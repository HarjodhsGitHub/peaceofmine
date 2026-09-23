"""MX-64 Protocol 1.0 transport and bounded, non-queued calibration moves."""
import time
import math


class ServoAlarm(RuntimeError):
    """A complete servo reply carrying an alarm and usable register data."""

    def __init__(self, ident, address, code, data, message):
        names = ('input voltage', 'angle limit', 'overheating', 'range', 'checksum', 'overload', 'instruction')
        alarms = ', '.join(name for bit, name in enumerate(names) if code & (1 << bit))
        super().__init__(f'{message}; servo alarm: {alarms}')
        self.ident, self.address, self.code, self.data = ident, address, code, bytes(data)


class FirmwareMotionFault(RuntimeError):
    """A responding controller rejected motion, not a disconnected port."""

    def __init__(self, code, message):
        super().__init__(message)
        self.code = code


def limits(minimum, center, maximum, low=0, high=4095):
    values = (minimum, center, maximum)
    if any(type(v) is not int for v in values) or not low <= minimum < center < maximum <= high:
        raise ValueError(f'Require {low} <= minimum < center < maximum <= {high} ticks')
    return dict(minimum=minimum, center=center, maximum=maximum)


class ArbotiX:
    def __init__(self, device, bus_baud=1000000):
        from dynamixel_sdk import PortHandler, PacketHandler
        self.port = PortHandler(device)
        self.packet = PacketHandler(1.0)
        self._motion_id = None
        self._sweep_profile = None
        self.read_retries = 0
        if bus_baud != 1000000:
            raise ValueError('Safe arm firmware requires a 1000000 baud servo bus')
        if not self.port.setBaudRate(115200):
            raise RuntimeError('Cannot open ArbotiX serial port')
        try:
            # Opening serial can reset the ArbotiX into its bootloader.
            # Keep the port open while waiting, as in the proven demo.
            deadline = time.monotonic() + 5.0
            while True:
                try:
                    if self.read(253, 0, 1) == 44:
                        break
                except RuntimeError:
                    pass
                if time.monotonic() >= deadline:
                    raise RuntimeError('Expected ArbotiX ROS gateway at ID 253 within 5 seconds')
                time.sleep(.05)
            if self.read(253, 80, 1) not in (1, 2):
                raise RuntimeError('Install safe_arm firmware v1 before enabling the arm watchdog')
        except Exception:
            self.port.closePort()
            raise

    def read(self, ident, address, size=2):
        # SDK typed helpers index data before checking device errors. Gateway
        # errors (including absent scan IDs) legitimately have no data payload.
        return int.from_bytes(self.read_block(ident, address, size), 'little')

    def read_block(self, ident, address, length):
        values, result, error = self.packet.readTxRx(self.port, ident, address, length)
        # Gateway status 32 can report one failed servo-bus read. Retry only
        # this read, once; never retry motion writes or renew permission here.
        if result == 0 and error == 32 and len(values) < length:
            self.read_retries = getattr(self, 'read_retries', 0) + 1
            values, result, error = self.packet.readTxRx(self.port, ident, address, length)
        if result != 0 or error or len(values) != length:
            source = 'servo alarm' if result == 0 and error and len(values) == length else 'bus/transport'
            message = (f'Servo telemetry read failed: id={ident}, register={address}, length={length}, '
                       f'communication={result}, device={error}, received={len(values)}, source={source}')
            if source == 'servo alarm' and ident != 253:
                raise ServoAlarm(ident, address, error, values, message)
            raise RuntimeError(message)
        return bytes(values)

    def write(self, ident, address, value, size=2):
        call = self.packet.write1ByteTxRx if size == 1 else self.packet.write2ByteTxRx
        # Register 83 performs three EEPROM writes with 20 ms settling each.
        # The SDK resets its ~34 ms timeout inside txRxPacket, so extending it
        # beforehand has no effect. Scope the override to this stopped-only
        # command; never retry writes or lengthen motion/heartbeat timeouts.
        setup = ident == 253 and address == 83 and size == 1 and value == 1
        original_timeout = self.port.setPacketTimeout if setup else None
        if setup:
            self.port.setPacketTimeout = lambda length: self.port.setPacketTimeoutMillis(250)
        try:
            result, error = call(self.port, ident, address, int(value) & (255 if size == 1 else 65535))
        finally:
            if setup:
                self.port.setPacketTimeout = original_timeout
        if result == 0 and ident != 253 and address == 24 and value == 0:
            self._motion_id = self._sweep_profile = None
        if result == 0 and error and ident != 253:
            raise ServoAlarm(ident, address, error, [], f'Servo write alarm: id={ident}, register={address}, device={error}')
        if result != 0 or error:
            detail = ''
            fault = 0
            if result == 0:
                try:
                    fault = self.read(253, 98, 1)
                    reason = {2: 'permission watchdog expired', 3: 'position read failed or outside limits',
                              4: 'servo torque disabled or torque limit zero', 5: 'no encoder progress for 2 seconds during sweep',
                              6: 'invalid sweep profile', 7: 'sweep limits/position check failed',
                              8: 'sweep torque-limit read failed or zero',
                              9: 'servo did not confirm startup writes',
                              10: 'endpoint did not settle within 1.5 seconds'}.get(fault, 'no latched fault')
                    detail = f', firmware_fault={fault} ({reason})'
                except RuntimeError:
                    detail = ', firmware fault read unavailable'
            message = (f'Servo write failed: id={ident}, register={address}, value={value}, '
                       f'size={size}, communication={result}, device={error}{detail}')
            if result == 0 and fault:
                raise FirmwareMotionFault(fault, message)
            raise RuntimeError(message)
        if ident != 253 and address == 24 and value == 0:
            self._motion_id = self._sweep_profile = None

    def prepare_motion(self, ident, allowed):
        operations = [(253, 82, 2, 1)]
        if self._motion_id is None:
            operations = [(253, 81, ident, 1), (253, 82, 1, 1)]
        elif self._motion_id != ident:
            raise RuntimeError('Stop before selecting a different servo')
        for operation in operations:
            if not allowed():
                self.write(ident, 24, 0, 1)
                return False
            try:
                self.write(*operation)
            except RuntimeError as error:
                try:
                    fault = self.read(253, 98, 1)
                except RuntimeError:
                    raise error
                reason = {2: 'permission watchdog expired', 3: 'position read failed or outside limits',
                          4: 'servo torque disabled or torque limit zero', 5: 'no encoder progress for 2 seconds during sweep',
                          6: 'invalid sweep profile', 7: 'sweep limits/position check failed',
                          8: 'sweep torque-limit read failed or zero',
                          9: 'servo did not confirm startup writes',
                          10: 'endpoint did not settle within 1.5 seconds'}.get(fault)
                if reason:
                    raise FirmwareMotionFault(fault, f'Arm firmware stopped: {reason}') from error
                raise
        self._motion_id = ident
        return True

    def run_sweep(self, ident, low, high, speed, acceleration, tolerance, allowed):
        profile = (ident, low, high, speed, acceleration, tolerance)
        if profile == self._sweep_profile:
            return True
        if (self._sweep_profile is not None and
                profile[:3] + profile[5:] == self._sweep_profile[:3] + self._sweep_profile[5:]):
            for address, value, size, old in ((88, speed, 2, self._sweep_profile[3]),
                                               (90, acceleration, 1, self._sweep_profile[4])):
                if value == old:
                    continue
                if not allowed():
                    self.write(ident, 24, 0, 1)
                    return False
                self.write(253, address, value, size)
            self._sweep_profile = profile
            return True
        if self._sweep_profile is not None:
            self.write(ident, 24, 0, 1)
            if not self.prepare_motion(ident, allowed):
                return False
        for address, value, size in ((84, low, 2), (86, high, 2), (88, speed, 2),
                                     (90, acceleration, 1), (91, tolerance, 2), (94, 1, 1)):
            if not allowed():
                self.write(ident, 24, 0, 1)
                return False
            self.write(253, address, value, size)
        self._sweep_profile = profile
        return True

    def close(self):
        try:
            if self._motion_id is not None:
                self.write(self._motion_id, 24, 0, 1)
        except RuntimeError:
            pass  # A broken link is handled by the firmware's permission timeout.
        finally:
            self.port.closePort()

    def enable_multiturn(self, ident):
        if self.read(253, 80, 1) < 2:
            raise ValueError('Install safe_arm firmware v2 before enabling multi-turn')
        self.write(ident, 24, 0, 1)
        self.write(253, 81, ident, 1)
        self.write(253, 83, 1, 1)
        deadline = time.monotonic() + .5
        while True:
            if (self.read(ident, 6) == self.read(ident, 8) == 4095
                    and self.read(ident, 22, 1) == 1 and self.read(ident, 24, 1) == 0):
                return
            if time.monotonic() >= deadline:
                raise RuntimeError('Multi-turn EEPROM readback failed; reconnect before moving')
            time.sleep(.02)


class ArmServo:
    def __init__(self, bus, ident, calibration=None, speed=20):
        if type(ident) is not int or not 0 <= ident <= 252:
            raise ValueError('Choose a unicast servo ID from 0 to 252')
        if not 1 <= speed <= 1023:
            raise ValueError('Calibration speed must be 1..1023 (zero means unlimited)')
        self.bus, self.ident, self.speed = bus, ident, speed
        self.calibration = None
        self.captured = {}
        self.target = None
        self.max_step = 16
        self.jog_limits = None
        self._applied_speed = None
        self.torque = False
        self.sweep_profile = None
        self._firmware_sweeping = False
        self.home_position = None
        self.home_status = 'Not homed'
        self._home = None
        self._home_request = None
        self._extension_request = None
        self._extension_active = False
        if bus.read(ident, 0) != 310:
            raise ValueError('Expected MX-64 Protocol 1.0 model 310')
        self.stop()
        if bus.read(ident, 70, 1) != 0:
            raise ValueError('Position control required; torque-control mode must be disabled')
        self.cw, self.ccw = bus.read(ident, 6), bus.read(ident, 8)
        self.multiturn = self.cw == self.ccw == 4095
        if self.multiturn:
            if bus.read(253, 80, 1) < 2:
                raise ValueError('Multi-turn requires safe_arm firmware v2')
            if bus.read(ident, 22, 1) != 1:
                raise ValueError('Multi-turn requires resolution divider 1')
            self.cw, self.ccw = -28672, 28672
        elif self.cw == self.ccw or self.ccw > 4095:
            raise ValueError('Joint mode required; this driver never rewrites EEPROM mode limits')
        self.calibration = limits(**calibration, low=self.cw, high=self.ccw) if calibration else None
        self.firmware = bus.read(ident, 2, 1)
        self.original_acceleration = bus.read(ident, 73, 1)
        self.acceleration = self.original_acceleration
        self._applied_acceleration = self.original_acceleration
        self.max_torque = bus.read(ident, 14)
        self.position = self.decode_position(bus.read(ident, 36))
        self.goal = self.position
        self._validate_limits()

    def _validate_limits(self):
        if self.calibration and not self.cw <= self.calibration['minimum'] < self.calibration['maximum'] <= self.ccw:
            raise ValueError('Calibration exceeds servo EEPROM limits')

    def decode_position(self, raw):
        return raw - 65536 if self.multiturn and raw >= 32768 else raw

    def stop(self):
        self._extension_active = False
        if self._home is not None:
            self.home_status = 'Homing interrupted; zero not set'
        self._home = None
        self.sweep_profile = None
        self._firmware_sweeping = False
        self.jog_limits = None
        self.target = None
        self.bus.write(self.ident, 24, 0, 1)
        self.torque = False

    def capture(self, point):
        if point not in ('minimum', 'center', 'maximum'):
            raise ValueError('Unknown calibration point')
        if self.torque or self.bus.read(self.ident, 24, 1):
            raise ValueError('Disarm and support the arm before recording positions')
        self.position = self.decode_position(self.bus.read(self.ident, 36))
        self.captured[point] = self.position
        return self.position

    def configure(self, calibration):
        if self.torque:
            raise ValueError('Disarm before changing calibration')
        proposed = limits(**calibration, low=self.cw, high=self.ccw)
        if not self.cw <= proposed['minimum'] < proposed['maximum'] <= self.ccw:
            raise ValueError('Calibration exceeds servo EEPROM limits')
        self.calibration = proposed

    def request(self, point):
        if not self.calibration or point not in self.calibration:
            raise ValueError('Record and apply minimum, center and maximum first')
        self.jog_limits = None
        self.sweep_profile = None
        self.target = self.calibration[point]
        self.acceleration = self.original_acceleration

    def request_sweep(self, point, speed, acceleration, tolerance=23):
        if self.multiturn:
            raise ValueError('Firmware arm sweeps require joint mode; multi-turn is for probe travel')
        self.request(point)
        if not 1 <= speed <= 1023 or not 1 <= acceleration <= 254:
            raise ValueError('Sweep speed must be 1..1023 and acceleration 1..254')
        span = self.calibration['maximum'] - self.calibration['minimum']
        tolerance = min(tolerance, span // 4)
        if tolerance < 1:
            raise ValueError('Sweep range is too narrow')
        self.sweep_profile = (self.calibration['minimum'], self.calibration['maximum'],
                              speed, acceleration, tolerance)
        self.speed = speed
        self.acceleration = acceleration
        self.max_step = 4095

    def movement_limits(self):
        # Calibration must be able to reach new endpoints outside saved bounds.
        return self.cw, self.ccw

    def request_jog(self, direction, speed_deg_s=2.0):
        if type(direction) is not int or direction not in (-1, 1):
            raise ValueError('Jog direction must be left or right')
        if isinstance(speed_deg_s, bool) or not isinstance(speed_deg_s, (int, float)) or not math.isfinite(speed_deg_s) or speed_deg_s <= 0:
            raise ValueError('Speed must be a finite positive number')
        if speed_deg_s > 1023 * .684:
            raise ValueError('Speed exceeds the MX-64 speed register range (1023 units)')
        self.jog_limits = self.movement_limits()
        self.sweep_profile = None
        self.speed = max(1, round(speed_deg_s / .684))
        self.acceleration = self.original_acceleration
        # Jog continuously while held; keep only a small outstanding goal.
        self.max_step = 56
        self.target = self.jog_limits[0 if direction < 0 else 1]

    def request_home(self, request, load_percent):
        # Repeated deadman packets must never restart a completed/failed search.
        if not isinstance(request, str) or not 1 <= len(request) <= 100:
            raise ValueError('Homing requires a unique hold identifier')
        if request == self._home_request:
            return
        if (isinstance(load_percent, bool) or not isinstance(load_percent, (int, float))
                or not math.isfinite(load_percent) or not 10 <= load_percent <= 60):
            raise ValueError('Home load threshold must be 10..60 percent')
        self.stop()
        self._home_request = request
        self.home_position = None
        self.request_jog(1, 50.0)
        self.max_step = 114  # 10 degree lead supports 50 degrees/s at the host update rate.
        self._home = dict(start=time.monotonic(), threshold=load_percent)
        self.home_status = 'Seeking top at 50 degrees/s; release to stop'

    def _check_home(self):
        search = self._home
        now = time.monotonic()
        if now - search['start'] >= 60:
            self.stop()
            self.home_status = 'Home failed: 60 second timeout; zero not set'
            return False
        if self._home_contact(self.bus.read(self.ident, 40)):
            return False
        if self.position >= self.ccw - 3:
            self.stop()
            self.home_status = 'Home failed: position range limit reached without confirmed resistance'
            return False
        return True

    def _home_contact(self, raw):
        if self._home is None:
            return False
        if not 0 <= raw <= 2047:
            raise ValueError('Invalid load telemetry during homing')
        # Direction bit is irrelevant to contact. Do not wait for the encoder
        # to stop: slipping/skipping teeth can keep the shaft moving at a stop.
        if (raw & 1023) * 100 / 1023 >= self._home['threshold']:
            position = self.position
            self.stop()
            self.home_position = position
            self.home_status = 'Load threshold reached; torque released and zero set. Verify top visually.'
            return True
        return False

    def request_position(self, position):
        low, high = self.movement_limits()
        if type(position) is not int or not low <= position <= high:
            raise ValueError('Position must be within the supported servo position range')
        self.jog_limits = (low, high)
        self.sweep_profile = None
        self.speed = 0  # MX-64 joint mode: maximum available speed.
        self.acceleration = self.original_acceleration
        self.max_step = high - low  # Send the user's target directly, including multiple turns.
        self.target = position

    def request_extension(self, request, depth, calibration):
        if not isinstance(request, str) or not request:
            raise ValueError('Probe target requires a request identifier')
        if request == self._extension_request:
            return
        if self.home_position is None:
            raise ValueError('Home the probe first')
        if not calibration or calibration['servo_id'] != self.ident:
            raise ValueError('Save maximum probe extension in Settings first')
        maximum = calibration['max_extension_mm']
        if isinstance(depth, bool) or not isinstance(depth, (int, float)) or not math.isfinite(depth) or not 0 <= depth <= maximum:
            raise ValueError('Probe target exceeds saved maximum extension')
        low, high = self.home_position - calibration['travel_ticks'], self.home_position
        if not self.cw <= low < high <= self.ccw:
            raise ValueError('Saved probe travel exceeds current position range; home/calibrate again')
        self.request_position(round(high - depth / maximum * calibration['travel_ticks']))
        self.jog_limits = (low, high)
        self.speed = 73  # Approximately 50 degrees/s, never register zero.
        self.max_step = 114
        self._extension_request = request
        self._extension_active = True

    def step(self, allowed):
        # Recheck the independent safety gate before each serial motion write.
        if not allowed() or self.target is None:
            if self.torque or self._home is not None:
                self.stop()
            self.target = None
            return
        if self._firmware_sweeping and self.sweep_profile is None:
            self.bus.write(self.ident, 24, 0, 1)
            self.torque = self._firmware_sweeping = False
        if not self.bus.prepare_motion(self.ident, allowed):
            self.stop()
            return
        self.position = self.decode_position(self.bus.read(self.ident, 36))
        if self._home is not None and not self._check_home():
            return
        low, high = self.jog_limits or (self.calibration['minimum'], self.calibration['maximum'])
        if not low <= self.position <= high:
            self.stop()
            raise ValueError('Present position outside calibration; reposition with torque off')
        if self._extension_active and abs(self.target - self.position) <= 3:
            self.stop()
            return
        if self.sweep_profile is not None:
            if not self.bus.run_sweep(self.ident, *self.sweep_profile, allowed):
                self.stop()
                return
            self.torque = self._firmware_sweeping = True
            # Firmware now owns endpoint reversals; only permission is refreshed.
            self._applied_speed = self.speed
            self._applied_acceleration = self.acceleration
            return
        if self._applied_acceleration != self.acceleration:
            if not allowed():
                self.stop()
                return
            self.bus.write(self.ident, 73, self.acceleration, 1)
            self._applied_acceleration = self.acceleration
        if not self.torque:
            if self.bus.read(self.ident, 34) == 0:
                self.stop()
                raise ValueError('Servo torque limit is 0%; resolve servo shutdown/configuration before moving')
            for address, value, size in ((30, self.position, 2), (32, self.speed, 2), (24, 1, 1)):
                if not allowed():
                    self.stop()
                    return
                self.bus.write(self.ident, address, value, size)
            self.torque = True
            self._applied_speed = self.speed
            self.goal = self.position
        if self._applied_speed != self.speed:
            if not allowed():
                self.stop()
                return
            self.bus.write(self.ident, 32, self.speed)
            self._applied_speed = self.speed
        # Jog/preset moves use a bounded lead; slider moves send the chosen target.
        goal = max(low, min(high, self.position + max(-self.max_step, min(self.max_step, self.target - self.position))))
        if goal != self.goal:
            if not allowed():
                self.stop()
                return
            self.bus.write(self.ident, 30, goal)
            self.goal = goal

    def snapshot(self):
        # One contiguous read keeps telemetry from delaying command/safety handling.
        data = self.bus.read_block(self.ident, 24, 23)
        def word(address):
            offset = address - 24
            return int.from_bytes(data[offset:offset + 2], 'little')
        def signed(value):
            return (value & 1023) * (-1 if value & 1024 else 1)
        self.position = self.decode_position(word(36))
        # A spike seen by the dashboard telemetry read must also stop homing,
        # even if it falls between the dedicated motion-loop load reads.
        contact = self._home_contact(word(40))
        return dict(connected=True, servo_id=self.ident, model=310, firmware=self.firmware,
                    sample_time=time.monotonic(),
                    position=self.position, torque=False if contact else bool(data[0]), commanded_torque=self.torque,
                    target=self.target, goal_position=self.decode_position(word(30)),
                    position_mode='multi-turn' if self.multiturn else 'joint',
                    position_minimum=self.cw, position_maximum=self.ccw,
                    home_position=self.home_position, home_status=self.home_status,
                    homing=self._home is not None,
                    speed_deg_s=signed(word(38)) * .684,
                    speed_limit_deg_s=word(32) * .684,
                    load_percent=signed(word(40)) * 100 / 1023,
                    torque_limit_percent=word(34) * 100 / 1023,
                    max_torque_percent=self.max_torque * 100 / 1023,
                    current_a=(self.bus.read(self.ident, 68) - 2048) * .0045,
                    moving=bool(data[46 - 24]),
                    calibration=self.calibration, captured=dict(self.captured),
                    eeprom_minimum=4095 if self.multiturn else self.cw,
                    eeprom_maximum=4095 if self.multiturn else self.ccw,
                    temperature_c=data[43 - 24], voltage=data[42 - 24] / 10)
