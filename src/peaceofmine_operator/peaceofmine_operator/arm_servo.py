"""MX-64 Protocol 1.0 transport and bounded, non-queued calibration moves."""
import time
import math


class FirmwareMotionFault(RuntimeError):
    """A responding controller rejected motion, not a disconnected port."""

    def __init__(self, code, message):
        super().__init__(message)
        self.code = code


def limits(minimum, center, maximum):
    values = (minimum, center, maximum)
    if any(type(v) is not int for v in values) or not 0 <= minimum < center < maximum <= 4095:
        raise ValueError('Require 0 <= minimum < center < maximum <= 4095 ticks')
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
            if self.read(253, 80, 1) != 1:
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
            raise RuntimeError(f'Servo telemetry read failed: id={ident}, register={address}, length={length}, '
                               f'communication={result}, device={error}, received={len(values)}, source={source}')
        return bytes(values)

    def write(self, ident, address, value, size=2):
        call = self.packet.write1ByteTxRx if size == 1 else self.packet.write2ByteTxRx
        result, error = call(self.port, ident, address, int(value))
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


class ArmServo:
    def __init__(self, bus, ident, calibration=None, speed=20):
        if type(ident) is not int or not 0 <= ident <= 252:
            raise ValueError('Choose a unicast servo ID from 0 to 252')
        if not 1 <= speed <= 1023:
            raise ValueError('Calibration speed must be 1..1023 (zero means unlimited)')
        self.bus, self.ident, self.speed = bus, ident, speed
        self.calibration = limits(**calibration) if calibration else None
        self.captured = {}
        self.target = None
        self.max_step = 16
        self.jog_limits = None
        self._applied_speed = None
        self.torque = False
        self.sweep_profile = None
        self._firmware_sweeping = False
        if bus.read(ident, 0) != 310:
            raise ValueError('Expected MX-64 Protocol 1.0 model 310')
        self.stop()
        if bus.read(ident, 70, 1) != 0:
            raise ValueError('Position control required; torque-control mode must be disabled')
        self.cw, self.ccw = bus.read(ident, 6), bus.read(ident, 8)
        if self.cw == self.ccw or self.ccw > 4095:
            raise ValueError('Joint mode required; this driver never rewrites EEPROM mode limits')
        self.firmware = bus.read(ident, 2, 1)
        self.original_acceleration = bus.read(ident, 73, 1)
        self.acceleration = self.original_acceleration
        self._applied_acceleration = self.original_acceleration
        self.max_torque = bus.read(ident, 14)
        self.position = bus.read(ident, 36)
        self.goal = self.position
        self._validate_limits()

    def _validate_limits(self):
        if self.calibration and not self.cw <= self.calibration['minimum'] < self.calibration['maximum'] <= self.ccw:
            raise ValueError('Calibration exceeds servo EEPROM limits')

    def stop(self):
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
        self.position = self.bus.read(self.ident, 36)
        self.captured[point] = self.position
        return self.position

    def configure(self, calibration):
        if self.torque:
            raise ValueError('Disarm before changing calibration')
        proposed = limits(**calibration)
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

    def request_position(self, position):
        low, high = self.movement_limits()
        if type(position) is not int or not low <= position <= high:
            raise ValueError('Position must be within servo EEPROM joint limits')
        self.jog_limits = (low, high)
        self.sweep_profile = None
        self.speed = 0  # MX-64 joint mode: maximum available speed.
        self.acceleration = self.original_acceleration
        self.max_step = 4095  # Send the user's target directly.
        self.target = position

    def step(self, allowed):
        # Recheck the independent safety gate before each serial motion write.
        if not allowed() or self.target is None:
            if self.torque:
                self.stop()
            self.target = None
            return
        if self._firmware_sweeping and self.sweep_profile is None:
            self.bus.write(self.ident, 24, 0, 1)
            self.torque = self._firmware_sweeping = False
        if not self.bus.prepare_motion(self.ident, allowed):
            self.stop()
            return
        self.position = self.bus.read(self.ident, 36)
        low, high = self.jog_limits or (self.calibration['minimum'], self.calibration['maximum'])
        if not low <= self.position <= high:
            self.stop()
            raise ValueError('Present position outside calibration; reposition with torque off')
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
        self.position = word(36)
        return dict(connected=True, servo_id=self.ident, model=310, firmware=self.firmware,
                    position=self.position, torque=bool(data[0]), commanded_torque=self.torque,
                    target=self.target, goal_position=word(30),
                    speed_deg_s=signed(word(38)) * .684,
                    speed_limit_deg_s=word(32) * .684,
                    load_percent=signed(word(40)) * 100 / 1023,
                    torque_limit_percent=word(34) * 100 / 1023,
                    max_torque_percent=self.max_torque * 100 / 1023,
                    current_a=(self.bus.read(self.ident, 68) - 2048) * .0045,
                    moving=bool(data[46 - 24]),
                    calibration=self.calibration, captured=dict(self.captured),
                    eeprom_minimum=self.cw, eeprom_maximum=self.ccw,
                    temperature_c=data[43 - 24], voltage=data[42 - 24] / 10)
