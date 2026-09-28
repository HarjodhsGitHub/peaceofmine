"""Bounded servo motion primitives used by actuator controllers."""
import math
import time

def limits(minimum, center, maximum, low=0, high=4095):
    values = (minimum, center, maximum)
    if any(type(v) is not int for v in values) or not low <= minimum < center < maximum <= high:
        raise ValueError(f'Require {low} <= minimum < center < maximum <= {high} ticks')
    return dict(minimum=minimum, center=center, maximum=maximum)


class ServoMotor:
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




    def request_position(self, position):
        low, high = self.movement_limits()
        if type(position) is not int or not low <= position <= high:
            raise ValueError('Position must be within the supported servo position range')
        self.jog_limits = (low, high)
        self.sweep_profile = None
        self.speed = max(1, self.speed)  # Zero is unlimited on MX-64.
        self.acceleration = self.original_acceleration
        self.max_step = high - low  # Send the user's target directly, including multiple turns.
        self.target = position


    def step(self, allowed):
        # Recheck the independent safety gate before each serial motion write.
        if not allowed() or self.target is None:
            if self.torque or self._home is not None or self._extension_active:
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

    def snapshot(self, fast=False):
        # One contiguous read keeps telemetry from delaying command/safety handling.
        data = self.bus.read_block(self.ident, 24, 18 if fast else 23)
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
                    **({} if fast else dict(moving=bool(data[46 - 24]))),
                    calibration=self.calibration, captured=dict(self.captured),
                    eeprom_minimum=4095 if self.multiturn else self.cw,
                    eeprom_maximum=4095 if self.multiturn else self.ccw,
                    **({} if fast else dict(temperature_c=data[43 - 24], voltage=data[42 - 24] / 10)))

    def _home_contact(self, raw):
        return False

    def _check_home(self):
        return True
