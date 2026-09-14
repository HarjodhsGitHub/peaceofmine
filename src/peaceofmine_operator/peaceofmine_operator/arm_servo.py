"""MX-64 Protocol 1.0 transport and bounded, non-queued calibration moves."""
import time


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
        if bus_baud not in (1000000, 57600):
            raise ValueError('Supported bus baud rates: 1000000 and 57600')
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
            self.write(253, 4, 1 if bus_baud == 1000000 else 34, 1)
        except Exception:
            self.port.closePort()
            raise

    def read(self, ident, address, size=2):
        call = self.packet.read1ByteTxRx if size == 1 else self.packet.read2ByteTxRx
        value, result, error = call(self.port, ident, address)
        if result != 0 or error:
            raise RuntimeError(f'Servo read failed: communication={result}, device={error}')
        return value

    def read_block(self, ident, address, length):
        values, result, error = self.packet.readTxRx(self.port, ident, address, length)
        if result != 0 or error or len(values) != length:
            raise RuntimeError(f'Servo telemetry read failed: communication={result}, device={error}')
        return bytes(values)

    def write(self, ident, address, value, size=2):
        call = self.packet.write1ByteTxRx if size == 1 else self.packet.write2ByteTxRx
        result, error = call(self.port, ident, address, int(value))
        if result != 0 or error:
            raise RuntimeError(f'Servo write failed: communication={result}, device={error}')

    def close(self):
        self.port.closePort()


class ArmServo:
    def __init__(self, bus, ident, calibration=None, speed=20):
        if type(ident) is not int or not 0 <= ident <= 252:
            raise ValueError('Choose a unicast servo ID from 0 to 252')
        if not 1 <= speed <= 80:
            raise ValueError('Calibration speed must be 1..80 (zero means unlimited)')
        self.bus, self.ident, self.speed = bus, ident, speed
        self.calibration = limits(**calibration) if calibration else None
        self.captured = {}
        self.target = None
        self.max_step = 16
        self.jog_limits = None
        self._applied_speed = None
        self.torque = False
        if bus.read(ident, 0) != 310:
            raise ValueError('Expected MX-64 Protocol 1.0 model 310')
        self.stop()
        if bus.read(ident, 70, 1) != 0:
            raise ValueError('Position control required; torque-control mode must be disabled')
        self.cw, self.ccw = bus.read(ident, 6), bus.read(ident, 8)
        if self.cw == self.ccw or self.ccw > 4095:
            raise ValueError('Joint mode required; this driver never rewrites EEPROM mode limits')
        self.firmware = bus.read(ident, 2, 1)
        self.max_torque = bus.read(ident, 14)
        self.position = bus.read(ident, 36)
        self.goal = self.position
        self._validate_limits()

    def _validate_limits(self):
        if self.calibration and not self.cw <= self.calibration['minimum'] < self.calibration['maximum'] <= self.ccw:
            raise ValueError('Calibration exceeds servo EEPROM limits')

    def stop(self):
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
        self.target = self.calibration[point]

    def request_jog(self, direction):
        if type(direction) is not int or direction not in (-1, 1):
            raise ValueError('Jog direction must be left or right')
        if self.jog_limits is None:
            position = self.bus.read(self.ident, 36)
            if not self.cw <= position <= self.ccw:
                raise ValueError('Present position is outside servo joint limits')
            if self.calibration:
                low, high = self.calibration['minimum'], self.calibration['maximum']
            else:
                # Unknown mechanical limits: at most five degrees per hold.
                low, high = max(self.cw, position - 56), min(self.ccw, position + 56)
            if not low <= position <= high:
                raise ValueError('Position outside saved limits; reposition with torque off')
            self.jog_limits = (low, high)
        self.speed = 3  # MX-64: approximately 2 degrees/second.
        self.max_step = 16  # Let the hardware speed controller move beyond tiny position errors.
        self.target = self.jog_limits[0 if direction < 0 else 1]

    def step(self, allowed):
        # Recheck the independent safety gate before each serial motion write.
        if not allowed() or self.target is None:
            if self.torque:
                self.stop()
            self.target = None
            return
        self.position = self.bus.read(self.ident, 36)
        low, high = self.jog_limits or (self.calibration['minimum'], self.calibration['maximum'])
        if not low <= self.position <= high:
            self.stop()
            raise ValueError('Present position outside calibration; reposition with torque off')
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
        if self._applied_speed != self.speed:
            if not allowed():
                self.stop()
                return
            self.bus.write(self.ident, 32, self.speed)
            self._applied_speed = self.speed
        # Only a small move can remain outstanding if the process/serial link dies.
        goal = max(low, min(high, self.position + max(-self.max_step, min(self.max_step, self.target - self.position))))
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
