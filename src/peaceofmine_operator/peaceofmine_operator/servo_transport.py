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


