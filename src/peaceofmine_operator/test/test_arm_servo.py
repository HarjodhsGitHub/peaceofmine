import unittest
from peaceofmine_operator.arm_servo import ArbotiX, ArmServo, limits
from unittest.mock import Mock, patch
from types import SimpleNamespace


class Bus:
    def __init__(self):
        self.values = {2: 40, 14: 1023, 30: 2000, 32: 3, 34: 1023, 38: 0, 40: 0, 46: 0, 68: 2048, 0: 310, 70: 0, 6: 0, 8: 4095, 24: 0, 36: 2000, 42: 120, 43: 25}
        self.writes = []
    def close(self):
        pass
    def read(self, ident, address, size=2):
        return self.values[address]
    def read_block(self, ident, address, length):
        data = bytearray(length)
        for register, value in self.values.items():
            size = 1 if register in (24, 42, 43, 46) else 2
            if address <= register and register + size <= address + length:
                data[register-address:register-address+size] = value.to_bytes(size, 'little')
        return bytes(data)
    def write(self, ident, address, value, size=2):
        self.writes.append((address, value))
        self.values[address] = value


class ArmServoTest(unittest.TestCase):
    def setUp(self):
        self.bus = Bus()
        self.servo = ArmServo(self.bus, 1)

    def test_unknown_limits_never_enable_torque(self):
        with self.assertRaises(ValueError):
            self.servo.request('center')
        self.servo.step(lambda: True)
        self.assertNotIn((24, 1), self.bus.writes)

    def test_uncalibrated_jog_has_fixed_small_window_and_stops_on_gate_loss(self):
        self.servo.request_jog(1)
        self.assertEqual(self.servo.target, 2056)
        self.servo.step(lambda: True)
        self.assertEqual(self.bus.values[32], 3)
        self.assertEqual(self.bus.values[30], 2016)
        self.bus.values[36] = 2040
        self.servo.request_jog(1)
        self.assertEqual(self.servo.target, 2056)  # Repeated packets cannot extend a hold.
        self.servo.step(lambda: False)
        self.assertFalse(self.servo.torque)
        self.assertIsNone(self.servo.target)
        self.assertIsNone(self.servo.jog_limits)

    def test_slow_jog_can_overcome_small_position_deadband(self):
        original = self.bus.write
        def motor(ident, address, value, size=2):
            original(ident, address, value, size)
            error = value - self.bus.values[36]
            if address == 30 and self.bus.values[24] and abs(error) > 3:
                self.bus.values[36] += 1 if error > 0 else -1
        self.bus.write = motor
        for _ in range(80):
            self.servo.request_jog(1)
            self.servo.step(lambda: True)
        self.assertGreater(self.bus.values[36], 2003)
        self.assertLessEqual(self.bus.values[36], 2056)
        self.assertEqual(self.bus.values[32], 3)

    def test_telemetry_reports_hardware_values_and_signed_units(self):
        self.bus.values.update({24: 1, 34: 512, 38: 1024 + 10, 40: 512, 68: 2148, 46: 1})
        data = self.servo.snapshot()
        self.assertTrue(data['torque'])
        self.assertTrue(data['moving'])
        self.assertAlmostEqual(data['load_percent'], 50.0489, places=3)
        self.assertAlmostEqual(data['torque_limit_percent'], 50.0489, places=3)
        self.assertAlmostEqual(data['current_a'], .45)
        self.assertAlmostEqual(data['speed_deg_s'], -6.84)

    def test_zero_torque_limit_explains_no_motion_without_reenabling(self):
        self.bus.values[34] = 0
        self.servo.request_jog(1)
        with self.assertRaisesRegex(ValueError, 'torque limit is 0'):
            self.servo.step(lambda: True)
        self.assertNotIn((24, 1), self.bus.writes)

    def test_jog_respects_recorded_limits_and_validates_direction(self):
        self.servo.configure(dict(minimum=1900, center=2000, maximum=2100))
        self.servo.request_jog(-1)
        self.assertEqual(self.servo.target, 1900)
        with self.assertRaises(ValueError):
            self.servo.request_jog(0)

    def test_capture_is_read_only_and_range_checked(self):
        before = list(self.bus.writes)
        self.assertEqual(self.servo.capture('center'), 2000)
        self.assertEqual(self.bus.writes, before)
        for values in [(0, 0, 4095), (3000, 2000, 1000), (0, 2048, 4096)]:
            with self.assertRaises(ValueError):
                limits(*values)

    def test_small_moves_and_px4_loss_disables_torque(self):
        self.servo.configure(dict(minimum=1500, center=2000, maximum=2500))
        self.servo.request('maximum')
        self.servo.step(lambda: True)
        self.assertIn((30, 2000), self.bus.writes)  # Goal set before torque.
        self.assertIn((24, 1), self.bus.writes)
        self.assertEqual(self.bus.writes[-1], (30, 2016))
        self.servo.step(lambda: False)
        self.assertEqual(self.bus.writes[-1], (24, 0))
        self.assertIsNone(self.servo.target)
        before = list(self.bus.writes)
        self.servo.step(lambda: True)
        self.assertEqual(self.bus.writes, before)  # No stale replay after recovery.

    def test_gate_rechecked_between_read_and_torque(self):
        self.servo.configure(dict(minimum=1500, center=2000, maximum=2500))
        self.servo.request('maximum')
        answers = iter([True, True, False])
        self.servo.step(lambda: next(answers))
        self.assertNotIn((24, 1), self.bus.writes)
        self.assertEqual(self.bus.writes[-1], (24, 0))


class ArbotiXStartupTest(unittest.TestCase):
    def test_waits_for_bootloader_without_reopening_port(self):
        port, packet = Mock(), Mock()
        packet.read1ByteTxRx.side_effect = [(0, -1, 0), (44, 0, 0)]
        packet.write1ByteTxRx.return_value = (0, 0)
        sdk = SimpleNamespace(PortHandler=Mock(return_value=port), PacketHandler=Mock(return_value=packet))
        with patch.dict('sys.modules', dynamixel_sdk=sdk), patch('peaceofmine_operator.arm_servo.time.sleep'):
            bus = ArbotiX('/dev/fake')
        sdk.PortHandler.assert_called_once_with('/dev/fake')
        self.assertEqual(packet.read1ByteTxRx.call_count, 2)
        packet.write1ByteTxRx.assert_called_once_with(port, 253, 4, 1)
        bus.close()
