import unittest
from peaceofmine_operator.servo_pid import apply_arm_pid


class Bus:
    def __init__(self):
        self.values = {24: 0, 28: 32, 27: 0, 26: 0}
        self.writes = []
        self.reject = False

    def read(self, ident, address, size=2):
        if ident == 253:
            return 1
        return 310 if address == 0 else self.values[address]

    def write(self, ident, address, value, size):
        self.writes.append((ident, address, value, size))
        if ident == 253:
            assert address == 81
            return
        assert address in (24, 26, 27, 28)
        assert address != 24 or value == 0
        if not (self.reject and address == 28 and value == 64):
            self.values[address] = value


class TestPid(unittest.TestCase):
    config = dict(arm=dict(servo_id=1), arm_pid=dict(servo_id=1, p=64, i=0, d=0))

    def test_apply_without_enabling_torque(self):
        bus = Bus()
        self.assertTrue(apply_arm_pid(bus, self.config))
        self.assertEqual(bus.values[28], 64)
        self.assertEqual(bus.values[24], 0)
        self.assertFalse(any(address == 82 for _, address, _, _ in bus.writes))

    def test_failure_restores_original(self):
        bus = Bus()
        bus.reject = True
        with self.assertRaisesRegex(RuntimeError, 'readback'):
            apply_arm_pid(bus, self.config)
        self.assertEqual(bus.values[28], 32)
        self.assertEqual(bus.values[24], 0)

    def test_wrong_servo_and_bad_gains_do_not_write(self):
        bus = Bus()
        with self.assertRaises(ValueError):
            apply_arm_pid(bus, self.config, 2)
        for p in (-1, 0, 255, True, 2.5):
            with self.assertRaises(ValueError):
                apply_arm_pid(bus, dict(arm_pid=dict(servo_id=1, p=p, i=0, d=0)))
        self.assertEqual(bus.writes, [])

    def test_absent_profile_preserves_existing_behavior(self):
        bus = Bus()
        self.assertFalse(apply_arm_pid(bus, {}))
        self.assertEqual(bus.writes, [])
