"""Driver safety invariants independent of controller or browser behavior."""
import unittest
from peaceofmine_interfaces.msg import ServoCommand
from peaceofmine_operator.servo_runtime import ServoRuntime
from peaceofmine_operator.simulated_servo_bus import SimulatedServoBus


class DriverTest(unittest.TestCase):
    def setUp(self):
        self.now=1.
        self.allowed=True
        self.runtime=ServoRuntime(SimulatedServoBus(),[1,2],lambda:self.allowed,lambda ident:(14,4),clock=lambda:self.now)

    def command(self,**changes):
        values=dict(owner='arm',operation_id='one',sequence=1,servo_id=1,mode=1,
                    target=2000,minimum=0,maximum=4095,speed=14,acceleration=4,lead=114)
        values.update(changes)
        return ServoCommand(**values)

    def tearDown(self):
        self.runtime.close()

    def test_watchdog_latches_even_if_controller_returns(self):
        self.runtime.submit(self.command())
        self.runtime.tick()
        self.assertTrue(self.runtime.bus.values[1][24])
        self.now+=.3
        self.runtime.enforce()
        self.assertFalse(self.runtime.bus.values[1][24])
        with self.assertRaisesRegex(ValueError,'retired'):
            self.runtime.submit(self.command(sequence=2))
        self.runtime.submit(self.command(operation_id='explicit-new-start',sequence=3))
        self.assertIsNotNone(self.runtime.active)

    def test_permission_loss_retires_operation(self):
        self.runtime.submit(self.command())
        self.allowed=False
        self.runtime.enforce()
        self.allowed=True
        with self.assertRaises(ValueError):self.runtime.submit(self.command(sequence=2))

    def test_all_motion_modes_enforce_speed_limit(self):
        for mode in (1,2):
            for speed in (0,15,1023):
                with self.subTest(mode=mode,speed=speed),self.assertRaises(ValueError):
                    self.runtime.submit(self.command(mode=mode,speed=speed))
        self.assertIsNone(self.runtime.active)

    def test_exclusive_owner_and_foreign_stop(self):
        self.runtime.submit(self.command())
        with self.assertRaises(ValueError):
            self.runtime.submit(self.command(owner='probe',servo_id=2))
        self.runtime.submit(self.command(owner='probe',mode=0))
        self.assertIsNotNone(self.runtime.active)
        self.runtime.submit(self.command(mode=0))
        self.assertIsNone(self.runtime.active)

    def test_duplicate_and_reordered_commands_cannot_renew(self):
        self.runtime.submit(self.command(sequence=3))
        for sequence in (3,2):
            with self.assertRaises(ValueError):self.runtime.submit(self.command(sequence=sequence))
        self.now+=.3
        self.runtime.enforce()
        self.assertIsNone(self.runtime.active)

    def test_invalid_bounds_rejected_before_torque(self):
        for changes in (dict(target=5000),dict(minimum=-1),dict(maximum=5000),dict(load_stop_percent=float('nan'))):
            with self.assertRaises(ValueError):self.runtime.submit(self.command(**changes))
        self.assertFalse(self.runtime.bus.values[1][24])

    def test_protective_load_stop_is_latched(self):
        self.runtime.submit(self.command(load_stop_percent=1.))
        self.runtime.tick()
        self.assertIsNone(self.runtime.active)
        self.assertEqual(self.runtime.outcomes[1][1],'load_stop')
        with self.assertRaises(ValueError):self.runtime.submit(self.command(sequence=2))
