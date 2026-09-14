"""Exercise the actual ROS driver with an in-memory serial bus (no hardware)."""
import importlib.util
from pathlib import Path
import unittest
from unittest.mock import patch
import json

import rclpy
from std_msgs.msg import Bool, String
from peaceofmine_operator.rc_safety import RcSafety
from peaceofmine_operator.arm_servo import ArmServo
try:
    from test_arm_servo import Bus
    from test_rc_safety import healthy, state, rc
except ImportError:
    from .test_arm_servo import Bus
    from .test_rc_safety import healthy, state, rc

spec = importlib.util.spec_from_file_location('arm_node', Path(__file__).parents[1] / 'scripts/arm_servo_node.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class ArmNodeTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.now = 100.0
        self.clock = patch.object(module.time, 'monotonic', side_effect=lambda: self.now)
        self.clock.start()
        self.node = module.ArmServoNode()
        self.node.safety = RcSafety(clock=lambda: self.now)
        healthy(self.node.safety)
        self.bus = Bus()
        self.node.bus = self.bus
        self.node.servo = ArmServo(self.bus, 1, dict(minimum=1500, center=2000, maximum=2500))
        self.node.permission_cb(Bool(data=True))

    def tearDown(self):
        self.node.destroy_node()
        self.clock.stop()

    def move(self):
        self.node.command_cb(String(data=json.dumps(dict(action='move', held=True, point='maximum'))))
        self.node.tick()
        self.assertTrue(self.node.servo.torque)

    def test_jog_without_limits_and_command_feedback(self):
        self.node.servo.calibration = None
        self.node.command_cb(String(data=json.dumps(dict(action='jog', direction=1, held=True, request_id='jog1'))))
        self.node.tick()
        self.assertTrue(self.node.servo.torque)
        self.assertTrue(self.node.last_command['success'])
        self.now += .26
        self.node.tick()
        self.assertFalse(self.node.servo.torque)
        self.node.command_cb(String(data=json.dumps(dict(action='capture', point='center', request_id='record1'))))
        self.assertTrue(self.node.last_command['success'])
        self.assertIn('Recorded center', self.node.last_command['message'])
        self.now += .1
        self.node.command_cb(String(data=json.dumps(dict(action='jog', direction=1, held=True, request_id='stale'))))
        self.assertFalse(self.node.last_command['success'])
        self.assertFalse(self.node.servo.torque)

    def test_px4_disarm_immediately_drops_torque(self):
        self.move()
        self.node.state_cb(state(2000000, armed=False, system_status=3))
        self.assertFalse(self.node.servo.torque)
        self.assertEqual(self.bus.writes[-1], (24, 0))
        healthy(self.node.safety, 3000000)
        self.node.tick()
        self.assertFalse(self.node.servo.torque)

    def test_command_timeout_and_lease_loss(self):
        self.move()
        self.now += .26
        self.node.tick()
        self.assertFalse(self.node.servo.torque)

        self.node.permission_cb(Bool(data=True))
        self.move()
        self.node.permission_cb(Bool(data=False))
        self.assertFalse(self.node.servo.torque)

    def test_destroy_disables_torque_before_serial_close(self):
        self.move()
        with patch.object(self.bus, 'close', side_effect=lambda: self.assertEqual(self.bus.writes[-1], (24, 0))):
            self.node.destroy_node()
        self.assertFalse(self.node.servo.torque)
        self.assertIsNone(self.node.servo.target)

    def test_sweep_ignores_rc_override_but_kill_clears_target(self):
        self.node.sweep_cb(Bool(data=True))
        self.node.tick()
        self.assertTrue(self.node.servo.torque)
        self.node.safety.update_rc(rc(2000000, pwm=1500))
        self.node.tick()
        self.assertTrue(self.node.sweeping)
        self.assertTrue(self.node.servo.torque)
        self.node.state_cb(state(2000000, system_status=8))
        self.assertFalse(self.node.sweeping)
        self.assertFalse(self.node.servo.torque)
        healthy(self.node.safety, 3000000)
        self.node.tick()
        self.assertFalse(self.node.servo.torque)
