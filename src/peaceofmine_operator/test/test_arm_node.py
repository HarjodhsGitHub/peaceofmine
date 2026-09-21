"""Exercise the actual ROS driver with an in-memory serial bus (no hardware)."""
import importlib.util
from pathlib import Path
import unittest
from unittest.mock import patch
from concurrent.futures import Future
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
        self.node.discovered_servos = [1, 2]
        self.node.servo = ArmServo(self.bus, 1, dict(minimum=1500, center=2000, maximum=2500))
        self.node.permission_cb(Bool(data=True))

    def tearDown(self):
        self.node.destroy_node()
        self.clock.stop()

    def move(self):
        self.node.command_cb(String(data=json.dumps(dict(action='move', held=True, point='maximum'))))
        self.node.tick()
        self.assertTrue(self.node.servo.torque)

    def test_scan_is_read_only_and_finds_both_servos_at_fixed_baud(self):
        def read(ident, address, size=2):
            if ident in (1, 2):
                return 310
            raise RuntimeError('No reply')
        with patch.object(module, 'ArbotiX', return_value=self.bus) as factory, \
                patch.object(self.bus, 'read', side_effect=read):
            before = list(self.bus.writes)
            bus, device, ids = self.node.discover('/dev/fake')
        self.assertIs(bus, self.bus)
        self.assertEqual(ids, [1, 2])
        self.assertEqual(self.bus.writes, before)
        factory.assert_called_once_with('/dev/fake', 1000000)

    def test_pending_scan_does_not_block_tick_or_select_a_servo(self):
        self.node.bus = self.node.servo = None
        self.node.discovery = Future()
        self.node.tick()
        self.assertIsNone(self.node.servo)
        self.assertFalse(self.node.discovery.done())
        self.node.discovery.set_result((self.bus, '/dev/fake', [1, 2]))
        self.node.tick()
        self.assertEqual(self.node.discovered_servos, [1, 2])
        self.assertIsNone(self.node.servo)
        self.assertNotIn((24, 1), self.bus.writes)

    def test_scan_cancellation_closes_port_without_motion(self):
        with patch.object(module, 'ArbotiX', return_value=self.bus), \
                patch.object(self.bus, 'read', side_effect=lambda *args: self.node.discovery_stop.set() or 310), \
                patch.object(self.bus, 'close') as close:
            self.assertIsNone(self.node.discover('/dev/fake'))
            close.assert_called_once()
        self.assertNotIn((24, 1), self.bus.writes)

    def test_failed_startup_scan_is_not_retried(self):
        self.node.bus = self.node.servo = None
        future = Future()
        future.set_exception(RuntimeError('No adapter'))
        with patch.object(self.node.discovery_pool, 'submit', return_value=future) as submit:
            self.node.tick()  # Schedule once without blocking the ROS callback.
            self.node.tick()  # Report failure.
            for _ in range(5):
                self.now += 10
                self.node.tick()
            submit.assert_called_once_with(self.node.discover, 'auto')
        self.assertEqual(self.node.reason, 'No adapter')

    def test_stop_does_not_hide_discovery_failure(self):
        self.node.fail(RuntimeError('Adapter did not answer'))
        self.node.command_cb(String(data=json.dumps(dict(action='stop'))))
        self.assertEqual(self.node.reason, 'Adapter did not answer')
        self.assertEqual(self.node.connection_error, 'Adapter did not answer')

    def test_disconnect_after_scan_does_not_restart_discovery(self):
        self.node.bus = self.node.servo = None
        self.node.discovery = Future()
        self.node.discovery.set_result((self.bus, '/dev/fake', [1, 2]))
        self.node.tick()
        self.node.fail(RuntimeError('Disconnected'))
        with patch.object(self.node.discovery_pool, 'submit') as submit:
            self.now += 10
            self.node.tick()
            submit.assert_not_called()

    def test_jog_without_limits_and_command_feedback(self):
        self.node.servo.calibration = None
        self.node.command_cb(String(data=json.dumps(dict(action='jog', direction=1, held=True, request_id='jog1'))))
        self.node.tick()
        self.assertTrue(self.node.servo.torque)
        self.assertTrue(self.node.last_command['success'])
        self.now += .26
        self.node.tick()
        self.assertFalse(self.node.servo.torque)

    def test_slider_command_full_speed_and_timeout(self):
        self.node.command_cb(String(data=json.dumps(dict(action='position', position=2030,
            held=True))))
        self.node.tick()
        self.assertEqual(self.bus.values[30], 2030)
        self.assertEqual(self.bus.values[32], 0)
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

    def test_sweep_commands_endpoint_with_speed_limit_and_stops_on_rc_kill(self):
        self.node.rc_cb(rc(2000000, pwm=1500))
        self.node.sweep_cb(Bool(data=True))
        self.node.tick()
        self.assertTrue(self.node.sweeping)
        self.assertEqual(self.bus.values[30], 2500)
        self.assertGreater(self.bus.values[32], 0)
        self.assertLessEqual(self.bus.values[32], self.node.get_parameter('speed').value)
        self.bus.values[36] = 2500
        self.node.servo.position = 2500
        self.now += .05
        self.node.tick()
        self.assertEqual(self.bus.values[30], 1500)
        self.node.rc_cb(rc(3000000, pwm=2000))
        self.assertFalse(self.node.sweeping)
        self.assertFalse(self.node.servo.torque)

    def test_sweep_rejection_explains_missing_limits(self):
        self.node.servo.calibration = None
        self.node.sweep_cb(Bool(data=True))
        self.assertFalse(self.node.sweeping)
        self.assertIn('minimum, center and maximum', self.node.reason)
        healthy(self.node.safety, 3000000)
        self.node.tick()
        self.assertFalse(self.node.servo.torque)

    def test_sweep_reverses_when_servo_settles_short_of_each_endpoint(self):
        self.node.sweep_cb(Bool(data=True))
        for position, goal in ((2480, 1500), (1520, 2500)):
            self.bus.values[36] = self.node.servo.position = position
            self.node.tick()
            self.assertTrue(self.node.sweeping)
            self.assertEqual(self.bus.values[30], goal)

    def test_sweep_does_not_reverse_early_or_chatter_in_a_narrow_range(self):
        self.node.sweep_cb(Bool(data=True))
        self.bus.values[36] = self.node.servo.position = 2460
        self.node.tick()
        self.assertEqual(self.bus.values[30], 2500)
        self.node.servo.calibration = dict(minimum=1980, center=2000, maximum=2020)
        self.bus.values[36] = self.node.servo.position = 2000
        self.node.tick()
        self.assertEqual(self.bus.values[30], 2020)
        self.bus.values[36] = self.node.servo.position = 2015
        self.node.tick()
        self.assertEqual(self.bus.values[30], 1980)
        self.node.tick()
        self.assertEqual(self.bus.values[30], 1980)
