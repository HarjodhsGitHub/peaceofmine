import sys
import unittest
from unittest.mock import MagicMock, patch
import time

# Mock ROS modules for host execution
import types
class MockNode:
    pass

class MockModule(types.ModuleType):
    def __getattr__(self, name):
        if name == 'Node':
            return MockNode
        return MagicMock()

for mod_name in [
    'rclpy', 'rclpy.node', 'rclpy.qos', 'rclpy.parameter', 'rclpy.action', 'rclpy.task', 'rclpy.signals',
    'nav_msgs', 'nav_msgs.msg', 'geometry_msgs', 'geometry_msgs.msg', 'sensor_msgs', 'sensor_msgs.msg',
    'std_msgs', 'std_msgs.msg', 'mavros_msgs', 'mavros_msgs.msg', 'peaceofmine_interfaces',
    'peaceofmine_interfaces.msg', 'peaceofmine_interfaces.srv', 'peaceofmine_interfaces.action'
]:
    if mod_name not in sys.modules:
        m = MockModule(mod_name)
        m.__path__ = []
        sys.modules[mod_name] = m

import importlib.util
from pathlib import Path
spec = importlib.util.spec_from_file_location(
    'probe_controller_node', Path(__file__).parents[1] / 'scripts/probe_controller_node.py')
probe_mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(probe_mod)
ProbeController = probe_mod.ProbeController


class ProbeCycleTest(unittest.TestCase):
    def setUp(self):
        # Create an instance without calling full ROS Node __init__
        self.probe = ProbeController.__new__(ProbeController)

        self.probe.ident = 2
        self.probe.home_position = 4000
        self.probe.telemetry = MagicMock()
        self.probe.telemetry.position = 4000
        self.probe.telemetry.minimum = 0
        self.probe.telemetry.maximum = 4095
        self.probe.telemetry.operation_id = 'op1'
        self.probe.extension = {'servo_id': 2, 'max_extension_mm': 120.0, 'travel_ticks': 3000}
        self.probe.contact = MagicMock()
        self.probe.contact.spike_detected = False
        self.probe.contact.detected = False
        self.probe.speed = 0.0
        self.probe.speed_limit = 0.03
        self.probe.odometry_at = time.monotonic()
        self.probe.fault = False
        self.probe.cycle_active = False
        self.probe.cycle_phase = 'IDLE'
        self.probe.cycle_target_depth = 80.0
        self.probe.contact_detected = False
        self.probe.contact_depth_mm = None
        self.probe.last_probe_result = None
        self.probe.motion = None
        self.probe.operation = None
        self.probe.started = time.monotonic()
        self.probe.reason = 'Idle'
        self.probe.lifecycle = 'IDLE'
        self.probe.motion_speed = 100
        self.probe.acceleration = 10

        self.probe.connected = lambda: True
        self.probe.interlock = lambda: self.probe.speed <= self.probe.speed_limit and not self.probe.fault
        self.probe.start = MagicMock(side_effect=self._mock_start)
        self.probe.stop = MagicMock(side_effect=self._mock_stop)

    def _mock_start(self, target, low, high, **kw):
        self.probe.operation = MagicMock()
        self.probe.operation.target = target
        self.probe.operation.operation_id = 'op1'
        self.probe.lifecycle = 'MOVING'

    def _mock_stop(self, reason='Stopped', success=False):
        if self.probe.cycle_active:
            self.probe.cycle_active = False
            self.probe.cycle_phase = 'IDLE'
            if not success:
                self.probe.last_probe_result = dict(status='ABORTED', reason=reason, timestamp=time.time())
        self.probe.operation = None
        self.probe.motion = None
        self.probe.lifecycle = 'IDLE'

    def test_start_probe_cycle_validates_interlock(self):
        # Rover moving (speed = 0.1 m/s > 0.03 limit)
        self.probe.speed = 0.1
        with self.assertRaises(ValueError):
            self.probe.start_probe_cycle(80.0)

        # Rover stationary -> succeeds
        self.probe.speed = 0.0
        self.probe.start_probe_cycle(80.0)
        self.assertTrue(self.probe.cycle_active)
        self.assertEqual(self.probe.cycle_phase, 'DESCENDING')
        self.assertEqual(self.probe.cycle_target_depth, 80.0)
        self.assertIsNotNone(self.probe.operation)

    def test_probe_cycle_spike_detection_and_retraction(self):
        self.probe.start_probe_cycle(80.0)
        self.assertEqual(self.probe.cycle_phase, 'DESCENDING')

        # Simulate descending to 45 mm:
        # ticks = 45 / 120 * 3000 = 1125 -> position = 4000 - 1125 = 2875
        self.probe.telemetry.position = 2875

        # No spike yet
        self.probe.update_behavior()
        self.assertEqual(self.probe.cycle_phase, 'DESCENDING')
        self.assertFalse(self.probe.contact_detected)

        # FSR contact spike occurs!
        self.probe.contact.spike_detected = True
        self.probe.update_behavior()

        # Should immediately detect contact, record depth, and switch to RETRACTING towards 0.0 mm
        self.assertTrue(self.probe.contact_detected)
        self.assertAlmostEqual(self.probe.contact_depth_mm, 45.0, delta=0.5)
        self.assertEqual(self.probe.cycle_phase, 'RETRACTING')
        # Target should now be home (4000)
        self.assertEqual(self.probe.operation.target, 4000)

        # While retracting, probe moves back to top
        self.probe.telemetry.position = 3980 # depth <= 2 mm
        self.probe.update_behavior()

        # Cycle completes with CONTACT
        self.assertFalse(self.probe.cycle_active)
        self.assertEqual(self.probe.cycle_phase, 'IDLE')
        self.assertIsNotNone(self.probe.last_probe_result)
        self.assertEqual(self.probe.last_probe_result['status'], 'CONTACT')
        self.assertTrue(self.probe.last_probe_result['contact'])
        self.assertAlmostEqual(self.probe.last_probe_result['contact_depth_mm'], 45.0, delta=0.5)

    def test_probe_cycle_clear_when_target_reached_without_spike(self):
        self.probe.start_probe_cycle(80.0)
        self.assertEqual(self.probe.cycle_phase, 'DESCENDING')

        # Probe reaches 80 mm target position (target = 4000 - 80/120*3000 = 2000)
        self.probe.telemetry.position = 2000
        self.probe.contact.spike_detected = False
        self.probe.contact.detected = False

        self.probe.update_behavior()
        # Should switch to RETRACTING clear
        self.assertEqual(self.probe.cycle_phase, 'RETRACTING')
        self.assertFalse(self.probe.contact_detected)
        self.assertIsNone(self.probe.contact_depth_mm)

        # Returns to stow
        self.probe.telemetry.position = 4000
        self.probe.update_behavior()

        # Cycle completes with CLEAR
        self.assertFalse(self.probe.cycle_active)
        self.assertEqual(self.probe.last_probe_result['status'], 'CLEAR')
        self.assertFalse(self.probe.last_probe_result['contact'])
        self.assertEqual(self.probe.last_probe_result['target_depth_mm'], 80.0)

    def test_probe_cycle_aborts_if_rover_moves_during_descent(self):
        self.probe.start_probe_cycle(80.0)
        self.assertEqual(self.probe.cycle_phase, 'DESCENDING')

        # Vehicle starts moving
        self.probe.speed = 0.08 # exceeds 0.03 m/s limit
        self.probe.update_behavior()

        # Cycle should immediately abort
        self.assertFalse(self.probe.cycle_active)
        self.assertEqual(self.probe.last_probe_result['status'], 'ABORTED')


spec_ctrl = importlib.util.spec_from_file_location(
    'operator_control_node', Path(__file__).parents[1] / 'scripts/operator_control_node.py')
ctrl_mod = importlib.util.module_from_spec(spec_ctrl)
spec_ctrl.loader.exec_module(ctrl_mod)
OperatorControl = ctrl_mod.OperatorControl


class OperatorControlProbeTest(unittest.TestCase):
    def setUp(self):
        self.ctrl = OperatorControl.__new__(OperatorControl)
        self.ctrl._lock = MagicMock()
        self.ctrl._lock.__enter__ = lambda s: None
        self.ctrl._lock.__exit__ = lambda s, *a: None
        self.ctrl._shutting_down = False
        self.ctrl._lease = 'client1'
        self.ctrl._armed = True
        self.ctrl._calibrating = False
        self.ctrl._speed = 0.0
        self.ctrl._probe_speed_limit = 0.03
        self.ctrl._probe_fault = False
        self.ctrl._probe_state = {'probe': {'ready': True, 'max_depth_mm': 120.0, 'cycle_active': False}}
        self.ctrl._probe_state_at = time.monotonic()
        self.ctrl._arm_state = {'sweeping': False}
        self.ctrl._probe_command_pub = MagicMock()
        self.ctrl._rc_safety = MagicMock()
        self.ctrl._rc_safety.snapshot.return_value = {'allowed': True, 'servo_allowed': True}
        self.ctrl._rc_safety.servo_snapshot.return_value = {'allowed': True, 'servo_allowed': True}
        self.ctrl._enforce_safety_locked = lambda: {'allowed': True, 'servo_allowed': True}
        self.ctrl._joy_live = lambda now: False
        self.ctrl._apply_drive_locked = MagicMock()

    def test_start_probe_cycle_dispatches_when_stationary(self):
        res = self.ctrl.handle_command('client1', {'type': 'start_probe_cycle', 'depth_mm': 80.0, 'request_id': 'req1'})
        self.assertIsNone(res)
        self.ctrl._probe_command_pub.publish.assert_called_once()

    def test_start_probe_cycle_blocked_when_moving(self):
        self.ctrl._speed = 0.05 # > 0.03 m/s limit
        res = self.ctrl.handle_command('client1', {'type': 'start_probe_cycle', 'depth_mm': 80.0, 'request_id': 'req1'})
        self.assertIsNotNone(res)
        self.assertIn('rover is moving', res['message'])
        self.ctrl._probe_command_pub.publish.assert_not_called()

    def test_drive_blocked_when_probe_cycle_active(self):
        self.ctrl._probe_state['probe']['cycle_active'] = True
        res = self.ctrl.handle_command('client1', {'type': 'drive', 'linear_x': 0.4, 'angular_z': 0.0, 'deadman': True})
        self.assertIsNotNone(res)
        self.assertIn('probe cycle is active', res['message'])
        self.ctrl._apply_drive_locked.assert_not_called()


if __name__ == '__main__':
    unittest.main()
