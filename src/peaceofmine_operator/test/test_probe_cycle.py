import unittest
from unittest.mock import MagicMock, patch
import time

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

        self.probe.reverse = False
        self.probe.extension_sign = -1
        self.probe.home_hold = None
        self.probe.ident = 2
        self.probe.home_position = 4000
        self.probe.telemetry = MagicMock()
        self.probe.telemetry.position = 4000
        self.probe.telemetry.minimum = 0
        self.probe.telemetry.maximum = 4095
        self.probe.telemetry.speed_deg_s = 0.
        self.probe.telemetry.operation_id = 'op1'
        self.probe.extension = {'servo_id': 2, 'max_extension_mm': 120.0, 'travel_ticks': 3000}
        self.probe.contact = MagicMock()
        self.probe.contact.spike_detected = False
        self.probe.contact.detected = False
        self.probe.contact.snapshot.side_effect = lambda: dict(enabled=True, fresh=True, detected=self.probe.contact.spike_detected or self.probe.contact.detected)
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

        self.probe.reload_contact = MagicMock()
        self.probe.connected = lambda: True
        self.probe.interlock = lambda: not self.probe.fault
        self.probe.start = MagicMock(side_effect=self._mock_start)
        self.probe.stop = MagicMock(side_effect=self._mock_stop)

    def _mock_start(self, target, low, high, **kw):
        self.probe.operation = MagicMock()
        self.probe.operation.target = target
        self.probe.operation.operation_id = 'op1'
        self.probe.operation.speed = self.probe.motion_speed
        self.probe.operation.tolerance = 23
        self.probe.telemetry.goal = target
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
        self.probe.fault=True
        with self.assertRaises(ValueError):
            self.probe.start_probe_cycle(80.)
        self.probe.fault=False
        self.probe.start_probe_cycle(80.)
        self.assertTrue(self.probe.cycle_active)
        self.assertEqual(self.probe.cycle_phase,'DESCENDING')
        self.assertEqual(self.probe.cycle_target_depth,80.)

    def test_cycle_rejects_stale_disabled_and_already_triggered_contact(self):
        for contact in (dict(enabled=False, fresh=False, detected=None),
                        dict(enabled=True, fresh=False, detected=None),
                        dict(enabled=True, fresh=True, detected=True)):
            self.probe.contact.snapshot.side_effect = None
            self.probe.contact.snapshot.return_value = contact
            with self.assertRaises(ValueError):
                self.probe.start_probe_cycle(80.)
            self.assertFalse(self.probe.cycle_active)
            self.probe.start.assert_not_called()

    def test_cycle_never_reports_clear_when_contact_input_dies(self):
        self.probe.start_probe_cycle(80.)
        self.probe.contact.snapshot.side_effect = None
        self.probe.contact.snapshot.return_value = dict(enabled=True, fresh=False, detected=None)
        self.probe.telemetry.position = self.probe.operation.target
        self.probe.update_behavior()
        self.assertFalse(self.probe.cycle_active)
        self.assertEqual(self.probe.last_probe_result['status'], 'ABORTED')

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
        self.probe.telemetry.goal = self.probe.home_position
        self.probe.telemetry.position = 4000 # confirmed home encoder position
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
        self.probe.contact.snapshot.side_effect = lambda: dict(enabled=True, fresh=True, detected=self.probe.contact.spike_detected or self.probe.contact.detected)

        self.probe.update_behavior()
        # Should switch to RETRACTING clear
        self.assertEqual(self.probe.cycle_phase, 'RETRACTING')
        self.assertFalse(self.probe.contact_detected)
        self.assertIsNone(self.probe.contact_depth_mm)

        # Returns to stow
        self.probe.telemetry.position = 4000
        self.probe.telemetry.goal = self.probe.home_position
        self.probe.update_behavior()

        # Cycle completes with CLEAR
        self.assertFalse(self.probe.cycle_active)
        self.assertEqual(self.probe.last_probe_result['status'], 'CLEAR')
        self.assertFalse(self.probe.last_probe_result['contact'])
        self.assertEqual(self.probe.last_probe_result['target_depth_mm'], 80.0)

    def test_probe_cycle_aborts_if_rc_permission_is_lost(self):
        self.probe.start_probe_cycle(80.)
        self.probe.interlock=lambda:False
        self.probe.update_behavior()
        self.assertFalse(self.probe.cycle_active)
        self.assertEqual(self.probe.last_probe_result['status'],'ABORTED')

    def test_one_mm_cycle_requires_encoder_descent_and_retraction(self):
        self.probe.start_probe_cycle(1.)
        self.probe.update_behavior()
        self.assertEqual(self.probe.cycle_phase,'DESCENDING')
        self.assertIsNone(self.probe.last_probe_result)
        self.probe.telemetry.position=self.probe.operation.target
        self.probe.telemetry.operation_id='old-operation'
        self.probe.update_behavior()
        self.assertEqual(self.probe.cycle_phase,'DESCENDING')
        self.probe.telemetry.operation_id='op1'
        self.probe.update_behavior()
        self.assertEqual(self.probe.cycle_phase,'RETRACTING')
        self.probe.update_behavior()
        self.assertTrue(self.probe.cycle_active,'Depth below 2 mm alone does not establish retraction')
        self.probe.telemetry.position=self.probe.home_position
        self.probe.telemetry.goal=self.probe.home_position
        self.probe.update_behavior()
        self.assertEqual(self.probe.last_probe_result['status'],'CLEAR')

    def test_reversed_probe_uses_opposite_encoder_direction(self):
        self.probe.apply_direction(dict(reverse=True))
        self.assertIsNone(self.probe.home_position)
        self.probe.start_home('reverse-home',20.)
        self.assertEqual(self.probe.start.call_args.args[0],self.probe.telemetry.minimum)
        self.probe.home_position=1000
        self.probe.operation=None
        self.probe.start_target(20.)
        self.assertEqual(self.probe.operation.target,1500)
        self.assertEqual(self.probe.start.call_args.args[1:],(1000,4000))
        self.probe.operation=None
        self.probe.motion=None
        command=dict(direction=-1)
        self.assertFalse(self.probe.special_command('jog',command))
        self.assertEqual(command['direction'],1)
        self.probe.telemetry.position=1500
        self.probe.telemetry.torque=False
        self.probe.save=MagicMock()
        self.probe.special_command('save_probe_extension',dict(max_extension_mm=20.))
        self.assertEqual(self.probe.save.call_args.args[2]['travel_ticks'],500)

    def test_reversed_cycle_descends_and_returns_home(self):
        self.probe.apply_direction(dict(reverse=True))
        self.probe.home_position=1000
        self.probe.start_probe_cycle(1.)
        self.assertEqual(self.probe.operation.target,1025)
        self.probe.update_behavior()
        self.assertEqual(self.probe.cycle_phase,'DESCENDING')
        self.probe.telemetry.position=1025
        self.probe.update_behavior()
        self.assertEqual(self.probe.cycle_phase,'RETRACTING')
        self.probe.update_behavior()
        self.assertTrue(self.probe.cycle_active)
        self.probe.telemetry.position=1000
        self.probe.telemetry.goal=self.probe.home_position
        self.probe.update_behavior()
        self.assertEqual(self.probe.last_probe_result['status'],'CLEAR')

    def test_manual_extension_stops_on_contact_but_retraction_is_allowed(self):
        self.probe.start_target(20.)
        self.probe.telemetry.position=3900
        self.probe.contact.detected=True
        self.probe.update_behavior()
        self.assertIsNone(self.probe.operation)
        self.assertIn('contact',self.probe.stop.call_args.args[0].lower())
        self.probe.start_target(0.)
        self.probe.update_behavior()
        self.assertIsNotNone(self.probe.operation)

    def test_cycle_action_converts_slow_mm_per_second(self):
        from types import SimpleNamespace
        self.probe.start_action(SimpleNamespace(operation='probe_cycle',target=1.,speed=5.))
        self.assertEqual(self.probe.operation.speed,16)
        self.assertTrue(self.probe.action_probe_cycle)

    def test_sub_encoder_cycle_depth_is_rejected(self):
        with self.assertRaisesRegex(ValueError,'encoder tick'):
            self.probe.start_probe_cycle(.001)
        self.probe.start.assert_not_called()


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
        from peaceofmine_operator.inspection import InspectionWorkflow
        self.ctrl._inspection = InspectionWorkflow(MagicMock(),MagicMock(),MagicMock())
        self.ctrl._lease = 'client1'
        self.ctrl._armed = True
        self.ctrl._calibrating = False
        self.ctrl._speed = 0.0
        self.ctrl._stationary = lambda: self.ctrl._speed <= self.ctrl._probe_speed_limit
        self.ctrl._probe_speed_limit = 0.03
        self.ctrl._probe_fault = False
        self.ctrl._probe_state = {'probe': {'ready': True, 'max_depth_mm': 120.0, 'depth_mm': 0.0, 'cycle_active': False}}
        self.ctrl._probe_state_at = time.monotonic()
        self.ctrl._arm_state = {'sweeping': False, 'connected': True, 'calibration': {'minimum': 1024, 'center': 2048, 'maximum': 3072}}
        self.ctrl._arm_state_at = time.monotonic()
        self.ctrl._probe_command_pub = MagicMock()
        self.ctrl._arm_command_pub = MagicMock()
        self.ctrl._sweep_enabled_pub = MagicMock()
        self.ctrl._permission_pub = MagicMock()
        self.ctrl._pump_pub = MagicMock()
        self.ctrl._pump_active = False
        self.ctrl._pump_timer = None
        self.ctrl.create_timer = MagicMock()
        self.ctrl.destroy_timer = MagicMock()
        self.ctrl._rc_safety = MagicMock()
        self.ctrl._rc_safety.simulation = True
        self.ctrl._rc_safety.snapshot.return_value = {'allowed': True, 'servo_allowed': True}
        self.ctrl._rc_safety.servo_snapshot.return_value = {'allowed': True, 'servo_allowed': True}
        self.ctrl._enforce_safety_locked = lambda: {'allowed': True, 'servo_allowed': True}
        self.ctrl._joy_live = lambda now: False
        self.ctrl._apply_drive_locked = MagicMock()

    def test_start_probe_cycle_dispatches_when_stationary(self):
        res = self.ctrl.handle_command('client1', {'type': 'start_probe_cycle', 'depth_mm': 80.0, 'request_id': 'req1'})
        self.assertIsNone(res)
        self.ctrl._probe_command_pub.publish.assert_called_once()
        self.assertEqual(self.ctrl._probe_command_pub.publish.call_args.args[0].depth_mm, 80.0)

    def test_probe_cycle_dispatch_does_not_depend_on_odometry(self):
        self.ctrl._stationary=lambda:False
        res=self.ctrl.handle_command('client1',dict(type='start_probe_cycle',depth_mm=80.))
        self.assertIsNone(res)
        self.ctrl._probe_command_pub.publish.assert_called_once()

    def test_drive_blocked_when_probe_cycle_active(self):
        self.ctrl._probe_state['probe']['cycle_active'] = True
        res = self.ctrl.handle_command('client1', {'type': 'drive', 'linear_x': 0.4, 'angular_z': 0.0, 'deadman': True})
        self.assertIsNotNone(res)
        self.assertIn('probe cycle is active', res['message'])
        self.ctrl._apply_drive_locked.assert_not_called()

    def test_align_probe_dispatches_when_stowed(self):
        res = self.ctrl.handle_command('client1', {'type': 'align_probe', 'angle_deg': -15.0, 'request_id': 'req_align'})
        self.assertIsNone(res)
        self.ctrl._permission_pub.publish.assert_called_once()
        self.ctrl._arm_command_pub.publish.assert_called_once()

    def test_alignment_blocks_active_probe_but_not_setup_or_parked_depth(self):
        self.ctrl._probe_state['probe']['cycle_active']=True
        res=self.ctrl.handle_command('client1',dict(type='align_probe',angle_deg=-15.))
        self.assertIn('active probe',res['message'])
        self.ctrl._probe_state={'probe':dict(ready=False,depth_mm=20.)}
        self.ctrl._stationary=lambda:False
        self.assertIsNone(self.ctrl.handle_command('client1',dict(type='align_probe',angle_deg=-15.)))

    def test_pump_mark_dispatches_when_stationary_and_stowed(self):
        res = self.ctrl.handle_command('client1', {'type': 'pump_mark', 'duration_s': 2.0})
        self.assertIsNone(res)
        self.assertTrue(self.ctrl._pump_active)
        self.ctrl._pump_pub.publish.assert_called_once()
        published_msg = self.ctrl._pump_pub.publish.call_args[0][0]
        self.assertTrue(published_msg.data)
        self.ctrl.create_timer.assert_called_once_with(2.0, self.ctrl._pump_timeout_cb)

    def test_mark_does_not_require_odometry_or_probe_telemetry(self):
        self.ctrl._probe_state={}
        self.ctrl._stationary=lambda:False
        self.assertIsNone(self.ctrl.handle_command('client1',dict(type='pump_mark',duration_s=2.)))
        self.assertTrue(self.ctrl._pump_active)

    def test_mark_blocks_active_probe_operation(self):
        for active in ('cycle_active','active','holding'):
            self.ctrl._probe_state={'probe':{active:True}}
            res=self.ctrl.handle_command('client1',dict(type='pump_mark',duration_s=2.))
            self.assertIn('active probe',res['message'])
        self.ctrl._probe_state={'probe':dict(depth_mm=15.,ready=False)}
        self.assertIsNone(self.ctrl.handle_command('client1',dict(type='pump_mark',duration_s=2.)))

    def test_pump_mark_blocked_when_already_active(self):
        self.ctrl._pump_active = True
        res = self.ctrl.handle_command('client1', {'type': 'pump_mark', 'duration_s': 2.0})
        self.assertIsNotNone(res)
        self.assertIn('already in progress', res['message'])

    def test_drive_blocked_when_pump_active(self):
        self.ctrl._pump_active = True
        res = self.ctrl.handle_command('client1', {'type': 'drive', 'linear_x': 0.2, 'angular_z': 0.0, 'deadman': True})
        self.assertIsNotNone(res)
        self.assertIn('ground marking is active', res['message'])

    def test_pump_timeout_deactivates_pump(self):
        self.ctrl._pump_active = True
        mock_timer = MagicMock()
        self.ctrl._pump_timer = mock_timer
        self.ctrl._pump_timeout_cb()
        self.assertFalse(self.ctrl._pump_active)
        self.assertIsNone(self.ctrl._pump_timer)
        mock_timer.cancel.assert_called_once()
        self.ctrl._pump_pub.publish.assert_called_once()
        published_msg = self.ctrl._pump_pub.publish.call_args[0][0]
        self.assertFalse(published_msg.data)

    def test_stop_motion_aborts_pump(self):
        self.ctrl._pump_active = True
        mock_timer = MagicMock()
        self.ctrl._pump_timer = mock_timer
        self.ctrl._stop_motion_locked()
        self.assertFalse(self.ctrl._pump_active)
        self.assertIsNone(self.ctrl._pump_timer)
        mock_timer.cancel.assert_called_once()
        self.ctrl._pump_pub.publish.assert_called_once()
        published_msg = self.ctrl._pump_pub.publish.call_args[0][0]
        self.assertFalse(published_msg.data)


spec_arm = importlib.util.spec_from_file_location(
    'arm_controller_node', Path(__file__).parents[1] / 'scripts/arm_controller_node.py')
arm_mod = importlib.util.module_from_spec(spec_arm)
spec_arm.loader.exec_module(arm_mod)
ArmController = arm_mod.ArmController


class ArmControllerAngleTest(unittest.TestCase):
    def setUp(self):
        self.arm = ArmController.__new__(ArmController)
        self.arm.offset = 0.0
        self.arm.sweep_speed = 60.0
        self.arm.sweeping = False
        self.arm.auto_position = False
        self.arm.started = 0.0
        self.arm.position_timeout = 60.
        self.arm.tolerance_deg = 4.
        self.arm.motion_speed = 116
        self.arm.action_handle = None
        self.arm.lifecycle = 'IDLE'
        self.arm.reason = 'Idle'
        self.arm.operation = None
        self.arm.telemetry = MagicMock()
        self.arm.telemetry.operation_id = 'op_angle'
        self.arm.telemetry.position = 2048
        self.arm.telemetry.goal = 2048
        self.arm.telemetry.speed_deg_s = 0.
        self.arm.calibration = lambda: {'minimum': 1024, 'center': 2048, 'maximum': 3072}
        self.arm.reply = MagicMock()
        self.arm.start = MagicMock(side_effect=self._mock_start)
        self.arm.stop = MagicMock(side_effect=self._mock_stop)

    def _mock_start(self, target, low, high, speed=None):
        self.arm.operation = MagicMock()
        self.arm.operation.target = target
        self.arm.operation.operation_id = 'op_angle'
        self.arm.operation.speed = speed or self.arm.motion_speed
        self.arm.telemetry.goal = target
        self.arm.lifecycle = 'MOVING'

    def _mock_stop(self, reason='Stopped', success=False):
        self.arm.sweeping = self.arm.auto_position = False
        self.arm.operation = None
        self.arm.lifecycle = 'IDLE'

    def test_special_command_angle_converts_degrees_to_ticks(self):
        # 0° -> center (2048)
        self.arm.special_command('angle', {'action': 'angle', 'angle_deg': 0.0})
        self.assertTrue(self.arm.auto_position)
        self.arm.start.assert_called_with(2048, 1024, 3072, speed=round(60.0 / 0.684))

        # +45° -> 2048 + 45*4096/360 = 2048 + 512 = 2560
        self.arm.special_command('angle', {'action': 'angle', 'angle_deg': 45.0})
        self.assertEqual(self.arm.operation.target, 2560)

        # -45° -> 2048 - 45*4096/360 = 2048 - 512 = 1536
        self.arm.special_command('angle', {'action': 'angle', 'angle_deg': -45.0})
        self.assertEqual(self.arm.operation.target, 1536)

    def test_special_command_angle_rejects_out_of_bounds(self):
        # Limit is 1024 to 3072 (±90°). +100° would be 2048 + 1138 = 3186 > 3072
        with self.assertRaises(ValueError):
            self.arm.special_command('angle', {'action': 'angle', 'angle_deg': 100.0})

        with self.assertRaises(ValueError):
            self.arm.special_command('angle', {'action': 'angle', 'angle_deg': -100.0})

    def test_update_behavior_holds_position_when_angle_target_reached(self):
        self.arm.special_command('angle', {'action': 'angle', 'angle_deg': -15.0})
        target_tick = self.arm.operation.target
        self.arm.telemetry.position = target_tick
        self.arm.telemetry.operation_id = self.arm.operation.operation_id

        self.arm.update_behavior()
        self.assertFalse(self.arm.auto_position)
        self.assertEqual(self.arm.lifecycle, 'HOLDING')
        self.assertIn('holding position', self.arm.reason)


class KinematicAlignmentTest(unittest.TestCase):
    def test_probe_alignment_math_and_reachability(self):
        # Detector is 45° to the right of the probe.
        # Pivot formula: theta_pivot = theta_peak - 45°
        # With arm travel limits [-45°, +45°]:
        # Minimum reachable peak = -45° + 45° = 0°
        # 1. Target detected at +30° (left):
        peak_angle = 30.0
        pivot_angle = peak_angle - 45.0 # -15.0°
        self.assertEqual(pivot_angle, -15.0)
        self.assertTrue(-45.0 <= pivot_angle <= 45.0)

        # 2. Target detected at 0° (ahead):
        peak_angle = 0.0
        pivot_angle = peak_angle - 45.0 # -45.0°
        self.assertEqual(pivot_angle, -45.0)
        self.assertTrue(-45.0 <= pivot_angle <= 45.0)

        # 3. Target detected at -20° (right, dead zone):
        peak_angle = -20.0
        pivot_angle = peak_angle - 45.0 # -65.0°
        self.assertEqual(pivot_angle, -65.0)
        self.assertFalse(-45.0 <= pivot_angle <= 45.0)
        # Deficit to reach:
        deficit = -45.0 - pivot_angle
        self.assertEqual(deficit, 20.0) # Rover must steer 20° right to reach target


if __name__ == '__main__':
    unittest.main()
