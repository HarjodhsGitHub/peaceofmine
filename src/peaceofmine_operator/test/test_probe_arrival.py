"""Probe arrival regressions using production code, without ROS or motor I/O."""
import ast
import math
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import Mock


class ProbeArrivalTest(unittest.TestCase):
    def setUp(self):
        self.now = 100.
        path = Path(__file__).parents[1]/'scripts/probe_controller_node.py'
        tree = ast.parse(path.read_text())
        kind = next(node for node in tree.body if isinstance(node, ast.ClassDef))
        scope = dict(ActuatorController=object, math=math,
                     time=SimpleNamespace(monotonic=lambda:self.now, time=lambda:self.now))
        exec(compile(ast.Module(body=[kind], type_ignores=[]), str(path), 'exec'), scope)
        self.probe = scope['ProbeController'].__new__(scope['ProbeController'])
        self.probe.ident = 2
        self.probe.home_position = 4000
        self.probe.extension_sign = -1
        self.probe.extension = dict(servo_id=2, max_extension_mm=120., travel_ticks=3000)
        self.probe.telemetry = SimpleNamespace(position=3517, goal=2000,
                                               operation_id='cycle', speed_deg_s=0., minimum=0, maximum=4095)
        self.probe.operation = SimpleNamespace(target=2000, operation_id='cycle', tolerance=23, speed=73)
        self.probe.arrival_tolerance = 23
        self.probe.motion = 'target'
        self.probe.started = 99.
        self.probe.cycle_timeout = 60.
        self.probe.cycle_active = True
        self.probe.cycle_phase = 'DESCENDING'
        self.probe.cycle_target_depth = 80.
        self.probe.contact = SimpleNamespace(snapshot=lambda:dict(enabled=True, fresh=True, detected=True))
        self.probe.interlock = self.probe.connected = lambda:True
        self.probe.stop = Mock()
        self.probe.last_probe_result = None

    def retract(self):
        self.probe.update_behavior()
        self.assertEqual(self.probe.cycle_phase, 'RETRACTING')
        self.assertEqual(self.probe.contact_depth_mm, 19.3)
        self.assertEqual(self.probe.operation.target, 4000)
        self.probe.telemetry.goal = 4000

    def test_settled_return_uses_command_tolerance_and_keeps_contact_depth(self):
        self.retract()
        self.probe.telemetry.position = 3980
        self.probe.update_behavior()
        self.probe.stop.assert_called_once_with('Probe cycle complete: CONTACT', success=True)
        self.assertFalse(self.probe.cycle_active)
        self.assertEqual(self.probe.last_probe_result['contact_depth_mm'], 19.3)

    def test_old_goal_does_not_complete_return(self):
        self.retract()
        self.probe.telemetry.position = 3980
        self.probe.telemetry.goal = 2000
        self.probe.update_behavior()
        self.probe.stop.assert_not_called()

    def test_old_operation_does_not_complete_return(self):
        self.retract()
        self.probe.telemetry.position = 4000
        self.probe.telemetry.operation_id = 'previous'
        self.probe.update_behavior()
        self.probe.stop.assert_not_called()

    def test_moving_or_outside_command_tolerance_does_not_complete(self):
        self.retract()
        self.probe.telemetry.position = 3980
        self.probe.telemetry.speed_deg_s = 20.
        self.probe.update_behavior()
        self.probe.stop.assert_not_called()
        self.probe.telemetry.speed_deg_s = 0.
        self.probe.telemetry.position = 3976
        self.probe.update_behavior()
        self.probe.stop.assert_not_called()

    def test_short_contact_return_keeps_tighter_tolerance(self):
        self.probe.telemetry.position = 3992
        self.probe.update_behavior()
        self.assertEqual(self.probe.arrival_tolerance, 2)
        self.probe.telemetry.goal = 4000
        self.probe.update_behavior()
        self.probe.stop.assert_not_called()
        self.probe.telemetry.position = 3998
        self.probe.update_behavior()
        self.probe.stop.assert_called_once()

    def test_short_requested_target_limits_tolerance_to_quarter_stroke(self):
        self.probe.telemetry.position = 4000
        self.probe.start = Mock()
        self.probe.start_target(1., hold=False)
        self.assertEqual(self.probe.arrival_tolerance, 6)

    def test_reversed_cycle_returns_using_the_same_arrival_contract(self):
        self.probe.extension_sign = 1
        self.probe.home_position = 1000
        self.probe.telemetry.position = 1483
        self.probe.update_behavior()
        self.probe.telemetry.goal = 1000
        self.probe.telemetry.position = 1020
        self.probe.update_behavior()
        self.probe.stop.assert_called_once_with('Probe cycle complete: CONTACT', success=True)
        self.assertEqual(self.probe.last_probe_result['contact_depth_mm'], 19.3)

    def test_clear_descent_returns_without_requiring_three_tick_accuracy(self):
        self.probe.contact.snapshot = lambda:dict(enabled=True, fresh=True, detected=False)
        self.probe.telemetry.position = 2020
        self.probe.update_behavior()
        self.assertEqual(self.probe.cycle_phase, 'RETRACTING')
        self.probe.telemetry.goal = 4000
        self.probe.telemetry.position = 3980
        self.probe.update_behavior()
        self.assertFalse(self.probe.last_probe_result['contact'])
        self.assertEqual(self.probe.last_probe_result['status'], 'CLEAR')

    def test_completed_retraction_advances_scan_to_next_angle(self):
        from peaceofmine_operator.inspection import InspectionWorkflow
        execute = Mock()
        flow = InspectionWorkflow(execute, Mock(), Mock(), clock=lambda:self.now)
        flow.confirmed_angle = 45.
        flow.probe(-50., 50., 80.)
        flow.tick(True, None)
        execute.call_args.args[-1](dict(success=True))
        flow.tick(True, None)
        complete = execute.call_args.args[-1]
        self.retract()
        self.probe.stop.side_effect = lambda message, success=False:complete(dict(
            success=success, message=message, **self.probe.last_probe_result))
        self.probe.telemetry.position = 3980
        self.probe.update_behavior()
        self.assertEqual(flow.completed, 1)
        self.assertEqual(flow.points[0]['contact_depth_mm'], 19.3)
        flow.tick(True, None)
        self.assertEqual(execute.call_args.args[:4], ('arm', 'angle', -5., 5.))

    def test_timeout_reports_actual_phase_and_remaining_distance(self):
        self.retract()
        self.now = 160.
        self.probe.update_behavior()
        self.assertEqual(self.probe.last_probe_result['status'], 'TIMEOUT')
        self.assertIn('retracting timed out: 19.3 mm', self.probe.stop.call_args.args[0])
        self.assertFalse(self.probe.stop.call_args.kwargs.get('success', False))


if __name__ == '__main__':
    unittest.main()
