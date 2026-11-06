"""Production arm-arrival regressions without starting ROS or servo I/O."""
import ast
import math
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import Mock
from peaceofmine_operator.inspection import InspectionWorkflow


class ArmArrivalTest(unittest.TestCase):
    def setUp(self):
        self.now=100.
        path=Path(__file__).parents[1]/'scripts/arm_controller_node.py'
        source=ast.parse(path.read_text())
        kind=next(node for node in source.body if isinstance(node,ast.ClassDef))
        scope=dict(ActuatorController=object,math=math,time=SimpleNamespace(monotonic=lambda:self.now))
        exec(compile(ast.Module(body=[kind],type_ignores=[]),str(path),'exec'),scope)
        self.arm=scope['ArmController'].__new__(scope['ArmController'])
        self.arm.sweeping=False
        self.arm.auto_position=True
        self.arm.started=99.
        self.arm.position_timeout=30.
        self.arm.tolerance_deg=4.
        self.arm.action_handle=object()
        self.arm.operation=SimpleNamespace(operation_id='arrival',target=511)
        self.arm.telemetry=SimpleNamespace(operation_id='arrival',position=541,goal=511,speed_deg_s=0.)
        self.arm.stop=Mock()

    def test_settled_30_tick_error_completes_with_configured_four_degrees(self):
        self.arm.update_behavior()
        self.arm.stop.assert_called_once_with('Arm target reached; torque released',success=True)

    def test_existing_narrower_tolerance_is_respected(self):
        self.arm.tolerance_deg=2.
        self.arm.update_behavior()
        self.arm.stop.assert_not_called()

    def test_outside_tolerance_does_not_claim_arrival(self):
        self.arm.telemetry.position=571
        self.arm.update_behavior()
        self.arm.stop.assert_not_called()

    def test_current_goal_must_have_been_applied(self):
        self.arm.telemetry.goal=600
        self.arm.update_behavior()
        self.arm.stop.assert_not_called()

    def test_old_operation_telemetry_does_not_complete_new_action(self):
        self.arm.telemetry.operation_id='previous'
        self.arm.update_behavior()
        self.arm.stop.assert_not_called()

    def test_passing_through_tolerance_at_speed_does_not_complete(self):
        for speed in (-36.,36.,3*.684+.01):
            self.arm.telemetry.speed_deg_s=speed
            self.arm.update_behavior()
            self.arm.stop.assert_not_called()
        self.arm.telemetry.speed_deg_s=3*.684
        self.arm.update_behavior()
        self.arm.stop.assert_called_once()

    def test_timeout_still_fails_and_reports_error_and_allowed_tolerance(self):
        self.now=130.
        self.arm.telemetry.position=571
        self.arm.update_behavior()
        self.assertFalse(self.arm.stop.call_args.kwargs.get('success',False))
        message=self.arm.stop.call_args.args[0]
        self.assertIn('timed out',message)
        self.assertIn('5.3° error',message)
        self.assertIn('allowed 4°',message)

    def test_manual_angle_keeps_existing_hold_after_arrival(self):
        self.arm.action_handle=None
        self.arm.update_behavior()
        self.assertFalse(self.arm.auto_position)
        self.assertEqual(self.arm.lifecycle,'HOLDING')
        self.arm.stop.assert_not_called()

    def test_arrival_advances_confirmation_to_the_first_full_pass(self):
        execute=Mock()
        flow=InspectionWorkflow(execute,Mock(),Mock(),clock=lambda:self.now)
        lower=(511-1514)*360/4096
        upper=(2555-1514)*360/4096
        flow.confirm(lower,upper,36.)
        flow.tick(True,None)
        self.assertEqual(flow.pending,'start')
        callback=execute.call_args.args[-1]
        self.arm.stop.side_effect=lambda message,success=False:callback(dict(success=success,message=message))
        self.arm.update_behavior()
        self.assertIsNone(flow.pending)
        flow.tick(True,None)
        self.assertEqual(flow.pending,'pass')
        self.assertEqual(execute.call_args.args[:4],('arm','angle',upper,36.))
        self.assertEqual(flow.progress,0.)


if __name__=='__main__':unittest.main()
