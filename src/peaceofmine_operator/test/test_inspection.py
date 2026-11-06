import ast
import math
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

from peaceofmine_operator.inspection import InspectionWorkflow


class InspectionTest(unittest.TestCase):
    def setUp(self):
        self.now = 1.
        self.execute, self.cancel, self.clear = Mock(), Mock(), Mock()
        self.flow = InspectionWorkflow(self.execute, self.cancel, self.clear, clock=lambda:self.now)

    def tick(self, peak=None, idle=True):
        self.flow.tick(idle, peak)

    def complete(self, **result):
        self.execute.call_args.args[-1](dict(success=True, **result))

    def feed_arc(self, center, baseline=2.45, height=.015, sigma=6.):
        angles=range(-90,91) if not self.flow.passes else range(90,-91,-1)
        for angle in angles:
            self.now+=.01
            volts=baseline+height*math.exp(-.5*((angle-center)/sigma)**2)
            self.tick(dict(fresh=True,angle_fresh=True,sample_time=self.now,
                           angle_deg=angle,volts=volts,filtered_volts=volts,response_direction=1))

    def confirm(self):
        self.flow.confirm(-90.,90.,36.)
        self.tick()
        self.assertEqual(self.execute.call_args.args[:4],('arm','angle',-90.,36.))
        self.complete()
        for angle in (2.,4.):
            self.tick()
            self.feed_arc(angle)
            self.complete()
        self.assertEqual(self.flow.phase,'CONFIRMED')
        self.assertEqual(self.flow.confirmed_angle,3.)

    def test_two_opposed_passes_compute_center(self):
        self.confirm()
        targets=[c.args[2] for c in self.execute.call_args_list]
        self.assertEqual(targets,[-90.,90.,-90.])
        self.assertEqual(self.clear.call_count,2)
        self.assertEqual(self.flow.completed,2)
        self.assertFalse(self.flow.active)

    def test_selected_search_speed_is_used_for_all_confirmation_moves(self):
        self.flow.confirm(-90.,90.,24.)
        self.tick();self.complete()
        for _ in range(2):
            self.tick();self.feed_arc(17.);self.complete()
        self.assertEqual([call.args[3] for call in self.execute.call_args_list],[24.,24.,24.])

    def test_confirmation_progress_uses_actual_angle_in_both_directions(self):
        self.flow.confirm(-90.,90.,36.)
        self.assertIsNone(self.flow.snapshot()['progress'])
        self.tick();self.complete();self.tick()
        def reading(angle):
            self.now+=.01
            self.tick(dict(fresh=True,angle_fresh=True,sample_time=self.now,
                           angle_deg=angle,volts=2.4,response_direction=1))
        reading(-45.)
        self.assertEqual(self.flow.snapshot()['progress'],.25)
        reading(-50.)
        self.assertEqual(self.flow.snapshot()['progress'],.25)
        self.feed_arc(17.);self.complete()
        self.assertEqual(self.flow.snapshot()['progress'],1.)
        self.tick();reading(0.)
        self.assertEqual(self.flow.snapshot()['progress'],1.5)
        self.feed_arc(17.);self.complete()
        self.assertEqual(self.flow.snapshot()['progress'],2.)
        self.assertEqual(self.flow.phase,'CONFIRMED')

    def test_missing_or_invalid_search_speed_fails_explicitly(self):
        for speed in (None,0.,-1.,math.nan,math.inf,True):
            with self.assertRaisesRegex(ValueError,'search sweep speed'):
                self.flow.confirm(-90.,90.,speed)
        self.execute.assert_not_called()

    def test_operator_starts_confirmation_at_reported_search_speed(self):
        path=Path(__file__).parents[1]/'scripts/operator_control_node.py'
        source=ast.parse(path.read_text())
        kind=next(node for node in source.body if isinstance(node,ast.ClassDef))
        method=next(node for node in kind.body if isinstance(node,ast.FunctionDef) and node.name=='_start_inspection')
        scope=dict(math=math,time=SimpleNamespace(monotonic=lambda:100.),STOP_TAIL_S=.5)
        exec(compile(ast.Module(body=[method],type_ignores=[]),str(path),'exec'),scope)
        owner=SimpleNamespace(_inspection=self.flow,_calibrating=False,_probe_active=lambda:False,
            _pump_active=False,_arm_state=dict(connected=True,calibration=dict(minimum=1024,center=2048,maximum=3072)),
            _arm_state_at=100.,get_parameter=lambda name:SimpleNamespace(value=0.),
            _inspection_clients={'arm':SimpleNamespace(server_is_ready=lambda:True)},
            _inspection_detector_at=100.,_inspection_detector=dict(fresh=True,sweep_speed_deg_s=36.),
            _stop_motion_locked=Mock(),_publish_twist=Mock())
        scope['_start_inspection'](owner,dict(action='confirm',speed_deg_s=99.))
        self.tick()
        self.assertEqual(self.execute.call_args.args[:4],('arm','angle',-90.,36.))
        self.assertEqual(self.flow.confirm_speed,36.)

    def test_stale_old_peak_cannot_confirm_a_pass(self):
        self.flow.confirm(-90.,90.,36.)
        self.tick();self.complete();self.tick()
        self.tick(dict(fresh=True,angle_fresh=True,sample_time=0.,angle_deg=2.,volts=4.,filtered_volts=4.,response_direction=1))
        self.complete()
        self.assertEqual(self.flow.phase,'ERROR')
        self.assertIn('Pass 1',self.flow.message)
        self.assertIsNone(self.flow.confirmed_angle)

    def test_confirmation_matches_arcs_without_any_absolute_mine_level(self):
        self.flow.confirm(-90.,90.,36.);self.tick();self.complete()
        self.tick();self.feed_arc(17.,baseline=1.3,height=.005);self.complete()
        self.tick();self.feed_arc(18.,baseline=3.2,height=.025);self.complete()
        self.assertEqual(self.flow.phase,'CONFIRMED')
        self.assertAlmostEqual(self.flow.confirmed_angle,17.5,delta=.2)
        self.assertEqual(len(self.flow.passes),2)

    def test_broad_arcs_like_recorded_failure_match_despite_six_degree_shift(self):
        self.flow.confirm(-90.,90.,36.);self.tick();self.complete()
        self.tick();self.feed_arc(-13.0558,height=.0145,sigma=16.6);self.complete()
        self.tick();self.feed_arc(-6.6516,baseline=2.46,height=.015875,sigma=19.2);self.complete()
        self.assertEqual(self.flow.phase,'CONFIRMED')
        self.assertAlmostEqual(self.flow.confirmed_angle,-9.8537,delta=.3)
        self.assertGreater(self.flow.arc_overlap,.7)
        self.assertLess(self.flow.arc_overlap,.85)
        overlap=self.flow.arc_overlap
        self.flow.probe(-90.,90.,80.)
        self.assertEqual(self.flow.snapshot()['arc_overlap_ratio'],overlap)

    def test_same_shift_on_narrow_arcs_is_rejected(self):
        self.flow.confirm(-90.,90.,36.);self.tick();self.complete()
        self.tick();self.feed_arc(0.,sigma=2.);self.complete()
        self.tick();self.feed_arc(6.4,sigma=2.);self.complete()
        self.assertEqual(self.flow.phase,'ERROR')
        self.assertIsNone(self.flow.confirmed_angle)
        self.assertLess(self.flow.arc_overlap,.5)

    def test_coincident_peaks_with_inconsistent_widths_are_rejected(self):
        self.flow.confirm(-90.,90.,36.);self.tick();self.complete()
        self.tick();self.feed_arc(17.,sigma=3.);self.complete()
        self.tick();self.feed_arc(17.,sigma=18.);self.complete()
        self.assertEqual(self.flow.phase,'ERROR')
        self.assertIsNone(self.flow.confirmed_angle)
        self.assertLess(self.flow.arc_overlap,.5)

    def test_disagreeing_arc_locations_cannot_start_a_probe_scan(self):
        self.flow.confirm(-90.,90.,36.);self.tick();self.complete()
        self.tick();self.feed_arc(17.);self.complete()
        self.tick();self.feed_arc(32.);self.complete()
        self.assertEqual(self.flow.phase,'ERROR')
        self.assertIsNone(self.flow.confirmed_angle)
        self.assertIn('overlap',self.flow.message)

    def test_probe_waits_for_each_retraction_before_next_angle(self):
        self.confirm()
        self.flow.probe(-90.,90.,80.)
        for index,angle in enumerate((-7.,-2.,3.,8.,13.)):
            self.tick()
            self.assertEqual(self.execute.call_args.args[:4],('arm','angle',angle-45.,5.))
            count=self.execute.call_count;self.tick()
            self.assertEqual(self.execute.call_count,count)
            self.complete();self.tick()
            self.assertEqual(self.execute.call_args.args[:4],('probe','probe_cycle',80.,5.))
            self.tick();self.assertEqual(self.execute.call_count,count+1)
            self.complete(contact=index%2==0,contact_depth_mm=12.+index)
        self.assertEqual(self.flow.phase,'COMPLETE')
        self.assertEqual(self.flow.completed,5)
        self.assertEqual(self.flow.points[0],dict(angle_deg=-7.,status='CONTACT',contact_depth_mm=12.))
        self.assertIsNone(self.flow.points[1]['contact_depth_mm'])

    def test_probe_positions_are_clipped_and_deduplicated(self):
        self.flow.confirmed_angle=88.
        self.flow.probe(-45.,45.,50.)
        self.assertEqual([target for kind,target in self.flow.queue if kind=='position'],[33.,38.,43.,45.])
        self.assertEqual(self.flow.total,4)

    def test_target_outside_probe_reach_requires_repositioning(self):
        self.flow.confirmed_angle=-20.
        self.flow.lower,self.flow.upper=-45.,45.
        self.assertFalse(self.flow.snapshot()['probe_reachable'])
        with self.assertRaisesRegex(ValueError,'outside probe reach'):
            self.flow.probe(-45.,45.,80.)
        self.execute.assert_not_called()

    def test_confirm_does_not_depend_on_probe_setup(self):
        self.flow.confirm(-10.,10.,36.)
        self.tick()
        self.assertEqual(self.execute.call_args.args[0],'arm')

    def test_idle_handoff_is_required(self):
        self.flow.confirm(-10.,10.,36.)
        self.tick(idle=False)
        self.execute.assert_not_called()
        self.tick()
        self.execute.assert_called_once()

    def test_failed_servo_handoff_reports_error(self):
        self.flow.confirm(-10.,10.,36.)
        self.now+=2.1
        self.tick(idle=False)
        self.assertEqual(self.flow.phase,'ERROR')
        self.assertIn('Servo did not release',self.flow.message)
        self.execute.assert_not_called()

    def test_cancel_ignores_late_action_result(self):
        self.flow.confirm(-10.,10.,36.);self.tick()
        late=self.execute.call_args.args[-1]
        self.flow.stop('RC kill')
        late(dict(success=True));self.tick()
        self.assertEqual(self.flow.phase,'STOPPED')
        self.assertEqual(self.execute.call_count,1)
        self.cancel.assert_called_once()

    def test_failed_operation_is_not_reported_as_clear(self):
        self.flow.confirmed_angle=45.;self.flow.probe(-10.,10.,80.);self.tick();self.complete();self.tick()
        self.execute.call_args.args[-1](dict(success=False,message='Contact input lost'))
        self.assertEqual(self.flow.phase,'ERROR')
        self.assertEqual(self.flow.points,[])
        self.assertEqual(self.flow.message,'Contact input lost')

    def test_invalid_contact_depth_fails(self):
        self.flow.confirmed_angle=45.;self.flow.probe(-10.,10.,80.);self.tick();self.complete();self.tick()
        self.complete(contact=True,contact_depth_mm=math.nan)
        self.assertEqual(self.flow.phase,'ERROR')
        self.assertEqual(self.flow.points,[])

    def test_probe_requires_confirmation_and_valid_depth(self):
        with self.assertRaisesRegex(ValueError,'Confirm'):
            self.flow.probe(-10.,10.,80.)
        self.flow.confirmed_angle=45.
        for depth in (math.nan,0.,-1.):
            with self.assertRaises(ValueError):
                self.flow.probe(-10.,10.,depth)


if __name__ == '__main__':
    unittest.main()
