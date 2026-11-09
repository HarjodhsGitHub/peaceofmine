"""Exercise the production Twist adapter without starting ROS or vehicle I/O."""
import ast
import math
import importlib.util
import time
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

_spec = importlib.util.spec_from_file_location('crawl', Path(__file__).parents[2]/'svea_core/svea_core/controllers/crawl.py')
_module = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_module)
CrawlController = _module.CrawlController


def adapter_type():
    path = Path(__file__).parents[1]/'scripts/twist_consumer.py'
    source = ast.parse(path.read_text())
    controller = next(node for node in source.body if isinstance(node, ast.ClassDef))
    methods = [node for node in controller.body if isinstance(node, ast.FunctionDef)]
    for node in methods:
        node.decorator_list = []
    controller.bases, controller.body = [], methods
    scope = dict(math=math, time=time, Twist=SimpleNamespace, Bool=object)
    exec(compile(ast.Module(body=[controller], type_ignores=[]), str(path), 'exec'),scope)
    return scope['twist_consumer']


class TwistConsumerTest(unittest.TestCase):
    def setUp(self):
        self.node=adapter_type()()
        self.node.wheelbase=.324
        self.node.max_steering=.524
        self.node.max_velocity=.8
        self.node.min_speed_for_steering=.05
        self.node.velocity_sign=self.node.steering_sign=1.
        self.node.cmd_timeout=.25
        self.node._pump_until=None
        self.node._crawl=None
        self.node.actuation=Mock()
        self.now=1_000_000_000
        class Stamp:
            def __init__(stamp, value): stamp.nanoseconds=value
            def __sub__(stamp, other): return Stamp(stamp.nanoseconds-other.nanoseconds)
        self.node.get_clock=lambda:SimpleNamespace(now=lambda:Stamp(self.now))

    def command(self, speed, yaw):
        self.node.twist_cb(SimpleNamespace(linear=SimpleNamespace(x=speed), angular=SimpleNamespace(z=yaw)))
        self.node.loop()
        return self.node.actuation.send_control.call_args.args

    def test_left_and_right_steer_at_rest_without_any_throttle(self):
        for yaw,sign in ((1.4,1),(-1.4,-1)):
            steering,velocity=self.command(0.,yaw)
            self.assertEqual(velocity,0.)
            self.assertEqual(steering,sign*self.node.max_steering)
        self.assertEqual(self.command(0.,0.),(0.,0.))

    def test_small_stationary_input_is_proportional_and_bounded(self):
        low=self.node._omega_to_steering(0.,.01)
        high=self.node._omega_to_steering(0.,.02)
        self.assertGreater(low,0.)
        self.assertGreater(high,low)
        self.assertLess(high,self.node.max_steering)
        for speed in (-.04,0.,.04):
            self.assertAlmostEqual(self.node._omega_to_steering(speed,.01),low)

    def test_moving_ackermann_geometry_and_reverse_are_preserved(self):
        expected=math.atan(.324*.2/.4)
        self.assertAlmostEqual(self.node._omega_to_steering(.4,.2),expected)
        self.assertAlmostEqual(self.node._omega_to_steering(-.4,.2),expected)
        self.assertEqual(self.command(2.,0.)[1],.8)

    def test_stationary_steering_still_neutralizes_on_command_timeout(self):
        self.command(0.,1.4)
        self.now+=300_000_000
        self.node.loop()
        self.assertEqual(self.node.actuation.send_control.call_args.args,(0.,0.))

    def test_invalid_speed_floor_fails_startup(self):
        self.node.twist_type='geometry_msgs/msg/Twist'
        for floor in (0.,-1.,math.nan,math.inf):
            self.node.min_speed_for_steering=floor
            with self.assertRaisesRegex(ValueError,'finite and positive'):
                self.node.on_startup()

    def enable_crawl(self):
        self.node._crawl=CrawlController()
        self.node.crawl_speed=.1
        self.node._wheel_speed=0.
        self.node._wheel_at=time.monotonic()
        self.node._wheel_stamp=None
        self.node.actuation.max_speed=1.7
        self.node.get_logger=lambda:Mock()

    def test_crawl_limits_output_and_timeout_resets_integrator(self):
        self.enable_crawl()
        self.assertAlmostEqual(self.command(.8,0.)[1],.03*1.7)
        self.assertEqual(self.node._velocity,.1)
        self.now+=300_000_000
        self.node.loop()
        self.assertEqual(self.node.actuation.send_control.call_args.args,(0.,0.))
        self.assertEqual(self.node._crawl.integral,0.)

    def test_crawl_missing_feedback_never_falls_back_to_open_loop(self):
        self.enable_crawl()
        self.node._wheel_at=None
        self.assertEqual(self.command(.8,0.)[1],0.)
        self.assertIn('stale',self.node._crawl.fault)

    def test_wheel_callback_rejects_old_repeated_and_nonfinite_feedback(self):
        self.enable_crawl()
        def sample(stamp,speed):
            return SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=0,nanosec=stamp)),
                                   twist=SimpleNamespace(twist=SimpleNamespace(linear=SimpleNamespace(x=speed))))
        self.node.wheel_speed_cb(sample(900_000_000,.05))
        self.assertEqual(self.node._wheel_speed,.05)
        for stamp,speed in ((900_000_000,.05),(500_000_000,.05),(1_100_000_000,.05),(950_000_000,math.nan)):
            self.node.wheel_speed_cb(sample(stamp,speed))
            self.assertTrue(math.isnan(self.node._wheel_speed))

    def test_nonfinite_command_neutralizes(self):
        for velocity,yaw in ((math.nan,0.),(.5,math.inf)):
            self.assertEqual(self.command(velocity,yaw),(0.,0.))


if __name__ == '__main__':
    unittest.main()
