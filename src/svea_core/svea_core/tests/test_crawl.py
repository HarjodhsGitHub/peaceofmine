"""Regulator checks using a simple simulated drivetrain, without ROS."""
import math
import importlib.util
from pathlib import Path
import unittest


spec = importlib.util.spec_from_file_location('crawl', Path(__file__).parents[1]/'controllers/crawl.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
CrawlController = module.CrawlController


class CrawlTest(unittest.TestCase):
    def test_simulated_drivetrain_tracks_both_directions(self):
        for direction in (-1, 1):
            controller = CrawlController()
            speed = 0.
            for step in range(600):
                throttle = controller.update(direction*.1, speed, 0., step*.05)
                self.assertLessEqual(abs(throttle), .2)
                # First-order plant with a small motor deadband; not a hardware model.
                desired = math.copysign(max(0., abs(throttle)-.01)*1.7, throttle)
                speed += (desired-speed)*.05/.3
            self.assertFalse(controller.fault)
            self.assertAlmostEqual(speed, direction*.1, delta=.005)

    def test_stall_latches_until_release(self):
        controller = CrawlController()
        for step in range(50):
            output = controller.update(.1, 0., 0., step*.1)
        self.assertEqual(output, 0.)
        self.assertIn('no wheel motion', controller.fault)
        self.assertEqual(controller.update(.1, .05, 0., 6.), 0.)
        self.assertEqual(controller.update(0., 0., 0., 6.1), 0.)
        self.assertFalse(controller.fault)
        self.assertGreater(controller.update(.1, 0., 0., 6.2), 0.)

    def test_bad_feedback_and_overspeed_stop(self):
        for speed, age in ((math.nan, 0.), (.01, math.inf), (.01, .3), (.01, -.1), (.2, 0.)):
            controller = CrawlController()
            self.assertEqual(controller.update(.1, speed, age, 1.), 0.)
            self.assertTrue(controller.fault)

    def test_neutral_invalid_commands_and_reversal(self):
        controller = CrawlController()
        self.assertGreater(controller.update(.1, 0., 0., 0.), 0.)
        self.assertEqual(controller.update(0., .05, 0., .1), 0.)
        self.assertEqual(controller.integral, 0.)
        self.assertEqual(controller.update(-.1, .05, 0., .2), 0.)
        self.assertLess(controller.update(-.1, 0., 0., .3), 0.)
        self.assertEqual(controller.update(math.nan, 0., 0., .4), 0.)

    def test_parameters_are_validated(self):
        for kwargs in ({'max_speed': 0}, {'ki': math.nan}, {'max_throttle': 1.}, {'stop_speed': .1}):
            with self.assertRaises(ValueError):
                CrawlController(**kwargs)

    def test_speed_request_capped_and_no_reverse_braking(self):
        controller = CrawlController()
        self.assertAlmostEqual(controller.update(1., 0., 0., 0.), .03)
        self.assertEqual(controller.update(.03, .08, 0., .1), 0.)
