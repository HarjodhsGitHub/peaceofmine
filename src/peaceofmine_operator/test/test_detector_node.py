import json
import unittest
from unittest.mock import Mock
import rclpy
from std_msgs.msg import String, Float32
from nav_msgs.msg import Odometry
from test_arm_node import module


class DetectorTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = module('detector_node').DetectorNode()
        self.node.publisher.publish = Mock()

    def tearDown(self):
        self.node.destroy_node()

    def test_threshold_hit_requires_fresh_position(self):
        self.node.signal_cb(Float32(data=1.))
        self.assertEqual(self.node.hits, [])
        self.node.signal_cb(Float32(data=.5))
        self.node.odometry_cb(Odometry())
        self.node.signal_cb(Float32(data=1.))
        self.assertEqual(len(self.node.hits), 1)
        self.node.publish()
        self.assertTrue(json.loads(self.node.publisher.publish.call_args.args[0].data)['detector']['detected'])

    def test_calibrated_plot_bounds_and_reset(self):
        self.node.arm_cb(String(data=json.dumps(dict(arm_calibration=dict(minimum=100, center=1500, maximum=4000)))))
        for angle, index in ((self.node.lower, 0), (self.node.upper, -1)):
            self.node.angle_cb(Float32(data=angle))
            self.node.signal_cb(Float32(data=.5))
            self.assertEqual(self.node.beam[index], .5)
        self.node.arm_cb(String(data=json.dumps(dict(arm_calibration=dict(minimum=100, center=1600, maximum=4000)))))
        self.assertTrue(all(value == 0 for value in self.node.beam))

    def test_successful_calibration_resets_threshold(self):
        self.node.threshold = .65
        self.node.command_cb(String(data='{"action":"apply","request_id":"x"}'))
        self.node.sensor_cb(String(data='{"command_result":{"request_id":"x","ok":true}}'))
        self.assertEqual(self.node.threshold, 1.)
