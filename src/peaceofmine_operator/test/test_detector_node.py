import json
import unittest
from unittest.mock import Mock, patch
import time
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

    def test_adc_uses_A2_once_and_rejects_stale_or_replayed_samples(self):
        from peaceofmine_interfaces.msg import AdcSample, AdcSamples
        packet = AdcSamples(valid=True, session='sensor', config_revision=self.node.adc_revision)
        sample = AdcSample(channel=6, sequence=1, volts=2.)
        now = time.time()
        sample.stamp.sec = int(now)
        sample.stamp.nanosec = int((now-int(now))*1e9)
        packet.samples.append(sample)
        self.node.adc_samples_cb(packet)
        self.assertEqual(self.node.seq, 1)
        self.assertAlmostEqual(self.node.last_raw_adc, 102.)
        self.node.adc_samples_cb(packet)
        self.assertEqual(len(self.node.recent_samples), 1)
        sample.sequence = 2
        sample.stamp.sec -= 10
        packet.samples = [sample]
        self.node.adc_samples_cb(packet)
        self.assertEqual(self.node.seq, 1)
        packet.valid = False
        self.node.adc_samples_cb(packet)
        self.node.publish()
        self.assertFalse(json.loads(self.node.publisher.publish.call_args.args[0].data)['detector']['fresh'])

    def test_detection_peak_expires_and_invalidates_after_movement(self):
        self.node.angle_cb(Float32(data=0.))
        self.node.odometry_cb(Odometry())
        self.node.signal_cb(Float32(data=1.))
        self.node.publish()
        self.assertTrue(json.loads(self.node.publisher.publish.call_args.args[0].data)['detector']['latest_peak']['fresh'])
        with patch('time.monotonic', return_value=time.monotonic()+31):
            self.node.publish()
            self.assertFalse(json.loads(self.node.publisher.publish.call_args.args[0].data)['detector']['latest_peak']['fresh'])
        odom = Odometry()
        odom.pose.pose.position.x = 1.
        self.node.odometry_cb(odom)
        self.assertIsNone(self.node.latest_peak)

    def test_peak_keeps_tracking_above_saturated_meter(self):
        self.node.angle_cb(Float32(data=1.))
        self.node.signal_cb(Float32(data=1.2))
        self.node.angle_cb(Float32(data=2.))
        self.node.signal_cb(Float32(data=1.5))
        self.assertEqual(self.node.signal,1.)
        self.assertEqual(self.node.latest_peak['angle_deg'],2.)
        self.assertEqual(self.node.latest_peak['signal'],1.5)
        self.node.angle_cb(Float32(data=3.))
        self.node.signal_cb(Float32(data=1.4))
        self.assertEqual(self.node.latest_peak['angle_deg'],2.)

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
