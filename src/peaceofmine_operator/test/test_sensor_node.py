"""ROS integration using a pseudo-terminal, with no physical hardware."""

import importlib.util
import json
import os
from pathlib import Path
import pty
import time
import unittest

try:
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from std_msgs.msg import Bool, Float32, UInt16, String
except ImportError:
    rclpy = None


@unittest.skipIf(rclpy is None, 'ROS environment required')
class SensorNodeTest(unittest.TestCase):
    def test_publish_calibrate_and_expire(self):
        script = Path(__file__).resolve().parents[1] / 'scripts/sensor_serial_node.py'
        spec = importlib.util.spec_from_file_location('sensor_serial_node', script)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        import tempfile
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        master, slave = pty.openpty()
        rclpy.init(args=['--ros-args', '-p', f'serial_port:={os.ttyname(slave)}',
                        '-p', 'baseline_adc:=20.0', '-p', 'full_response_adc:=120.0',
                        '-p', f'calibration_file:={directory.name}/calibration.json', '-p', 'stale_timeout:=0.2'])
        node = module.SensorSerialNode()
        observer = rclpy.create_node('sensor_observer')
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        executor.add_node(observer)
        raw, ratio, fresh = [], [], []
        subscriptions = [
            observer.create_subscription(UInt16, 'detector/amplitude_adc', lambda m: raw.append(m.data), 10),
            observer.create_subscription(Float32, 'detector/signal_ratio', lambda m: ratio.append(m.data), 10),
            observer.create_subscription(Bool, 'detector/fresh', lambda m: fresh.append(m.data), 10),
        ]

        def spin(seconds):
            end = time.monotonic() + seconds
            while time.monotonic() < end:
                executor.spin_once(timeout_sec=0.01)

        try:
            spin(0.5)
            for seq in range(10):
                os.write(master, (json.dumps(dict(v=1, seq=seq, uptime_ms=seq*50,
                                                 amplitude_adc=70)) + '\n').encode())
                spin(0.05)
            self.assertEqual(raw[-1], 70)
            self.assertAlmostEqual(ratio[-1], 0.5)
            self.assertTrue(fresh[-1])
            node.calibrate(String(data=json.dumps(dict(action='zero', request_id='zero'))))
            self.assertTrue(node.command_result['ok'])
            self.assertEqual(node.baseline, 70)
            node.calibrate(String(data=json.dumps(dict(action='apply', request_id='apply', baseline_adc=20,
                                                       full_response_adc=150, reference_voltage=5.0))))
            self.assertTrue(node.command_result['ok'])
            self.assertAlmostEqual(20 * node.reference_voltage / 255, .3921568627)
            node.calibrate(String(data=json.dumps(dict(action='apply', request_id='bad', baseline_adc=20,
                                                       full_response_adc=20))))
            self.assertFalse(node.command_result['ok'])
            self.assertEqual(node.full_response, 150)
            restarted = module.SensorSerialNode()
            try:
                self.assertEqual(restarted.baseline, 20)
                self.assertEqual(restarted.full_response, 150)
            finally:
                restarted.destroy_node()
            now = time.monotonic()
            node.recent_samples.clear()
            node.recent_samples.append((now - 11, 255))
            for index in range(500):
                node.recent_samples.append((now - (499 - index) * .02, 20 if index < 450 else 100))
            node.calibrate(String(data=json.dumps(dict(action='zero', request_id='ten-seconds'))))
            self.assertTrue(node.command_result['ok'])
            self.assertEqual(node.baseline, 28)
            self.assertEqual(node.full_response, 150)
            self.assertEqual(len(node.recent_samples), 500)
            self.assertIn('500 readings', node.command_result['message'])
            spin(0.1)
            count = len(raw)
            spin(0.3)
            self.assertFalse(fresh[-1])
            node.calibrate(String(data=json.dumps(dict(action='zero', request_id='stale'))))
            self.assertFalse(node.command_result['ok'])
            self.assertEqual(len(raw), count)
        finally:
            executor.shutdown()
            node.destroy_node()
            observer.destroy_node()
            rclpy.shutdown()
            os.close(master)
            os.close(slave)
