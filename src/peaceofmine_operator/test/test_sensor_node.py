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
    from std_msgs.msg import Bool, Float32, UInt16
except ImportError:
    rclpy = None


@unittest.skipIf(rclpy is None, 'ROS environment required')
class SensorNodeTest(unittest.TestCase):
    def test_publish_calibrate_and_expire(self):
        script = Path(__file__).resolve().parents[1] / 'scripts/sensor_serial_node.py'
        spec = importlib.util.spec_from_file_location('sensor_serial_node', script)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        master, slave = pty.openpty()
        rclpy.init(args=['--ros-args', '-p', f'serial_port:={os.ttyname(slave)}',
                        '-p', 'baseline_adc:=20.0', '-p', 'full_response_adc:=120.0',
                        '-p', 'stale_timeout:=0.2'])
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
            spin(0.1)
            count = len(raw)
            spin(0.3)
            self.assertFalse(fresh[-1])
            self.assertEqual(len(raw), count)
        finally:
            executor.shutdown()
            node.destroy_node()
            observer.destroy_node()
            rclpy.shutdown()
            os.close(master)
            os.close(slave)
