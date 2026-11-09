"""Exercise real ROS subscriptions/timers against a synthetic drivetrain only."""
import importlib.util
import math
import os
from pathlib import Path
import time
import unittest


@unittest.skipUnless(importlib.util.find_spec('rclpy'), 'ROS 2 required')
class CrawlRosTest(unittest.TestCase):
    def test_feedback_tracking_and_watchdogs(self):
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
        from rclpy.parameter import Parameter
        from geometry_msgs.msg import Twist, TwistWithCovarianceStamped
        from mavros_msgs.msg import ManualControl

        spec = importlib.util.spec_from_file_location('crawl_adapter', Path(__file__).parents[1]/'scripts/twist_consumer.py')
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        rclpy.init()
        namespace = f'/crawl_test_{os.getpid()}'
        adapter = module.twist_consumer(namespace=namespace, parameter_overrides=[
            Parameter('crawl_enabled', value=True), Parameter('cmd_timeout', value=.25)])
        plant = Node('synthetic_drivetrain', namespace=namespace)
        executor = SingleThreadedExecutor()
        executor.add_node(adapter)
        executor.add_node(plant)
        speed, throttle, outputs = 0., 0., []

        def control(msg):
            nonlocal throttle
            throttle = (500.-msg.z)/500.
            outputs.append(throttle)

        subscription = plant.create_subscription(ManualControl, 'mavros/manual_control/send', control, 10)
        commands = plant.create_publisher(Twist, 'cmd_vel', 10)
        feedback = plant.create_publisher(TwistWithCovarianceStamped, 'mavros/wheel_odometry/velocity', 10)

        def run(seconds, request, publish_feedback=True):
            nonlocal speed
            end = time.monotonic()+seconds
            previous = time.monotonic()
            while time.monotonic() < end:
                now = time.monotonic()
                dt = min(.05, now-previous)
                previous = now
                desired = math.copysign(max(0., abs(throttle)-.01)*1.7, throttle)
                speed += (desired-speed)*dt/.3
                if publish_feedback:
                    msg = TwistWithCovarianceStamped()
                    msg.header.stamp = plant.get_clock().now().to_msg()
                    msg.twist.twist.linear.x = speed
                    feedback.publish(msg)
                if request is not None:
                    msg = Twist()
                    msg.linear.x = request
                    commands.publish(msg)
                executor.spin_once(timeout_sec=.02)

        try:
            adapter.__rosonic_startup__(adapter)
            run(.5, 0.)  # Discovery and feedback before requesting motion.
            for direction in (1., -1.):
                run(16., direction*.8)
                self.assertFalse(adapter._crawl.fault)
                self.assertAlmostEqual(speed, direction*.1, delta=.012)
                run(.8, 0.)
            self.assertTrue(outputs)
            self.assertLessEqual(max(map(abs, outputs)), .2)
            run(.3, .1)
            run(.5, None)
            self.assertAlmostEqual(throttle, 0.)  # Command watchdog.
            run(.3, 0.)
            run(.5, .1, publish_feedback=False)
            self.assertAlmostEqual(throttle, 0.)
            self.assertIn('stale', adapter._crawl.fault)
        finally:
            if adapter._is_started():
                adapter.__rosonic_shutdown__(adapter)
            executor.shutdown()
            plant.destroy_subscription(subscription)
            plant.destroy_node()
            adapter.destroy_node()
            rclpy.shutdown()
