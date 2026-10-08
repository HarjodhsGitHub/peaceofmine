"""GNSS ROS presentation and existing bridge snapshots; no physical UART."""
import json
import time
import unittest
from unittest.mock import patch
import rclpy
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import TwistWithCovarianceStamped
from std_msgs.msg import String
from test_operator_transport import script
from peaceofmine_operator.web_bridge import OperatorWebBridge


class GNSSNodeTest(unittest.TestCase):
    def test_simulation_ros_fix_state_and_bridge_staleness(self):
        rclpy.init(args=['--ros-args','-p','simulation:=true','-p','safety_simulation:=true','-r','__ns:=/gnss_test'])
        gnss=script('gnss_node').GNSSNode()
        bridge=OperatorWebBridge()
        control=script('operator_control_node').OperatorControl()
        observer=rclpy.create_node('observer')
        executor=SingleThreadedExecutor()
        nodes=(gnss,bridge,control,observer)
        for node in nodes: executor.add_node(node)
        fixes,states=[],[]
        observer.create_subscription(NavSatFix,'gnss/fix',fixes.append,10)
        observer.create_subscription(String,'gnss/state',lambda m:states.append(json.loads(m.data)),10)
        def spin(seconds):
            end=time.monotonic()+seconds
            while time.monotonic()<end: executor.spin_once(timeout_sec=.01)
        try:
            spin(.9)
            self.assertTrue(fixes and states)
            self.assertFalse(control.snapshot()['robot']['pose_available'])
            self.assertIsNone(control.snapshot()['robot']['x'])
            wheel=TwistWithCovarianceStamped(); wheel.header.stamp.sec=1; wheel.twist.twist.linear.x=.2
            control._wheel_velocity_cb(wheel)
            self.assertEqual(control.snapshot()['robot']['speed_mps'],.2)
            self.assertFalse(control.snapshot()['robot']['pose_available'])
            self.assertEqual(fixes[-1].status.status,0)
            self.assertEqual(states[-1]['fix_label'],'RTK FIXED')
            self.assertTrue(states[-1]['simulation'])
            self.assertEqual(gnss.threads,[])
            self.assertEqual(bridge.snapshot()['gnss']['fix_quality'],4)
            gnss.simulation=False
            gnss.state.field_updated['latitude']-=10
            gnss.publish();spin(.2)
            self.assertEqual(fixes[-1].status.status,-1)
            self.assertIsNone(states[-1]['latitude'])
            self.assertFalse(states[-1]['fresh'])
            bridge._gnss_at-=3
            self.assertFalse(bridge.snapshot()['gnss']['connected'])
        finally:
            executor.shutdown()
            for node in nodes: node.destroy_node()
            rclpy.shutdown()
