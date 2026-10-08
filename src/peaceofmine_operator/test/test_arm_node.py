"""Production controllers exercised with the production driver and a fake bus."""
import importlib.util
import json
from pathlib import Path
import tempfile
import time
import unittest
import rclpy
from rclpy.parameter import Parameter
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import Bool, String
from nav_msgs.msg import Odometry
from peaceofmine_operator import configuration


def module(name):
    spec = importlib.util.spec_from_file_location(name, Path(__file__).parents[1]/'scripts'/f'{name}.py')
    result = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(result)
    return result


class ActuatorTest(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        self.tmp = tempfile.TemporaryDirectory()
        self.path = str(Path(self.tmp.name)/'operator_config.json')
        configuration.save_section(self.path,'arm',dict(servo_id=1,minimum=920,center=1533,maximum=2224))
        configuration.save_section(self.path,'probe',dict(servo_id=2,max_extension_mm=120.,travel_ticks=5430))
        params = [Parameter('config_file',value=self.path), Parameter('simulation',value=True)]
        self.driver = module('servo_driver_node').ServoDriver(parameter_overrides=params)
        self.arm = module('arm_controller_node').ArmController(parameter_overrides=params)
        self.probe = module('probe_controller_node').ProbeController(parameter_overrides=params)
        self.settings = module('operator_settings_node').OperatorSettings(parameter_overrides=params)
        self.observer = rclpy.create_node('test_vehicle')
        self.permission = self.observer.create_publisher(Bool,'operator/actuation_enabled',1)
        self.odom = self.observer.create_publisher(Odometry,'odometry/local',1)
        self.allow = True
        self.publish_velocity = True
        self.velocity = 0.
        self.observer.create_timer(.04,self.inputs)
        self.nodes = (self.driver,self.arm,self.probe,self.settings,self.observer)
        self.executor = SingleThreadedExecutor()
        for node in self.nodes:
            self.executor.add_node(node)
        self.spin(.6)
        self.assertTrue(self.arm.connected(),self.driver.reason)

    def inputs(self):
        if getattr(self, 'send_permission', True):
            self.permission.publish(Bool(data=self.allow))
        if self.publish_velocity:
            msg = Odometry()
            msg.header.stamp = self.observer.get_clock().now().to_msg()
            msg.twist.twist.linear.x = self.velocity
            self.odom.publish(msg)

    def spin(self, seconds):
        end = time.monotonic()+seconds
        while time.monotonic()<end:
            self.executor.spin_once(timeout_sec=.005)

    def command(self, node, **value):
        node.command_cb(String(data=json.dumps(dict(request_id='test',**value))))
        return node.last_command

    def tearDown(self):
        for node in (self.arm,self.probe):
            node.stop()
        self.spin(.1)
        self.executor.shutdown()
        for node in self.nodes:
            node.destroy_node()
        rclpy.shutdown()
        self.tmp.cleanup()

    def test_configured_speed_limit_survives_typed_status(self):
        from peaceofmine_interfaces.msg import ActuatorStatus
        from peaceofmine_operator.actuator_protocol import status_dict
        received=[]
        sub=self.observer.create_subscription(ActuatorStatus,'arm/state',received.append,1)
        self.arm.motion_speed=116
        self.spin(.15)
        self.assertTrue(received)
        value=status_dict(received[-1])
        self.assertEqual(value['motion_speed_limit'],116)
        self.assertAlmostEqual(value['motion_speed_limit']*.684,79.344)
        self.observer.destroy_subscription(sub)

    def test_idle_permission_updates_preserve_stop_reason(self):
        self.arm.stop('Servo overload; inspect the mechanism')
        self.arm.permission_cb(Bool(data=False))
        self.arm.permission_cb(Bool(data=False))
        self.assertEqual(self.arm.reason, 'Servo overload; inspect the mechanism')
        self.assertIsNone(self.arm.operation)

    def test_slider_and_jog_obey_saved_speed(self):
        self.command(self.arm,action='configure_motion',max_speed_deg_s=10.,acceleration_deg_s2=34.332)
        self.spin(1.1)
        saved=configuration.load(self.path)
        self.assertNotIn('arm_motion',saved)
        self.assertEqual(saved['arm_motion_simulation']['max_speed_deg_s'],10.)
        self.command(self.arm,action='position',position=2100,held=True)
        self.spin(.12)
        self.assertIsNotNone(self.driver.runtime.active,self.arm.reason)
        self.assertEqual(self.driver.runtime.bus.values[1][32],14)
        self.command(self.arm,action='stop')
        self.spin(.1)
        self.command(self.arm,action='jog',direction=1,speed_deg_s=100.,held=True)
        self.spin(.12)
        self.assertEqual(self.driver.runtime.bus.values[1][32],14)

    def test_home_and_jog_reject_fault(self):
        self.probe.fault = True
        for command in (dict(action='home',held=True,hold_id='fault',load_percent=20),
                        dict(action='jog',held=True,direction=1)):
            self.assertFalse(self.command(self.probe,**command)['success'])
            self.assertIsNone(self.probe.operation)

    def test_vehicle_motion_does_not_stop_homing(self):
        self.command(self.probe,action='home',held=True,hold_id='home',load_percent=20)
        self.spin(.1)
        self.assertIsNotNone(self.probe.operation,self.probe.reason)
        self.velocity=1.
        self.spin(.12)
        self.assertIsNotNone(self.probe.operation,self.probe.reason)
        self.allow=False
        self.spin(.12)
        self.assertIsNone(self.probe.operation)
        self.assertIsNone(self.probe.home_position)

    def test_jog_without_velocity_still_times_out(self):
        self.publish_velocity=False
        self.spin(.6)
        self.assertTrue(self.command(self.probe,action='jog',held=True,direction=1)['success'])
        self.spin(.1)
        self.assertIsNotNone(self.driver.runtime.active)
        self.spin(.45)
        self.assertIsNone(self.driver.runtime.active)

    def test_expired_manual_hold_does_not_restart(self):
        self.command(self.arm,action='jog',held=True,direction=1)
        self.spin(.45)
        self.assertIsNone(self.driver.runtime.active)
        self.command(self.arm,action='jog',held=True,direction=1)
        self.spin(.1)
        self.assertIsNone(self.driver.runtime.active)
        self.command(self.arm,action='stop')
        self.command(self.arm,action='jog',held=True,direction=1)
        self.spin(.1)
        self.assertIsNotNone(self.driver.runtime.active)

    def test_homing_contact_and_reconnect_invalidates_zero(self):
        deadline=time.monotonic()+3
        while self.probe.home_position is None and time.monotonic()<deadline:
            self.command(self.probe,action='home',held=True,hold_id='home',load_percent=20)
            self.spin(.08)
        self.assertIsNotNone(self.probe.home_position,self.probe.reason)
        self.assertFalse(self.driver.runtime.bus.values[2][24])
        self.driver.disconnect('test unplug')
        self.spin(.1)
        self.assertIsNone(self.probe.home_position)

    def test_stale_odometry_allows_extension(self):
        self.probe.home_position=0
        self.publish_velocity=False
        self.spin(.6)
        self.assertTrue(self.command(self.probe,action='target',depth_mm=20)['success'])
        self.spin(.1)
        self.assertIsNotNone(self.driver.runtime.active)

    def test_action_home_feedback_result_and_cancel(self):
        from rclpy.action import ActionClient
        from peaceofmine_interfaces.action import ActuatorOperation
        client=ActionClient(self.observer,ActuatorOperation,'probe/execute')
        feedback=[]
        self.spin(.2)
        goal=client.send_goal_async(ActuatorOperation.Goal(operation='home',load_percent=20.),feedback_callback=feedback.append)
        deadline=time.monotonic()+4
        while not goal.done() and time.monotonic()<deadline:self.spin(.02)
        self.assertTrue(goal.done())
        handle=goal.result()
        self.assertTrue(handle.accepted)
        result=handle.get_result_async()
        while not result.done() and time.monotonic()<deadline:self.spin(.02)
        self.assertTrue(result.done(),self.probe.reason)
        self.assertTrue(result.result().result.success,result.result().result.message)
        self.assertTrue(feedback)
        goal=client.send_goal_async(ActuatorOperation.Goal(operation='target',target=100.))
        while not goal.done():self.spin(.02)
        handle=goal.result()
        self.assertTrue(handle.accepted)
        self.spin(.1)
        cancel=handle.cancel_goal_async()
        result=handle.get_result_async()
        deadline=time.monotonic()+2
        while not result.done() and time.monotonic()<deadline:self.spin(.02)
        self.assertTrue(result.done())
        self.assertFalse(result.result().result.success)
        self.spin(.1)
        self.assertIsNone(self.driver.runtime.active)
        client.destroy()
