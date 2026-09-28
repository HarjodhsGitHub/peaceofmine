"""Browser -> ROS control -> real controllers -> simulated hardware -> browser."""
import unittest
import test_arm_node as fixtures
from rclpy.parameter import Parameter
from peaceofmine_operator.web_bridge import OperatorWebBridge


class SimulationPipelineTest(unittest.TestCase):
    def test_same_controllers_through_web_and_disconnect(self):
        fixture=fixtures.ActuatorTest()
        fixture.setUp()
        fixture.send_permission=False
        control=fixtures.module('operator_control_node').OperatorControl(parameter_overrides=[Parameter('safety_simulation',value=True)])
        bridge=OperatorWebBridge()
        for node in (control,bridge):fixture.executor.add_node(node)
        try:
            client=object()
            bridge.connect_client(client)
            fixture.spin(.2)
            bridge.handle_command(client,dict(type='take_control'))
            fixture.spin(.2)
            bridge.handle_command(client,dict(type='arm',calibration=True))
            fixture.spin(.1)
            for _ in range(16):
                bridge.handle_command(client,dict(type='arm_servo',role='probe',action='home',held=True,hold_id='pipeline',load_percent=20))
                fixture.spin(.08)
                if fixture.probe.home_position is not None:break
            self.assertIsNotNone(fixture.probe.home_position,fixture.probe.reason)
            bridge.handle_command(client,dict(type='disarm'))
            fixture.spin(.1)
            bridge.handle_command(client,dict(type='arm'))
            fixture.spin(.1)
            bridge.handle_command(client,dict(type='probe_target',depth_mm=20.))
            fixture.spin(.3)
            self.assertEqual(bridge.snapshot()['probe']['target_mm'],20.)
            self.assertGreater(bridge.snapshot()['probe']['depth_mm'],0.)
            bridge.handle_command(client,dict(type='arm_servo',action='stop',role='probe'))
            fixture.spin(.15)
            bridge.handle_command(client,dict(type='sweep_enabled',enabled=True))
            fixture.spin(.25)
            self.assertTrue(fixture.arm.sweeping,fixture.arm.reason)
            bridge._shutting_down=True
            fixture.spin(.9)
            self.assertIsNone(control._lease)
            self.assertIsNone(fixture.driver.runtime.active)
            self.assertFalse(fixture.arm.sweeping)
        finally:
            for node in (bridge,control):
                fixture.executor.remove_node(node)
                node.destroy_node()
            fixture.tearDown()
