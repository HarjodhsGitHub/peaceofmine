"""Run with a sourced ROS overlay on an isolated ROS_DOMAIN_ID."""

import importlib.util
from pathlib import Path
import unittest
from unittest.mock import patch

import rclpy
from sensor_msgs.msg import Joy

spec = importlib.util.spec_from_file_location(
    'operator_gateway', Path(__file__).parents[1] / 'scripts/operator_gateway.py')
gateway_module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(gateway_module)


class TeleopTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init(args=['--ros-args', '-r', '__ns:=/teleop_test'])

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = gateway_module.OperatorGateway()
        self.owner = object()
        self.clock = patch.object(gateway_module.time, 'monotonic', return_value=100.0)
        self.now = self.clock.start()
        self.output = patch.object(self.node, '_publish_twist').start()

    def tearDown(self):
        patch.stopall()
        self.node.destroy_node()

    def command(self, kind, **kwargs):
        return self.node.handle_command(self.owner, dict(type=kind, **kwargs))

    def arm(self):
        self.command('take_control')
        self.command('arm')

    def test_browser_requires_lease_arm_and_deadman(self):
        drive = dict(linear_x=0.4, angular_z=-0.2, deadman=True)
        self.assertIsNotNone(self.command('drive', **drive))
        self.command('take_control')
        self.command('drive', **drive)
        self.output.assert_not_called()
        self.command('arm')
        self.output.assert_called_with(0.4, -0.2)
        self.command('drive', **dict(drive, deadman=False))
        self.output.assert_called_with(0.0, 0.0)

    def test_browser_timeout_disarm_and_disconnect_stop(self):
        for stop in ('timeout', 'disarm', 'disconnect'):
            with self.subTest(stop=stop):
                self.arm()
                self.command('drive', linear_x=0.4, angular_z=0.0, deadman=True)
                self.output.assert_called_with(0.4, 0.0)
                if stop == 'timeout':
                    self.now.return_value += 0.3
                    self.node._drive_watchdog()
                elif stop == 'disarm':
                    self.command('disarm')
                else:
                    self.node.release(self.owner)
                self.output.assert_called_with(0.0, 0.0)
                self.now.return_value += 1.0
                self.output.reset_mock()
                self.node._drive_watchdog()
                self.output.assert_not_called()

    def test_ros_xbox_triggers_and_back_start(self):
        self.arm()
        buttons = [0] * 11
        for lt, rt, velocity in ((1., 1., 0.), (1., -1., 0.8),
                                 (-1., 1., -0.8), (-1., -1., 0.)):
            with self.subTest(lt=lt, rt=rt):
                self.node._joy_cb(Joy(axes=[0., 0., lt, 0., 0., rt], buttons=buttons))
                self.assertAlmostEqual(self.node._joy['linear'], velocity)
                self.assertEqual(self.node._joy['deadman'], velocity != 0.)
        buttons[6] = buttons[7] = 1
        self.node._joy_cb(Joy(axes=[0., 0., 1., 0., 0., 1.], buttons=buttons))
        self.assertEqual(self.node._joy['linear'], 0.)
        self.assertFalse(self.node._joy['deadman'])

    def test_uninitialized_ros_triggers_are_released(self):
        parsed = self.node._parse_xbox_joy(Joy(axes=[0.] * 6, buttons=[0] * 11))
        self.assertEqual((parsed['lt'], parsed['rt']), (0., 0.))
        self.assertFalse(parsed['deadman'])


if __name__ == '__main__':
    unittest.main()
