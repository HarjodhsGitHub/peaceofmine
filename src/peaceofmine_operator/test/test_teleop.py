"""Run with a sourced ROS overlay on an isolated ROS_DOMAIN_ID."""

import importlib.util
from pathlib import Path
import unittest
from unittest.mock import patch

import rclpy
from sensor_msgs.msg import CameraInfo, Joy

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
        self.node._rc_safety.simulation = True
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

    def test_shutdown_stops_outputs_and_rejects_new_control(self):
        self.arm()
        self.command('drive', linear_x=.4, deadman=True)
        with patch.object(self.node._permission_pub, 'publish') as permission, \
                patch.object(self.node._arm_command_pub, 'publish') as servo:
            self.node.begin_shutdown()
            self.output.assert_called_with(0.0, 0.0)
            self.assertFalse(permission.call_args.args[0].data)
            self.assertEqual(servo.call_args.args[0].data, '{"action":"stop"}')
            self.assertIsNotNone(self.command('take_control'))
            self.assertIsNotNone(self.command('arm'))
            self.node._drive_watchdog()
            self.assertFalse(permission.call_args.args[0].data)
            self.assertFalse(self.node._armed)

    def test_servo_commands_work_in_rc_override_while_drive_is_blocked(self):
        from mavros_msgs.msg import State
        self.node._rc_safety.simulation = False
        message = State(connected=True, armed=False, system_status=4)
        message.header.stamp.sec = 100
        self.node._state_cb(message)  # No RC channel stream required for servos.
        self.arm()
        self.assertTrue(self.node._armed)
        with patch.object(self.node._arm_command_pub, 'publish') as servo, \
                patch.object(self.node._permission_pub, 'publish') as permission:
            self.assertIsNone(self.command('arm_servo', action='move', point='center', held=True))
            self.assertTrue(servo.called)
            self.node._drive_watchdog()
            self.assertTrue(permission.call_args.args[0].data)
            self.assertIsNotNone(self.command('drive', linear_x=.4, deadman=True))
            message.system_status = 8
            message.header.stamp.sec = 101
            self.node._state_cb(message)
            self.assertFalse(self.node._armed)
            self.assertFalse(permission.call_args.args[0].data)
            self.assertIsNotNone(self.command('arm'))

    def test_camera_dimensions_survive_controller_merge(self):
        self.node._camera_frames['front'] = dict(topic='/front/camera_info',
            label='Front', last_frame=0.0, window_started=99.0, window_frames=0, fps=0.0)
        self.node._camera_frame_cb('front', CameraInfo(width=1280, height=720))
        camera = self.node.snapshot()['cameras']['front']
        self.assertEqual((camera['width'], camera['height']), (1280, 720))
        self.assertEqual(camera['frame_age_ms'], 0)

    def test_px4_safety_disarms_all_outputs_and_blocks_rearming(self):
        try:
            from test_rc_safety import healthy
        except ImportError:
            from .test_rc_safety import healthy
        self.node._rc_safety.simulation = False
        healthy(self.node._rc_safety)
        self.arm()
        self.command('drive', linear_x=.4, deadman=True)
        self.assertTrue(self.node._armed)
        with patch.object(self.node._probe_pub, 'publish') as probe, patch.object(self.node._sweep_enabled_pub, 'publish') as sweep:
            self.node._rc_safety.samples['state'][1].system_status = 8
            self.node._drive_watchdog()
            self.assertFalse(self.node._armed)
            self.output.assert_called_with(0.0, 0.0)
            self.assertFalse(sweep.call_args.args[0].data)
            for kind in ('arm', 'drive', 'probe_target', 'sweep_enabled', 'sweep_speed'):
                self.assertIsNotNone(self.command(kind))
            probe.assert_not_called()
            self.node._rc_safety.samples['state'][1].system_status = 4
            self.node._drive_watchdog()
            self.assertFalse(self.node._armed)

    def test_calibration_blocks_browser_and_ros_vehicle_input(self):
        self.command('take_control')
        self.command('arm', calibration=True)
        self.assertTrue(self.node._armed)
        self.assertIsNotNone(self.command('drive', linear_x=.4, deadman=True))
        self.assertIsNotNone(self.command('probe_target', depth_mm=20))
        self.node._joy_cb(Joy(axes=[0., 0., 1., 0., 0., -1.], buttons=[0] * 11))
        self.node._drive_watchdog()
        self.assertEqual(self.node._command, (0.0, 0.0))
        self.assertFalse(self.node.snapshot()['drive']['publishing'])
        self.command('disarm')
        self.assertFalse(self.node._calibrating)

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
