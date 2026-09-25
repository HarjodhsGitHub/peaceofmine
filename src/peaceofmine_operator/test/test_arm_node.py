"""Exercise the actual ROS driver with an in-memory serial bus (no hardware)."""
import importlib.util
from pathlib import Path
import unittest
from unittest.mock import patch
from concurrent.futures import Future
import json
import tempfile

import rclpy
from rclpy.parameter import Parameter
from std_msgs.msg import Bool, Float32, String
from peaceofmine_operator.rc_safety import RcSafety
from peaceofmine_operator.arm_servo import ArmServo, FirmwareMotionFault, ServoAlarm
try:
    from test_arm_servo import Bus
    from test_rc_safety import healthy, state, rc
except ImportError:
    from .test_arm_servo import Bus
    from .test_rc_safety import healthy, state, rc

spec = importlib.util.spec_from_file_location('arm_node', Path(__file__).parents[1] / 'scripts/arm_servo_node.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class ArmNodeTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.now = 100.0
        self.clock = patch.object(module.time, 'monotonic', side_effect=lambda: self.now)
        self.clock.start()
        self.calibration_dir = tempfile.TemporaryDirectory()
        self.addCleanup(self.calibration_dir.cleanup)
        self.node = module.ArmServoNode(parameter_overrides=[Parameter('calibration_file', value=str(Path(self.calibration_dir.name) / 'calibration.json'))])
        self.node.safety = RcSafety(clock=lambda: self.now)
        healthy(self.node.safety)
        self.bus = Bus()
        self.node.bus = self.bus
        self.node.discovered_servos = [1, 2]
        self.node.servo = ArmServo(self.bus, 1, dict(minimum=1500, center=2000, maximum=2500))
        self.node.role_ids['arm'] = 1
        self.node.permission_cb(Bool(data=True))

    def tearDown(self):
        self.node.destroy_node()
        self.clock.stop()

    def test_auto_discovery_enumerates_ftdi_and_finds_both_servos(self):
        device = Path('/dev/serial/by-id/usb-FTDI-test')
        def read(ident, address):
            if ident in (1, 2):
                return 310
            raise RuntimeError('No reply')
        with patch.object(Path, 'glob', return_value=[device]) as glob, \
                patch.object(module, 'ArbotiX') as adapter:
            adapter.return_value.read.side_effect = read
            bus, port, ids = self.node.discover('auto')
            glob.assert_called_once_with('usb-FTDI*')
            adapter.assert_called_once_with(str(device), 1000000)
            self.assertIs(bus, adapter.return_value)
            self.assertEqual(port, str(device))
            self.assertEqual(ids, [1, 2])

    def test_auto_discovery_reports_missing_adapter(self):
        with patch.object(Path, 'glob', return_value=[]):
            with self.assertRaisesRegex(RuntimeError, 'No FTDI arm adapter found'):
                self.node.discover('auto')

    def test_arm_limits_persist_with_probe_and_restart_stopped(self):
        from peaceofmine_operator import calibration
        calibration.save_section(self.node.calibration_file, 'probe',
                                 dict(servo_id=2, max_extension_mm=120, travel_ticks=5430))
        self.node.command_cb(String(data=json.dumps(dict(action='configure',
                             minimum=1400, center=1900, maximum=2400))))
        saved = calibration.load(self.node.calibration_file)
        self.assertEqual(saved['arm']['center'], 1900)
        self.assertEqual(saved['probe']['travel_ticks'], 5430)
        restarted = module.ArmServoNode(parameter_overrides=[Parameter('calibration_file', value=self.node.calibration_file)])
        try:
            restarted.bus = Bus()
            restarted.discovered_servos = [1, 2]
            restarted.select(1)
            self.assertEqual(restarted.servo.calibration['center'], 1900)
            self.assertFalse(restarted.servo.torque)
            self.assertIsNone(restarted.servo.target)
        finally:
            restarted.destroy_node()

    def test_probe_maximum_persists_but_home_does_not(self):
        import tempfile
        from peaceofmine_operator import probe_calibration
        with tempfile.TemporaryDirectory() as directory:
            self.node.calibration_file = str(Path(directory) / 'probe.json')
            self.node.active_role = 'probe'
            self.node.role_ids['probe'] = 1
            self.node.servo.home_position = 3000
            self.node.command_cb(String(data=json.dumps(dict(action='save_probe_extension', role='probe', max_extension_mm=100))))
            self.assertEqual(self.node.probe_extension['travel_ticks'], 1000)
            saved = probe_calibration.load(self.node.calibration_file)
            self.assertEqual(saved['max_extension_mm'], 100)
            restarted = module.ArmServoNode(parameter_overrides=[Parameter('calibration_file', value=self.node.calibration_file)])
            try:
                self.assertEqual(restarted.probe_extension, saved)
                self.assertFalse(restarted.probe_state()['homed'])
                self.assertFalse(restarted.probe_state()['ready'])
            finally:
                restarted.destroy_node()

    def test_zero_contact_requires_owner_and_fresh_holding_samples(self):
        from test_teleop import gateway_module
        gateway = gateway_module.OperatorGateway()
        owner = object()
        try:
            gateway._rc_safety = self.node.safety
            for index in range(12):
                gateway._arm_state_cb(String(data=json.dumps(dict(connected=True, active_role='probe',
                    servo_id=2, sample_time=index, load_percent=-5, current_a=.1,
                    position=2000, goal_position=2000, torque=True, probe=dict(holding=True)))))
                self.now += .05
            healthy(self.node.safety, stamp=2000000)
            self.assertIsNotNone(gateway.handle_command(object(), dict(type='probe_zero_contact')))
            self.assertIsNone(gateway._probe_contact.baseline)
            gateway.handle_command(owner, dict(type='take_control'))
            self.assertIsNone(gateway.handle_command(owner, dict(type='probe_zero_contact')))
            self.assertAlmostEqual(gateway._probe_contact.baseline[0], -5)
            self.now += .4
            healthy(self.node.safety, stamp=2000001)
            self.assertIsNotNone(gateway.handle_command(owner, dict(type='probe_zero_contact')))
        finally:
            gateway.destroy_node()

    def test_probe_fast_poll_keeps_slow_diagnostics_at_one_hz(self):
        self.node.active_role = 'probe'
        self.node.last_poll = self.now
        with patch.object(self.node.servo, 'snapshot', wraps=self.node.servo.snapshot) as snapshot:
            for i in range(20):
                self.node.tick()
                self.now += .05
            self.assertEqual(snapshot.call_count, 20)
            self.assertTrue(all(call.kwargs['fast'] for call in snapshot.call_args_list))
            self.now += .01
            self.node.tick()
            self.assertFalse(snapshot.call_args.kwargs['fast'])

    def test_main_probe_load_uses_magnitude_and_expires(self):
        from test_teleop import gateway_module
        gateway = gateway_module.OperatorGateway()
        try:
            gateway._rc_safety = self.node.safety
            state = dict(connected=True, active_role='probe', servo_id=2,
                         sample_time=1, load_percent=-45)
            gateway._arm_state_cb(String(data=json.dumps(state)))
            self.assertEqual(gateway.snapshot()['probe']['pressure_ratio'], .45)
            gateway._arm_state_cb(String(data=json.dumps(state)))
            self.assertEqual(len(gateway._probe_load_history), 1)
            gateway._probe_pressure_cb(Float32(data=0))
            self.assertEqual(gateway.snapshot()['probe']['pressure_ratio'], .45)
            self.now += .7
            gateway._arm_state_cb(String(data=json.dumps(state)))
            self.assertIsNone(gateway.snapshot()['probe']['pressure_ratio'])
            state.update(sample_time=2, load_percent=32)
            gateway._arm_state_cb(String(data=json.dumps(state)))
            self.assertEqual(gateway.snapshot()['probe']['pressure_ratio'], .32)
            state['active_role'] = 'arm'
            gateway._arm_state_cb(String(data=json.dumps(state)))
            self.assertIsNone(gateway.snapshot()['probe']['pressure_ratio'])
        finally:
            gateway.destroy_node()

    def test_main_gateway_probe_target_holds_and_safety_still_stops(self):
        from test_teleop import gateway_module
        gateway = gateway_module.OperatorGateway()
        owner = object()
        try:
            gateway._rc_safety = self.node.safety
            self.node.active_role = 'probe'
            self.node.role_ids['probe'] = 1
            self.node.servo.home_position = 3000
            self.node.probe_extension = dict(servo_id=1, max_extension_mm=100., travel_ticks=1000)
            self.bus.values[36] = self.node.servo.position = 3000
            gateway._arm_state = dict(connected=True, active_role='probe', probe=self.node.probe_state())
            gateway._arm_state_at = self.now
            with patch.object(gateway._arm_command_pub, 'publish', side_effect=self.node.command_cb), patch.object(gateway._permission_pub, 'publish', side_effect=self.node.permission_cb):
                self.assertIsNone(gateway.handle_command(owner, dict(type='take_control')))
                self.assertIsNone(gateway.handle_command(owner, dict(type='probe_target', depth_mm=50)))
                for index in range(12):
                    healthy(self.node.safety, stamp=1000001 + index)
                    gateway._drive_watchdog()
                    self.node.tick()
                    self.bus.values[36] = self.bus.values[30]
                    gateway._arm_state['probe'] = self.node.probe_state()
                    gateway._arm_state_at = self.now
                    self.now += .05
                self.assertEqual(self.node.servo.position, 2500)
                self.assertEqual(self.bus.values[24], 1)
                self.assertTrue(self.node.probe_state()['holding'])
                self.assertFalse(self.node.probe_state()['moving'])
                self.assertIsNotNone(gateway._probe_motion)
                gateway._probe_motion['started'] = self.now - 61
                gateway._drive_watchdog()
                self.node.tick()
                self.assertEqual(self.bus.values[24], 0)
                self.assertIsNone(gateway._probe_motion)
                healthy(self.node.safety, stamp=2000000)
                self.assertIsNotNone(gateway.handle_command(owner, dict(type='probe_target', depth_mm=101)))
                self.assertIsNone(gateway.handle_command(owner, dict(type='probe_target', depth_mm=80)))
                self.node.tick()
                gateway._speed = .2
                gateway._drive_watchdog()
                self.node.tick()
                self.assertEqual(self.bus.values[24], 0)
                gateway._speed = 0
                self.assertIsNone(gateway.handle_command(owner, dict(type='probe_target', depth_mm=80)))
                self.node.tick()
                gateway._lease = None
                gateway._drive_watchdog()
                self.node.tick()
                self.assertEqual(self.bus.values[24], 0)
        finally:
            gateway.destroy_node()

    def test_probe_home_heartbeat_keeps_slow_speed_and_permission_loss_stops(self):
        self.node.active_role = 'probe'
        command = String(data=json.dumps(dict(action='home', role='probe', held=True,
                                             hold_id='test-home', load_percent=30)))
        self.node.command_cb(command)
        self.node.tick()
        self.assertEqual(self.bus.values[32], 73)
        self.now += .08
        self.node.command_cb(command)
        self.node.tick()
        self.assertEqual(self.bus.values[32], 73)
        self.assertEqual(self.node.servo.max_step, 114)
        self.now += .4
        self.node.tick()
        self.assertEqual(self.bus.values[24], 0)
        self.assertIsNone(self.node.servo.home_position)
        self.assertIsNone(self.node.servo._home)

    def test_home_rejects_arm_role_and_missing_hold(self):
        for role, held in [('arm', True), ('probe', False)]:
            self.node.active_role = role
            self.node.command_cb(String(data=json.dumps(dict(action='home', role=role,
                held=held, hold_id='test-home', load_percent=30))))
            self.assertIsNone(self.node.servo.target)
            self.assertEqual(self.bus.values[24], 0)

    def test_enable_multiturn_clears_probe_reference_without_moving(self):
        self.bus.gateway[80] = 2
        self.node.active_role = 'probe'
        self.node.servo.home_position = 2000
        self.node.command_cb(String(data=json.dumps(dict(action='enable_multiturn', role='probe'))))
        self.assertTrue(self.node.servo.multiturn)
        self.assertIsNone(self.node.servo.home_position)
        self.assertIsNone(self.node.servo.calibration)
        self.assertEqual(self.bus.values[24], 0)
        self.assertNotIn((24, 1), self.bus.writes)

    def test_motion_fault_retains_connection_without_restarting(self):
        servo = self.node.servo
        self.node.sweeping = True
        servo.request_sweep('maximum', 10, 4)
        self.node.fail(FirmwareMotionFault(10, 'Endpoint did not settle'))
        self.assertIs(self.node.bus, self.bus)
        self.assertIs(self.node.servo, servo)
        self.assertEqual(self.node.discovered_servos, [1, 2])
        self.assertFalse(self.node.sweeping)
        self.assertIsNone(servo.target)
        self.assertEqual(self.bus.values[24], 0)
        self.node.tick()
        self.assertFalse(self.node.sweeping)
        self.assertEqual(self.bus.values[24], 0)

    def test_servo_alarm_retains_connection_with_persistent_alarm_on_stop(self):
        servo = self.node.servo
        self.node.sweeping = True
        servo.request_sweep('maximum', 10, 4)
        alarm = ServoAlarm(1, 36, 32, [0, 8], 'Overload')
        original_write = self.bus.write
        original_read = self.bus.read
        def write(ident, address, value, size=2):
            original_write(ident, address, value, size)
            raise ServoAlarm(ident, address, 32, [], 'Overload')
        def read(ident, address, size=2):
            value = original_read(ident, address, size)
            raise ServoAlarm(ident, address, 32, value.to_bytes(size, 'little'), 'Overload')
        with patch.object(self.bus, 'write', side_effect=write), patch.object(self.bus, 'read', side_effect=read):
            self.node.fail(alarm)
            self.node.tick()
        self.assertIs(self.node.bus, self.bus)
        self.assertEqual(self.node.discovered_servos, [1, 2])
        self.assertIsNone(servo.target)
        self.assertFalse(self.node.sweeping)
        self.assertFalse(servo.torque)
        self.assertEqual(self.bus.values[24], 0)
        self.assertIn('connection retained', self.node.reason)

    def test_servo_alarm_with_unconfirmed_stop_disconnects(self):
        with patch.object(self.bus, 'read', side_effect=ServoAlarm(1, 24, 32, [1], 'Overload')):
            self.node.fail(ServoAlarm(1, 36, 32, [0, 8], 'Overload'))
        self.assertIsNone(self.node.bus)
        self.assertIn('stop verification failed', self.node.reason)

    def test_failed_fault_stop_verification_disconnects(self):
        original_read = self.bus.read
        with patch.object(self.bus, 'read', side_effect=lambda ident, address, size=2:
                          1 if address == 24 else original_read(ident, address, size)):
            self.node.fail(FirmwareMotionFault(10, 'Endpoint did not settle'))
        self.assertIsNone(self.node.bus)
        self.assertIn('stop verification failed', self.node.reason)

    def test_position_publishes_each_tick_between_diagnostic_polls(self):
        self.node.last_poll = self.now
        with patch.object(self.node.angle_pub, 'publish') as publish:
            for index in range(3):
                self.now += .05
                self.bus.values[36] = 2000 + index * 10
                self.node.tick()
                self.assertEqual(self.node.state['position'], 2000 + index * 10)
            self.assertEqual(publish.call_count, 3)

    def test_launch_speed_100_is_supported_without_losing_discovered_servo(self):
        node = module.ArmServoNode(parameter_overrides=[Parameter('calibration_file', value=self.node.calibration_file), Parameter('speed', value=100),
                                                       Parameter('servo_id', value=1)])
        try:
            node.discovery = Future()
            node.discovery.set_result((Bus(), '/dev/fake', [1]))
            node.poll_discovery()
            self.assertEqual(node.motion_speed, 100)
            self.assertEqual(node.servo.ident, 1)
            self.assertEqual(node.discovered_servos, [1])
            self.assertIsNotNone(node.bus)
        finally:
            node.destroy_node()

    def test_bad_calibration_does_not_clear_servo_discovery(self):
        node = module.ArmServoNode(parameter_overrides=[Parameter('calibration_file', value=self.node.calibration_file), Parameter('servo_id', value=1),
                                                       Parameter('minimum', value=3000),
                                                       Parameter('center', value=2000),
                                                       Parameter('maximum', value=1000)])
        try:
            node.discovery = Future()
            node.discovery.set_result((Bus(), '/dev/fake', [1]))
            node.poll_discovery()
            self.assertEqual(node.discovered_servos, [1])
            self.assertIsNotNone(node.bus)
            self.assertIsNone(node.servo)
        finally:
            node.destroy_node()

    def test_unplugged_probe_can_be_viewed_without_losing_arm(self):
        arm = self.node.servo
        self.node.discovered_servos = [1]
        self.node.select_role('probe')
        self.assertIsNone(self.node.servo)
        self.assertIn('not connected', self.node.reason)
        self.assertEqual(self.node.discovered_servos, [1])
        self.assertIsNotNone(self.node.bus)
        self.node.select_role('arm')
        self.assertIs(self.node.servo, arm)
        self.assertEqual(self.node.servo.calibration['minimum'], 1500)

    def test_role_calibrations_are_independent_and_switch_requires_stop(self):
        arm = self.node.servo
        self.node.select_role('probe')
        self.assertEqual(self.node.servo.ident, 2)
        self.assertIsNone(self.node.servo.calibration)
        self.node.servo.configure(dict(minimum=1000, center=1800, maximum=3000))
        probe = self.node.servo
        self.node.select_role('arm')
        self.assertIs(self.node.servo, arm)
        self.move()
        with self.assertRaisesRegex(ValueError, 'Stop'):
            self.node.select_role('probe')
        self.node.stop()
        self.node.select_role('probe')
        self.assertIs(self.node.servo, probe)
        self.assertEqual(probe.calibration['minimum'], 1000)

    def test_panel_selection_and_stale_role_command(self):
        arm = self.node.servo
        self.node.command_cb(String(data=json.dumps(dict(action='select', role='probe', servo_id=2))))
        self.assertEqual(self.node.active_role, 'probe')
        self.assertEqual(self.node.servo.ident, 2)
        self.node.command_cb(String(data=json.dumps(dict(action='jog', role='arm', held=True, direction=1))))
        self.assertIn('Select this actuator', self.node.reason)
        self.assertIsNone(self.node.servo.target)
        self.node.command_cb(String(data=json.dumps(dict(action='select', role='arm', servo_id=1))))
        self.assertIs(self.node.servo, arm)
        self.assertEqual(arm.calibration['minimum'], 1500)

    def test_sweep_selects_arm_not_probe(self):
        self.node.select_role('probe')
        self.node.sweep_cb(Bool(data=True))
        self.node.tick()
        self.assertEqual(self.node.active_role, 'arm')
        self.assertEqual(self.node.servo.ident, 1)
        self.assertTrue(self.node.sweeping)

    def test_reconnect_requires_stop_and_never_replays_motion(self):
        self.move()
        with self.assertRaisesRegex(ValueError, 'Stop'):
            self.node.reconnect()
        self.node.fail(RuntimeError('Telemetry failure'))
        self.node.reconnect()
        self.assertFalse(self.node.sweeping)
        self.assertFalse(self.node.permission)
        self.assertEqual(self.node.command_at, 0)
        self.node.discovery_started = True
        self.node.discovery = Future()
        self.node.discovery.set_result((Bus(), '/dev/fake', [1]))
        self.node.poll_discovery()
        self.node.select(1)
        self.assertEqual(self.node.servo.calibration['minimum'], 1500)
        self.assertIsNone(self.node.servo.target)
        self.assertFalse(self.node.servo.torque)

    def move(self):
        self.node.command_cb(String(data=json.dumps(dict(action='move', held=True, point='maximum'))))
        self.node.tick()
        self.assertTrue(self.node.servo.torque)

    def test_scan_is_read_only_and_finds_both_servos_at_fixed_baud(self):
        def read(ident, address, size=2):
            if ident in (1, 2):
                return 310
            raise RuntimeError('No reply')
        with patch.object(module, 'ArbotiX', return_value=self.bus) as factory, \
                patch.object(self.bus, 'read', side_effect=read):
            before = list(self.bus.writes)
            bus, device, ids = self.node.discover('/dev/fake')
        self.assertIs(bus, self.bus)
        self.assertEqual(ids, [1, 2])
        self.assertEqual(self.bus.writes, before)
        factory.assert_called_once_with('/dev/fake', 1000000)

    def test_pending_scan_does_not_block_tick_or_select_a_servo(self):
        self.node.bus = self.node.servo = None
        self.node.role_ids['arm'] = -1
        self.node.discovery = Future()
        self.node.tick()
        self.assertIsNone(self.node.servo)
        self.assertFalse(self.node.discovery.done())
        self.node.discovery.set_result((self.bus, '/dev/fake', [1, 2]))
        self.node.tick()
        self.assertEqual(self.node.discovered_servos, [1, 2])
        self.assertIsNone(self.node.servo)
        self.assertNotIn((24, 1), self.bus.writes)

    def test_scan_cancellation_closes_port_without_motion(self):
        with patch.object(module, 'ArbotiX', return_value=self.bus), \
                patch.object(self.bus, 'read', side_effect=lambda *args: self.node.discovery_stop.set() or 310), \
                patch.object(self.bus, 'close') as close:
            self.assertIsNone(self.node.discover('/dev/fake'))
            close.assert_called_once()
        self.assertNotIn((24, 1), self.bus.writes)

    def test_failed_startup_scan_retries_after_backoff(self):
        self.node.bus = self.node.servo = None
        future = Future()
        future.set_exception(RuntimeError('No adapter'))
        with patch.object(self.node.discovery_pool, 'submit', return_value=future) as submit:
            self.node.tick()  # Schedule once without blocking the ROS callback.
            self.node.tick()  # Report failure.
            self.now += 1
            self.node.tick()
            submit.assert_called_once_with(self.node.discover, 'auto')
            self.now += 1
            self.node.tick()
            self.assertEqual(submit.call_count, 2)
            self.node.tick()  # Consume the failed retry before shutdown.

    def test_stop_does_not_hide_discovery_failure(self):
        self.node.fail(RuntimeError('Adapter did not answer'))
        self.node.command_cb(String(data=json.dumps(dict(action='stop'))))
        self.assertEqual(self.node.reason, 'Adapter did not answer')
        self.assertEqual(self.node.connection_error, 'Adapter did not answer')

    def test_disconnect_recovers_calibration_without_resuming(self):
        self.node.fail(RuntimeError('Disconnected'))
        recovered = Bus()
        future = Future()
        future.set_result((recovered, '/dev/fake', [1, 2]))
        with patch.object(self.node.discovery_pool, 'submit', return_value=future) as submit:
            self.now += 2
            self.node.tick()
            submit.assert_called_once()
            self.node.tick()
        self.assertIs(self.node.bus, recovered)
        self.assertEqual(self.node.servo.calibration['center'], 2000)
        self.assertFalse(self.node.sweeping)
        self.assertIsNone(self.node.servo.target)
        self.assertEqual(recovered.values[24], 0)

    def test_jog_without_limits_and_command_feedback(self):
        self.node.servo.calibration = None
        self.node.command_cb(String(data=json.dumps(dict(action='jog', direction=1, held=True, request_id='jog1'))))
        self.node.tick()
        self.assertTrue(self.node.servo.torque)
        self.assertTrue(self.node.last_command['success'])
        self.now += .26
        self.node.tick()
        self.assertFalse(self.node.servo.torque)

    def test_motion_settings_and_unclipped_sweep_speed(self):
        self.node.command_cb(String(data=json.dumps(dict(action='configure_motion',
            max_speed_deg_s=200, acceleration_deg_s2=100))))
        self.assertEqual(self.node.motion_speed, 292)
        self.assertEqual(self.node.sweep_acceleration, 11)
        self.node.sweep_speed_cb(Float32(data=150))
        self.assertEqual(self.node.sweep_speed, 150)
        self.node.sweep_speed_cb(Float32(data=600))
        self.assertAlmostEqual(self.node.sweep_speed, 292 * .684)
        self.node.servo.torque = True
        self.node.command_cb(String(data=json.dumps(dict(action='configure_motion',
            max_speed_deg_s=50, acceleration_deg_s2=100))))
        self.assertEqual(self.node.motion_speed, 292)

    def test_slider_command_full_speed_and_timeout(self):
        self.node.command_cb(String(data=json.dumps(dict(action='position', position=2030,
            held=True))))
        self.node.tick()
        self.assertEqual(self.bus.values[30], 2030)
        self.assertEqual(self.bus.values[32], 0)
        self.now += .26
        self.node.tick()
        self.assertFalse(self.node.servo.torque)
        self.node.command_cb(String(data=json.dumps(dict(action='capture', point='center', request_id='record1'))))
        self.assertTrue(self.node.last_command['success'])
        self.assertIn('Recorded center', self.node.last_command['message'])
        self.now += .1
        self.node.command_cb(String(data=json.dumps(dict(action='jog', direction=1, held=True, request_id='stale'))))
        self.assertFalse(self.node.last_command['success'])
        self.assertFalse(self.node.servo.torque)

    def test_px4_disarm_immediately_drops_torque(self):
        self.move()
        self.node.state_cb(state(2000000, armed=False, system_status=3))
        self.assertFalse(self.node.servo.torque)
        self.assertEqual(self.bus.writes[-1], (24, 0))
        healthy(self.node.safety, 3000000)
        self.node.tick()
        self.assertFalse(self.node.servo.torque)

    def test_command_timeout_and_lease_loss(self):
        self.move()
        self.now += .26
        self.node.tick()
        self.assertFalse(self.node.servo.torque)

        self.node.permission_cb(Bool(data=True))
        self.move()
        self.node.permission_cb(Bool(data=False))
        self.assertFalse(self.node.servo.torque)

    def test_destroy_disables_torque_before_serial_close(self):
        self.move()
        with patch.object(self.bus, 'close', side_effect=lambda: self.assertEqual(self.bus.writes[-1], (24, 0))):
            self.node.destroy_node()
        self.assertFalse(self.node.servo.torque)
        self.assertIsNone(self.node.servo.target)

    def test_sweep_ignores_rc_override_but_kill_clears_target(self):
        self.node.sweep_cb(Bool(data=True))
        self.node.tick()
        self.assertTrue(self.node.servo.torque)
        self.node.safety.update_rc(rc(2000000, pwm=1500))
        self.node.tick()
        self.assertTrue(self.node.sweeping)
        self.assertTrue(self.node.servo.torque)
        self.node.state_cb(state(2000000, system_status=8))
        self.assertFalse(self.node.sweeping)
        self.assertFalse(self.node.servo.torque)
        healthy(self.node.safety, 3000000)
        self.node.tick()
        self.assertFalse(self.node.servo.torque)

    def test_sweep_commands_endpoint_with_speed_limit_and_stops_on_rc_kill(self):
        self.node.rc_cb(rc(2000000, pwm=1500))
        self.node.sweep_cb(Bool(data=True))
        self.node.tick()
        self.assertTrue(self.node.sweeping)
        self.assertEqual(self.bus.values[30], 2500)
        self.assertGreater(self.bus.values[32], 0)
        self.assertLessEqual(self.bus.values[32], self.node.get_parameter('speed').value)
        self.bus.values[36] = 2500
        self.node.servo.position = 2500
        self.now += .05
        self.node.tick()
        self.assertEqual(self.bus.gateway_writes.count((94, 1)), 1)
        self.node.rc_cb(rc(3000000, pwm=2000))
        self.assertFalse(self.node.sweeping)
        self.assertFalse(self.node.servo.torque)

    def test_sweep_rejection_explains_missing_limits(self):
        self.node.servo.calibration = None
        self.node.sweep_cb(Bool(data=True))
        self.assertFalse(self.node.sweeping)
        self.assertIn('minimum, center and maximum', self.node.reason)
        healthy(self.node.safety, 3000000)
        self.node.tick()
        self.assertFalse(self.node.servo.torque)

    def test_sweep_delegates_endpoint_reversals_to_firmware(self):
        self.node.sweep_cb(Bool(data=True))
        for position in (2480, 1520):
            self.bus.values[36] = self.node.servo.position = position
            self.node.tick()
            self.assertTrue(self.node.sweeping)
        self.assertEqual(self.bus.gateway_writes.count((94, 1)), 1)
        self.assertEqual(self.bus.gateway[91], self.node.sweep_endpoint_tolerance)

    def test_sweep_does_not_reverse_early_or_chatter_in_a_narrow_range(self):
        self.node.sweep_cb(Bool(data=True))
        self.bus.values[36] = self.node.servo.position = 2460
        self.node.tick()
        self.assertEqual(self.bus.values[30], 2500)
        self.node.servo.calibration = dict(minimum=1980, center=2000, maximum=2020)
        self.bus.values[36] = self.node.servo.position = 2000
        self.node.tick()
        self.assertEqual(self.bus.values[30], 2020)
        self.bus.values[36] = self.node.servo.position = 2015
        self.node.tick()
        self.assertEqual(self.bus.gateway[91], 10)
        self.node.tick()
        self.assertEqual(self.bus.gateway_writes.count((94, 1)), 2)
