import unittest
from peaceofmine_operator.arm_servo import ArbotiX, ArmServo, ServoAlarm, limits
from unittest.mock import Mock, patch
from types import SimpleNamespace


class Bus(ArbotiX):
    def __init__(self):
        self._motion_id = self._sweep_profile = None
        self.gateway = {80: 1, 96: 0, 98: 0}
        self.gateway_writes = []
        self.values = {2: 40, 14: 1023, 30: 2000, 32: 3, 34: 1023, 38: 0, 40: 0, 46: 0, 68: 2048, 0: 310, 70: 0, 6: 0, 8: 4095, 24: 0, 36: 2000, 42: 120, 43: 25}
        self.writes = []
        self.values[73] = 0
    def close(self):
        pass
    def read(self, ident, address, size=2):
        if ident == 253:
            return self.gateway[address]
        return self.values[address]
    def read_block(self, ident, address, length):
        data = bytearray(length)
        for register, value in self.values.items():
            size = 1 if register in (24, 42, 43, 46) else 2
            if address <= register and register + size <= address + length:
                data[register-address:register-address+size] = value.to_bytes(size, 'little')
        return bytes(data)
    def write(self, ident, address, value, size=2):
        if ident == 253:
            self.gateway_writes.append((address, value))
            if address == 82 and value == 2 and self.gateway[96] not in (1, 2):
                raise RuntimeError('Firmware watchdog expired')
            self.gateway[address] = value
            if address == 82 and value == 1:
                self.gateway[96] = 1
            if address == 83 and value == 1:
                self.values.update({6: 4095, 8: 4095, 22: 1})
            if address == 94 and value == 1:
                self.gateway[96] = 2
                low, high, tolerance = (self.gateway[k] for k in (84, 86, 91))
                self.values.update({24: 1, 30: low if self.values[36] >= high - tolerance else high,
                                    32: 1, 73: self.gateway[90]})
            if address == 90 and self.gateway[96] == 2:
                self.values[73] = value
            return
        self.writes.append((address, value))
        self.values[address] = value & (255 if size == 1 else 65535)
        if address == 24 and value == 0:
            self._motion_id = self._sweep_profile = None
            self.gateway[96] = 0


class ArmServoTest(unittest.TestCase):
    def setUp(self):
        self.bus = Bus()
        self.servo = ArmServo(self.bus, 1)

    def test_home_stops_on_first_spike_even_when_encoder_keeps_moving(self):
        for direction_bit in (0, 1024):
            with self.subTest(direction_bit=direction_bit):
                servo = ArmServo(self.bus, 1)
                self.bus.values[40] = direction_bit | 100
                servo.request_home('hold-1', 20)
                servo.step(lambda: True)
                self.assertEqual(self.bus.values[32], 73)
                self.assertIsNone(servo.home_position)
                self.bus.values[36] += 25  # Teeth skipping must not suppress contact.
                self.bus.values[40] = direction_bit | 450
                self.bus.writes.clear()
                servo.step(lambda: True)
                self.assertEqual(servo.home_position, self.bus.values[36])
                self.assertEqual(self.bus.writes, [(24, 0)])
                servo.request_home('hold-1', 20)
                servo.step(lambda: True)
                self.assertIsNone(servo.target)
                self.assertEqual(self.bus.values[24], 0)

    def test_telemetry_only_spike_also_stops_homing(self):
        self.servo.request_home('hold-1', 20)
        self.servo.step(lambda: True)
        self.bus.values[36] += 30
        self.bus.values[40] = 1024 | 450
        snapshot = self.servo.snapshot()
        self.assertFalse(snapshot['torque'])
        self.assertFalse(snapshot['homing'])
        self.assertLess(snapshot['load_percent'], -20)
        self.assertEqual(snapshot['home_position'], self.bus.values[36])
        self.assertEqual(self.bus.values[24], 0)

    def test_home_below_threshold_still_requires_permission(self):
        self.servo.request_home('hold-1', 20)
        self.bus.values[40] = 1024 | 100
        self.servo.step(lambda: True)
        self.assertIsNone(self.servo.home_position)
        self.servo.step(lambda: False)
        self.assertEqual(self.bus.values[24], 0)
        self.assertIsNone(self.servo._home)

    @patch('peaceofmine_operator.arm_servo.time.monotonic')
    def test_home_timeout_and_joint_limit_never_set_zero(self, clock):
        clock.return_value = 10
        self.servo.request_home('hold-1', 30)
        self.servo.step(lambda: True)
        clock.return_value = 70
        self.servo.request_home('hold-1', 30)
        self.servo.step(lambda: True)
        self.assertIn('timeout', self.servo.home_status)
        self.assertIsNone(self.servo.home_position)
        self.assertEqual(self.bus.values[24], 0)
        self.servo.request_home('hold-2', 30)
        self.bus.values[36] = 4095
        self.servo.step(lambda: True)
        self.assertIn('position range limit', self.servo.home_status)
        self.assertIsNone(self.servo.home_position)
        self.assertEqual(self.bus.values[24], 0)

    def test_home_threshold_validation_and_reconnect_clear_zero(self):
        for threshold in (None, True, float('nan'), 0, 61):
            with self.assertRaises(ValueError):
                self.servo.request_home('hold', threshold)
        self.servo.home_position = 2000
        self.assertIsNone(ArmServo(self.bus, 1).home_position)

    def test_multiturn_signed_positions_and_home_cross_one_revolution(self):
        self.bus.gateway[80] = 2
        self.bus.values.update({6: 4095, 8: 4095, 22: 1, 36: 65535})
        servo = ArmServo(self.bus, 2)
        self.assertEqual(servo.position, -1)
        self.assertEqual(servo.movement_limits(), (-28672, 28672))
        servo.configure(dict(minimum=-10000, center=0, maximum=12000))
        servo.request_position(-5000)
        servo.step(lambda: True)
        self.assertLess(servo.goal, 0)
        self.assertEqual(servo.snapshot()['goal_position'], servo.goal)
        servo.stop()
        self.bus.values[36] = 6000
        servo.request_home('multi-home', 30)
        servo.step(lambda: True)
        self.assertEqual(servo.target, 28672)
        self.assertEqual(self.bus.values[32], 73)
        self.assertEqual(servo.goal, 6114)
        servo.stop()
        self.assertEqual(servo.capture('maximum'), 6000)
        with self.assertRaises(ValueError):
            servo.request_position(28673)
        with self.assertRaises(ValueError):
            servo.request_sweep('maximum', 10, 4)

    def test_multiturn_rejects_old_gateway_and_wrong_resolution(self):
        self.bus.values.update({6: 4095, 8: 4095, 22: 1})
        with self.assertRaisesRegex(ValueError, 'firmware v2'):
            ArmServo(self.bus, 2)
        self.bus.gateway[80] = 2
        self.bus.values[22] = 2
        with self.assertRaisesRegex(ValueError, 'divider 1'):
            ArmServo(self.bus, 2)

    def test_multiturn_setup_keeps_torque_off_and_checks_firmware(self):
        with self.assertRaisesRegex(ValueError, 'firmware v2'):
            self.bus.enable_multiturn(2)
        self.bus.gateway[80] = 2
        self.bus.enable_multiturn(2)
        self.assertEqual(self.bus.values[24], 0)
        self.assertEqual(self.bus.values[6], 4095)
        self.assertEqual(self.bus.values[8], 4095)
        self.assertEqual(self.bus.values[22], 1)

    def test_transport_encodes_negative_position_as_two_complement(self):
        bus = ArbotiX.__new__(ArbotiX)
        bus.port = object()
        write = Mock(return_value=(0, 0))
        bus.packet = SimpleNamespace(write2ByteTxRx=write)
        bus.write(2, 30, -5000)
        write.assert_called_once_with(bus.port, 2, 30, 60536)

    def test_setup_timeout_restored_when_sdk_raises(self):
        bus = ArbotiX.__new__(ArbotiX)
        normal_timeout = Mock()
        bus.port = SimpleNamespace(setPacketTimeout=normal_timeout, setPacketTimeoutMillis=Mock())
        def failed_write(port, *args):
            port.setPacketTimeout(6)
            raise OSError('disconnected')
        bus.packet = SimpleNamespace(write1ByteTxRx=failed_write)
        with self.assertRaises(OSError):
            bus.write(253, 83, 1, 1)
        bus.port.setPacketTimeoutMillis.assert_called_once_with(250)
        self.assertIs(bus.port.setPacketTimeout, normal_timeout)

    def test_real_sdk_accepts_delayed_eeprom_ack_without_retry(self):
        try:
            from dynamixel_sdk import PortHandler, PacketHandler
        except ImportError:
            self.skipTest('Dynamixel SDK required')
        import os
        import pty
        import select
        import threading
        import time
        master, slave = pty.openpty()
        bus = ArbotiX.__new__(ArbotiX)
        bus.port = PortHandler(os.ttyname(slave))
        bus.packet = PacketHandler(1.0)
        packets = []
        def controller():
            if select.select([master], [], [], 1)[0]:
                packet = os.read(master, 64)
                packets.append(packet)
                time.sleep(.065)  # Real firmware takes >60 ms to reply.
                os.write(master, bytes([255, 255, 253, 2, 0, 0]))
        worker = threading.Thread(target=controller)
        try:
            self.assertTrue(bus.port.setBaudRate(115200))
            worker.start()
            bus.write(253, 83, 1, 1)
            worker.join(1)
            self.assertEqual(len(packets), 1)
            self.assertEqual(packets[0][2:7], bytes([253, 4, 3, 83, 1]))
            bus.port.setPacketTimeout(6)
            self.assertLess(bus.port.packet_timeout, 100)
        finally:
            if worker.ident is not None:
                worker.join(1)
            bus.port.closePort()
            os.close(master)
            os.close(slave)

    def test_unknown_limits_never_enable_torque(self):
        with self.assertRaises(ValueError):
            self.servo.request('center')
        self.servo.step(lambda: True)
        self.assertNotIn((24, 1), self.bus.writes)

    def test_extension_requires_home_and_enforces_saved_relative_travel(self):
        calibration = dict(servo_id=1, max_extension_mm=100., travel_ticks=1000)
        with self.assertRaisesRegex(ValueError, 'Home'):
            self.servo.request_extension('a', 50, calibration)
        self.servo.home_position = 3000
        for depth in (-1, 101, float('nan')):
            with self.assertRaises(ValueError):
                self.servo.request_extension('a', depth, calibration)
        self.servo.request_extension('a', 50, calibration)
        self.assertEqual(self.servo.target, 2500)
        self.assertEqual(self.servo.jog_limits, (2000, 3000))
        self.bus.values[36] = 2500
        self.servo.step(lambda: True)
        self.servo.request_extension('a', 50, calibration)
        self.servo.step(lambda: True)
        self.assertEqual(self.servo.target, 2500)
        self.assertEqual(self.bus.values[24], 1)
        self.servo.step(lambda: False)
        self.servo.request_extension('a', 50, calibration)
        self.servo.step(lambda: True)
        self.assertIsNone(self.servo.target)
        self.assertEqual(self.bus.values[24], 0)

    def test_native_sweep_writes_profile_once_and_keeps_checking_safety(self):
        self.servo.configure(dict(minimum=1500, center=2000, maximum=2500))
        self.servo.request_sweep('maximum', speed=10, acceleration=4)
        self.servo.step(lambda: True)
        self.assertEqual(self.bus.values[73], 4)
        self.assertEqual(self.bus.values[32], 1)
        self.assertEqual(self.bus.gateway[88], 10)
        self.assertEqual(self.bus.values[30], 2500)
        self.bus.writes.clear()
        for _ in range(10):
            self.servo.request_sweep('maximum', speed=10, acceleration=4)
            self.servo.step(lambda: True)
        self.assertEqual(self.bus.writes, [])
        self.servo.request_sweep('minimum', speed=10, acceleration=4)
        self.servo.step(lambda: True)
        self.assertEqual(self.bus.writes, [])  # Endpoints belong to firmware now.
        self.assertEqual(self.bus.gateway_writes.count((94, 1)), 1)
        self.assertEqual(self.bus.gateway_writes.count((82, 2)), 11)
        self.servo.step(lambda: False)
        self.assertEqual(self.bus.writes[-1], (24, 0))
        self.assertFalse(self.servo.torque)

    def test_sweep_acceleration_does_not_change_settings_slider_profile(self):
        self.servo.configure(dict(minimum=1500, center=2000, maximum=2500))
        self.servo.request_sweep('maximum', speed=10, acceleration=4)
        self.servo.step(lambda: True)
        self.servo.stop()
        self.servo.request_position(2100)
        self.servo.step(lambda: True)
        self.assertEqual(self.bus.values[73], 0)
        self.assertEqual(self.bus.values[30], 2100)

    def test_jog_continues_to_joint_limit_and_stops_on_gate_loss(self):
        self.servo.request_jog(1)
        self.assertEqual(self.servo.target, 4095)
        self.servo.step(lambda: True)
        self.assertEqual(self.bus.values[32], 3)
        self.assertEqual(self.bus.values[30], 2056)
        self.bus.values[36] = 2040
        self.servo.request_jog(1)
        self.assertEqual(self.servo.target, 4095)
        self.servo.step(lambda: False)
        self.assertFalse(self.servo.torque)
        self.assertIsNone(self.servo.target)
        self.assertIsNone(self.servo.jog_limits)

    def test_slow_jog_can_overcome_loaded_position_deadband(self):
        original = self.bus.write
        def motor(ident, address, value, size=2):
            original(ident, address, value, size)
            error = value - self.bus.values[36]
            if address == 30 and self.bus.values[24] and abs(error) > 20:
                self.bus.values[36] += 1 if error > 0 else -1
        self.bus.write = motor
        for _ in range(80):
            self.servo.request_jog(1)
            self.servo.step(lambda: True)
        self.assertGreater(self.bus.values[36], 2003)
        self.assertGreater(self.bus.values[36], 2056)
        self.assertEqual(self.bus.values[32], 3)

    def test_jog_goal_is_bounded_in_both_directions_and_after_release(self):
        for direction in (-1, 1):
            self.servo.stop()
            self.bus.values[36] = 2000
            for _ in range(100):
                self.servo.request_jog(direction)
                self.servo.step(lambda: True)
                self.assertEqual(self.bus.values[30], 2000 + direction * 56)
            self.servo.stop()
            before = list(self.bus.writes)
            self.servo.step(lambda: True)
            self.assertEqual(self.bus.writes, before)
            self.assertEqual(self.bus.values[24], 0)

    def test_calibration_jog_can_pass_saved_sweep_limits(self):
        self.servo.configure(dict(minimum=1980, center=2000, maximum=2020))
        for direction, goal in [(-1, 1944), (1, 2056)]:
            self.servo.stop()
            self.servo.request_jog(direction)
            self.servo.step(lambda: True)
            self.assertEqual(self.bus.values[30], goal)

    def test_telemetry_reports_hardware_values_and_signed_units(self):
        self.bus.values.update({24: 1, 34: 512, 38: 1024 + 10, 40: 512, 68: 2148, 46: 1})
        data = self.servo.snapshot()
        self.assertTrue(data['torque'])
        self.assertTrue(data['moving'])
        self.assertAlmostEqual(data['load_percent'], 50.0489, places=3)
        self.assertAlmostEqual(data['torque_limit_percent'], 50.0489, places=3)
        self.assertAlmostEqual(data['current_a'], .45)
        self.assertAlmostEqual(data['speed_deg_s'], -6.84)

    def test_zero_torque_limit_explains_no_motion_without_reenabling(self):
        self.bus.values[34] = 0
        self.servo.request_jog(1)
        with self.assertRaisesRegex(ValueError, 'torque limit is 0'):
            self.servo.step(lambda: True)
        self.assertNotIn((24, 1), self.bus.writes)

    def test_jog_uses_eeprom_limits_and_validates_direction(self):
        self.servo.configure(dict(minimum=1900, center=2000, maximum=2100))
        self.servo.request_jog(-1)
        self.assertEqual(self.servo.target, 0)
        with self.assertRaises(ValueError):
            self.servo.request_jog(0)

    def test_jog_speed_applies_without_hold_travel_limit(self):
        self.servo.request_jog(1, 100)
        self.servo.step(lambda: True)
        self.assertEqual(self.bus.values[32], round(100 / .684))
        self.bus.values[36] = 3000
        self.servo.request_jog(1, 100)
        self.servo.step(lambda: True)
        self.assertEqual(self.bus.values[30], 3056)
        self.assertEqual(self.servo.target, 4095)

    def test_invalid_speed_and_out_of_range_target_are_rejected(self):
        for speed in (0, -1, float('nan'), float('inf'), True, 700):
            with self.assertRaises(ValueError):
                self.servo.request_jog(1, speed)
        self.servo.configure(dict(minimum=1900, center=2000, maximum=2100))
        self.servo.request_position(2500)
        with self.assertRaises(ValueError):
            self.servo.request_position(4096)
        self.assertNotIn((24, 1), self.bus.writes)

    def test_fast_sweep_keeps_saved_endpoints(self):
        self.servo.configure(dict(minimum=1500, center=2000, maximum=2500))
        self.servo.request_sweep('maximum', 500, 20)
        self.assertEqual(self.servo.sweep_profile[:3], (1500, 2500, 500))
        with self.assertRaises(ValueError):
            self.servo.request_sweep('maximum', 1024, 20)

    def test_slider_uses_full_speed_direct_target_and_permission(self):
        self.servo.request_jog(1, 2)
        self.servo.step(lambda: True)
        self.servo.request_position(3500)
        self.servo.step(lambda: True)
        self.assertEqual(self.bus.values[32], 0)
        self.assertEqual(self.bus.values[30], 3500)
        self.servo.step(lambda: False)
        self.assertEqual(self.bus.values[24], 0)

    def test_capture_is_read_only_and_range_checked(self):
        before = list(self.bus.writes)
        self.assertEqual(self.servo.capture('center'), 2000)
        self.assertEqual(self.bus.writes, before)
        for values in [(0, 0, 4095), (3000, 2000, 1000), (0, 2048, 4096)]:
            with self.assertRaises(ValueError):
                limits(*values)

    def test_small_moves_and_px4_loss_disables_torque(self):
        self.servo.configure(dict(minimum=1500, center=2000, maximum=2500))
        self.servo.request('maximum')
        self.servo.step(lambda: True)
        self.assertIn((30, 2000), self.bus.writes)  # Goal set before torque.
        self.assertIn((24, 1), self.bus.writes)
        self.assertEqual(self.bus.writes[-1], (30, 2016))
        self.servo.step(lambda: False)
        self.assertEqual(self.bus.writes[-1], (24, 0))
        self.assertIsNone(self.servo.target)
        before = list(self.bus.writes)
        self.servo.step(lambda: True)
        self.assertEqual(self.bus.writes, before)  # No stale replay after recovery.

    def test_gate_rechecked_between_read_and_torque(self):
        self.servo.configure(dict(minimum=1500, center=2000, maximum=2500))
        self.servo.request('maximum')
        answers = iter([True, True, False])
        self.servo.step(lambda: next(answers))
        self.assertNotIn((24, 1), self.bus.writes)
        self.assertEqual(self.bus.writes[-1], (24, 0))


class ArbotiXStartupTest(unittest.TestCase):
    def test_waits_for_bootloader_without_reopening_port(self):
        port, packet = Mock(), Mock()
        packet.readTxRx.side_effect = [([], -1, 0), ([44], 0, 0), ([1], 0, 0)]
        packet.write1ByteTxRx.return_value = (0, 0)
        sdk = SimpleNamespace(PortHandler=Mock(return_value=port), PacketHandler=Mock(return_value=packet))
        with patch.dict('sys.modules', dynamixel_sdk=sdk), patch('peaceofmine_operator.arm_servo.time.sleep'):
            bus = ArbotiX('/dev/fake')
        sdk.PortHandler.assert_called_once_with('/dev/fake')
        self.assertEqual(packet.readTxRx.call_count, 3)
        packet.write1ByteTxRx.assert_not_called()
        bus.close()

    def test_legacy_firmware_is_rejected_without_motion(self):
        port, packet = Mock(), Mock()
        packet.readTxRx.side_effect = [([44], 0, 0), ([0], 0, 0)]
        sdk = SimpleNamespace(PortHandler=Mock(return_value=port), PacketHandler=Mock(return_value=packet))
        with patch.dict('sys.modules', dynamixel_sdk=sdk):
            with self.assertRaisesRegex(RuntimeError, 'safe_arm firmware'):
                ArbotiX('/dev/fake')
        packet.write1ByteTxRx.assert_not_called()
        port.closePort.assert_called_once()


class ArbotiXReadTest(unittest.TestCase):
    def setUp(self):
        self.bus = ArbotiX.__new__(ArbotiX)
        self.bus.port, self.bus.packet = Mock(), Mock()

    def test_sweep_rejection_preserves_firmware_fault_before_cleanup(self):
        self.bus.packet.write1ByteTxRx.return_value = (0, 8)
        self.bus.packet.readTxRx.return_value = ([7], 0, 0)
        with self.assertRaisesRegex(RuntimeError, 'register=94.*firmware_fault=7'):
            self.bus.write(253, 94, 1, 1)
        self.bus.packet.write1ByteTxRx.assert_called_once()

    def test_error_only_reply_is_rejected_before_decoding(self):
        for size in (1, 2):
            for data in ([], [220]):
                self.bus.packet.readTxRx.return_value = (data, 0, 32)
                with self.assertRaisesRegex(RuntimeError, 'device=32'):
                    self.bus.read(0, 0, size)
        self.bus.packet.read1ByteTxRx.assert_not_called()
        self.bus.packet.read2ByteTxRx.assert_not_called()

    def test_short_reply_and_timeout_are_rejected(self):
        for reply in (([], 0, 0), ([54], 0, 0), ([], -3001, 0)):
            self.bus.packet.readTxRx.return_value = reply
            with self.assertRaises(RuntimeError):
                self.bus.read(1, 0)

    def test_scan_can_continue_after_absent_id(self):
        self.bus.packet.readTxRx.side_effect = [([], 0, 32), ([], 0, 32), ([54, 1], 0, 0), ([54, 1], 0, 0)]
        found = []
        for ident in range(3):
            try:
                if self.bus.read(ident, 0) == 310:
                    found.append(ident)
            except RuntimeError:
                continue
        self.assertEqual(found, [1, 2])

    def test_one_transient_read_failure_retries_without_writes(self):
        self.bus.packet.readTxRx.side_effect = [([], 0, 32), ([54, 1], 0, 0)]
        self.assertEqual(self.bus.read(1, 0), 310)
        self.assertEqual(self.bus.read_retries, 1)
        self.assertEqual(self.bus.packet.readTxRx.call_count, 2)
        self.bus.packet.write1ByteTxRx.assert_not_called()
        self.bus.packet.write2ByteTxRx.assert_not_called()

    def test_persistent_read_failure_is_not_retried_forever(self):
        self.bus.packet.readTxRx.return_value = ([], 0, 32)
        with self.assertRaisesRegex(RuntimeError, 'register=36.*device=32'):
            self.bus.read(1, 36)
        self.assertEqual(self.bus.packet.readTxRx.call_count, 2)

    def test_complete_servo_alarm_reply_is_never_retried_as_a_bus_fault(self):
        self.bus.packet.readTxRx.return_value = ([1, 0], 0, 32)
        with self.assertRaisesRegex(ServoAlarm, 'source=servo alarm.*overload') as caught:
            self.bus.read(1, 24)
        self.assertEqual(caught.exception.data, bytes([1, 0]))
        self.assertEqual(self.bus.packet.readTxRx.call_count, 1)

    def test_sdk_error_packet_without_parameters(self):
        try:
            from dynamixel_sdk import PacketHandler
        except ImportError:
            self.skipTest('Dynamixel SDK not installed in this interpreter')
        self.bus.packet = PacketHandler(1.0)
        # Valid Protocol 1 status packet: error=32, no parameters, checksum=221.
        self.bus.packet.txRxPacket = Mock(return_value=([255, 255, 0, 2, 32, 221], 0, 32))
        with self.assertRaisesRegex(RuntimeError, 'device=32'):
            self.bus.read(0, 0)


class WatchdogTransportTest(unittest.TestCase):
    def setUp(self):
        self.bus = Bus()
        self.servo = ArmServo(self.bus, 1, dict(minimum=1500, center=2000, maximum=2500))

    def test_no_heartbeat_without_a_motion_request_or_permission(self):
        self.servo.step(lambda: True)
        self.assertEqual(self.bus.gateway_writes, [])
        self.servo.request_sweep('maximum', 10, 4)
        self.servo.step(lambda: False)
        self.assertEqual(self.bus.gateway_writes, [])

    def test_expired_watchdog_is_not_automatically_rearmed(self):
        self.servo.request_sweep('maximum', 10, 4)
        self.servo.step(lambda: True)
        self.bus.gateway[96] = 3
        self.bus.gateway[98] = 2
        with self.assertRaisesRegex(RuntimeError, 'watchdog expired'):
            self.servo.step(lambda: True)
        self.assertEqual(self.bus.gateway_writes.count((82, 1)), 1)

    def test_firmware_stall_is_reported_to_operator(self):
        self.servo.request_sweep('maximum', 10, 4)
        self.servo.step(lambda: True)
        self.bus.gateway.update({96: 3, 98: 5})
        with self.assertRaisesRegex(RuntimeError, 'no encoder progress'):
            self.servo.step(lambda: True)

    def test_endpoint_settle_fault_is_distinct(self):
        self.servo.request_sweep('maximum', 10, 4)
        self.servo.step(lambda: True)
        self.bus.gateway.update({96: 3, 98: 10})
        with self.assertRaisesRegex(RuntimeError, 'endpoint did not settle'):
            self.servo.step(lambda: True)

    def test_speed_update_does_not_stop_or_restart_sweep(self):
        self.servo.request_sweep('maximum', 10, 4)
        self.servo.step(lambda: True)
        self.bus.writes.clear()
        self.servo.request_sweep('maximum', 15, 2)
        self.servo.step(lambda: True)
        self.assertEqual(self.bus.gateway[88], 15)
        self.assertEqual(self.bus.gateway[90], 2)
        self.assertEqual(self.bus.gateway_writes.count((94, 1)), 1)
        self.assertEqual(self.bus.writes, [])

    def test_switching_to_slider_stops_firmware_sweep_first(self):
        self.servo.request_sweep('maximum', 10, 4)
        self.servo.step(lambda: True)
        self.bus.writes.clear()
        self.servo.request_position(2100)
        self.servo.step(lambda: True)
        self.assertEqual(self.bus.writes[0], (24, 0))
        self.assertEqual(self.bus.values[30], 2100)
        self.assertEqual(self.bus.gateway_writes.count((94, 1)), 1)
