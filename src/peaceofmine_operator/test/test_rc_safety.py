import unittest
from mavros_msgs.msg import State, RCIn
from peaceofmine_operator.rc_safety import RcSafety


def state(stamp=1000000, **kwargs):
    msg = State(connected=True, armed=True, system_status=4, mode='MANUAL')
    msg.header.stamp.sec = stamp
    for key, value in kwargs.items():
        setattr(msg, key, value)
    return msg


def rc(stamp=1000000, pwm=1000, rssi=100):
    msg = RCIn(channels=[1500, 1500, 1500, 1500, pwm, 1500, 1500], rssi=rssi)
    msg.header.stamp.sec = stamp
    return msg


def healthy(safety, stamp=1000000):
    safety.update_state(state(stamp))
    safety.update_rc(rc(stamp))


class RcSafetyTest(unittest.TestCase):
    def setUp(self):
        self.now = 10.0
        self.safety = RcSafety(clock=lambda: self.now)

    def test_fresh_rc_is_authority_and_replay_cannot_renew_it(self):
        self.assertFalse(self.safety.snapshot()['allowed'])
        self.safety.update_rc(rc())
        self.assertTrue(self.safety.snapshot()['allowed'])
        self.now += .41
        self.safety.update_rc(rc())
        self.assertFalse(self.safety.snapshot()['allowed'])
        self.safety.update_rc(rc(2000000))
        self.assertTrue(self.safety.snapshot()['allowed'])
        self.now += 2
        self.safety.update_rc(rc(3000000))
        self.assertTrue(self.safety.snapshot()['allowed'])

    def test_rc_override_kill_transitions_and_signal_loss(self):
        for pwm,rssi,mode,servo in ((1500,100,'override',True), (2000,100,'kill',False),
                                    (1200,100,'unknown',False), (1000,0,'rc_lost',False)):
            with self.subTest(pwm=pwm,rssi=rssi):
                self.safety.samples.clear()
                self.safety.update_rc(rc(pwm=pwm,rssi=rssi))
                snapshot=self.safety.snapshot()
                self.assertFalse(snapshot['allowed'])
                self.assertEqual(snapshot['servo_allowed'],servo)
                self.assertEqual(snapshot['mode'],mode)

    def test_invalid_channels_reboot_and_disconnect_fail_closed(self):
        healthy(self.safety)
        message = rc(2000000)
        message.channels = [1500]
        self.safety.update_rc(message)
        self.assertFalse(self.safety.snapshot()['allowed'])
        healthy(self.safety, 3000000)
        self.safety.update_rc(rc(1))
        self.assertFalse(self.safety.snapshot()['allowed'])
        healthy(self.safety, 4000000)
        self.safety.update_state(state(4000000, connected=False))
        self.assertFalse(self.safety.snapshot()['allowed'])

    def test_px4_status_and_arming_do_not_add_a_second_rc_gate(self):
        for status in range(9):
            with self.subTest(status=status):
                self.safety.samples.clear()
                self.safety.update_state(state(system_status=status,armed=False))
                self.safety.update_rc(rc())
                self.assertTrue(self.safety.servo_snapshot()['allowed'])
                self.assertTrue(self.safety.snapshot()['allowed'])
        self.safety.samples.clear()
        self.safety.update_rc(rc(pwm=1500))
        self.assertTrue(self.safety.servo_snapshot()['allowed'])
        self.assertFalse(self.safety.snapshot()['allowed'])
        self.now += .41
        self.assertFalse(self.safety.servo_snapshot()['allowed'])

    def test_servo_rc_kill_and_loss_without_heartbeat_change(self):
        healthy(self.safety)
        self.safety.update_rc(rc(2000000, pwm=1500))
        self.assertTrue(self.safety.servo_snapshot()['allowed'])
        self.safety.update_rc(rc(3000000, pwm=2000))
        self.assertFalse(self.safety.servo_snapshot()['allowed'])
        self.safety.update_rc(rc(4000000, pwm=1500))
        self.now += .41
        self.assertFalse(self.safety.servo_snapshot()['allowed'])
