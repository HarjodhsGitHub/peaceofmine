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

    def test_requires_complete_fresh_set_and_rejects_replay(self):
        self.assertFalse(self.safety.snapshot()['allowed'])
        healthy(self.safety)
        self.assertTrue(self.safety.snapshot()['allowed'])
        self.now += .41
        healthy(self.safety)
        self.assertFalse(self.safety.snapshot()['allowed'])
        healthy(self.safety, 2000000)
        self.assertTrue(self.safety.snapshot()['allowed'])
        self.now += 1.51
        self.safety.update_rc(rc(3000000))
        self.assertFalse(self.safety.snapshot()['allowed'])

    def test_px4_disarm_override_kill_and_rc_loss(self):
        for message, expected in [(state(2000000, armed=False), 'ros_disarmed'),
                                  (state(2000000, system_status=8), 'kill'),
                                  (state(2000000, system_status=5), 'unknown'),
                                  (state(2000000, connected=False), 'unknown'),
                                  (rc(2000000, pwm=1500), 'override'),
                                  (rc(2000000, pwm=2000), 'kill'),
                                  (rc(2000000, pwm=1200), 'unknown'),
                                  (rc(2000000, rssi=0), 'rc_lost')]:
            with self.subTest(expected=expected, message=message):
                self.safety.samples.clear()
                healthy(self.safety)
                (self.safety.update_state if isinstance(message, State) else self.safety.update_rc)(message)
                snapshot = self.safety.snapshot()
                self.assertFalse(snapshot['allowed'])
                self.assertEqual(snapshot['mode'], expected)

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

    def test_servo_requires_fresh_rc_and_connected_status_four(self):
        for status in range(9):
            with self.subTest(status=status):
                self.safety.samples.clear()
                self.safety.update_state(state(system_status=status, armed=False))
                self.safety.update_rc(rc())
                self.assertEqual(self.safety.servo_snapshot()['allowed'], status == 4)
        self.safety.samples.clear()
        self.safety.update_state(state(system_status=4, armed=False))
        self.safety.update_rc(rc(pwm=1500))
        self.assertTrue(self.safety.servo_snapshot()['allowed'])
        self.assertFalse(self.safety.snapshot()['allowed'])
        self.now += 1.51
        self.assertFalse(self.safety.servo_snapshot()['allowed'])
        self.safety.update_state(state(2000000, system_status=4))
        self.safety.update_state(state(2000000, connected=False))
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
