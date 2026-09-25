import unittest
from peaceofmine_operator.probe_contact import ProbeContact


class ProbeContactTest(unittest.TestCase):
    def sample(self, stamp, **changes):
        return dict(dict(connected=True, active_role='probe', servo_id=2, serial_port='test',
                         sample_time=stamp, load_percent=-4., current_a=.1, position=2000,
                         goal_position=2000, torque=True, probe=dict(holding=True)), **changes)

    def test_zero_is_frozen_and_peaks_survive_filter(self):
        contact = ProbeContact()
        for i in range(10):
            contact.update(self.sample(i), i * .05)
        contact.zero(.45)
        contact.update(self.sample(10, load_percent=-6, current_a=.13), .5)
        value = contact.snapshot(.5)
        self.assertAlmostEqual(value['contact_load_percent'], 2)
        self.assertAlmostEqual(value['current_change_ma'], 30)
        contact.update(self.sample(11), .55)
        self.assertAlmostEqual(contact.snapshot(.55)['contact_peak_percent'], 2)
        self.assertEqual(contact.baseline, (-4, .1))
        self.assertIsNone(contact.snapshot(.9)['current_a'])
        self.assertFalse(contact.can_zero(.9))

    def test_duplicate_samples_do_not_refresh_or_zero(self):
        contact = ProbeContact()
        for i in range(10):
            contact.update(self.sample(1), i * .05)
        self.assertEqual(len(contact.samples), 1)
        with self.assertRaises(ValueError):
            contact.zero(.45)
        self.assertFalse(contact.snapshot(.45)['contact_fresh'])

    def test_motion_disconnect_and_servo_switch_invalidate_zero(self):
        contact = ProbeContact()
        for i in range(10):
            contact.update(self.sample(i, probe=dict(holding=False)), i * .05)
        with self.assertRaises(ValueError):
            contact.zero(.45)
        for i in range(10, 21):
            contact.update(self.sample(i), i * .05)
        contact.zero(1)
        contact.update(self.sample(21, servo_id=3), 1.05)
        self.assertIsNone(contact.baseline)
        contact.update(dict(connected=False), 1.1)
        self.assertFalse(contact.snapshot(1.1)['contact_fresh'])

    def test_30_second_history_at_20_hz(self):
        contact = ProbeContact()
        for i in range(800):
            contact.update(self.sample(i), i * .05)
        self.assertGreaterEqual(len(contact.snapshot(39.95)['contact_samples']), 600)
