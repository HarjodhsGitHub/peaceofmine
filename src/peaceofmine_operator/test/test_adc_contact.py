import unittest
from peaceofmine_operator.adc_contact import AdcContact


class ContactTest(unittest.TestCase):
    def setUp(self):
        self.now=1.
        self.contact=AdcContact(clock=lambda:self.now)
        self.contact.configure(dict(enabled=True,mux=5,level=1.,direction='above',hysteresis=.1,debounce_ms=100))

    def sample(self,value,sequence):
        self.contact.sample(value,sequence,'session')
        return self.contact.snapshot()

    def test_hysteresis_debounce_and_staleness(self):
        self.assertFalse(self.sample(1.1,1)['detected'])
        self.now+=.11
        self.assertTrue(self.sample(1.1,2)['detected'])
        self.assertTrue(self.sample(.95,3)['detected'])
        self.sample(.8,4)
        self.now+=.11
        self.assertFalse(self.sample(.8,5)['detected'])
        self.now+=.6
        self.assertIsNone(self.contact.snapshot()['detected'])

    def test_reordered_packets_do_not_refresh_input(self):
        self.sample(1.1,4)
        self.now+=.6
        self.assertIsNone(self.sample(1.1,3)['detected'])

    def test_below_threshold_and_session_restart(self):
        config=dict(self.contact.config,direction='below',debounce_ms=0.)
        self.contact.configure(config)
        self.assertTrue(self.sample(.9,1)['detected'])
        self.contact.sample(2.,1,'new-session')
        self.assertFalse(self.contact.snapshot()['detected'])
