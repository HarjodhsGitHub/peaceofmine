import unittest
from peaceofmine_operator.adc_detector import ADCDetector, validate_calibration


class ADCDetectorTest(unittest.TestCase):
    def setUp(self):
        self.now = 100.
        self.sensor = ADCDetector(clock=lambda:self.now)

    def sample(self, mux=7, seq=1, volts=1., **extra):
        return dict(mux=mux, seq=seq, volts=volts, time=self.now, **extra)

    def test_routing_requires_new_voltage_calibration(self):
        self.sensor.update([self.sample(mux=4)], session='one')
        self.assertFalse(self.sensor.snapshot()['fresh'])
        self.sensor.update([self.sample()], session='one')
        self.assertTrue(self.sensor.snapshot()['fresh'])
        self.assertIsNone(self.sensor.snapshot()['signal_ratio'])
        self.sensor.calibration = validate_calibration(dict(baseline_volts=0.,trigger_volts=2.))
        self.assertEqual(self.sensor.snapshot()['signal_ratio'],.5)
        self.assertEqual(self.sensor.snapshot()['unit'],'V')
        with self.assertRaises(ValueError): validate_calibration(dict(baseline_adc=36.,full_response_adc=105.))

    def test_failure_staleness_and_recovery(self):
        self.sensor.update([self.sample()], session='one')
        self.sensor.update([], valid=False, session='one')
        self.assertFalse(self.sensor.snapshot()['fresh'])
        self.now += 1
        self.sensor.update([self.sample(seq=2)], session='one')
        self.assertTrue(self.sensor.snapshot()['fresh'])
        self.now += 1
        self.assertFalse(self.sensor.snapshot()['fresh'])
        self.sensor.update([self.sample(seq=3, clipped=True)], session='one')
        self.assertFalse(self.sensor.snapshot()['fresh'])
        self.sensor.update([self.sample(seq=1)], session='restart')
        self.assertTrue(self.sensor.snapshot()['fresh'])

    def test_rate_is_measured_per_a3_not_adc_total(self):
        for sequence in range(1,11):
            self.sensor.update([self.sample(seq=sequence)], session='one')
            self.now += .02
        self.assertEqual(self.sensor.snapshot()['rate_hz'],50.)
