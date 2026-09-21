import unittest
from peaceofmine_operator.detector import DetectorTelemetry


class DetectorTest(unittest.TestCase):
    def test_freshness_deduplication_and_history_expiry(self):
        now = [0.0]
        detector = DetectorTelemetry(clock=lambda: now[0])
        self.assertFalse(detector.snapshot()['fresh'])
        value = dict(fresh=True, age_ms=0, packet=dict(seq=0, uptime_ms=50, amplitude_adc=20))
        detector.update(value)
        detector.update(value)
        self.assertEqual(len(detector.snapshot()['history']), 1)
        now[0] = 1.1
        self.assertFalse(detector.snapshot()['fresh'])
        self.assertEqual(detector.snapshot()['age_ms'], 1100)
        now[0] = 31
        self.assertEqual(detector.snapshot()['history'], [])
