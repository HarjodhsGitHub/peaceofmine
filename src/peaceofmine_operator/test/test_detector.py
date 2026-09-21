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
        now[0] = 121
        self.assertEqual(detector.snapshot()['history'], [])

    def test_full_time_window_at_fifty_hz(self):
        now = [0.0]
        detector = DetectorTelemetry(clock=lambda: now[0])
        for seq in range(6501):
            now[0] = seq / 50
            detector.update(dict(fresh=True, packet=dict(
                seq=seq, uptime_ms=seq * 20, amplitude_adc=seq % 256)))
        history = detector.snapshot()['history']
        self.assertEqual(history[0]['age_s'], 120)
        self.assertEqual(history[-1]['age_s'], 0)
        self.assertEqual(len(history), 6001)
        self.assertEqual(len([s for s in history if s['age_s'] <= 30]), 1501)
        now[0] += 121
        self.assertEqual(detector.snapshot()['history'], [])
        self.assertEqual(len(detector.history), 0)
