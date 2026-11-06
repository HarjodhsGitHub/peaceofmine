import math
import unittest
from peaceofmine_operator.detector import DetectorTelemetry, DetectorLowPass


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


class DetectorLowPassTest(unittest.TestCase):
    def test_step_response_is_independent_of_sample_rate(self):
        outputs=[]
        for hz in (20,50,100):
            f=DetectorLowPass()
            f.update(0.,0.)
            for i in range(1,hz+1):
                value=f.update(1.,i/hz)
            outputs.append(value)
        for value in outputs:
            self.assertAlmostEqual(value,1-math.exp(-10),places=10)

    def test_noise_is_reduced_and_small_signals_keep_precision(self):
        f=DetectorLowPass()
        raw=[];smoothed=[]
        for i in range(300):
            value=125.7+.1*math.sin(i*2*math.pi*10/50)
            result=f.update(value,i/50)
            if i>50:
                raw.append(value);smoothed.append(result)
        self.assertLess(max(smoothed)-min(smoothed),(max(raw)-min(raw))/4)
        self.assertTrue(any(v!=round(v) for v in smoothed))

    def test_gap_and_session_reset_do_not_blend_old_values(self):
        f=DetectorLowPass()
        self.assertEqual(f.update(2.465,1.),2.465)
        self.assertEqual(f.update(100.,1.),2.465)
        self.assertEqual(f.update(2.47,2.),2.47)
        f.reset()
        self.assertEqual(f.update(2.468,0.),2.468)
        with self.assertRaises(ValueError):
            f.update(math.nan,1.)

    def test_telemetry_retains_precise_raw_and_filtered_history(self):
        sensor=DetectorTelemetry(clock=lambda:1.)
        sensor.update(dict(fresh=True,packet=dict(seq=1,uptime_ms=1,
            amplitude_adc=126,precise_adc=125.753,filtered_adc=125.712)))
        sample=sensor.snapshot()['history'][0]
        self.assertEqual(sample['adc'],125.753)
        self.assertEqual(sample['filtered_adc'],125.712)
