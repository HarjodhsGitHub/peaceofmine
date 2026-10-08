import unittest
from peaceofmine_operator.adc_telemetry import ADCTelemetry


class TelemetryTest(unittest.TestCase):
    def test_independent_cursors_restarts_and_freshness(self):
        now = [0.0]
        telemetry = ADCTelemetry(clock=lambda: now[0])
        state = dict(session='one', revision=1, now=100, sequence=2,
                     connected=True, probe_contact=dict(fresh=True, detected=True, value=1),
                     samples=[dict(seq=1,time=99.9), dict(seq=2,time=100)])
        telemetry.update(state)
        self.assertEqual(len(telemetry.snapshot()['samples']), 2)
        self.assertEqual(len(telemetry.snapshot(1)['samples']), 1)
        self.assertEqual(telemetry.snapshot(2)['samples'], [])
        telemetry.update(state)
        self.assertEqual(len(telemetry.snapshot()['samples']), 2)
        now[0] = 2
        stale = telemetry.snapshot()
        self.assertFalse(stale['available'])
        self.assertIsNone(stale['probe_contact']['detected'])
        self.assertEqual(stale['now'], 102)
        state.update(session='two', sequence=1, samples=[dict(seq=1,time=100)])
        telemetry.update(state)
        self.assertEqual(len(telemetry.snapshot(2, 'one')['samples']), 1)
