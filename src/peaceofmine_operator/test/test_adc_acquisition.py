"""Persistent operator settings, bounded telemetry and hardware failure behavior."""
import copy
from pathlib import Path
import tempfile
import time
import unittest
from unittest.mock import patch
from peaceofmine_operator.adc_acquisition import OperatorADC


class OperatorADCTest(unittest.TestCase):
    def test_persist_routes_scale_stop_and_restart(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / 'adc.json'
            adc = OperatorADC(path, demo=True)
            try:
                initial = adc.snapshot()
                self.assertFalse(initial['running'])
                settings = {key: initial[key] for key in ('config', 'routes')}
                settings['routes'][4] = 'both'
                settings['config']['channels'][4].update(multiplier=3, offset=-2)
                adc.command(dict(action='settings', settings=settings))
                adc.command(dict(action='run', running=True))
                deadline = time.monotonic() + 2
                while adc.snapshot()['sequence'] < 10 and time.monotonic() < deadline:
                    time.sleep(.02)
                adc.command(dict(action='run', running=False))
                result = adc.snapshot()
                self.assertGreaterEqual(result['sequence'], 10)
                self.assertEqual(result['applied'], result['revision'])
                self.assertEqual({s['mux'] for s in result['samples']}, {4, 5})
                sample = next(s for s in result['samples'] if s['mux'] == 4)
                self.assertAlmostEqual(sample['scaled'], sample['volts'] * 3 - 2)
                self.assertEqual(adc.snapshot(result['sequence'])['samples'], [])
                time.sleep(.1)
                self.assertEqual(adc.snapshot()['sequence'], result['sequence'])
                saved = path.read_text()
                invalid = copy.deepcopy(settings)
                invalid['config']['mode'] = 'continuous'
                with self.assertRaises(ValueError):
                    adc.command(dict(action='settings', settings=invalid))
                self.assertEqual(path.read_text(), saved)
                invalid = copy.deepcopy(settings)
                invalid['routes'][0] = 'wrong'
                with self.assertRaises(ValueError):
                    adc.command(dict(action='settings', settings=invalid))
            finally:
                adc.close()
            restarted = OperatorADC(path, demo=True)
            try:
                self.assertEqual(restarted.snapshot()['routes'][4], 'both')
                self.assertEqual(restarted.snapshot()['config'], settings['config'])
                self.assertFalse(restarted.snapshot()['running'])
            finally:
                restarted.close()
            self.assertFalse(adc.acquisition.thread.is_alive())

    def test_hardware_is_opt_in_and_failure_does_not_fake_samples(self):
        with tempfile.TemporaryDirectory() as tmp, patch(
                'peaceofmine_operator.adc_acquisition.ADS1115', side_effect=OSError('missing device')) as device:
            adc = OperatorADC(Path(tmp) / 'adc.json')
            try:
                time.sleep(.1)
                device.assert_not_called()
                adc.command(dict(action='run', running=True))
                deadline = time.monotonic() + 2
                while not adc.snapshot()['error'] and time.monotonic() < deadline:
                    time.sleep(.02)
                state = adc.snapshot()
                self.assertIn('missing device', state['error'])
                self.assertFalse(state['connected'])
                self.assertEqual(state['samples'], [])
            finally:
                adc.close()

    def test_plot_routing_does_not_control_trigger_source(self):
        with tempfile.TemporaryDirectory() as tmp:
            adc = OperatorADC(Path(tmp)/'adc.json', demo=True)
            try:
                state = adc.snapshot()
                settings = {k:state[k] for k in ('config','routes','probe_trigger')}
                settings['probe_trigger'].update(enabled=True,level=.5)
                settings['routes'][5]='detector'
                adc.command(dict(action='settings',settings=settings))
                self.assertTrue(adc.probe_trigger['enabled'])
            finally:
                adc.close()

    def test_settings_interrupt_long_scan_pause(self):
        with tempfile.TemporaryDirectory() as tmp:
            adc = OperatorADC(Path(tmp) / 'adc.json', demo=True)
            try:
                state = adc.snapshot()
                settings = {k: state[k] for k in ('config', 'routes')}
                settings['config']['interval_ms'] = 60000
                adc.command(dict(action='settings', settings=settings))
                adc.command(dict(action='run', running=True))
                deadline = time.monotonic() + 1
                while adc.snapshot()['sequence'] < 2 and time.monotonic() < deadline:
                    time.sleep(.01)
                settings['config']['interval_ms'] = 0
                adc.command(dict(action='settings', settings=settings))
                deadline = time.monotonic() + 1
                while adc.snapshot()['sequence'] < 4 and time.monotonic() < deadline:
                    time.sleep(.01)
                self.assertGreaterEqual(adc.snapshot()['sequence'], 4)
            finally:
                adc.close()
