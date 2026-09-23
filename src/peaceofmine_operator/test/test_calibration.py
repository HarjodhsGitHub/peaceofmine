"""Calibration files remain shared, restartable and independent of launch cwd."""
import json
import multiprocessing
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from peaceofmine_operator import calibration, probe_calibration


class CalibrationTest(unittest.TestCase):
    def test_default_does_not_use_working_directory(self):
        with patch.object(Path, 'cwd', side_effect=AssertionError('cwd used')):
            path = Path(calibration.default_path())
        self.assertTrue(path.is_absolute())
        self.assertEqual(path.name, 'calibration.json')
        self.assertEqual(path.parent.name, 'peaceofmine_operator')

    def test_symlink_saves_source_and_preserves_other_sections(self):
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory) / 'source.json'
            source.write_text(json.dumps(dict(arm=dict(servo_id=1), metal_detector=dict(baseline_adc=36))))
            source.chmod(0o644)
            installed = Path(directory) / 'installed.json'
            installed.symlink_to(source)
            probe_calibration.save(str(installed), dict(servo_id=2, max_extension_mm=120, travel_ticks=5430))
            self.assertTrue(installed.is_symlink())
            self.assertEqual(source.stat().st_mode & 0o777, 0o644)
            self.assertEqual(calibration.load(str(source))['arm'], dict(servo_id=1))
            self.assertEqual(probe_calibration.load(str(source))['max_extension_mm'], 120)

    def test_concurrent_nodes_preserve_sections(self):
        with tempfile.TemporaryDirectory() as directory:
            path = str(Path(directory) / 'calibration.json')
            workers = [multiprocessing.get_context('spawn').Process(target=calibration.save_section,
                       args=(path, section, dict(value=index)))
                       for index, section in enumerate(('arm', 'probe', 'metal_detector'))]
            for worker in workers:
                worker.start()
            for worker in workers:
                worker.join(5)
                self.assertEqual(worker.exitcode, 0)
            self.assertEqual(set(calibration.load(path)), {'arm', 'probe', 'metal_detector'})

    def test_invalid_json_is_not_overwritten(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'calibration.json'
            path.write_text('{broken')
            with self.assertRaises(ValueError):
                calibration.save_section(str(path), 'arm', {})
            self.assertEqual(path.read_text(), '{broken')
