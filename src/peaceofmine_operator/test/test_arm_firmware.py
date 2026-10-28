"""Compile and exercise the real firmware with a fake serial/servo transport."""
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest


class ArmFirmwareTest(unittest.TestCase):
    @unittest.skipUnless(shutil.which('g++'), 'Native C++ compiler required')
    def test_firmware_parser_watchdog_and_sweep(self):
        repo = Path(__file__).resolve().parents[3]
        source = repo / 'DevTools/servodemo/firmware/safe_arm/test/test_firmware.cpp'
        with tempfile.TemporaryDirectory() as directory:
            binary = str(Path(directory) / 'test_firmware')
            subprocess.run(['g++', '-std=c++11', '-Wall', '-Wextra', '-Werror',
                            '-fsanitize=undefined', '-fno-sanitize-recover=all',
                            f'-I{source.parent / "support"}', str(source), '-o', binary], check=True)
            subprocess.run([binary], check=True, timeout=15)


if __name__ == '__main__':
    unittest.main()
