import copy
import time
import unittest
from unittest.mock import patch

from peaceofmine_operator.ads1115 import ADS1115, DEFAULTS, RANGES, RATES, config_word, signed, validate
from peaceofmine_operator.adc_acquisition import Acquisition


class Bus:
    def __init__(self):
        self.regs = {0: 0xfffe, 1: 0x8583, 2: 0x8000, 3: 0x7fff}
        self.writes = []
        self.closed = False
        self.busy = 0

    def read_i2c_block_data(self, address, register, count):
        value = self.regs[register]
        if register == 1:
            if self.busy:
                self.busy -= 1
                value &= 0x7fff
            else:
                value |= 0x8000
        return [value >> 8, value & 255]

    def write_i2c_block_data(self, address, register, data):
        self.writes.append((address, register, data))
        self.regs[register] = data[0] << 8 | data[1]

    def close(self):
        self.closed = True


class ADCTests(unittest.TestCase):
    def setUp(self):
        self.c = copy.deepcopy(DEFAULTS)
        self.bus = Bus()
        self.adc = ADS1115(1, 0x48, self.bus)

    def test_datasheet_config_and_byte_order(self):
        self.c.update(pga=2, mode='continuous')
        self.assertEqual(config_word(self.c, 0), 0x0483)
        self.adc.write(1, 0x8483)
        self.assertEqual(self.bus.writes[-1], (0x48, 1, [0x84, 0x83]))
        self.assertEqual(self.adc.read(0), 0xfffe)
        for pga in range(6):
            for mux in range(8):
                for rate in RATES:
                    self.c.update(pga=pga, rate=rate)
                    word = config_word(self.c, mux)
                    self.assertEqual((word >> 12) & 7, mux)
                    self.assertEqual((word >> 9) & 7, pga)
                    self.assertEqual((word >> 5) & 7, RATES.index(rate))

    def test_signed_voltage(self):
        self.assertEqual([signed(x) for x in [0, 1, 0x7fff, 0x8000, 0xffff]],
                         [0, 1, 32767, -32768, -1])
        self.assertEqual(signed(0x8000) * RANGES[1] / 32768, -4.096)

    def test_single_shot_waits_for_ready(self):
        self.bus.busy = 2
        with patch('peaceofmine_operator.ads1115.time.sleep') as sleep:
            self.assertEqual(self.adc.sample(self.c, 7), -2)
        self.assertEqual(len(sleep.call_args_list), 3)
        self.assertTrue(self.bus.regs[1] & 0x8100 == 0x8100)

    def test_conversion_timeout(self):
        self.bus.busy = 100
        with patch('peaceofmine_operator.ads1115.time.sleep'), patch('peaceofmine_operator.ads1115.time.monotonic', side_effect=[0, 1]):
            with self.assertRaises(TimeoutError):
                self.adc.sample(self.c, 4)

    def test_continuous_writes_once_and_waits(self):
        self.c['mode'] = 'continuous'
        with patch('peaceofmine_operator.ads1115.time.sleep') as sleep:
            self.adc.sample(self.c, 4)
            self.adc.sample(self.c, 4)
        self.assertEqual(len(self.bus.writes), 1)
        self.assertGreater(sleep.call_args.args[0], 1 / (self.c['rate'] * .9))

    def test_ready_thresholds_and_idle(self):
        self.c['alert'] = 'ready'
        self.adc.configure(self.c)
        self.assertEqual(self.bus.regs[2], 0)
        self.assertEqual(self.bus.regs[3], 0x8000)
        self.assertEqual(self.bus.regs[1] & 3, 0)
        self.adc.close()
        self.assertTrue(self.bus.closed)
        self.assertEqual(self.bus.regs[1] & 0x8103, 0x0103)

    def test_validation(self):
        for key, value in [('rate', 100), ('address', 0), ('bus', True), ('mode', 'x'), ('low', -32769), ('high', 32768)]:
            bad = copy.deepcopy(self.c)
            bad[key] = value
            with self.subTest(key=key), self.assertRaises(ValueError):
                validate(bad)
        for mode in ['continuous', 'single']:
            bad = copy.deepcopy(self.c)
            bad.update(mode=mode, alert='traditional')
            with self.assertRaises(ValueError):
                validate(bad)
        self.c['channels'][0]['multiplier'] = float('nan')
        with self.assertRaises(ValueError):
            validate(self.c)

    def test_demo_lifecycle_and_scaling(self):
        c = copy.deepcopy(DEFAULTS)
        c['channels'][4].update(multiplier=3, offset=-1)
        acquisition = Acquisition(True, c)
        acquisition.thread.start()
        try:
            acquisition.run(True)
            deadline = time.monotonic() + 2
            while len(acquisition.snapshot()['samples']) < 8 and time.monotonic() < deadline:
                time.sleep(.02)
            acquisition.run(False)
            s = acquisition.snapshot()
            self.assertGreaterEqual(len(s['samples']), 8)
            self.assertEqual({x['mux'] for x in s['samples']}, {4, 5, 6, 7})
            x = next(x for x in s['samples'] if x['mux'] == 4)
            self.assertAlmostEqual(x['scaled'], x['volts'] * 3 - 1)
            time.sleep(.1)
            self.assertEqual(acquisition.snapshot()['sequence'], s['sequence'])
            acquisition.update(c)
            self.assertEqual(acquisition.snapshot()['samples'], [])
        finally:
            acquisition.close()
        self.assertFalse(acquisition.thread.is_alive())


if __name__ == '__main__':
    unittest.main()
