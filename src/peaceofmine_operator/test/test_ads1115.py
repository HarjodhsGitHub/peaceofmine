import unittest

from peaceofmine_operator.ads1115 import Ads1115


class FakeBus:
    def __init__(self, read_bytes=(0x40, 0x00)):
        self.read_bytes = read_bytes
        self.writes = []
        self.closed = False

    def write_i2c_block_data(self, address, register, data):
        self.writes.append((address, register, data))

    def read_i2c_block_data(self, address, register, length):
        self.last_read = (address, register, length)
        return list(self.read_bytes)

    def close(self):
        self.closed = True


class Ads1115Test(unittest.TestCase):
    def test_configures_continuous_a0_and_scales_voltage(self):
        bus = FakeBus()
        adc = Ads1115(full_scale_voltage=4.096, data_rate_sps=128,
                      bus_factory=lambda _: bus)
        counts, voltage = adc.read()
        self.assertEqual(bus.writes, [(0x48, 0x01, [0xc2, 0x83])])
        self.assertEqual(bus.last_read, (0x48, 0x00, 2))
        self.assertEqual(counts, 16384)
        self.assertAlmostEqual(voltage, 2.048)
        self.assertTrue(adc.connected)
        adc.close()
        self.assertTrue(bus.closed)

    def test_negative_code_is_signed(self):
        bus = FakeBus((0xff, 0xff))
        adc = Ads1115(full_scale_voltage=2.048, bus_factory=lambda _: bus)
        counts, voltage = adc.read()
        self.assertEqual(counts, -1)
        self.assertAlmostEqual(voltage, -2.048 / 32768)

    def test_invalid_configuration_is_rejected(self):
        with self.assertRaisesRegex(ValueError, 'channel'):
            Ads1115(channel=4)
        with self.assertRaisesRegex(ValueError, 'full_scale_voltage'):
            Ads1115(full_scale_voltage=3.3)
        with self.assertRaisesRegex(ValueError, 'data_rate_sps'):
            Ads1115(data_rate_sps=100)
