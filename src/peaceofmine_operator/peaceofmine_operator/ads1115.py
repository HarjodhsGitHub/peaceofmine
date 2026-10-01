"""Small, testable I²C transport for a single-ended ADS1115 channel.

The ROS node owns sampling and calibration.  This module does only the
ADS1115 register protocol so it can be tested without an RPi or I²C hardware.
"""

from __future__ import annotations


class Ads1115:
    """Read a continuously converted, single-ended ADS1115 input in volts."""

    CONVERSION_REGISTER = 0x00
    CONFIG_REGISTER = 0x01

    # Config-register bit fields from the ADS1115 data sheet.
    _CHANNEL_BITS = {0: 0x4000, 1: 0x5000, 2: 0x6000, 3: 0x7000}
    _FULL_SCALE_BITS = {
        6.144: 0x0000,
        4.096: 0x0200,
        2.048: 0x0400,
        1.024: 0x0600,
        0.512: 0x0800,
        0.256: 0x0A00,
    }
    _DATA_RATE_BITS = {
        8: 0x0000,
        16: 0x0020,
        32: 0x0040,
        64: 0x0060,
        128: 0x0080,
        250: 0x00A0,
        475: 0x00C0,
        860: 0x00E0,
    }

    def __init__(self, bus_number=1, address=0x48, channel=0,
                 full_scale_voltage=4.096, data_rate_sps=128, bus_factory=None):
        if isinstance(bus_number, bool) or not isinstance(bus_number, int) or bus_number < 0:
            raise ValueError('I²C bus number must be a non-negative integer')
        if isinstance(address, bool) or not isinstance(address, int) or not 0x03 <= address <= 0x77:
            raise ValueError('I²C address must be between 0x03 and 0x77')
        if channel not in self._CHANNEL_BITS:
            raise ValueError('ADS1115 channel must be 0, 1, 2, or 3')
        if full_scale_voltage not in self._FULL_SCALE_BITS:
            choices = ', '.join(str(value) for value in self._FULL_SCALE_BITS)
            raise ValueError(f'full_scale_voltage must be one of: {choices}')
        if data_rate_sps not in self._DATA_RATE_BITS:
            choices = ', '.join(str(value) for value in self._DATA_RATE_BITS)
            raise ValueError(f'data_rate_sps must be one of: {choices}')

        self.bus_number = bus_number
        self.address = address
        self.channel = channel
        self.full_scale_voltage = full_scale_voltage
        self.data_rate_sps = data_rate_sps
        self._bus_factory = bus_factory or self._default_bus_factory
        self._bus = None

    @staticmethod
    def _default_bus_factory(bus_number):
        try:
            from smbus2 import SMBus
        except ImportError as exc:
            raise RuntimeError(
                'The ADS1115 node needs smbus2. Install it with: python3 -m pip install smbus2') from exc
        return SMBus(bus_number)

    @property
    def connected(self):
        return self._bus is not None

    @property
    def device_name(self):
        return f'/dev/i2c-{self.bus_number} at 0x{self.address:02x}'

    def close(self):
        if self._bus is not None:
            self._bus.close()
        self._bus = None

    def _config_word(self):
        # OS starts continuous conversion. MODE=0 selects continuous mode, and
        # COMP_QUE=11 disables the comparator/ALERT pin.
        return (0x8000 | self._CHANNEL_BITS[self.channel]
                | self._FULL_SCALE_BITS[self.full_scale_voltage]
                | self._DATA_RATE_BITS[self.data_rate_sps] | 0x0003)

    def open(self):
        if self._bus is not None:
            return
        bus = self._bus_factory(self.bus_number)
        try:
            config = self._config_word()
            bus.write_i2c_block_data(self.address, self.CONFIG_REGISTER,
                                     [(config >> 8) & 0xff, config & 0xff])
        except Exception:
            bus.close()
            raise
        self._bus = bus

    def read(self):
        """Return ``(signed_counts, voltage)`` from the configured input."""
        self.open()
        data = self._bus.read_i2c_block_data(self.address, self.CONVERSION_REGISTER, 2)
        if len(data) != 2:
            raise OSError(f'ADS1115 returned {len(data)} conversion bytes, expected 2')
        counts = (data[0] << 8) | data[1]
        if counts & 0x8000:
            counts -= 0x10000
        return counts, counts * self.full_scale_voltage / 32768.0
