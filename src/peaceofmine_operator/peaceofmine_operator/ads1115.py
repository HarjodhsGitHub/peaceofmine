"""ADS1115 register interface, TI SBAS444E sections 7 and 8."""
import copy
import math
import time

RATES = (8, 16, 32, 64, 128, 250, 475, 860)
RANGES = (6.144, 4.096, 2.048, 1.024, 0.512, 0.256)
INPUTS = ('AIN0 - AIN1', 'AIN0 - AIN3', 'AIN1 - AIN3', 'AIN2 - AIN3',
          'AIN0', 'AIN1', 'AIN2', 'AIN3')
DEFAULTS = dict(bus=1, address=0x48, rate=128, pga=1, mode='single',
                alert='disabled', polarity=0, latch=False, queue=1,
                low=-1000, high=1000, interval_ms=0,
                channels=[dict(mux=i, enabled=i >= 4, multiplier=1.0,
                               offset=0.0) for i in range(8)])


def validate(value):
    if not isinstance(value, dict) or set(value) != set(DEFAULTS):
        raise ValueError('Settings must contain exactly the supported fields')
    c = copy.deepcopy(value)
    for key, allowed in [('bus', range(256)), ('address', range(0x48, 0x4c)),
                         ('rate', RATES), ('pga', range(6)), ('polarity', (0, 1)),
                         ('queue', (1, 2, 4)), ('low', range(-32768, 32768)),
                         ('high', range(-32768, 32768)), ('interval_ms', range(60001))]:
        if type(c[key]) is not int or c[key] not in allowed:
            raise ValueError(f'Invalid {key}')
    if c['mode'] not in ('single', 'continuous'):
        raise ValueError('Invalid conversion mode')
    if c['alert'] not in ('disabled', 'traditional', 'window', 'ready'):
        raise ValueError('Invalid ALERT function')
    if type(c['latch']) is not bool:
        raise ValueError('Invalid latch value')
    if not isinstance(c['channels'], list) or len(c['channels']) != 8:
        raise ValueError('Eight input configurations are required')
    for i, ch in enumerate(c['channels']):
        if not isinstance(ch, dict) or set(ch) != {'mux', 'enabled', 'multiplier', 'offset'}:
            raise ValueError('Invalid channel configuration')
        if type(ch['mux']) is not int or ch['mux'] != i or type(ch['enabled']) is not bool:
            raise ValueError('Invalid channel selection')
        for key in ('multiplier', 'offset'):
            if type(ch[key]) not in (int, float) or not math.isfinite(ch[key]) or abs(ch[key]) > 1e9:
                raise ValueError(f'Invalid channel {key}')
    enabled = sum(ch['enabled'] for ch in c['channels'])
    if c['mode'] == 'continuous' and enabled != 1:
        raise ValueError('Continuous mode requires exactly one enabled input')
    if c['alert'] in ('traditional', 'window'):
        if enabled != 1:
            raise ValueError('Comparator testing requires exactly one enabled input')
        if c['low'] >= c['high']:
            raise ValueError('Low threshold must be less than high threshold')
    return c


def config_word(c, mux, start=False, idle=False):
    queue = 3 if c['alert'] == 'disabled' else {1: 0, 2: 1, 4: 2}[c['queue']]
    return ((int(start) << 15) | (mux << 12) | (c['pga'] << 9)
            | (int(idle or c['mode'] == 'single') << 8) | (RATES.index(c['rate']) << 5)
            | (int(c['alert'] == 'window') << 4) | (c['polarity'] << 3)
            | (int(c['latch'] and c['alert'] != 'ready') << 2) | queue)


def signed(word):
    return word - 65536 if word & 0x8000 else word


class ADS1115:
    def __init__(self, bus, address, transport=None):
        if transport is None:
            from smbus2 import SMBus
            transport = SMBus(bus)
        self.bus = transport
        self.address = address
        self.last_config = None

    def read(self, register):
        data = self.bus.read_i2c_block_data(self.address, register, 2)
        return (data[0] << 8) | data[1]

    def write(self, register, value):
        self.bus.write_i2c_block_data(self.address, register, [(value >> 8) & 255, value & 255])

    def idle(self):
        # Complete the current conversion before changing MUX/PGA or thresholds.
        current = self.read(1)
        self.write(1, (current & 0x7fff) | 0x0103)
        deadline = time.monotonic() + 0.3
        while not self.read(1) & 0x8000:
            if time.monotonic() > deadline:
                raise TimeoutError('ADC did not enter power-down')
            time.sleep(0.001)
        self.last_config = None

    def configure(self, c):
        self.idle()
        lo, hi = (0, 0x8000) if c['alert'] == 'ready' else (c['low'], c['high'])
        self.write(2, lo)
        self.write(3, hi)
        mux = next((ch['mux'] for ch in c['channels'] if ch['enabled']), 4)
        self.write(1, config_word(c, mux, idle=True))
        registers = self.registers()
        if registers['low'] != lo & 0xffff or registers['high'] != hi & 0xffff:
            raise IOError('ADC threshold readback mismatch')
        if registers['config'] & 0x7fff != config_word(c, mux, idle=True):
            raise IOError('ADC configuration readback mismatch')
        return registers

    def sample(self, c, mux):
        word = config_word(c, mux, start=c['mode'] == 'single')
        if c['mode'] == 'single':
            self.write(1, word)
            deadline = time.monotonic() + 2 / c['rate'] + 0.05
            time.sleep(0.8 / c['rate'])
            while not self.read(1) & 0x8000:
                if time.monotonic() > deadline:
                    raise TimeoutError('ADC conversion timed out')
                time.sleep(min(0.001, 0.1 / c['rate']))
        else:
            if self.last_config != word:
                self.write(1, word)
                self.last_config = word
                # No RDY GPIO is required. Account for the specified -10% rate.
            time.sleep(1 / (c['rate'] * 0.9) + 0.00005)
        return signed(self.read(0))

    def registers(self):
        return dict(zip(('config', 'low', 'high'), (self.read(i) for i in (1, 2, 3))))

    def close(self):
        try:
            self.idle()
        finally:
            self.bus.close()


def amplitude_counts(raw, full_scale, reference):
    """Map a signed ADS1115 count onto the dashboard's 0..255 detector scale."""
    if type(raw) is not int or not -32768 <= raw <= 32767:
        raise ValueError('ADC count must be a signed 16-bit value')
    if type(full_scale) not in (int, float) or not math.isfinite(full_scale) or full_scale <= 0:
        raise ValueError('ADC full-scale voltage must be positive')
    if type(reference) not in (int, float) or not math.isfinite(reference) or reference <= 0:
        raise ValueError('ADC reference voltage must be positive')
    volts = raw * full_scale / 32768
    counts = round(volts / reference * 255)
    return max(0, min(255, counts)), volts
