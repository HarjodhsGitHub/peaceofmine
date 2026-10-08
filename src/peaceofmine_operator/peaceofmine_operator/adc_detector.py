"""Timestamped A3 voltage interpretation; never reuse legacy 8-bit calibration."""
from collections import deque
import math
import time


def validate_calibration(value):
    if not isinstance(value, dict) or set(value) != {'baseline_volts', 'trigger_volts'}:
        raise ValueError('Expected baseline_volts and trigger_volts')
    low, high = value['baseline_volts'], value['trigger_volts']
    if not all(type(v) in (int, float) and math.isfinite(v) and abs(v) <= 6.144 for v in (low, high)) or abs(high-low) < .001:
        raise ValueError('Calibration voltages must be distinct by at least 1 mV and within ±6.144 V')
    return dict(baseline_volts=float(low), trigger_volts=float(high))


class ADCDetector:
    def __init__(self, calibration=None, clock=time.time):
        self.clock = clock
        self.calibration = validate_calibration(calibration) if calibration else None
        self.samples = deque(maxlen=60000)
        self.valid = False
        self.last_sequence = 0
        self.session = None
        self.received = 0
        self.error = 'New A3 voltage calibration required'

    def update(self, samples, valid=True, session=None):
        if session != self.session:
            self.samples.clear()
            self.last_sequence = 0
            self.session = session
        self.valid = bool(valid)
        now = self.clock()
        if not valid:
            self.error = 'ADS1115 acquisition unavailable'
            return
        for sample in samples:
            if sample['mux'] != 7 or sample['seq'] <= self.last_sequence:
                continue
            self.last_sequence = sample['seq']
            voltage, at = sample['volts'], sample['time']
            if not all(type(v) in (int, float) and math.isfinite(v) for v in (voltage, at)) or not 0 <= now-at < .5 or sample.get('clipped', False):
                self.valid = False
                self.error = 'Invalid, clipped or stale A3 sample'
                continue
            self.samples.append((at, voltage, sample['seq']))
            self.received += 1
        self.trim(now)

    def trim(self, now):
        while self.samples and now-self.samples[0][0] > 120:
            self.samples.popleft()

    def snapshot(self, include_history=True):
        now = self.clock()
        self.trim(now)
        last = self.samples[-1] if self.samples else None
        fresh = bool(self.valid and last and 0 <= now-last[0] < .5)
        recent = [sample for sample in self.samples if now-sample[0] < 2]
        rate = (len(recent)-1)/(recent[-1][0]-recent[0][0]) if len(recent)>1 and recent[-1][0]>recent[0][0] else 0.
        ratio = None
        if fresh and self.calibration:
            low, high = self.calibration['baseline_volts'], self.calibration['trigger_volts']
            ratio = min(1., max(0., (last[1]-low)/(high-low)))
        return dict(connected=self.valid, fresh=fresh, unit='V', source='ADS1115 A3',
                    port='shared Pi I²C · A3 / MUX 7', age_ms=round((now-last[0])*1000) if last else None,
                    rate_hz=round(rate, 1), received=self.received,
                    packet=dict(seq=last[2], voltage_volts=last[1], amplitude_adc=last[1]) if last else None,
                    calibration_required=self.calibration is None, signal_ratio=ratio,
                    baseline_adc=self.calibration['baseline_volts'] if self.calibration else None,
                    full_response_adc=self.calibration['trigger_volts'] if self.calibration else None,
                    error=None if ratio is not None else ('New A3 voltage calibration required' if fresh else self.error),
                    history=[dict(age_s=round(now-at,3), adc=volts) for at,volts,_ in self.samples] if include_history else [])
