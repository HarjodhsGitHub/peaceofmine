"""Adapt shared ADC acquisition to the detector's calibrated 8-bit scale."""
import math
import time


class ADCDetectorInput:
    def __init__(self, channel=7, reference=lambda: 5.0, clock=time.monotonic):
        self.channel, self.reference, self.clock = channel, reference, clock
        self.port, self.baudrate = f'ADS1115 mux {channel}', 0
        self.connection = self.last_sample = None
        self.invalid_lines = 0
        self.error = 'Start acquisition in Settings → ADC'
        self.pending = []
        self.key = None
        self.sequence = -1

    @property
    def fresh(self):
        return self.connection is not None and self.last_sample is not None and self.clock()-self.last_sample < .5

    def update(self, session, revision, valid, samples, epoch=None):
        epoch = time.time() if epoch is None else epoch
        key = session, revision
        if key != self.key or not valid:
            self.pending.clear()
            self.last_sample = self.connection = None
            self.sequence = -1
            self.key = key
        if not valid:
            self.error = 'ADC stopped, disconnected, or applying settings'
            return
        for sample in samples:
            if sample['channel'] != self.channel or sample['sequence'] <= self.sequence:
                continue
            if not math.isfinite(sample['volts']) or not 0 <= epoch-sample['stamp'] < .5:
                continue
            self.sequence = sample['sequence']
            raw = max(0, min(255, round(sample['volts']/self.reference()*255)))
            self.pending.append(dict(v=1, seq=self.sequence, uptime_ms=0, amplitude_adc=raw))
            self.pending = self.pending[-256:]
            self.last_sample = self.clock() - (epoch-sample['stamp'])
            self.connection, self.error = True, None

    def poll(self):
        samples, self.pending = self.pending, []
        return samples if self.fresh else []

    def close(self):
        self.connection = self.last_sample = None
        self.pending.clear()
