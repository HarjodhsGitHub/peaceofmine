"""Bounded detector history and freshness for dashboard telemetry."""

from collections import deque
import math
import time


class DetectorLowPass:
    time_constant = .1

    def __init__(self):
        self.reset()

    def reset(self):
        self.value = self.timestamp = None

    def update(self, value, timestamp):
        if not math.isfinite(value) or not math.isfinite(timestamp):
            raise ValueError('Detector sample and timestamp must be finite')
        if self.timestamp is not None and timestamp <= self.timestamp:
            return self.value
        if self.timestamp is None or timestamp-self.timestamp > .5:
            self.value = value
        else:
            self.value += -math.expm1(-(timestamp-self.timestamp)/self.time_constant)*(value-self.value)
        self.timestamp = timestamp
        return self.value


class DetectorTelemetry:
    def __init__(self, clock=time.monotonic):
        self.clock = clock
        self.state = {}
        self.updated = None
        # Retain by elapsed time: 600 samples cover only 12 seconds at 50 Hz.
        self.history = deque()
        self.history_seconds = 120
        self.last_packet = None

    def update(self, value):
        self.state = value
        self.updated = self.clock()
        self._expire_history(self.updated)
        packet = value.get('packet')
        if isinstance(packet, dict) and value.get('fresh'):
            key = (packet.get('seq'), packet.get('uptime_ms'))
            raw = packet.get('precise_adc', packet.get('amplitude_adc'))
            filtered = packet.get('filtered_adc', raw)
            if (key != self.last_packet and all(type(v) in (int,float) and math.isfinite(v) and 0 <= v <= 255 for v in (raw,filtered))):
                self.history.append((self.updated, raw, filtered))
                self.last_packet = key

    def _expire_history(self, now):
        while self.history and now - self.history[0][0] > self.history_seconds:
            self.history.popleft()

    def snapshot(self):
        now = self.clock()
        self._expire_history(now)
        available = self.updated is not None and now - self.updated < 1.0
        age = self.state.get('age_ms')
        if age is not None and self.updated is not None:
            age += round((now - self.updated) * 1000)
        return {**self.state, 'available': available,
                'fresh': available and self.state.get('fresh', False), 'age_ms': age,
                'history': [{'age_s': round(now - at, 3), 'adc': raw, 'filtered_adc': filtered}
                            for at, raw, filtered in self.history]}
