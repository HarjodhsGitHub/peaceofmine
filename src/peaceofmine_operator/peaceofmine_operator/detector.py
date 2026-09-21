"""Bounded detector history and freshness for dashboard telemetry."""

from collections import deque
import time


class DetectorTelemetry:
    def __init__(self, clock=time.monotonic):
        self.clock = clock
        self.state = {}
        self.updated = None
        self.history = deque(maxlen=600)
        self.last_packet = None

    def update(self, value):
        self.state = value
        self.updated = self.clock()
        packet = value.get('packet')
        if isinstance(packet, dict) and value.get('fresh'):
            key = (packet.get('seq'), packet.get('uptime_ms'))
            raw = packet.get('amplitude_adc')
            if key != self.last_packet and type(raw) is int and 0 <= raw <= 255:
                self.history.append((self.updated, raw))
                self.last_packet = key

    def snapshot(self):
        now = self.clock()
        available = self.updated is not None and now - self.updated < 1.0
        age = self.state.get('age_ms')
        if age is not None and self.updated is not None:
            age += round((now - self.updated) * 1000)
        return {**self.state, 'available': available,
                'fresh': available and self.state.get('fresh', False), 'age_ms': age,
                'history': [{'age_s': round(now - at, 3), 'adc': raw}
                            for at, raw in self.history if now - at <= 30]}
