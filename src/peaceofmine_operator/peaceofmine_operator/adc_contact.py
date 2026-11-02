"""Probe-owned threshold evaluation, with hysteresis, debounce, and FSR spike detection."""
import math
import time


class AdcContact:
    def __init__(self, clock=time.monotonic):
        self.clock = clock
        self.config = dict(enabled=False, mux=5, level=1., direction='above', hysteresis=0., debounce_ms=0.,
                           spike_threshold=0.3)
        self.value = None
        self.received = 0.
        self.detected = False
        self.spike_detected = False
        self.spike_detected_at = 0.
        self.candidate = None
        self.since = 0.
        self.session = None
        self.sequence = -1
        self.max_age = .5
        self.history = []

    def configure(self, config, max_age=.5):
        if config != self.config:
            self.received = 0.
            self.candidate = None
            self.detected = False
            self.spike_detected = False
            self.history.clear()
        self.config = dict(config)
        self.max_age = max(.5, max_age)

    def sample(self, value, sequence, session):
        if session != self.session:
            self.session, self.sequence = session, -1
            self.detected, self.candidate, self.spike_detected = False, None, False
            self.history.clear()
        if sequence <= self.sequence or not math.isfinite(value):
            return
        now = self.clock()
        if now-self.received > self.max_age:
            self.detected, self.candidate, self.spike_detected = False, None, False
            self.history.clear()
        self.sequence, self.value, self.received = sequence, value, now

        # Maintain short-term history for FSR slope / spike detection
        self.history = [(t, v) for t, v in self.history if now - t <= 0.5]
        spike_thresh = float(self.config.get('spike_threshold', 0.3))
        # Find past sample from 50ms to 200ms ago to detect sudden impact against soil baseline
        past_samples = [v for t, v in self.history if 0.05 <= (now - t) <= 0.25]
        if past_samples:
            past_val = past_samples[0]
            sign = 1 if self.config.get('direction', 'above') == 'above' else -1
            delta = sign * (value - past_val)
            if delta >= spike_thresh:
                self.spike_detected = True
                self.spike_detected_at = now
        elif now - self.spike_detected_at > 0.5:
            self.spike_detected = False

        self.history.append((now, value))

        # Standard static threshold check
        config = self.config
        sign = 1 if config.get('direction', 'above') == 'above' else -1
        distance = sign*(value-config.get('level', 1.0))
        state = distance >= (-config.get('hysteresis',0.) if self.detected else 0.)
        if state != self.candidate:
            self.candidate, self.since = state, now
        if now-self.since >= config.get('debounce_ms',0.)/1000:
            self.detected = state

    def snapshot(self):
        valid = self.config.get('enabled', False) and self.value is not None and self.clock()-self.received < self.max_age
        is_hit = (self.detected or self.spike_detected) if valid else None
        return dict(enabled=self.config.get('enabled', False), fresh=valid, detected=is_hit,
            spike_detected=self.spike_detected if valid else False,
            value=self.value if valid else None, mux=self.config.get('mux', 5), level=self.config.get('level', 1.0),
            direction=self.config.get('direction', 'above'),
            spike_threshold=self.config.get('spike_threshold', 0.3))
