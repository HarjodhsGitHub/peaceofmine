"""Bounded ROS ADC telemetry cache for independent browser sample cursors."""
from collections import deque
import copy
import threading
import time


class ADCTelemetry:
    def __init__(self, clock=time.monotonic):
        self.clock = clock
        self.lock = threading.RLock()
        self.updated = None
        self.state = None
        self.samples = deque(maxlen=30000)

    def update(self, value):
        with self.lock:
            if self.state is None or (value['session'], value['revision']) != (
                    self.state['session'], self.state['revision']):
                self.samples.clear()
            last = self.samples[-1]['seq'] if self.samples else 0
            self.samples.extend(sample for sample in value['samples'] if sample['seq'] > last)
            while self.samples and value['now'] - self.samples[0]['time'] > 60:
                self.samples.popleft()
            self.state = {key: val for key, val in value.items() if key != 'samples'}
            self.updated = self.clock()

    def snapshot(self, after=0, session=None):
        with self.lock:
            if self.state is None:
                return None
            result = copy.deepcopy(self.state)
            fresh = self.clock() - self.updated < 1
            result['now'] += self.clock() - self.updated
            if after > result['sequence'] or (session is not None and session != result['session']):
                after = 0
            samples = [sample for sample in self.samples if sample['seq'] > after][:5000]
            result.update(samples=samples, available=fresh,
                          dropped=max(0, samples[0]['seq'] - after - 1) if samples and after else 0)
            if not fresh:
                result.update(connected=False, error='ADC ROS node unavailable')
                result['probe_contact'].update(fresh=False, detected=None, value=None)
            return result
