"""Two confirmation passes followed by a small, sequential probe scan."""
from collections import deque
import math
import time
from .sweep_arc import sweep_arc


# Fixed mounting geometry already used by manual detector-to-probe alignment.
PROBE_OFFSET_DEG = 45.


class InspectionWorkflow:
    def __init__(self, execute, cancel, clear_peak, clock=time.monotonic):
        self.execute, self.cancel_io, self.clear_peak, self.clock = execute, cancel, clear_peak, clock
        self.generation = 0
        self.phase, self.message = 'IDLE', 'Search while driving, then confirm a detection.'
        self.confirmed_angle = None
        self.lower = self.upper = None
        self.completed = self.total = 0
        self.progress = 0.
        self.confirm_speed = None
        self.pass_from = self.pass_to = None
        self.points, self.passes = [], []
        self.arc_overlap = None
        self.queue = deque()
        self.pending = None
        self.samples = []
        self.pass_polarity = None
        self.pass_started = 0.
        self.angle = None
        self.wait_started = self.clock()

    @property
    def active(self):
        return self.phase in ('CONFIRMING', 'PROBING')

    def snapshot(self):
        return dict(phase=self.phase, active=self.active, message=self.message,
                    completed=self.completed, total=self.total, progress=self.progress,
                    confirmed_angle_deg=self.confirmed_angle, arc_overlap_ratio=self.arc_overlap,
                    probe_reachable=self.confirmed_angle is not None and self.lower <= self.confirmed_angle-PROBE_OFFSET_DEG <= self.upper, points=list(self.points), passes=list(self.passes))

    def confirm(self, lower, upper, speed):
        if not all(math.isfinite(v) for v in (lower, upper)) or lower >= upper:
            raise ValueError('Calibrated arm limits are required')
        if type(speed) not in (int, float) or not math.isfinite(speed) or speed <= 0:
            raise ValueError('Current search sweep speed is unavailable or invalid')
        self._begin('CONFIRMING')
        self.arc_overlap = None
        self.confirm_speed = speed
        self.progress = None
        self.lower, self.upper = lower, upper
        self.confirmed_angle = None
        self.passes, self.points = [], []
        self.total = 2
        self.queue.extend([('start', lower), ('pass', upper), ('pass', lower)])
        self.message = f'Moving to sweep start at {speed:g}°/s.'

    def probe(self, lower, upper, depth):
        if self.confirmed_angle is None:
            raise ValueError('Confirm a detection before starting the probe scan')
        if not all(math.isfinite(v) for v in (lower, upper, depth)) or lower >= upper or depth <= 0:
            raise ValueError('Valid arm limits and maximum probe depth are required')
        center = self.confirmed_angle-PROBE_OFFSET_DEG
        if not lower <= center <= upper:
            raise ValueError('Confirmed target is outside probe reach. Reposition the rover and confirm again.')
        angles = sorted({round(max(lower, min(upper, center+offset)), 4)
                         for offset in (-10, -5, 0, 5, 10)})
        self._begin('PROBING')
        self.lower, self.upper = lower, upper
        self.points = []
        self.total = len(angles)
        for angle in angles:
            self.queue.extend([('position', angle), ('probe', depth)])
        self.message = f'Probing {self.total} positions around {self.confirmed_angle:.1f}°.'

    def _begin(self, phase):
        if self.active:
            raise ValueError('Stop the current inspection first')
        self.generation += 1
        self.phase, self.completed = phase, 0
        self.progress = 0.
        self.pass_from = self.pass_to = None
        self.queue.clear()
        self.pending = None
        self.samples = []
        self.pass_polarity = None
        self.wait_started = self.clock()

    def stop(self, message='Inspection stopped', error=False):
        if not self.active and self.confirmed_angle is None:
            return
        self.generation += 1
        self.queue.clear()
        self.pending = None
        self.confirmed_angle = None
        self.phase, self.message = ('ERROR' if error else 'STOPPED'), message
        self.cancel_io()

    def tick(self, idle, sample):
        if not self.active:
            return
        if self.pending == 'pass' and sample and sample.get('fresh') and sample.get('angle_fresh'):
            stamp = sample.get('sample_time')
            keys = ('angle_deg', 'volts')
            if (type(stamp) in (int, float) and math.isfinite(stamp) and stamp >= self.pass_started
                    and all(type(sample.get(k)) in (int, float) and math.isfinite(sample[k]) for k in keys)
                    and (not self.samples or stamp > self.samples[-1]['sample_time'])):
                polarity = sample.get('response_direction')
                if polarity not in (-1, 1):
                    self.stop('Detector response direction unavailable', error=True)
                    return
                if self.pass_polarity is not None and polarity != self.pass_polarity:
                    self.stop('Detector response direction changed; confirm again', error=True)
                    return
                self.pass_polarity = polarity
                self.samples.append({k: sample[k] for k in (*keys, 'sample_time')})
                fraction = (sample['angle_deg']-self.pass_from)/(self.pass_to-self.pass_from)
                # Reserve completion for the action result and arc validation.
                self.progress = max(self.progress, self.completed+max(0., min(.999, fraction)))
        if self.pending is not None:
            return
        if not idle:
            if self.clock()-self.wait_started > 2:
                self.stop('Servo did not release after stopping; check the driver.', error=True)
            return
        if not self.queue:
            return
        kind, target = self.queue.popleft()
        self.pending = kind
        if kind == 'pass':
            self.clear_peak()
            self.pass_started = self.clock()
            self.samples, self.pass_polarity = [], None
            self.pass_from = self.lower if self.completed == 0 else self.upper
            self.pass_to = target
            self.progress = float(self.completed)
            self.message = f'Confirmation pass {self.completed+1} of 2 at {self.confirm_speed:g}°/s.'
        elif kind == 'position':
            self.angle = target
            self.message = f'Position {self.completed+1} of {self.total}: moving to {target+PROBE_OFFSET_DEG:.1f}°.'
        elif kind == 'probe':
            self.message = f'Position {self.completed+1} of {self.total}: probing at {self.angle+PROBE_OFFSET_DEG:.1f}°.'
        generation = self.generation

        def complete(result):
            if generation != self.generation:
                return
            self.pending = None
            self.wait_started = self.clock()
            if not result.get('success'):
                self.stop(result.get('message') or 'Actuator operation failed', error=True)
                return
            if kind == 'pass':
                try:
                    arc = sweep_arc(self.samples, self.pass_polarity)
                except ValueError as exc:
                    self.stop(f'Pass {self.completed+1}: {exc}', error=True)
                    return
                self.passes.append(arc)
                self.completed += 1
                if self.completed == 2:
                    left = max(p['left_deg'] for p in self.passes)
                    right = min(p['right_deg'] for p in self.passes)
                    span = max(p['right_deg'] for p in self.passes)-min(p['left_deg'] for p in self.passes)
                    # Match relative footprint overlap instead of a fixed angular cutoff.
                    self.arc_overlap = max(0., right-left)/span
                    if self.arc_overlap < .5:
                        self.stop(f'The two arcs have only {self.arc_overlap:.0%} overlap. Reposition or repeat confirmation.', error=True)
                        return
                    self.confirmed_angle = sum(p['angle_deg'] for p in self.passes)/2
                    self.phase = 'CONFIRMED'
                    self.message = f'Arc matched at {self.confirmed_angle:.1f}° ({self.arc_overlap:.0%} overlap across both passes).'
                self.progress = float(self.completed)
            elif kind == 'probe':
                contact = result.get('contact') is True
                depth = result.get('contact_depth_mm') if contact else None
                if contact and (type(depth) not in (int,float) or not math.isfinite(depth) or depth < 0):
                    self.stop('Probe returned an invalid contact depth', error=True)
                    return
                self.points.append(dict(angle_deg=self.angle+PROBE_OFFSET_DEG, status='CONTACT' if contact else 'CLEAR',
                                        contact_depth_mm=depth))
                self.completed += 1
                self.progress = float(self.completed)
                if not self.queue:
                    self.phase = 'COMPLETE'
                    hits = sum(p['status'] == 'CONTACT' for p in self.points)
                    self.message = f'Probe scan complete: contact at {hits} of {self.total} positions.'

        try:
            self.execute('probe' if kind == 'probe' else 'arm',
                         'probe_cycle' if kind == 'probe' else 'angle', target,
                         self.confirm_speed if self.phase == 'CONFIRMING' else 5., complete)
        except (ValueError, RuntimeError) as exc:
            self.stop(str(exc), error=True)
