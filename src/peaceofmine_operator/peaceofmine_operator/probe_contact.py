"""Session-only probe contact telemetry; never interprets motor output as force."""
from collections import deque
import math


class ProbeContact:
    def __init__(self):
        self.samples = deque(maxlen=700)
        self.key = None
        self.baseline = None
        self.holding = False
        self.valid = False
        self.filtered = None

    def update(self, state, now):
        values = [state.get(k) for k in ('sample_time', 'load_percent', 'current_a', 'position', 'goal_position')]
        valid = (state.get('connected') and state.get('active_role') == 'probe'
                 and all(type(v) in (int, float) and math.isfinite(v) for v in values))
        if not valid:
            self.__init__()
            return
        stamp, load, current, position, goal = values
        key = (state.get('serial_port'), state.get('servo_id'), stamp)
        if self.key and (key[:2] != self.key[:2] or stamp < self.key[2]):
            self.__init__()
        self.valid = True
        self.holding = bool(state.get('torque') and state.get('probe', {}).get('holding'))
        if key == self.key:
            return
        self.key = key
        base_load, base_current = self.baseline or (0, 0)
        magnitude = (abs(load - base_load), abs(current - base_current) * 1000)
        if self.filtered is None or (self.samples and now - self.samples[-1][0] > .3):
            self.filtered = magnitude
        else:
            self.filtered = tuple(.5 * v + .5 * old for v, old in zip(magnitude, self.filtered))
        self.samples.append((now, load, current, goal - position, self.holding, *self.filtered))

    def can_zero(self, now):
        recent = [s for s in self.samples if now - s[0] <= .6]
        last_motion = max((i for i, sample in enumerate(recent) if not sample[4]), default=-1)
        recent = recent[last_motion + 1:]
        return bool(self.valid and self.holding and recent and now - recent[-1][0] < .3
                    and len(recent) >= 5 and recent[-1][0] - recent[0][0] >= .45 and all(s[4] for s in recent))

    def zero(self, now):
        if not self.can_zero(now):
            raise ValueError('Hold the probe clear of contact for at least half a second with fresh telemetry')
        recent = [s for s in self.samples if now - s[0] <= .5]
        self.baseline = (sum(s[1] for s in recent) / len(recent), sum(s[2] for s in recent) / len(recent))
        self.samples.clear()
        self.filtered = None

    def snapshot(self, now):
        fresh = bool(self.valid and self.samples and now - self.samples[-1][0] < .3)
        base_load, base_current = self.baseline or (0, 0)
        latest = self.samples[-1] if fresh else None
        recent = [s for s in self.samples if now - s[0] <= 2]
        return dict(contact_rate_hz=(len(recent) - 1) / (recent[-1][0] - recent[0][0]) if fresh and len(recent) > 1 and recent[-1][0] > recent[0][0] else 0,
                    contact_fresh=fresh, contact_zeroed=self.baseline is not None,
                    contact_can_zero=self.can_zero(now),
                    current_a=latest[2] if latest else None,
                    current_change_ma=abs(latest[2] - base_current) * 1000 if latest else None,
                    contact_load_percent=abs(latest[1] - base_load) if latest else None,
                    position_error_ticks=latest[3] if latest else None,
                    contact_peak_percent=max((abs(s[1] - base_load) for s in recent), default=None) if fresh else None,
                    current_peak_ma=max((abs(s[2] - base_current) * 1000 for s in recent), default=None) if fresh else None,
                    contact_samples=[dict(x=s[0] - now, load=s[5], current=s[6]) for s in self.samples if now - s[0] <= 30])
