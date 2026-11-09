"""Opt-in, bounded speed-magnitude regulator; no ROS or hardware dependencies."""
import math


class CrawlController:
    """Return signed throttle fractions, not speed commands.

    Feedback may be unsigned (the SVEA wheel encoder cannot sense direction).
    Never apply a reversing/braking pulse. Faults latch until a zero request.
    Gains and limits require vehicle-specific validation before ground use.
    """

    def __init__(self, max_speed=.1, max_throttle=.2, kp=.3, ki=.2,
                 feedback_timeout=.25, stall_timeout=2., stop_speed=.02):
        values = (max_speed, max_throttle, kp, ki, feedback_timeout,
                  stall_timeout, stop_speed)
        if not all(math.isfinite(v) and v > 0 for v in values):
            raise ValueError('Crawl parameters must be finite and positive')
        if max_throttle > .9 or stop_speed >= max_speed:
            raise ValueError('Invalid crawl throttle limit or stop speed')
        self.max_speed, self.max_throttle = max_speed, max_throttle
        self.kp, self.ki = kp, ki
        self.feedback_timeout, self.stall_timeout = feedback_timeout, stall_timeout
        self.stop_speed = stop_speed
        self.fault = ''
        self.integral = 0.
        self.direction = 0
        self.last_time = None
        self.stalled_since = None

    def update(self, request, speed, feedback_age, now):
        dt = 0. if self.last_time is None else max(0., min(.1, now-self.last_time))
        self.last_time = now
        if not math.isfinite(request):
            self.fault = 'invalid command'
        elif request == 0:
            self.fault = ''
            self.integral = 0.
            self.stalled_since = None
            # Keep the previous direction until a measured stop permits reversal.
            return 0.
        if self.fault:
            return 0.
        if (not all(math.isfinite(v) for v in (speed, feedback_age, now))
                or not 0 <= feedback_age <= self.feedback_timeout):
            self.fault = 'missing or stale wheel speed'
            return 0.
        speed = abs(speed)
        if speed > self.max_speed * 1.5:
            self.fault = 'crawl overspeed'
            return 0.
        direction = 1 if request > 0 else -1
        if direction != self.direction:
            self.integral = 0.
            self.stalled_since = None
            if speed > self.stop_speed:
                return 0.
            self.direction = direction
        if speed < self.stop_speed:
            if self.stalled_since is None:
                self.stalled_since = now
            elif now-self.stalled_since >= self.stall_timeout:
                self.fault = 'no wheel motion; release drive input'
                self.integral = 0.
                return 0.
        else:
            self.stalled_since = None
        error = min(abs(request), self.max_speed) - speed
        candidate = self.integral + self.ki * error * dt
        output = self.kp * error + candidate
        # Conditional integration prevents windup at either throttle bound.
        if 0 <= output <= self.max_throttle or (output > self.max_throttle and error < 0) or (output < 0 and error > 0):
            self.integral = candidate
        return direction * max(0., min(self.max_throttle, self.kp*error+self.integral))
