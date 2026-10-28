"""Probe-only load homing and millimetre-to-encoder travel control."""
import math
import time
from .servo_motor import ServoMotor


class ProbeMotor(ServoMotor):
    def request_home(self, request, load_percent):
        # Repeated deadman packets must never restart a completed/failed search.
        if not isinstance(request, str) or not 1 <= len(request) <= 100:
            raise ValueError('Homing requires a unique hold identifier')
        if request == self._home_request:
            return
        if (isinstance(load_percent, bool) or not isinstance(load_percent, (int, float))
                or not math.isfinite(load_percent) or not 10 <= load_percent <= 60):
            raise ValueError('Home load threshold must be 10..60 percent')
        self.stop()
        self._home_request = request
        self.home_position = None
        self.request_jog(1, 50.0)
        self.max_step = 114  # 10 degree lead supports 50 degrees/s at the host update rate.
        self._home = dict(start=time.monotonic(), threshold=load_percent)
        self.home_status = 'Seeking top at 50 degrees/s; release to stop'


    def _check_home(self):
        search = self._home
        now = time.monotonic()
        if now - search['start'] >= 60:
            self.stop()
            self.home_status = 'Home failed: 60 second timeout; zero not set'
            return False
        if self._home_contact(self.bus.read(self.ident, 40)):
            return False
        if self.position >= self.ccw - 3:
            self.stop()
            self.home_status = 'Home failed: position range limit reached without confirmed resistance'
            return False
        return True


    def _home_contact(self, raw):
        if self._home is None:
            return False
        if not 0 <= raw <= 2047:
            raise ValueError('Invalid load telemetry during homing')
        # Direction bit is irrelevant to contact. Do not wait for the encoder
        # to stop: slipping/skipping teeth can keep the shaft moving at a stop.
        if (raw & 1023) * 100 / 1023 >= self._home['threshold']:
            position = self.position
            self.stop()
            self.home_position = position
            self.home_status = 'Load threshold reached; torque released and zero set. Verify top visually.'
            return True
        return False


    def request_extension(self, request, depth, calibration):
        if not isinstance(request, str) or not request:
            raise ValueError('Probe target requires a request identifier')
        if request == self._extension_request:
            return
        if self.home_position is None:
            raise ValueError('Home the probe first')
        if not calibration or calibration['servo_id'] != self.ident:
            raise ValueError('Save maximum probe extension in Settings first')
        maximum = calibration['max_extension_mm']
        if isinstance(depth, bool) or not isinstance(depth, (int, float)) or not math.isfinite(depth) or not 0 <= depth <= maximum:
            raise ValueError('Probe target exceeds saved maximum extension')
        low, high = self.home_position - calibration['travel_ticks'], self.home_position
        if not self.cw <= low < high <= self.ccw:
            raise ValueError('Saved probe travel exceeds current position range; home/calibrate again')
        self.request_position(round(high - depth / maximum * calibration['travel_ticks']))
        self.jog_limits = (low, high)
        self.speed = 73  # Approximately 50 degrees/s, never register zero.
        self.max_step = 114
        self._extension_request = request
        self._extension_active = True


