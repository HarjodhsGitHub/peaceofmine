"""SVEA interlock using the firmware's existing MAVROS heartbeat and RC streams."""
import time


class RcSafety:
    TIMEOUTS = {'state': 1.5, 'rc': 0.4}

    def __init__(self, simulation=False, clock=time.monotonic):
        self.simulation = simulation
        self.clock = clock
        self.samples = {}

    def _update(self, topic, message):
        stamp = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
        old = self.samples.get(topic)
        if stamp <= 0:
            self.samples.pop(topic, None)
            return
        if old and stamp <= old[2]:
            if stamp < old[2]:
                self.samples.clear()
            return
        self.samples[topic] = (self.clock(), message, stamp)

    def update_state(self, message):
        # A disconnect must revoke permission even if it repeats the last stamp.
        if not message.connected:
            self.samples.clear()
            return
        self._update('state', message)

    def update_rc(self, message):
        self._update('rc', message)

    def servo_snapshot(self):
        if self.simulation:
            return dict(allowed=True, reason='Simulation interlock')
        sample = self.samples.get('state')
        if sample is None or self.clock() - sample[0] > self.TIMEOUTS['state']:
            return dict(allowed=False, reason='PX4 disconnected or status stale')
        status = sample[1].system_status
        if status != 4:
            return dict(allowed=False, reason=f'PX4 status {status}: servo movement disabled')
        rc_sample = self.samples.get('rc')
        if rc_sample is None or self.clock() - rc_sample[0] > self.TIMEOUTS['rc']:
            return dict(allowed=False, reason='RC input missing or stale')
        rc = rc_sample[1]
        if len(rc.channels) < 5 or rc.rssi == 0:
            return dict(allowed=False, reason='RC signal unavailable')
        pwm = rc.channels[4]
        allowed = 800 <= pwm <= 1100 or 1300 <= pwm <= 1700
        return dict(allowed=allowed, reason='RC permits servo movement' if allowed else 'RC kill or unverified switch position')

    def controls(self, steering_channel=1, throttle_channel=2):
        sample = self.samples.get('rc')
        if sample is None or self.clock() - sample[0] > self.TIMEOUTS['rc'] or sample[1].rssi == 0:
            return dict(fresh=False)
        channels = list(sample[1].channels)
        def axis(channel):
            if not 1 <= channel <= len(channels):
                return 0.0
            value = max(-1.0, min(1.0, (channels[channel - 1] - 1500) / 500))
            return 0.0 if abs(value) < .05 else value
        return dict(fresh=True, channels=channels, steering=axis(steering_channel),
                    throttle=axis(throttle_channel), steering_channel=steering_channel,
                    throttle_channel=throttle_channel)

    def snapshot(self):
        if self.simulation:
            return dict(mode='simulation', allowed=True, servo_allowed=True, reason='Simulation interlock', simulated=True)
        def result(mode, reason, allowed=False):
            return dict(mode=mode, reason=reason, allowed=allowed, simulated=False,
                        servo_allowed=self.servo_snapshot()['allowed'])
        now = self.clock()
        state_sample = self.samples.get('state')
        if state_sample is None or now - state_sample[0] > self.TIMEOUTS['state']:
            return result('unknown', 'MAVROS disconnected or state stale: check mavros/state and hardware launch')
        state = state_sample[1]
        # SVEA HEARTBEAT.hpp explicitly maps kill/lockdown/termination to 8.
        if state.system_status in (6, 8):
            return result('kill', 'PX4 kill / lockdown / termination')
        rc_sample = self.samples.get('rc')
        if rc_sample is None or now - rc_sample[0] > self.TIMEOUTS['rc']:
            return result('rc_lost', 'RC input missing or stale: check mavros/rc/in')
        rc = rc_sample[1]
        if len(rc.channels) < 5 or rc.rssi == 0:
            return result('rc_lost', 'RC channels unavailable or signal lost')
        pwm = rc.channels[4]  # Documented SVEA board default: SWB on CH5.
        if 1800 <= pwm <= 2200:
            return result('kill', 'RC kill switch')
        if 1300 <= pwm <= 1700:
            return result('override', 'RC override: remote drives; UI servo controls available when permitted')
        # A narrow low-end band stays inside PX4 mode slot 1; transitions deny.
        if not 800 <= pwm <= 1100:
            return result('unknown', f'RC CH5 is not in a verified switch position ({pwm} us)')
        if not state.armed:
            return result('ros_disarmed', 'ROS mode selected; PX4 disarmed')
        if state.system_status != 4:
            return result('unknown', 'PX4 is not active (failsafe, calibration, or other inhibited state)')
        return result('ros', 'ROS mode; PX4 armed', True)
