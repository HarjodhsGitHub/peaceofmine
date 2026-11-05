#!/usr/bin/env python3
"""Detector signal interpretation, sweep plot, and georeferenced hit history.

NOTE: The legacy Arduino Uno and ESP32 USB-serial transports (sensor_serial)
are deprecated and no longer used. The metal detector signal is now acquired
directly by the Raspberry Pi over I2C via the onboard ADS1115 ADC.
"""
from collections import deque
import json
import math
import time

from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String, UInt16
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistWithCovarianceStamped
from rclpy.qos import qos_profile_sensor_data
from peaceofmine_operator.detector import DetectorTelemetry
from peaceofmine_interfaces.msg import ActuatorStatus, AdcSamples
from peaceofmine_operator import configuration
from peaceofmine_operator.actuator_protocol import status_dict
from peaceofmine_operator.settings_client import SettingsClient
from peaceofmine_operator.node_runner import run_node


class DetectorNode(Node):
    def __init__(self, **kwargs):
        super().__init__('detector', **kwargs)
        self.threshold = float(self.declare_parameter('detector_threshold_ratio', 1.0).value)
        self.odometry_topic = str(self.declare_parameter('odometry_topic', 'odometry/local').value)
        self.config_file = str(self.declare_parameter('config_file', '').value or configuration.default_path())
        self.use_adc = bool(self.declare_parameter('use_adc', True).value)
        self.adc_channel = 6
        self.adc_revision = 0
        self.adc_session = None
        self.adc_sequence = -1
        self.configuration_error = None
        self.adc_topic = str(self.declare_parameter('adc_topic', 'adc/samples').value)

        # Calibration parameters
        self.settings = SettingsClient(self, self.config_file)
        self.baseline = 0.0
        self.full_response = 255.0
        self.reference_voltage = 5.0
        self.reload_calibration()
        self.initial_calibration = (self.baseline, self.full_response, self.reference_voltage)

        self.sensor = DetectorTelemetry()
        self.signal = self.angle = self.speed = 0.0
        self.signal_at = 0.0
        self.angle_at = 0.0
        self.lower, self.upper = -45.0, 45.0
        self.beam = [0.0] * 91
        self.sweep = False
        self.sweep_speed = 60.0
        self.arm = {}
        self.arm_at = 0.0
        self.pose = dict(x=0., y=0., yaw=0.)
        self.pose_at = 0.0
        self.detected = False
        self.hits = []
        self.latest_peak = None
        self.threshold_request = None

        # Sample tracking for zeroing and rate statistics
        self.recent_samples = deque(maxlen=200)
        self.last_raw_adc = 0.0
        self.seq = 0
        self.start_time = time.monotonic()
        self.sample_times = deque(maxlen=50)
        self.command_result = None

        # Publishers
        self.publisher = self.create_publisher(String, 'detector/telemetry', 10)
        self.ratio_pub = self.create_publisher(Float32, 'detector/signal_ratio', 10)
        self.fresh_pub = self.create_publisher(Bool, 'detector/fresh', 10)
        self.raw_pub = self.create_publisher(UInt16, 'detector/amplitude_adc', 10)
        self.state_pub = self.create_publisher(String, 'detector/state', 10)

        # Direct I2C ADC subscriptions
        if self.use_adc:
            self.create_subscription(AdcSamples, self.adc_topic, self.adc_samples_cb, 10)
        else:
            self.create_subscription(String, 'detector/state', self.sensor_cb, 10)

        # Coordination subscriptions
        self.create_subscription(String, 'detector/command', self.command_cb, 10)
        self.create_subscription(ActuatorStatus, 'arm/state', self.arm_cb, 10)
        if not self.use_adc:
            self.create_subscription(Float32, 'detector/signal_ratio', self.signal_cb, 10)
        self.create_subscription(Float32, 'fixture/angle_deg', self.angle_cb, 10)
        self.create_subscription(Bool, 'fixture/sweep_enabled_state', lambda msg: setattr(self, 'sweep', msg.data), 10)
        self.create_subscription(Float32, 'fixture/sweep_speed_deg_s_state', lambda msg: setattr(self, 'sweep_speed', msg.data), 10)
        self.create_subscription(Odometry, self.odometry_topic, self.odometry_cb, 10)
        velocity_topic = self.declare_parameter('velocity_topic', '').value
        if velocity_topic:
            self.create_subscription(TwistWithCovarianceStamped, velocity_topic, self.velocity_cb, qos_profile_sensor_data)

        self.create_timer(0.05, self.publish)
        self.create_timer(1.0, self.reload_calibration)

    def load_calibration(self):
        previous = (self.baseline, self.full_response, self.reference_voltage, self.adc_channel, self.adc_revision)
        saved = configuration.load(self.config_file)
        value = saved.get('metal_detector', dict(baseline_adc=0., full_response_adc=255., reference_voltage=5.))
        base, full, ref = (value[k] for k in ('baseline_adc', 'full_response_adc', 'reference_voltage'))
        if (not all(type(v) in (int, float) and math.isfinite(v) for v in (base, full, ref))
                or not (0 <= base <= 255 and 0 <= full <= 255 and abs(full-base) >= 1 and 0 < ref <= 5.5)):
            raise ValueError('Invalid saved detector calibration')
        self.baseline, self.full_response, self.reference_voltage = float(base), float(full), float(ref)
        adc = saved.get('adc', {})
        channels = adc.get('config', {}).get('channels', [])
        routes = adc.get('routes', ['none'] * 8)
        assigned = [c['mux'] for c in channels if c['enabled'] and routes[c['mux']] in ('detector', 'both')] if 'adc' in saved else [6]
        if self.use_adc and len(assigned) != 1:
            raise ValueError('Assign exactly one enabled ADC input to the metal detector in Settings')
        if assigned:
            self.adc_channel = assigned[0]
        self.adc_revision = saved.get('section_revisions', {}).get('adc', 0)
        self.configuration_error = None
        if previous != (self.baseline, self.full_response, self.reference_voltage, self.adc_channel, self.adc_revision) and hasattr(self, 'latest_peak'):
            self.latest_peak = None
            self.signal_at = 0.
            self.detected = False
            self.recent_samples.clear()
            self.beam = [0.] * len(self.beam)

    def reload_calibration(self):
        try:
            self.load_calibration()
        except (ValueError, KeyError, TypeError, OSError) as exc:
            self.configuration_error = str(exc)
            self.signal_at = 0.
            self.detected = False

    def adc_samples_cb(self, msg: AdcSamples):
        if not msg.valid:
            self.signal_at = 0.
            self.detected = False
            return
        if msg.config_revision != self.adc_revision:
            self.reload_calibration()
        if self.configuration_error or msg.config_revision != self.adc_revision:
            return
        if msg.session != self.adc_session:
            self.adc_session, self.adc_sequence = msg.session, -1
        for sample in msg.samples:
            age = time.time() - (sample.stamp.sec + sample.stamp.nanosec / 1e9)
            if sample.channel != self.adc_channel or not 0 <= age < .5 or sample.sequence <= self.adc_sequence:
                continue
            if not math.isfinite(sample.volts):
                continue
            self.adc_sequence = sample.sequence
            self.process_adc_sample(sample.volts, sample.sequence)

    def process_adc_sample(self, volts, seq):
        now = time.monotonic()
        self.seq = seq
        self.sample_times.append(now)
        raw_count = max(0., min(255., volts / self.reference_voltage * 255.))

        self.last_raw_adc = raw_count
        self.recent_samples.append((now, raw_count))

        ratio = (raw_count-self.baseline)/(self.full_response-self.baseline)
        normalized = max(0., min(1., ratio))
        int_adc = int(round(raw_count))
        self.sensor.update(dict(
            fresh=True,
            age_ms=0,
            packet=dict(seq=self.seq, uptime_ms=int((now - self.start_time) * 1000), amplitude_adc=int_adc)
        ))

        self.raw_pub.publish(UInt16(data=min(65535, max(0, int_adc))))
        self.ratio_pub.publish(Float32(data=normalized))
        self.fresh_pub.publish(Bool(data=True))

        self._update_signal(normalized, now)

    def command_cb(self, msg):
        request_id = None
        try:
            value = json.loads(msg.data)
            if not isinstance(value, dict):
                return
            request_id = value.get('request_id')
            action = value.get('action')

            if action == 'zero':
                now = time.monotonic()
                recent = [val for t, val in self.recent_samples if now - t <= 5.0]
                if len(recent) < 3:
                    raise ValueError('Wait for at least three recent I2C ADC readings before zeroing')
                new_baseline = sum(recent) / len(recent)

                def on_saved(ok, message):
                    if ok:
                        self.baseline = float(new_baseline)
                    self.command_result = dict(request_id=request_id, ok=ok, message=message)
                    self._publish_detector_state()

                self.settings.save('metal_detector', dict(
                    baseline_adc=float(new_baseline),
                    full_response_adc=float(self.full_response),
                    reference_voltage=float(self.reference_voltage)
                ), on_saved)
                return

            elif action == 'apply':
                self.threshold_request = request_id
                base = value.get('baseline_adc', self.baseline)
                full = value.get('full_response_adc', self.full_response)
                ref = value.get('reference_voltage', self.reference_voltage)
                if not all(type(v) in (int, float) and math.isfinite(v) for v in (base, full, ref)):
                    raise ValueError('Invalid calibration values')

                def on_saved(ok, message):
                    if ok:
                        self.baseline, self.full_response, self.reference_voltage = float(base), float(full), float(ref)
                        self.threshold = 1.0
                    self.command_result = dict(request_id=request_id, ok=ok, message=message)
                    self._publish_detector_state()

                self.settings.save('metal_detector', dict(
                    baseline_adc=float(base),
                    full_response_adc=float(full),
                    reference_voltage=float(ref)
                ), on_saved)
                return

            elif action == 'reset':
                base, full, ref = self.initial_calibration

                def on_saved(ok, message):
                    if ok:
                        self.baseline, self.full_response, self.reference_voltage = float(base), float(full), float(ref)
                    self.command_result = dict(request_id=request_id, ok=ok, message=message)
                    self._publish_detector_state()

                self.settings.save('metal_detector', dict(
                    baseline_adc=float(base),
                    full_response_adc=float(full),
                    reference_voltage=float(ref)
                ), on_saved)
                return

            raise ValueError('Unknown detector calibration action')

        except (ValueError, TypeError, OSError) as exc:
            self.command_result = dict(request_id=request_id, ok=False, message=str(exc))
            self._publish_detector_state()

    def sensor_cb(self, msg):
        try:
            value = json.loads(msg.data)
            self.sensor.update(value)
            result = value.get('command_result') or {}
            if self.threshold_request and result.get('request_id') == self.threshold_request:
                if result.get('ok'):
                    self.threshold = 1.0
                self.threshold_request = None
        except (ValueError, TypeError, AttributeError):
            pass

    def arm_cb(self, msg):
        try:
            value = status_dict(msg)
            calibration = value.get('arm_calibration') or value.get('calibration')
            self.arm, self.arm_at = value, time.monotonic()
            if calibration:
                low, center, high = (calibration.get(k) for k in ('minimum', 'center', 'maximum'))
                if all(type(v) is int for v in (low, center, high)) and 0 <= low < center < high <= 4095:
                    lower, upper = (low-center)*360/4096, (high-center)*360/4096
                    if (lower, upper) != (self.lower, self.upper):
                        self.lower, self.upper = lower, upper
                        self.beam = [0.] * (math.ceil(upper-lower)+1)
        except (ValueError, TypeError, AttributeError):
            pass

    def angle_cb(self, msg):
        if math.isfinite(msg.data):
            self.angle = msg.data
            self.angle_at = time.monotonic()

    def velocity_cb(self, msg):
        twist = msg.twist.twist
        values = (twist.linear.x, twist.linear.y, twist.linear.z, twist.angular.x, twist.angular.y, twist.angular.z)
        if not all(math.isfinite(v) and abs(v) <= .03 for v in values):
            self.latest_peak = None

    def odometry_cb(self, msg):
        q = msg.pose.pose.orientation
        self.pose = dict(x=msg.pose.pose.position.x, y=msg.pose.pose.position.y,
            yaw=math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)))
        self.pose_at = time.monotonic()
        if self.latest_peak:
            origin = self.latest_peak['pose']
            distance = math.hypot(self.pose['x']-origin['x'], self.pose['y']-origin['y'])
            yaw_change = abs(math.atan2(math.sin(self.pose['yaw']-origin['yaw']), math.cos(self.pose['yaw']-origin['yaw'])))
            if distance > .05 or yaw_change > math.radians(2):
                self.latest_peak = None

    def signal_cb(self, msg):
        if not math.isfinite(msg.data):
            return
        self._update_signal(msg.data, time.monotonic())

    def _update_signal(self, value, now):
        self.signal = max(0., min(1., float(value)))
        self.signal_at = now
        if self.lower <= self.angle <= self.upper and len(self.beam) > 1:
            idx = round((self.angle - self.lower) / (self.upper - self.lower) * (len(self.beam) - 1))
            if 0 <= idx < len(self.beam):
                self.beam[idx] = self.signal
        detected = self.signal >= self.threshold
        if detected and now - self.angle_at < .5:
            if not self.latest_peak or self.signal > self.latest_peak.get('signal', 0.0) or (now - self.latest_peak.get('timestamp', 0) > 30.0):
                self.latest_peak = dict(
                    angle_deg=round(self.angle, 1),
                    signal=round(self.signal, 3),
                    timestamp=now, pose=dict(self.pose)
                )
        if detected and not self.detected and now - self.pose_at < 0.5:
            self.hits.append(dict(
                x=self.pose['x'],
                y=self.pose['y'],
                heading_deg=math.degrees(self.pose['yaw']) + self.angle,
                fixture_angle_deg=round(self.angle, 1),
                signal=round(self.signal, 3)
            ))
            self.hits = self.hits[-20:]
        self.detected = detected

    def _publish_detector_state(self):
        if not self.use_adc:
            return
        now = time.monotonic()
        recent = [t for t in self.sample_times if now - t <= 2.0]
        rate = (len(recent) - 1) / (recent[-1] - recent[0]) if len(recent) > 1 and recent[-1] > recent[0] else 0.0
        fresh = (now - self.signal_at < 0.5) if self.signal_at else False

        channel_name = f'AIN{self.adc_channel - 4}' if self.adc_channel >= 4 else f'MUX{self.adc_channel}'
        state_data = dict(
            connected=fresh,
            fresh=fresh,
            port=f'I2C bus 1, 0x48 ({channel_name})',
            baud_rate=0,
            age_ms=round((now - self.signal_at) * 1000) if self.signal_at else None,
            rate_hz=round(rate, 1) if fresh else 0.0,
            received=self.seq,
            invalid_lines=0,
            packet=dict(
                seq=self.seq,
                uptime_ms=int((now - self.start_time) * 1000),
                amplitude_adc=int(round(self.last_raw_adc))
            ),
            baseline_adc=self.baseline,
            full_response_adc=self.full_response,
            error=self.configuration_error or (None if fresh else 'No fresh ADC samples; start acquisition and check A2 wiring'),
            reference_voltage=self.reference_voltage,
            command_result=self.command_result
        )
        self.state_pub.publish(String(data=json.dumps(state_data, allow_nan=False)))

    def publish(self):
        arm = self.arm if time.monotonic() - self.arm_at < 0.5 else {}
        fresh = self.signal_at > 0 and time.monotonic() - self.signal_at < 0.5 and not self.configuration_error
        self._publish_detector_state()

        now = time.monotonic()
        recent = [t for t in self.sample_times if now - t <= 2.0]
        rate_hz = round((len(recent) - 1) / (recent[-1] - recent[0]), 1) if len(recent) > 1 and recent[-1] > recent[0] else 0.0
        channel_name = f'AIN{self.adc_channel - 4}' if self.adc_channel >= 4 else f'MUX{self.adc_channel}'

        sensor_info = dict(
            available=bool(self.signal_at),
            fresh=fresh,
            device=f'I2C ADS1115 ({channel_name})',
            age_ms=round((now - self.signal_at) * 1000) if self.signal_at else None,
            rate_hz=rate_hz,
            received=self.seq,
            invalid_lines=0,
            baseline_adc=self.baseline,
            full_response_adc=self.full_response,
            reference_voltage=self.reference_voltage,
            packet=dict(
                seq=self.seq,
                uptime_ms=int((now - self.start_time) * 1000),
                amplitude_adc=int(round(self.last_raw_adc))
            ),
            history=self.sensor.snapshot().get('history', []),
            command_result=self.command_result, error=self.configuration_error or (None if fresh else 'No fresh ADC samples; start acquisition and check wiring')
        )
        if not self.use_adc:
            sensor_info = self.sensor.snapshot()
        peak = {**self.latest_peak, 'fresh': fresh and now-self.latest_peak['timestamp'] <= 30} if self.latest_peak else None

        self.publisher.publish(String(data=json.dumps(dict(detector=dict(
            fresh=fresh, sensor=sensor_info, signal_ratio=self.signal,
            threshold_ratio=self.threshold, detected=fresh and self.detected,
            fixture_angle_deg=self.angle, sweep_enabled=self.sweep, sweep_speed_deg_s=self.sweep_speed,
            sweep_speed_min=0.684 if arm.get('connected') else 5.0,
            sweep_speed_max=arm.get('motion_speed_limit', 180 / 0.684) * 0.684,
            beam_half_angle_deg=45, beam_min_angle_deg=self.lower, beam_max_angle_deg=self.upper,
            beam=self.beam, latest_peak=peak), detections=self.hits), allow_nan=False)))


def main():
    run_node(DetectorNode)


if __name__ == '__main__':
    main()

