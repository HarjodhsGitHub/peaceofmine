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
from std_msgs.msg import Bool, Float32, Float64, String, UInt16
from nav_msgs.msg import Odometry
from peaceofmine_operator.detector import DetectorTelemetry
from peaceofmine_interfaces.msg import ActuatorStatus, AdcSamples
from peaceofmine_operator import configuration
from peaceofmine_operator.actuator_protocol import status_dict
from peaceofmine_operator.settings_client import SettingsClient
from peaceofmine_operator.node_runner import run_node


class DetectorNode(Node):
    def __init__(self):
        super().__init__('detector')
        self.threshold = float(self.declare_parameter('detector_threshold_ratio', 1.0).value)
        self.odometry_topic = str(self.declare_parameter('odometry_topic', 'odometry/local').value)
        self.config_file = str(self.declare_parameter('config_file', '').value or configuration.default_path())
        self.adc_channel = int(self.declare_parameter('adc_channel', 7).value)  # 7 = single-ended AIN3
        self.adc_topic = str(self.declare_parameter('adc_topic', 'adc/samples').value)

        # Calibration parameters
        self.settings = SettingsClient(self, self.config_file)
        self.baseline = 0.0
        self.full_response = 255.0
        self.reference_voltage = 5.0
        self.load_calibration()
        self.initial_calibration = (self.baseline, self.full_response, self.reference_voltage)

        self.sensor = DetectorTelemetry()
        self.signal = self.angle = self.speed = 0.0
        self.signal_at = 0.0
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
        self.create_subscription(AdcSamples, self.adc_topic, self.adc_samples_cb, 10)
        channel_name = f'a{self.adc_channel - 4}' if self.adc_channel >= 4 else f'mux{self.adc_channel}'
        self.create_subscription(Float64, f'adc/{channel_name}/scaled', self.adc_scaled_cb, 10)

        # Fallback / coordination subscriptions
        self.create_subscription(String, 'detector/state', self.sensor_cb, 10)
        self.create_subscription(String, 'detector/command', self.command_cb, 10)
        self.create_subscription(ActuatorStatus, 'arm/state', self.arm_cb, 10)
        self.create_subscription(Float32, 'detector/signal_ratio', self.signal_cb, 10)
        self.create_subscription(Float32, 'fixture/angle_deg', self.angle_cb, 10)
        self.create_subscription(Bool, 'fixture/sweep_enabled_state', lambda msg: setattr(self, 'sweep', msg.data), 10)
        self.create_subscription(Float32, 'fixture/sweep_speed_deg_s_state', lambda msg: setattr(self, 'sweep_speed', msg.data), 10)
        self.create_subscription(Odometry, self.odometry_topic, self.odometry_cb, 10)

        self.create_timer(0.05, self.publish)
        self.create_timer(1.0, self.reload_calibration)

    def load_calibration(self):
        try:
            saved = configuration.load(self.config_file).get('metal_detector', {})
            if saved:
                self.baseline = float(saved.get('baseline_adc', self.baseline))
                self.full_response = float(saved.get('full_response_adc', self.full_response))
                self.reference_voltage = float(saved.get('reference_voltage', self.reference_voltage))
        except (ValueError, OSError, TypeError):
            pass

    def reload_calibration(self):
        self.load_calibration()

    def adc_samples_cb(self, msg: AdcSamples):
        if not msg.valid:
            return
        for sample in msg.samples:
            if sample.channel == self.adc_channel:
                self.process_adc_sample(sample.scaled, sample.raw, sample.volts, sample.sequence)

    def adc_scaled_cb(self, msg: Float64):
        self.process_adc_sample(msg.data)

    def process_adc_sample(self, scaled_val, raw_val=None, volts=None, seq=None):
        now = time.monotonic()
        self.seq = seq if seq is not None else (self.seq + 1)
        self.sample_times.append(now)

        # Determine raw count equivalent (nominal 0..255 range to preserve UI display expectations)
        if raw_val is not None and volts is not None:
            ref = self.reference_voltage if self.reference_voltage > 0 else 5.0
            raw_count = max(0.0, min(255.0, (volts / ref) * 255.0))
        elif isinstance(scaled_val, (int, float)):
            if self.full_response > 5.0 and scaled_val <= 5.0:
                ref = self.reference_voltage if self.reference_voltage > 0 else 5.0
                raw_count = max(0.0, min(255.0, (scaled_val / ref) * 255.0))
            else:
                raw_count = float(scaled_val)
        else:
            return

        self.last_raw_adc = raw_count
        self.recent_samples.append((now, raw_count))

        # Calculate normalized ratio (0.0 to 1.0)
        denom = self.full_response - self.baseline
        ratio = (raw_count - self.baseline) / denom if abs(denom) > 1e-6 else 0.0
        normalized = max(0.0, min(1.0, ratio))

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

    def odometry_cb(self, msg):
        q = msg.pose.pose.orientation
        self.pose = dict(x=msg.pose.pose.position.x, y=msg.pose.pose.position.y,
            yaw=math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)))
        self.pose_at = time.monotonic()

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
        if detected and not self.detected and now - self.pose_at < 0.5:
            self.hits.append(dict(
                x=self.pose['x'],
                y=self.pose['y'],
                heading_deg=math.degrees(self.pose['yaw']) + self.angle,
                signal=self.signal
            ))
            self.hits = self.hits[-20:]
        self.detected = detected

    def _publish_detector_state(self):
        now = time.monotonic()
        recent = [t for t in self.sample_times if now - t <= 2.0]
        rate = (len(recent) - 1) / (recent[-1] - recent[0]) if len(recent) > 1 and recent[-1] > recent[0] else 0.0
        fresh = (now - self.signal_at < 0.5) if self.signal_at else False

        channel_name = f'AIN{self.adc_channel - 4}' if self.adc_channel >= 4 else f'MUX{self.adc_channel}'
        state_data = dict(
            connected=True,
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
            error=None,
            reference_voltage=self.reference_voltage,
            command_result=self.command_result
        )
        self.state_pub.publish(String(data=json.dumps(state_data, allow_nan=False)))

    def publish(self):
        arm = self.arm if time.monotonic() - self.arm_at < 0.5 else {}
        fresh = time.monotonic() - self.signal_at < 0.5
        self._publish_detector_state()

        now = time.monotonic()
        recent = [t for t in self.sample_times if now - t <= 2.0]
        rate_hz = round((len(recent) - 1) / (recent[-1] - recent[0]), 1) if len(recent) > 1 and recent[-1] > recent[0] else 0.0
        channel_name = f'AIN{self.adc_channel - 4}' if self.adc_channel >= 4 else f'MUX{self.adc_channel}'

        sensor_info = dict(
            available=True,
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
            command_result=self.command_result
        )

        self.publisher.publish(String(data=json.dumps(dict(detector=dict(
            fresh=fresh, sensor=sensor_info, signal_ratio=self.signal,
            threshold_ratio=self.threshold, detected=fresh and self.detected,
            fixture_angle_deg=self.angle, sweep_enabled=self.sweep, sweep_speed_deg_s=self.sweep_speed,
            sweep_speed_min=0.684 if arm.get('connected') else 5.0,
            sweep_speed_max=arm.get('motion_speed_limit', 180 / 0.684) * 0.684,
            beam_half_angle_deg=45, beam_min_angle_deg=self.lower, beam_max_angle_deg=self.upper,
            beam=self.beam), detections=self.hits), allow_nan=False)))


def main():
    run_node(DetectorNode)


if __name__ == '__main__':
    main()

