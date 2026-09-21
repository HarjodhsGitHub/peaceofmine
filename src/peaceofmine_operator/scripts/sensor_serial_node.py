#!/usr/bin/env python3
"""Publish Arduino sensor telemetry without blocking the ROS executor."""

import math
import json
import time
from collections import deque
import signal
import threading

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Bool, Float32, UInt16, String

from peaceofmine_operator.sensor_serial import SensorSerial


class SensorSerialNode(Node):
    def __init__(self):
        super().__init__('sensor_serial')
        defaults = dict(serial_port='/dev/ttyACM0', baud_rate=115200,
                        stale_timeout=0.5, reconnect_interval=1.0,
                        baseline_adc=0.0, full_response_adc=255.0, reference_voltage=5.0,
                        amplitude_topic='detector/amplitude_adc',
                        signal_topic='detector/signal_ratio',
                        fresh_topic='detector/fresh')
        values = {key: self.declare_parameter(key, value).value
                  for key, value in defaults.items()}
        self.baseline = float(values['baseline_adc'])
        self.full_response = float(values['full_response_adc'])
        self.reference_voltage = float(values['reference_voltage'])
        if not math.isfinite(self.reference_voltage) or not 0 < self.reference_voltage <= 5.5:
            raise ValueError('ADC reference voltage must be within 0..5.5 V')
        if (not all(math.isfinite(v) and 0 <= v <= 255
                    for v in (self.baseline, self.full_response))
                or self.baseline == self.full_response):
            raise ValueError('Calibration endpoints must be distinct and within 0..255')
        for key in ('stale_timeout', 'reconnect_interval'):
            if not math.isfinite(values[key]) or values[key] <= 0:
                raise ValueError(f'{key} must be finite and positive')
        if not values['serial_port'] or values['baud_rate'] <= 0:
            raise ValueError('Set a serial port and positive baud rate')
        self.sensor = SensorSerial(values['serial_port'], values['baud_rate'],
                                   values['stale_timeout'], values['reconnect_interval'])
        self.raw_pub = self.create_publisher(UInt16, values['amplitude_topic'], 10)
        self.ratio_pub = self.create_publisher(Float32, values['signal_topic'], 10)
        self.fresh_pub = self.create_publisher(Bool, values['fresh_topic'], 10)
        self.previous_status = None
        self.initial_calibration = (self.baseline, self.full_response, self.reference_voltage)
        self.recent_samples = deque()
        self.zero_window_seconds = 10.0
        self.latest_packet = None
        self.received = 0
        self.command_result = None
        self.state_pub = self.create_publisher(String, 'detector/state', 10)
        self.create_subscription(String, 'detector/command', self.calibrate, 10)
        self.create_timer(0.2, self.publish_state)
        self.create_timer(0.02, self.poll)

    def poll(self):
        samples = self.sensor.poll()
        if samples:
            now = time.monotonic()
            self.received += len(samples)
            self.latest_packet = {key: samples[-1][key] for key in ('v', 'seq', 'uptime_ms', 'amplitude_adc')}
            for sample in samples:
                self.recent_samples.append((now, sample['amplitude_adc']))
            self.trim_samples(now)
            # Publish the newest sample after a scheduling delay, not a backlog.
            raw = samples[-1]['amplitude_adc']
            ratio = (raw - self.baseline) / (self.full_response - self.baseline)
            self.raw_pub.publish(UInt16(data=raw))
            self.ratio_pub.publish(Float32(data=max(0.0, min(1.0, ratio))))
            self.publish_state()
        fresh = self.sensor.fresh
        self.fresh_pub.publish(Bool(data=fresh))
        status = (fresh, self.sensor.error)
        if status != self.previous_status:
            self.previous_status = status
            if fresh:
                self.get_logger().info('Sensor telemetry is fresh')
            else:
                self.get_logger().warning(self.sensor.error or 'Waiting for valid sensor telemetry')

    def trim_samples(self, now):
        while self.recent_samples and now - self.recent_samples[0][0] > self.zero_window_seconds:
            self.recent_samples.popleft()

    def calibrate(self, message):
        request_id = None
        try:
            command = json.loads(message.data)
            if not isinstance(command, dict):
                raise ValueError('Expected calibration object')
            request_id = command.get('request_id')
            if not isinstance(request_id, str) or len(request_id) > 100:
                raise ValueError('Invalid request ID')
            if not self.sensor.fresh:
                raise ValueError('Fresh Arduino readings required')
            baseline, full = self.baseline, self.full_response
            reference = self.reference_voltage
            action = command.get('action')
            if action == 'zero':
                now = time.monotonic()
                self.trim_samples(now)
                recent = [raw for _, raw in self.recent_samples]
                if len(recent) < 5:
                    raise ValueError('Wait for at least five recent readings')
                baseline = sum(recent) / len(recent)
            elif action == 'apply':
                baseline, full = command.get('baseline_adc'), command.get('full_response_adc')
                reference = command.get('reference_voltage', reference)
            elif action == 'reset':
                baseline, full, reference = self.initial_calibration
            else:
                raise ValueError('Unknown calibration action')
            if (not all(type(v) in (int, float) and math.isfinite(v) and 0 <= v <= 255
                        for v in (baseline, full)) or abs(full - baseline) < 1):
                raise ValueError('Endpoints must be within 0..255 and at least one count apart')
            if type(reference) not in (int, float) or not math.isfinite(reference) or not 0 < reference <= 5.5:
                raise ValueError('ADC reference voltage must be within 0..5.5 V')
            self.baseline, self.full_response = float(baseline), float(full)
            self.reference_voltage = float(reference)
            message = 'Zero level and detection trigger applied'
            if action == 'zero':
                span = now - self.recent_samples[0][0]
                message = f'Zero level {baseline:.2f} ADC: averaged {len(recent)} readings over {span:.1f} s'
            self.command_result = dict(request_id=request_id, ok=True, message=message)
            self.get_logger().info(f'Detector calibration: zero={baseline:.2f}, trigger={full:.2f}')
        except (ValueError, TypeError) as exc:
            self.command_result = dict(request_id=request_id, ok=False, message=str(exc))
        self.publish_state()

    def publish_state(self):
        now = time.monotonic()
        self.trim_samples(now)
        recent = [at for at, _ in self.recent_samples if now - at <= 2.0]
        rate = (len(recent) - 1) / (recent[-1] - recent[0]) if len(recent) > 1 and recent[-1] > recent[0] else 0.0
        value = dict(connected=self.sensor.connection is not None, fresh=self.sensor.fresh,
                     port=self.sensor.port, baud_rate=self.sensor.baudrate,
                     age_ms=round((now - self.sensor.last_sample) * 1000) if self.sensor.last_sample is not None else None,
                     rate_hz=round(rate, 1) if self.sensor.fresh else 0.0,
                     received=self.received, invalid_lines=self.sensor.invalid_lines,
                     packet=self.latest_packet, baseline_adc=self.baseline,
                     full_response_adc=self.full_response, error=self.sensor.error,
                     reference_voltage=self.reference_voltage,
                     command_result=self.command_result)
        self.state_pub.publish(String(data=json.dumps(value, allow_nan=False)))

    def destroy_node(self):
        self.sensor.close()
        super().destroy_node()


def main():
    stop = threading.Event()
    # Launch can forward SIGINT twice, including during node cleanup.
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop.set())
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = None
    try:
        node = SensorSerialNode()
        while not stop.is_set():
            rclpy.spin_once(node, timeout_sec=0.05)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
