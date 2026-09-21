#!/usr/bin/env python3
"""Publish Arduino sensor telemetry without blocking the ROS executor."""

import math
import signal
import threading

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Bool, Float32, UInt16

from peaceofmine_operator.sensor_serial import SensorSerial


class SensorSerialNode(Node):
    def __init__(self):
        super().__init__('sensor_serial')
        defaults = dict(serial_port='/dev/ttyACM0', baud_rate=115200,
                        stale_timeout=0.5, reconnect_interval=1.0,
                        baseline_adc=0.0, full_response_adc=255.0,
                        amplitude_topic='detector/amplitude_adc',
                        signal_topic='detector/signal_ratio',
                        fresh_topic='detector/fresh')
        values = {key: self.declare_parameter(key, value).value
                  for key, value in defaults.items()}
        self.baseline = float(values['baseline_adc'])
        self.full_response = float(values['full_response_adc'])
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
        self.create_timer(0.02, self.poll)

    def poll(self):
        samples = self.sensor.poll()
        if samples:
            # Publish the newest sample after a scheduling delay, not a backlog.
            raw = samples[-1]['amplitude_adc']
            ratio = (raw - self.baseline) / (self.full_response - self.baseline)
            self.raw_pub.publish(UInt16(data=raw))
            self.ratio_pub.publish(Float32(data=max(0.0, min(1.0, ratio))))
        fresh = self.sensor.fresh
        self.fresh_pub.publish(Bool(data=fresh))
        status = (fresh, self.sensor.error)
        if status != self.previous_status:
            self.previous_status = status
            if fresh:
                self.get_logger().info('Sensor telemetry is fresh')
            else:
                self.get_logger().warning(self.sensor.error or 'Waiting for valid sensor telemetry')

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
