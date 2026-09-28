#!/usr/bin/env python3
"""Simulation-only detector and soil-resistance sensor sources.

Publishes the same topics the hardware drivers will publish, so swapping in
real drivers does not touch the gateway or the dashboard.
"""

from __future__ import annotations

import math

from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool, Float32
from peaceofmine_operator.node_runner import run_node


class SimulatedPayload(Node):
    def __init__(self) -> None:
        super().__init__('simulated_payload')
        self.declare_parameter('odometry_topic', 'odometry/local')
        self.declare_parameter('detector_signal_topic', 'detector/signal_ratio')
        self.declare_parameter('probe_pressure_topic', 'simulation/probe_load_ratio')
        self.declare_parameter('probe_fault_topic', 'probe/fault')
        self.declare_parameter('max_probe_depth_mm', 110.0)
        self.declare_parameter('fixture_angle_offset_deg', 0.0)

        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0
        self._depth = 0.0
        self._max_depth = float(self.get_parameter('max_probe_depth_mm').value)
        self._fixture_offset = float(self.get_parameter('fixture_angle_offset_deg').value)
        self._fixture_angle = self._fixture_offset
        self.create_subscription(
            Odometry, str(self.get_parameter('odometry_topic').value), self._odometry, 10)
        self.create_subscription(Float32, 'probe/depth_mm', lambda msg: setattr(self, '_depth', msg.data), 10)
        self.create_subscription(Float32, 'fixture/angle_deg', lambda msg: setattr(self, '_fixture_angle', msg.data), 10)
        self._signal_pub = self.create_publisher(
            Float32, str(self.get_parameter('detector_signal_topic').value), 10)
        self._pressure_pub = self.create_publisher(
            Float32, str(self.get_parameter('probe_pressure_topic').value), 10)
        self._fault_pub = self.create_publisher(
            Bool, str(self.get_parameter('probe_fault_topic').value), 10)
        self.create_timer(0.05, self._publish)

    def _odometry(self, message: Odometry) -> None:
        self._x = message.pose.pose.position.x
        self._y = message.pose.pose.position.y
        orientation = message.pose.pose.orientation
        self._yaw = math.atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y ** 2 + orientation.z ** 2))

    def _publish(self) -> None:
        fixture_angle = self._fixture_angle

        # Two buried targets along the +X search lane make detector behaviour
        # repeatable. Response falls off with both bearing error and range.
        signal = 0.02
        detector_heading = self._yaw + math.radians(fixture_angle)
        for target_x, target_y, amplitude, radius in ((4.8, 0.8, 92.0, 1.2), (9.2, -1.3, 78.0, 0.9)):
            dx, dy = target_x - self._x, target_y - self._y
            distance_squared = dx ** 2 + dy ** 2
            bearing = math.atan2(dy, dx)
            bearing_error = math.atan2(math.sin(bearing - detector_heading), math.cos(bearing - detector_heading))
            forward_response = math.exp(-(bearing_error ** 2) / (2.0 * math.radians(18.0) ** 2))
            signal += (amplitude / 100.0) * forward_response * math.exp(-distance_squared / (2.0 * radius ** 2))

        # Soil resistance rises gradually with depth, and steeply once the
        # probe makes contact with a buried target.
        soil_pressure = 0.04 + self._depth / self._max_depth * 0.30
        mine_distance = math.hypot(self._x - 4.8, self._y - 0.8)
        contact_pressure = max(0.0, self._depth - 38.0) / (self._max_depth - 38.0) * 0.66 if mine_distance < 0.55 else 0.0
        pressure_ratio = min(1.0, soil_pressure + contact_pressure)

        self._signal_pub.publish(Float32(data=min(1.0, signal)))
        self._pressure_pub.publish(Float32(data=pressure_ratio))
        self._fault_pub.publish(Bool(data=False))


def main():
    run_node(SimulatedPayload)


if __name__ == '__main__':
    main()
