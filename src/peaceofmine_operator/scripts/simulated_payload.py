#!/usr/bin/env python3
"""Simulation-only detector and probe sources.

The dashboard gateway consumes the topics created here exactly as it will
consume real drivers later.  Keeping this node separate from the gateway is
intentional: replacing it with hardware must not alter browser safety or UI
contracts.
"""

from __future__ import annotations

import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool, Float32


class SimulatedPayload(Node):
    def __init__(self) -> None:
        super().__init__('simulated_payload')
        self.declare_parameter('odometry_topic', 'odometry/local')
        self.declare_parameter('probe_target_topic', 'probe/target_depth_mm')
        self.declare_parameter('detector_signal_topic', 'detector/signal_ratio')
        self.declare_parameter('probe_depth_topic', 'probe/depth_mm')
        self.declare_parameter('probe_pressure_topic', 'probe/pressure_ratio')
        self.declare_parameter('fixture_angle_topic', 'fixture/angle_deg')
        self.declare_parameter('fixture_sweep_enabled_topic', 'fixture/sweep_enabled')
        self.declare_parameter('fixture_sweep_speed_topic', 'fixture/sweep_speed_deg_s')
        self.declare_parameter('fixture_sweep_enabled_state_topic', 'fixture/sweep_enabled_state')
        self.declare_parameter('fixture_sweep_speed_state_topic', 'fixture/sweep_speed_deg_s_state')
        self.declare_parameter('probe_fault_topic', 'probe/fault')
        self.declare_parameter('max_probe_depth_mm', 110.0)
        self.declare_parameter('probe_speed_mm_s', 35.0)
        self.declare_parameter('fixture_angle_offset_deg', 0.0)
        self.declare_parameter('fixture_sweep_half_angle_deg', 45.0)
        self.declare_parameter('fixture_sweep_speed_deg_s', 60.0)

        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0
        self._speed = 0.0
        self._target_depth = 0.0
        self._depth = 0.0
        self._max_depth = float(self.get_parameter('max_probe_depth_mm').value)
        self._depth_rate = float(self.get_parameter('probe_speed_mm_s').value)
        self._fixture_offset = float(self.get_parameter('fixture_angle_offset_deg').value)
        self._fixture_half_angle = float(self.get_parameter('fixture_sweep_half_angle_deg').value)
        self._fixture_angle = self._fixture_offset
        self._fixture_sweep_enabled = True
        self._fixture_sweep_speed = float(self.get_parameter('fixture_sweep_speed_deg_s').value)
        self._fixture_sweep_direction = 1.0

        self.create_subscription(
            Odometry, str(self.get_parameter('odometry_topic').value), self._odometry, 10)
        self.create_subscription(
            Float32, str(self.get_parameter('probe_target_topic').value), self._probe_target, 10)
        self.create_subscription(
            Bool, str(self.get_parameter('fixture_sweep_enabled_topic').value), self._sweep_enabled, 10)
        self.create_subscription(
            Float32, str(self.get_parameter('fixture_sweep_speed_topic').value), self._sweep_speed, 10)
        self._signal_pub = self.create_publisher(
            Float32, str(self.get_parameter('detector_signal_topic').value), 10)
        self._depth_pub = self.create_publisher(
            Float32, str(self.get_parameter('probe_depth_topic').value), 10)
        self._pressure_pub = self.create_publisher(
            Float32, str(self.get_parameter('probe_pressure_topic').value), 10)
        self._fixture_angle_pub = self.create_publisher(
            Float32, str(self.get_parameter('fixture_angle_topic').value), 10)
        self._sweep_enabled_state_pub = self.create_publisher(
            Bool, str(self.get_parameter('fixture_sweep_enabled_state_topic').value), 10)
        self._sweep_speed_state_pub = self.create_publisher(
            Float32, str(self.get_parameter('fixture_sweep_speed_state_topic').value), 10)
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
        self._speed = message.twist.twist.linear.x

    def _probe_target(self, message: Float32) -> None:
        self._target_depth = max(0.0, min(self._max_depth, float(message.data)))

    def _sweep_enabled(self, message: Bool) -> None:
        self._fixture_sweep_enabled = bool(message.data)

    def _sweep_speed(self, message: Float32) -> None:
        self._fixture_sweep_speed = max(5.0, min(180.0, float(message.data)))

    def _publish(self) -> None:
        # The fixture is an actual simulated mechanism, not a browser effect.
        # It scans +/- 45 degrees around the forward-arm offset only while the
        # rover advances, and publishes its measured angle for the UI and later
        # hardware replacement.
        if self._fixture_sweep_enabled:
            self._fixture_angle += self._fixture_sweep_direction * self._fixture_sweep_speed * 0.05
            lower, upper = self._fixture_offset - self._fixture_half_angle, self._fixture_offset + self._fixture_half_angle
            if self._fixture_angle >= upper:
                self._fixture_angle, self._fixture_sweep_direction = upper, -1.0
            elif self._fixture_angle <= lower:
                self._fixture_angle, self._fixture_sweep_direction = lower, 1.0
        fixture_angle = self._fixture_angle

        # Two hidden targets make detector behaviour repeatable. The detector
        # response depends on its physical forward field of view, including the
        # fixture's published angle, rather than on anything in the dashboard.
        signal = 0.02
        detector_heading = self._yaw + math.radians(fixture_angle)
        for target_x, target_y, amplitude, radius in ((4.8, 0.8, 92.0, 1.2), (9.2, -1.3, 78.0, 0.9)):
            dx, dy = target_x - self._x, target_y - self._y
            distance_squared = dx ** 2 + dy ** 2
            bearing = math.atan2(dy, dx)
            bearing_error = math.atan2(math.sin(bearing - detector_heading), math.cos(bearing - detector_heading))
            forward_response = math.exp(-(bearing_error ** 2) / (2.0 * math.radians(18.0) ** 2))
            signal += (amplitude / 100.0) * forward_response * math.exp(-distance_squared / (2.0 * radius ** 2))

        self._depth += max(-self._depth_rate * 0.05,
                           min(self._depth_rate * 0.05, self._target_depth - self._depth))

        # Soil resistance rises gradually with depth. A probe that is close to a
        # simulated target sees a distinctly steeper increase after contact.
        soil_pressure = 0.04 + self._depth / self._max_depth * 0.30
        mine_distance = math.hypot(self._x - 4.8, self._y - 0.8)
        contact_pressure = max(0.0, self._depth - 38.0) / (self._max_depth - 38.0) * 0.66 if mine_distance < 0.55 else 0.0
        pressure_ratio = min(1.0, soil_pressure + contact_pressure)

        self._signal_pub.publish(Float32(data=min(1.0, signal)))
        self._depth_pub.publish(Float32(data=self._depth))
        self._pressure_pub.publish(Float32(data=pressure_ratio))
        self._fixture_angle_pub.publish(Float32(data=fixture_angle))
        self._sweep_enabled_state_pub.publish(Bool(data=self._fixture_sweep_enabled))
        self._sweep_speed_state_pub.publish(Float32(data=self._fixture_sweep_speed))
        self._fault_pub.publish(Bool(data=False))


def main() -> None:
    rclpy.init()
    node = SimulatedPayload()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
