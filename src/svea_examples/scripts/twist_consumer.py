#! /usr/bin/env python3

import math
import time

from svea_core import rosonic as rx
from svea_core.interfaces import ActuationInterface
from svea_core.controllers.crawl import CrawlController
from geometry_msgs.msg import Twist, TwistWithCovarianceStamped
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool


class twist_consumer(rx.Node):
    """Twist (v, omega) -> SVEA actuation (steering angle, velocity)."""

    twist_top  = rx.Parameter('cmd_vel')
    twist_type = rx.Parameter('geometry_msgs/msg/Twist')

    wheelbase    = rx.Parameter(0.324)   # [m]
    max_steering = rx.Parameter(0.524)   # [rad]
    max_velocity = rx.Parameter(0.8)     # [m/s]
    min_speed_for_steering = rx.Parameter(0.05)

    velocity_sign = rx.Parameter(1.0)
    steering_sign = rx.Parameter(1.0)

    cmd_timeout  = rx.Parameter(0.6)     # [s]
    use_difflock = rx.Parameter(True)
    pump_top     = rx.Parameter('actuation/pump')
    crawl_enabled = rx.Parameter(False)
    crawl_speed = rx.Parameter(0.1)
    crawl_max_throttle = rx.Parameter(0.2)  # Fraction, not m/s
    crawl_kp = rx.Parameter(0.3)
    crawl_ki = rx.Parameter(0.2)
    crawl_feedback_topic = rx.Parameter('mavros/wheel_odometry/velocity')

    actuation = ActuationInterface()

    _velocity = 0.0
    _steering = 0.0
    _last_msg = None
    _pump_until = None
    _crawl = None

    def on_startup(self):
        if self.twist_type not in ("geometry_msgs/msg/Twist",
                                   "geometry_msgs/msg/TwistStamped"):
            raise TypeError(f'Invalid message type for {self.twist_top}: {self.twist_type}')

        if not math.isfinite(self.min_speed_for_steering) or self.min_speed_for_steering <= 0:
            raise ValueError('min_speed_for_steering must be finite and positive')

        if self.crawl_enabled:
            self._crawl = CrawlController(max_speed=self.crawl_speed,
                                          max_throttle=self.crawl_max_throttle,
                                          kp=self.crawl_kp, ki=self.crawl_ki)
            self._wheel_speed = math.nan
            self._wheel_at = None
            self._wheel_stamp = None
            self.create_subscription(TwistWithCovarianceStamped, self.crawl_feedback_topic,
                                     self.wheel_speed_cb, qos_profile_sensor_data)
            self.get_logger().warn('Experimental crawl regulator enabled; validate wheel speed before ground use')

        if self.use_difflock:
            self.actuation.enable_difflock()

        self._last_msg = self.get_clock().now()
        self.get_logger().info(
            f'twist_consumer listening on "{self.twist_top}" ({self.twist_type})')

    def _omega_to_steering(self, v, w):
        """Ackermann steering with a speed floor so steering also works at rest."""
        speed = max(abs(v), self.min_speed_for_steering)
        delta = math.atan(self.wheelbase * w / speed)
        return max(-self.max_steering, min(self.max_steering, delta))

    def wheel_speed_cb(self, msg):
        now = self.get_clock().now().nanoseconds
        stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        speed = msg.twist.twist.linear.x
        if (not math.isfinite(speed) or not 0 <= (now-stamp)*1e-9 <= .25
                or (self._wheel_stamp is not None and stamp <= self._wheel_stamp)):
            self._wheel_speed = math.nan
            return
        self._wheel_speed = speed
        self._wheel_at = time.monotonic()
        self._wheel_stamp = stamp

    def on_shutdown(self):
        self.actuation.send_xtr(xtr1=0.0)
        self.actuation.send_control(0.0, 0.0)
        self.actuation.loop()  # Publish neutral before ROS publishers are destroyed.

    @rx.Subscriber(Bool, pump_top)
    def pump_cb(self, msg: Bool):
        self._pump_until = self.get_clock().now().nanoseconds * 1e-9 + 0.25 if msg.data else None
        self.actuation.send_xtr(xtr1=100.0 if msg.data else 0.0)

    @rx.Subscriber(twist_type, twist_top)
    def twist_cb(self, msg):
        tw = msg if isinstance(msg, Twist) else msg.twist

        v = float(tw.linear.x)
        w = float(tw.angular.z)
        if not math.isfinite(v) or not math.isfinite(w):
            v = w = 0.0
        v = max(-self.max_velocity, min(self.max_velocity, v))
        if getattr(self, '_crawl', None) is not None:
            v = max(-self.crawl_speed, min(self.crawl_speed, v))

        self._velocity = v
        self._steering = self._omega_to_steering(v, w)
        self._last_msg = self.get_clock().now()

    @rx.Timer(0.1)
    def loop(self):
        if self._pump_until is not None and self.get_clock().now().nanoseconds * 1e-9 >= self._pump_until:
            self._pump_until = None
            self.actuation.send_xtr(xtr1=0.0)
        age = (self.get_clock().now() - self._last_msg).nanoseconds * 1e-9
        if age > self.cmd_timeout:
            self._velocity = 0.0
            self._steering = 0.0

        velocity = self._velocity
        crawl = getattr(self, '_crawl', None)
        if crawl is not None:
            now = time.monotonic()
            feedback_age = math.inf if self._wheel_at is None else now-self._wheel_at
            previous_fault = crawl.fault
            throttle = crawl.update(velocity, self._wheel_speed, feedback_age, now)
            if crawl.fault and crawl.fault != previous_fault:
                self.get_logger().warn(f'Crawl stopped: {crawl.fault}')
            # ActuationInterface converts m/s back to a throttle fraction.
            velocity = throttle * self.actuation.max_speed
        self.actuation.send_control(self._steering * self.steering_sign,
                                    velocity * self.velocity_sign)


if __name__ == '__main__':
    twist_consumer.main()
