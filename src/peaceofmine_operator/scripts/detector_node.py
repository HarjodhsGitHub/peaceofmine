#!/usr/bin/env python3
"""Detector signal interpretation, sweep plot and georeferenced hit history."""
import json
import math
import time
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String
from nav_msgs.msg import Odometry
from peaceofmine_operator.detector import DetectorTelemetry
from peaceofmine_interfaces.msg import ActuatorStatus
from peaceofmine_operator.actuator_protocol import status_dict
from peaceofmine_operator.node_runner import run_node


class DetectorNode(Node):
    def __init__(self):
        super().__init__('detector')
        self.threshold = self.declare_parameter('detector_threshold_ratio', 1.0).value
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
        self.publisher = self.create_publisher(String, 'detector/telemetry', 10)
        self.create_subscription(String, 'detector/state', self.sensor_cb, 10)
        self.create_subscription(String, 'detector/command', self.command_cb, 10)
        self.create_subscription(ActuatorStatus, 'arm/state', self.arm_cb, 10)
        self.create_subscription(Float32, 'detector/signal_ratio', self.signal_cb, 10)
        self.create_subscription(Float32, 'fixture/angle_deg', self.angle_cb, 10)
        self.create_subscription(Bool, 'fixture/sweep_enabled_state', lambda msg: setattr(self, 'sweep', msg.data), 10)
        self.create_subscription(Float32, 'fixture/sweep_speed_deg_s_state', lambda msg: setattr(self, 'sweep_speed', msg.data), 10)
        self.create_subscription(Odometry, self.declare_parameter('odometry_topic', 'odometry/local').value, self.odometry_cb, 10)
        self.create_timer(.05, self.publish)

    def command_cb(self, msg):
        try:
            value = json.loads(msg.data)
            if value.get('action') == 'apply':
                self.threshold_request = value.get('request_id')
        except (ValueError, AttributeError):
            pass

    def sensor_cb(self, msg):
        try:
            value = json.loads(msg.data)
            self.sensor.update(value)
            if value.get('unit') == 'V' and (not value.get('fresh') or value.get('calibration_required')):
                self.signal_at = 0.
                self.detected = False
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
        self.signal, self.signal_at = max(0., min(1., msg.data)), time.monotonic()
        if self.lower <= self.angle <= self.upper:
            self.beam[round((self.angle-self.lower)/(self.upper-self.lower)*(len(self.beam)-1))] = self.signal
        detected = self.signal >= self.threshold
        if detected and not self.detected and time.monotonic()-self.pose_at < .5:
            self.hits.append(dict(x=self.pose['x'], y=self.pose['y'],
                heading_deg=math.degrees(self.pose['yaw'])+self.angle, signal=self.signal))
            self.hits = self.hits[-20:]
        self.detected = detected

    def publish(self):
        arm = self.arm if time.monotonic()-self.arm_at < .5 else {}
        fresh = time.monotonic()-self.signal_at < .5
        self.publisher.publish(String(data=json.dumps(dict(detector=dict(
            fresh=fresh, sensor=self.sensor.snapshot(), signal_ratio=self.signal,
            threshold_ratio=self.threshold, detected=fresh and self.detected,
            fixture_angle_deg=self.angle, sweep_enabled=self.sweep, sweep_speed_deg_s=self.sweep_speed,
            sweep_speed_min=.684 if arm.get('connected') else 5.,
            sweep_speed_max=arm.get('motion_speed_limit', 180/.684)*.684,
            beam_half_angle_deg=45, beam_min_angle_deg=self.lower, beam_max_angle_deg=self.upper,
            beam=self.beam), detections=self.hits), allow_nan=False)))


def main():
    run_node(DetectorNode)


if __name__ == '__main__':
    main()
