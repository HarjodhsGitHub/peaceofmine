#!/usr/bin/env python3
"""Browser-to-ROS gateway for the PeaceOfMine operator dashboard.

One browser holds the control lease. Physical RC state grants drive and servo
authority; command timeouts and the downstream SVEA watchdog remain independent
safety layers.
"""

from __future__ import annotations

import asyncio
import signal
import contextlib
import json
import math
import os
import socket
import ssl
import threading
import time
import re
from pathlib import Path
from typing import Any

try:
    import aiohttp
    from aiohttp import WSMsgType, web
except ImportError as error:  # pragma: no cover - depends on target image
    raise RuntimeError('aiohttp is required; install the workspace requirements first.') from error

import rclpy
from rclpy.signals import SignalHandlerOptions
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from mavros_msgs.msg import State, RCIn
from nav_msgs.msg import Odometry
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, BatteryState
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, Float32, String
from peaceofmine_operator.rc_safety import RcSafety
from peaceofmine_operator.power import PowerTelemetry
from peaceofmine_operator.camera_stream import CameraStreams
from peaceofmine_operator.detector import DetectorTelemetry
from peaceofmine_operator.probe_contact import ProbeContact

# Authoritative actuator limits. The dashboard reads these from telemetry so
# the browser never carries its own copy.
SWEEP_SPEED_MIN_DEG_S = 5.0
SWEEP_SPEED_MAX_DEG_S = 180.0

# Default sector before arm calibration arrives (also used by simulation).
BEAM_HALF_ANGLE_DEG = 45
BEAM_BINS = 2 * BEAM_HALF_ANGLE_DEG + 1

PRESSURE_SAMPLE_PERIOD_S = 0.2
PRESSURE_WINDOW_S = 30.0
PRESSURE_SAMPLES = int(PRESSURE_WINDOW_S / PRESSURE_SAMPLE_PERIOD_S)

TELEMETRY_PERIOD_S = 0.05

# After the drive command stops being valid, keep publishing zeros for this
# long and then go silent, so an idle gateway does not hold the downstream
# twist_consumer watchdog open or fight other cmd_vel publishers.
STOP_TAIL_S = 0.5


class OperatorGateway(Node):
    def __init__(self) -> None:
        super().__init__('operator_gateway')
        self.declare_parameter('safety_simulation', False)
        self.declare_parameter('host', '0.0.0.0')
        self.declare_parameter('port', 8080)
        self.declare_parameter('tls_cert', '')
        self.declare_parameter('tls_key', '')
        self.declare_parameter('cmd_vel_topic', 'cmd_vel')
        self.declare_parameter('joy_topic', 'joy')
        self.declare_parameter('odometry_topic', 'odometry/local')
        self.declare_parameter('detector_signal_topic', 'detector/signal_ratio')
        self.declare_parameter('probe_depth_topic', 'probe/depth_mm')
        self.declare_parameter('probe_pressure_topic', 'probe/pressure_ratio')
        self.declare_parameter('fixture_angle_topic', 'fixture/angle_deg')
        self.declare_parameter('fixture_sweep_enabled_topic', 'fixture/sweep_enabled')
        self.declare_parameter('fixture_sweep_speed_topic', 'fixture/sweep_speed_deg_s')
        self.declare_parameter('fixture_sweep_enabled_state_topic', 'fixture/sweep_enabled_state')
        self.declare_parameter('fixture_sweep_speed_state_topic', 'fixture/sweep_speed_deg_s_state')
        self.declare_parameter('probe_fault_topic', 'probe/fault')
        self.declare_parameter('probe_target_topic', 'probe/target_depth_mm')
        self.declare_parameter('camera_stream_base_url', 'http://127.0.0.1:8081')
        self.declare_parameter('front_camera_topic', '/self/camera_front/image_raw')
        self.declare_parameter('auxiliary_camera_topic', '/self/camera_auxiliary/image_raw')
        self.declare_parameter('front_camera_info_topic', '/self/camera_front/camera_info')
        self.declare_parameter('auxiliary_camera_info_topic', '/self/camera_auxiliary/camera_info')
        self.declare_parameter('command_timeout_s', 0.25)
        self.declare_parameter('max_velocity_mps', 0.8)
        self.declare_parameter('max_yaw_rate_rad_s', 1.4)
        self.declare_parameter('probe_motion_speed_limit_mps', 0.03)
        self.declare_parameter('max_probe_depth_mm', 110.0)
        self.declare_parameter('detector_threshold_ratio', 1.0)
        self.declare_parameter('rc_steering_channel', 1)
        self.declare_parameter('rc_throttle_channel', 2)

        self._lock = threading.RLock()
        self._power = PowerTelemetry()
        self.create_subscription(BatteryState, 'mavros/battery', self._battery_cb, qos_profile_sensor_data)
        # PX4 tunnel messages are optional on installations without px4_msgs.
        try:
            from px4_msgs.msg import EscStatus
        except ImportError:
            pass
        else:
            self.create_subscription(EscStatus, 'px4/uorb/esc_status', self._esc_cb, qos_profile_sensor_data)
        self._shutting_down = False
        self._rc_safety = RcSafety(bool(self.get_parameter('safety_simulation').value), clock=lambda: time.monotonic())
        self._arm_state = {'connected': False, 'reason': 'Arm driver not running'}
        self._arm_state_at = 0.0
        self._permission_pub = self.create_publisher(Bool, 'operator/actuation_enabled', 1)
        self._arm_command_pub = self.create_publisher(String, 'arm/command', 1)
        self.create_subscription(State, 'mavros/state', self._state_cb, 10)
        self.create_subscription(RCIn, 'mavros/rc/in', self._rc_cb, 10)
        self.create_subscription(String, 'arm/state', self._arm_state_cb, 1)
        self._lease: web.WebSocketResponse | None = None
        # Do not call this `_clients`: rclpy Node uses that private attribute
        # for ROS service clients while its executor spins.
        self._connected_sockets: set[web.WebSocketResponse] = set()
        self._calibrating = False
        self._armed = False
        self._deadman = False
        self._command = (0.0, 0.0)
        self._last_command = 0.0
        self._last_joy = 0.0
        self._joy_prev_a = False
        self._trigger_seen = {'lt': False, 'rt': False}
        self._joy = {
            'seen': False,
            'age_ms': None,
            'stick_x': 0.0,
            'stick_y': 0.0,
            'lt': 0.0,
            'rt': 0.0,
            'lb': False,
            'rb': False,
            'a': False,
            'b': False,
            'x': False,
            'y': False,
            'deadman': False,
        }
        self._stop_until = 0.0
        self._detector = 0.0
        self._detector_at = None
        self._detector_telemetry = DetectorTelemetry()
        self._detector_command_pub = self.create_publisher(String, 'detector/command', 10)
        self.create_subscription(String, 'detector/state', self._detector_state_cb, 10)
        self._probe_depth = 0.0
        self._probe_pressure_ratio = 0.0
        self._fixture_angle_deg = 0.0
        self._fixture_sweep_enabled = False
        self._fixture_sweep_speed = 60.0
        self._beam = [0.0] * BEAM_BINS
        self._beam_min = -BEAM_HALF_ANGLE_DEG
        self._beam_max = BEAM_HALF_ANGLE_DEG
        self._beam_calibration = None
        self._pressure_history = [0.0] * PRESSURE_SAMPLES
        self._last_pressure_sample = 0.0
        self._probe_fault = False
        self._probe_target = 0.0
        self._probe_motion = None
        self._probe_contact = ProbeContact()
        self._probe_load_history = []
        self._probe_load_sample = None
        self._probe_load_at = 0.0
        self._probe_load_ratio = None
        self._pose = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
        self._speed = 0.0
        self._was_detected = False
        self._detections: list[dict[str, float]] = []
        self._camera_frames: dict[str, dict[str, Any]] = {}
        self._camera_subscriptions: dict[str, Any] = {}

        self._max_velocity = float(self.get_parameter('max_velocity_mps').value)
        self._max_yaw_rate = float(self.get_parameter('max_yaw_rate_rad_s').value)
        self._timeout = float(self.get_parameter('command_timeout_s').value)
        self._probe_speed_limit = float(self.get_parameter('probe_motion_speed_limit_mps').value)
        self._max_probe_depth = float(self.get_parameter('max_probe_depth_mm').value)
        self._detector_threshold = float(self.get_parameter('detector_threshold_ratio').value)
        self._detector_threshold_request = None

        self._cmd_pub = self.create_publisher(Twist, str(self.get_parameter('cmd_vel_topic').value), 10)
        self._probe_pub = self.create_publisher(Float32, str(self.get_parameter('probe_target_topic').value), 10)
        self._sweep_enabled_pub = self.create_publisher(Bool, str(self.get_parameter('fixture_sweep_enabled_topic').value), 10)
        self._sweep_speed_pub = self.create_publisher(Float32, str(self.get_parameter('fixture_sweep_speed_topic').value), 10)
        self.create_subscription(Odometry, str(self.get_parameter('odometry_topic').value), self._odometry_cb, 10)
        self.create_subscription(Float32, str(self.get_parameter('detector_signal_topic').value), self._detector_cb, 10)
        self.create_subscription(Float32, str(self.get_parameter('probe_depth_topic').value), self._probe_depth_cb, 10)
        self.create_subscription(Float32, str(self.get_parameter('probe_pressure_topic').value), self._probe_pressure_cb, 10)
        self.create_subscription(Float32, str(self.get_parameter('fixture_angle_topic').value), self._fixture_angle_cb, 10)
        self.create_subscription(Bool, str(self.get_parameter('fixture_sweep_enabled_state_topic').value), self._sweep_enabled_cb, 10)
        self.create_subscription(Float32, str(self.get_parameter('fixture_sweep_speed_state_topic').value), self._sweep_speed_cb, 10)
        self.create_subscription(Bool, str(self.get_parameter('probe_fault_topic').value), self._probe_fault_cb, 10)
        # CameraInfo arrives once per captured frame but is tiny. Subscribing
        # this Python gateway to raw RGB images caused costly full-frame DDS
        # copies and starved both the dashboard and MJPEG proxy.
        self.create_timer(1.0, self._discover_cameras)
        self.create_subscription(Joy, str(self.get_parameter('joy_topic').value), self._joy_cb, 10)
        self.create_timer(0.05, self._drive_watchdog)
        self._discover_cameras()

    def _battery_cb(self, message):
        with self._lock:
            self._power.update_battery(message)

    def _esc_cb(self, message):
        with self._lock:
            self._power.update_esc(message)

    def _arm_state_cb(self, message):
        try:
            value = json.loads(message.data)
            if isinstance(value, dict):
                with self._lock:
                    self._arm_state = value
                    self._arm_state_at = time.monotonic()
                    if not self._rc_safety.simulation:
                        self._probe_contact.update(value, self._arm_state_at)
                        load = value.get('load_percent')
                        sample = value.get('sample_time')
                        if (value.get('connected') and value.get('active_role') == 'probe'
                                and type(load) in (int, float) and math.isfinite(load)
                                and type(sample) in (int, float) and math.isfinite(sample)):
                            key = (value.get('servo_id'), sample)
                            if key != self._probe_load_sample:
                                if self._probe_load_sample and key[0] != self._probe_load_sample[0]:
                                    self._probe_load_history.clear()
                                self._probe_load_sample = key
                                self._probe_load_at = self._arm_state_at
                                self._probe_load_ratio = min(1.0, abs(load) / 100.0)
                                self._probe_load_history.append((self._probe_load_at, self._probe_load_ratio))
                                self._probe_load_history = [(at, ratio) for at, ratio in self._probe_load_history
                                                            if self._probe_load_at - at <= PRESSURE_WINDOW_S][-700:]
                        else:
                            self._probe_load_ratio = None
                    calibration = value.get('arm_calibration')
                    if calibration is None and value.get('active_role', 'arm') == 'arm':
                        calibration = value.get('calibration')
                    if isinstance(calibration, dict):
                        points = tuple(calibration.get(key) for key in ('minimum', 'center', 'maximum'))
                        if (all(type(point) is int for point in points)
                                and 0 <= points[0] < points[1] < points[2] <= 4095
                                and points != self._beam_calibration):
                            self._beam_min = (points[0] - points[1]) * 360 / 4096
                            self._beam_max = (points[2] - points[1]) * 360 / 4096
                            self._beam = [0.0] * (math.ceil(self._beam_max - self._beam_min) + 1)
                            self._beam_calibration = points
        except (ValueError, TypeError):
            pass

    def _state_cb(self, message):
        with self._lock:
            self._rc_safety.update_state(message)
            self._enforce_safety_locked()
        self._emit_cmd_vel()

    def _rc_cb(self, message):
        with self._lock:
            self._rc_safety.update_rc(message)
            self._enforce_safety_locked()
        self._emit_cmd_vel()

    def _disable_actuation_locked(self):
        self._probe_motion = None
        self._calibrating = False
        self._armed = False
        self._deadman = False
        self._command = (0.0, 0.0)
        self._permission_pub.publish(Bool(data=False))
        self._sweep_enabled_pub.publish(Bool(data=False))
        self._arm_command_pub.publish(String(data='{"action":"stop"}'))

    def begin_shutdown(self):
        """Revoke control before closing transports or invalidating ROS publishers."""
        with self._lock:
            self._shutting_down = True
            self._lease = None
            self._disable_actuation_locked()
            self._stop_until = time.monotonic() + STOP_TAIL_S
            self._publish_twist(0.0, 0.0)

    def _enforce_safety_locked(self):
        safety = self._rc_safety.snapshot()
        if not safety['servo_allowed']:
            self._disable_actuation_locked()
        elif not safety['allowed']:
            self._deadman = False
            self._command = (0.0, 0.0)
        return safety

    ## ROS callbacks ##

    def _odometry_cb(self, message: Odometry) -> None:
        quaternion = message.pose.pose.orientation
        yaw = math.atan2(2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
                         1.0 - 2.0 * (quaternion.y ** 2 + quaternion.z ** 2))
        with self._lock:
            self._pose = {'x': message.pose.pose.position.x, 'y': message.pose.pose.position.y, 'yaw': yaw}
            self._speed = message.twist.twist.linear.x

    def _detector_state_cb(self, message):
        try:
            value = json.loads(message.data)
            if isinstance(value, dict):
                with self._lock:
                    self._detector_telemetry.update(value)
                    result = value.get('command_result') or {}
                    if self._detector_threshold_request and result.get('request_id') == self._detector_threshold_request:
                        if result.get('ok'):
                            self._detector_threshold = 1.0
                        self._detector_threshold_request = None
        except (ValueError, TypeError):
            pass

    def _detector_cb(self, message: Float32) -> None:
        with self._lock:
            self._detector_at = time.monotonic()
            self._detector = max(0.0, min(1.0, float(message.data)))
            bin_index = round((self._fixture_angle_deg - self._beam_min)
                              / (self._beam_max - self._beam_min) * (len(self._beam) - 1))
            if self._beam_min <= self._fixture_angle_deg <= self._beam_max:
                self._beam[bin_index] = self._detector
            detected = self._detector >= self._detector_threshold
            if detected and not self._was_detected:
                # Where the rover stood when the signal crossed the threshold.
                # A scalar signal carries no range, so this is a track marker
                # and not a target position.
                self._detections.append({
                    'x': self._pose['x'],
                    'y': self._pose['y'],
                    'heading_deg': math.degrees(self._pose['yaw']) + self._fixture_angle_deg,
                    'signal': self._detector,
                })
                self._detections = self._detections[-20:]
            self._was_detected = detected

    def _probe_depth_cb(self, message: Float32) -> None:
        with self._lock:
            self._probe_depth = float(message.data)

    def _probe_pressure_cb(self, message: Float32) -> None:
        if not self._rc_safety.simulation:
            return
        now = time.monotonic()
        with self._lock:
            self._probe_pressure_ratio = max(0.0, min(1.0, float(message.data)))
            if now - self._last_pressure_sample >= PRESSURE_SAMPLE_PERIOD_S:
                self._pressure_history.append(self._probe_pressure_ratio)
                del self._pressure_history[:-PRESSURE_SAMPLES]
                self._last_pressure_sample = now

    def _fixture_angle_cb(self, message: Float32) -> None:
        with self._lock:
            self._fixture_angle_deg = float(message.data)

    def _sweep_enabled_cb(self, message: Bool) -> None:
        with self._lock:
            self._fixture_sweep_enabled = bool(message.data)

    def _sweep_speed_cb(self, message: Float32) -> None:
        with self._lock:
            self._fixture_sweep_speed = float(message.data)

    def _probe_fault_cb(self, message: Bool) -> None:
        with self._lock:
            self._probe_fault = bool(message.data)

    def _camera_frame_cb(self, camera: str, message: CameraInfo) -> None:
        now = time.monotonic()
        with self._lock:
            sample = self._camera_frames[camera]
            sample['width'] = int(message.width)
            sample['height'] = int(message.height)
            sample['last_frame'] = now
            sample['window_frames'] += 1
            elapsed = now - sample['window_started']
            if elapsed >= 1.0:
                sample['fps'] = sample['window_frames'] / elapsed
                sample['window_frames'] = 0
                sample['window_started'] = now

    def _discover_cameras(self) -> None:
        for topic, types in self.get_topic_names_and_types():
            if 'sensor_msgs/msg/CameraInfo' not in types or not topic.endswith('/camera_info'):
                continue
            camera_id = re.sub(r'[^a-zA-Z0-9_-]+', '_', topic.strip('/')).strip('_')
            if not camera_id:
                continue
            if camera_id not in self._camera_frames:
                self._camera_frames[camera_id] = {
                    'topic': topic,
                    'label': topic.strip('/').replace('/', ' / '),
                    'last_frame': 0.0,
                    'window_started': time.monotonic(),
                    'window_frames': 0,
                    'fps': 0.0,
                }
            if camera_id not in self._camera_subscriptions:
                self._camera_subscriptions[camera_id] = self.create_subscription(
                    CameraInfo, topic, lambda message, camera=camera_id: self._camera_frame_cb(camera, message),
                    qos_profile_sensor_data)

    def _axis_trigger(self, key: str, axes: Any, index: int) -> float:
        if index >= len(axes):
            return 0.0
        value = float(axes[index])
        if not self._trigger_seen[key]:
            if value == 0.0:
                return 0.0
            self._trigger_seen[key] = True
        # joy_node inverts SDL axes: released is +1, pressed is -1.
        return max(0.0, min(1.0, (1.0 - value) / 2.0))

    @staticmethod
    def _deadzone(value: float) -> float:
        return 0.0 if abs(value) < 0.12 else value

    @staticmethod
    def _finite(value: float) -> float:
        return value if math.isfinite(value) else 0.0

    def _parse_xbox_joy(self, message: Joy) -> dict[str, Any]:
        axes = message.axes
        buttons = message.buttons

        def pressed(index: int) -> bool:
            return index < len(buttons) and bool(buttons[index])

        # ROS Xbox buttons 6/7 are Back/Start, not browser triggers.
        left_trigger = self._axis_trigger('lt', axes, 2)
        right_trigger = self._axis_trigger('rt', axes, 5)
        stick_x = float(axes[0]) if axes else 0.0
        stick_y = float(axes[1]) if len(axes) > 1 else 0.0
        steering = self._deadzone(stick_x)
        throttle = right_trigger - left_trigger
        left_bumper = pressed(4)
        right_bumper = pressed(5)
        return {
            'seen': True,
            'age_ms': 0,
            'stick_x': stick_x,
            'stick_y': stick_y,
            'lt': left_trigger,
            'rt': right_trigger,
            'lb': left_bumper,
            'rb': right_bumper,
            'a': pressed(0),
            'b': pressed(1),
            'x': pressed(2),
            'y': pressed(3),
            'deadman': left_bumper or right_bumper or abs(throttle) > 0.12,
            'linear': throttle * self._max_velocity,
            'angular': steering * self._max_yaw_rate,
        }

    def _apply_drive_locked(self, linear: float, angular: float, deadman: bool) -> None:
        self._deadman = deadman
        self._command = (
            max(-self._max_velocity, min(self._max_velocity, self._finite(linear))),
            max(-self._max_yaw_rate, min(self._max_yaw_rate, self._finite(angular))),
        )
        self._last_command = time.monotonic()

    def _publish_twist(self, linear: float, angular: float) -> None:
        command = Twist()
        command.linear.x = linear
        command.angular.z = angular
        self._cmd_pub.publish(command)

    def _emit_cmd_vel(self) -> None:
        now = time.monotonic()
        with self._lock:
            safety = self._enforce_safety_locked()
            active = (safety['allowed'] and not self._calibrating and self._lease is not None
                      and now - self._last_command <= self._timeout)
            if active:
                self._stop_until = now + STOP_TAIL_S
                linear, angular = self._command
            elif now < self._stop_until:
                linear, angular = 0.0, 0.0
            else:
                return
            self._publish_twist(linear, angular)

    def _joy_cb(self, message: Joy) -> None:
        flush = False
        with self._lock:
            parsed = self._parse_xbox_joy(message)
            self._joy = parsed
            self._last_joy = time.monotonic()
            if self._lease is None or self._calibrating or not self._enforce_safety_locked()['allowed']:
                self._joy_prev_a = bool(parsed['a'])
                return
            self._joy_prev_a = bool(parsed['a'])
            self._apply_drive_locked(float(parsed['linear']), float(parsed['angular']), True)
            flush = True
        if flush:
            self._emit_cmd_vel()

    def _joy_live(self, now: float) -> bool:
        return self._last_joy > 0.0 and now - self._last_joy < 0.4

    def _drive_watchdog(self) -> None:
        with self._lock:
            safety = self._enforce_safety_locked()
            self._permission_pub.publish(Bool(data=bool(safety['servo_allowed'] and self._lease is not None)))
            self._stream_probe_locked(safety)
        self._emit_cmd_vel()

    def _stream_probe_locked(self, safety):
        motion = self._probe_motion
        if not motion:
            return
        probe = self._arm_state.get('probe', {})
        if (self._lease is None or self._calibrating or not safety['servo_allowed']
                or abs(self._speed) > self._probe_speed_limit or self._probe_fault
                or time.monotonic() - self._arm_state_at >= 1 or not probe.get('ready')
                or time.monotonic() - motion['started'] > 60):
            self._probe_motion = None
            self._arm_command_pub.publish(String(data='{"action":"stop"}'))
            return
        if probe.get('request_id') == motion['command']['hold_id'] and not probe.get('active'):
            self._probe_motion = None
            return
        self._arm_command_pub.publish(String(data=json.dumps(motion['command'])))

    ## Dashboard protocol ##

    def snapshot(self) -> dict[str, Any]:
        """Telemetry shared by every client, without the per-client lease flag."""
        now = time.monotonic()
        with self._lock:
            cameras = {}
            for name, sample in self._camera_frames.items():
                if sample['last_frame']:
                    cameras[name] = {
                        'label': sample['label'],
                        'topic': sample['topic'],
                        'frame_age_ms': round((now - sample['last_frame']) * 1000),
                        'fps': round(sample['fps'], 1),
                        'width': sample.get('width', 0),
                        'height': sample.get('height', 0),
                    }
            publishing = (self._rc_safety.snapshot()['allowed'] and not self._calibrating and self._lease is not None
                          and now - self._last_command <= self._timeout)
            joy_live = self._joy_live(now)
            joy = {**self._joy, 'seen': joy_live}
            if self._last_joy:
                joy['age_ms'] = round((now - self._last_joy) * 1000)
            return {
                'type': 'state',
                'power': self._power.snapshot(),
                'safety': self._rc_safety.snapshot(),
                'arm_servo': self._arm_state if time.monotonic() - self._arm_state_at < 1.0 else {'connected': False, 'reason': 'Arm driver unavailable'},
                'drive': {
                    'armed': self._rc_safety.snapshot()['allowed'],
                    'rc': self._rc_safety.controls(int(self.get_parameter('rc_steering_channel').value),
                                                   int(self.get_parameter('rc_throttle_channel').value)),
                    'calibrating': self._calibrating,
                    'deadman': self._deadman,
                    'publishing': publishing,
                    'command_source': 'joy' if joy_live and publishing else ('browser' if publishing else 'none'),
                    'joy': joy,
                    'cmd_linear_x': self._command[0] if publishing else 0.0,
                    'cmd_angular_z': self._command[1] if publishing else 0.0,
                    'cmd_vel_topic': 'cmd_vel',
                    'timeout_ms': round(self._timeout * 1000),
                    'client_count': len(self._connected_sockets),
                    'control_owner_present': self._lease is not None,
                    'max_velocity_mps': self._max_velocity,
                    'max_yaw_rate_rad_s': self._max_yaw_rate,
                },
                'robot': {**self._pose, 'speed_mps': self._speed},
                'detector': {
                    'fresh': self._detector_at is not None and now - self._detector_at < 0.5,
                    'sensor': self._detector_telemetry.snapshot(),
                    'signal_ratio': self._detector,
                    'threshold_ratio': self._detector_threshold,
                    'detected': self._detector_at is not None and now - self._detector_at < 0.5 and self._detector >= self._detector_threshold,
                    'fixture_angle_deg': self._fixture_angle_deg,
                    'sweep_enabled': self._fixture_sweep_enabled,
                    'sweep_speed_deg_s': self._fixture_sweep_speed,
                    'sweep_speed_min': .684 if self._arm_state.get('connected') else SWEEP_SPEED_MIN_DEG_S,
                    'sweep_speed_max': self._arm_state.get('motion_speed_limit', SWEEP_SPEED_MAX_DEG_S / .684) * .684,
                    'beam_half_angle_deg': BEAM_HALF_ANGLE_DEG,
                    'beam_min_angle_deg': self._beam_min,
                    'beam_max_angle_deg': self._beam_max,
                    'beam': list(self._beam),
                },
                'probe': {
                    'depth_mm': self._probe_depth,
                    'target_mm': self._probe_target,
                    'max_depth_mm': self._max_probe_depth,
                    'pressure_ratio': self._probe_pressure_ratio,
                    'pressure_history': list(self._pressure_history),
                    'pressure_window_s': PRESSURE_WINDOW_S,
                    'fault': self._probe_fault,
                    **({} if self._rc_safety.simulation else {
                        **self._arm_state.get('probe', {}),
                        **self._probe_contact.snapshot(time.monotonic()),
                        'ready': time.monotonic() - self._arm_state_at < 1 and self._arm_state.get('probe', {}).get('ready', False),
                        'reason': self._arm_state.get('probe', {}).get('reason', 'Probe driver unavailable') if time.monotonic() - self._arm_state_at < 1 else 'Probe driver unavailable',
                        'pressure_ratio': self._probe_load_ratio if time.monotonic() - self._probe_load_at < .6 else None,
                        'pressure_samples': [{'x': at - time.monotonic(), 'y': ratio * 100}
                                             for at, ratio in self._probe_load_history if time.monotonic() - at <= PRESSURE_WINDOW_S],
                        'depth_mm': self._arm_state.get('probe', {}).get('depth_mm'),
                        'max_depth_mm': self._arm_state.get('probe', {}).get('max_depth_mm'),
                    }),
                },
                'detections': list(self._detections),
                'cameras': cameras,
            }

    def connected_sockets(self) -> list[web.WebSocketResponse]:
        with self._lock:
            return list(self._connected_sockets)

    def owns_lease(self, ws: web.WebSocketResponse) -> bool:
        with self._lock:
            return self._lease is ws

    def connect_client(self, ws: web.WebSocketResponse) -> None:
        with self._lock:
            self._connected_sockets.add(ws)

    def release(self, ws: web.WebSocketResponse) -> None:
        dropped = False
        with self._lock:
            self._connected_sockets.discard(ws)
            if self._lease is ws:
                dropped = True
                self._disable_actuation_locked()
                self._lease = None
                self._armed = False
                self._deadman = False
                self._command = (0.0, 0.0)
                self._joy_prev_a = False
        if dropped:
            self._emit_cmd_vel()

    def handle_command(self, ws: web.WebSocketResponse, payload: dict[str, Any]) -> dict[str, Any] | None:
        message_type = payload.get('type')
        error: dict[str, Any] | None = None
        flush_drive = False
        with self._lock:
            if self._shutting_down:
                return {'type': 'error', 'message': 'Operator gateway is shutting down.'}
            safety = self._enforce_safety_locked()
            if message_type == 'take_control':
                # Transferring a lease stops the rover; the new operator must
                # consciously arm again before commands can take effect.
                self._disable_actuation_locked()
                self._lease = ws
                self._armed = False
                self._deadman = False
                self._command = (0.0, 0.0)
                flush_drive = True
            elif self._lease is not ws:
                error = {'type': 'error', 'message': 'Spectator mode: take control before sending commands.'}
            elif message_type == 'probe_zero_contact':
                if self._rc_safety.simulation or abs(self._speed) > self._probe_speed_limit or not safety['servo_allowed']:
                    return {'type': 'error', 'message': 'Zero contact requires stationary hardware with probe permission'}
                try:
                    self._probe_contact.zero(time.monotonic())
                except ValueError as error:
                    return {'type': 'error', 'message': str(error)}
            elif message_type == 'detector_calibrate':
                if abs(self._speed) > self._probe_speed_limit:
                    return {'type': 'error', 'message': 'Stop the vehicle before detector calibration.'}
                if not self._detector_telemetry.snapshot()['fresh']:
                    return {'type': 'error', 'message': 'Fresh Arduino readings required for calibration.'}
                command = {key: payload[key] for key in ('action', 'request_id', 'baseline_adc', 'full_response_adc', 'reference_voltage') if key in payload}
                if command.get('action') == 'apply':
                    self._detector_threshold_request = command.get('request_id')
                self._detector_command_pub.publish(String(data=json.dumps(command)))
            elif message_type == 'arm_servo' and payload.get('action') in ('select', 'select_role', 'reconnect', 'capture', 'configure', 'configure_motion', 'enable_multiturn', 'save_probe_extension', 'stop'):
                self._probe_motion = None
                command = {key: payload[key] for key in ('action', 'request_id', 'servo_id', 'role', 'point', 'minimum', 'center', 'maximum', 'max_speed_deg_s', 'acceleration_deg_s2', 'max_extension_mm') if key in payload}
                self._arm_command_pub.publish(String(data=json.dumps(command)))
            elif message_type == 'drive' and not safety['allowed']:
                return {'type': 'error', 'message': safety['reason']}
            elif message_type not in ('disarm', 'estop') and not safety['servo_allowed']:
                return {'type': 'error', 'message': self._rc_safety.servo_snapshot()['reason']}
            elif message_type == 'arm':
                self._calibrating = payload.get('calibration') is True
                self._armed = True
                flush_drive = True
            elif message_type in ('disarm', 'estop'):
                self._disable_actuation_locked()
                self._armed = False
                self._deadman = False
                self._command = (0.0, 0.0)
                flush_drive = True
            elif self._calibrating and message_type in ('drive', 'probe_target', 'sweep_enabled', 'sweep_speed'):
                return {'type': 'error', 'message': 'Vehicle and probe motion are blocked during arm calibration.'}
            elif message_type == 'drive':
                # A live ROS pad on the SVEA owns the sticks so the browser
                # Gamepad API is not required on http://<lan-ip>.
                if not self._joy_live(time.monotonic()):
                    self._apply_drive_locked(
                            float(payload.get('linear_x', 0.0)),
                            float(payload.get('angular_z', 0.0)),
                            True,
                    )
                    flush_drive = True
            elif message_type == 'arm_servo':
                if payload.get('action') in ('jog', 'position', 'home') and not self._calibrating:
                    return {'type': 'error', 'message': 'Enable calibration before jogging.'}
                command = {key: payload[key] for key in ('action', 'request_id', 'role', 'point', 'held', 'direction', 'position', 'speed_deg_s', 'hold_id', 'load_percent') if key in payload}
                self._arm_command_pub.publish(String(data=json.dumps(command)))
            elif message_type == 'probe_target':
                requested = float(payload.get('depth_mm', 0.0))
                if not math.isfinite(requested):
                    return {'type': 'error', 'message': 'Probe extension must be finite'}
                if abs(self._speed) > self._probe_speed_limit:
                    error = {'type': 'error', 'message': 'Probe motion is blocked while the rover is moving.'}
                elif self._probe_fault:
                    error = {'type': 'error', 'message': 'Probe motion is blocked by a probe fault.'}
                else:
                    if self._rc_safety.simulation:
                        self._probe_target = max(0.0, min(self._max_probe_depth, requested))
                        self._probe_pub.publish(Float32(data=self._probe_target))
                    else:
                        probe = self._arm_state.get('probe', {})
                        if time.monotonic() - self._arm_state_at >= 1 or not probe.get('ready'):
                            return {'type': 'error', 'message': probe.get('reason', 'Home and calibrate probe in Settings first')}
                        if self._arm_state.get('sweeping'):
                            return {'type': 'error', 'message': 'Stop arm sweep before moving the probe'}
                        if not 0 <= requested <= probe['max_depth_mm']:
                            return {'type': 'error', 'message': 'Target exceeds saved maximum extension'}
                        self._probe_target = requested
                        self._probe_motion = dict(started=time.monotonic(), command=dict(action='probe_depth', role='probe',
                            held=True, hold_id=str(time.monotonic_ns()), depth_mm=requested))
                        self._permission_pub.publish(Bool(data=True))
                        self._stream_probe_locked(safety)
            elif message_type == 'sweep_enabled':
                if payload.get('enabled') and not self._rc_safety.simulation:
                    if time.monotonic() - self._arm_state_at >= 1 or not self._arm_state.get('connected'):
                        return {'type': 'error', 'message': 'Select a connected arm servo in Settings first.'}
                    if not self._arm_state.get('calibration'):
                        return {'type': 'error', 'message': 'Record and apply arm limits in Settings before sweeping.'}
                # Publish permission before the sweep request; separate topics may
                # still arrive out of order, so the periodic watchdog renews it.
                self._permission_pub.publish(Bool(data=True))
                self._sweep_enabled_pub.publish(Bool(data=bool(payload.get('enabled', False))))
            elif message_type == 'sweep_speed':
                maximum = self._arm_state.get('motion_speed_limit', SWEEP_SPEED_MAX_DEG_S / .684) * .684
                minimum = .684 if self._arm_state.get('connected') else SWEEP_SPEED_MIN_DEG_S
                speed = max(minimum, min(maximum, float(payload.get('deg_s', 60.0))))
                self._sweep_speed_pub.publish(Float32(data=speed))
            else:
                error = {'type': 'error', 'message': 'Unsupported operator command.'}
        if flush_drive:
            self._emit_cmd_vel()
        return error


def dashboard_directory() -> Path:
    return Path(get_package_share_directory('peaceofmine_operator')) / 'dashboard'


def local_ipv4_addresses() -> list[str]:
    """Return non-loopback IPv4 addresses visible from this process."""
    addresses: set[str] = set()
    try:
        results = socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET)
    except OSError:
        return []  # LAN URL discovery must not prevent the dashboard starting.
    for result in results:
        address = result[4][0]
        if not address.startswith('127.'):
            addresses.add(address)
    return sorted(addresses)


def log_dashboard_urls(node: OperatorGateway, scheme: str, host: str, port: int) -> None:
    if host not in {'0.0.0.0', '::'}:
        node.get_logger().info(f'Operator dashboard listening on {scheme}://{host}:{port}')
        return
    node.get_logger().info(
        f'Operator dashboard listening on port {port}. '
        f'Open {scheme}://localhost:{port} on this computer, or '
        f'{scheme}://<robot-computer-ip>:{port} from the remote operator computer.')
    for address in local_ipv4_addresses():
        node.get_logger().info(f'Direct container dashboard URL: {scheme}://{address}:{port}')
    host_address = os.environ.get('OPERATOR_DASHBOARD_HOST_IP')
    if host_address:
        node.get_logger().info(
            f'Host LAN dashboard URL (when Docker port forwarding is active): '
            f'{scheme}://{host_address}:{port}')


def build_ssl_context(cert: str, key: str) -> ssl.SSLContext | None:
    if bool(cert) != bool(key):
        raise RuntimeError('Set both tls_cert and tls_key, or neither.')
    if not cert:
        return None
    context = ssl.create_default_context(ssl.Purpose.CLIENT_AUTH)
    context.load_cert_chain(cert, key)
    return context


def build_app(node: OperatorGateway) -> web.Application:
    app = web.Application()
    dashboard = dashboard_directory()
    cameras = CameraStreams(str(node.get_parameter('camera_stream_base_url').value))
    app.on_shutdown.append(cameras.close)

    async def close_sockets(_app):
        await asyncio.gather(*(ws.close(code=1001, message=b'Server shutting down')
                               for ws in node.connected_sockets()))

    app.on_shutdown.append(close_sockets)

    async def index_handler(_: web.Request) -> web.FileResponse:
        return web.FileResponse(dashboard / 'index.html', headers={'Cache-Control': 'no-store'})

    async def asset_handler(request: web.Request) -> web.FileResponse:
        # ament's symlink install makes the packaged assets symlinks during
        # development, and aiohttp's generic static route refuses symlinks by
        # design, so serve this small explicit allow-list instead.
        filename = request.match_info['filename']
        if filename not in {'app.js', 'style.css', 'keydrown-1.3.0.js', 'chart-4.4.8.umd.js'}:
            raise web.HTTPNotFound()
        return web.FileResponse(dashboard / filename, headers={'Cache-Control': 'no-store'})

    async def camera_handler(request: web.Request) -> web.StreamResponse:
        camera = node._camera_frames.get(request.match_info['camera'])
        if camera is None:
            raise web.HTTPNotFound()
        topic = camera['topic'].replace('/camera_info', '/image_raw')
        return await cameras.serve(request, topic)

    async def socket_handler(request: web.Request) -> web.WebSocketResponse:
        # Per-message deflate on the same socket as 20+ Hz drive commands
        # costs CPU on the Pi and delays the event loop. LAN bandwidth is
        # cheap; leave the frames uncompressed.
        ws = web.WebSocketResponse(heartbeat=10.0, compress=False, timeout=0.5)
        await ws.prepare(request)
        node.connect_client(ws)
        try:
            async for message in ws:
                if message.type is not WSMsgType.TEXT:
                    continue
                try:
                    payload = json.loads(message.data)
                    if not isinstance(payload, dict):
                        raise ValueError('Operator command must be an object.')
                except (ValueError, json.JSONDecodeError) as error:
                    await ws.send_json({'type': 'error', 'message': f'Invalid operator command: {error}'})
                    continue
                if payload.get('type') == 'latency_ping':
                    await ws.send_json({'type': 'latency_pong', 'probe_id': payload.get('probe_id')})
                    continue
                try:
                    error_message = node.handle_command(ws, payload)
                except (ValueError, TypeError) as error:
                    error_message = {'type': 'error', 'message': f'Invalid operator command: {error}'}
                if error_message:
                    await ws.send_json(error_message)
        finally:
            node.release(ws)
        return ws

    app.router.add_get('/', index_handler)
    app.router.add_get('/assets/{filename}', asset_handler)
    app.router.add_get('/camera/{camera}', camera_handler)
    app.router.add_get('/ws', socket_handler)
    return app


async def broadcast_telemetry(node: OperatorGateway) -> None:
    """Build one snapshot per tick and fan it out to every connected client."""
    while True:
        await asyncio.sleep(TELEMETRY_PERIOD_S)
        sockets = node.connected_sockets()
        if not sockets:
            continue
        shared = node.snapshot()
        drive = shared['drive']
        for ws in sockets:
            if ws.closed:
                continue
            drive['you_control_owner'] = node.owns_lease(ws)
            try:
                await ws.send_str(json.dumps(shared, separators=(',', ':')))
            except (ConnectionResetError, RuntimeError):
                pass


async def start_server(node: OperatorGateway, stop: threading.Event) -> None:
    runner = web.AppRunner(build_app(node), shutdown_timeout=1.0)
    broadcaster = None
    try:
        await runner.setup()
        cert = str(node.get_parameter('tls_cert').value)
        key = str(node.get_parameter('tls_key').value)
        ssl_context = build_ssl_context(cert, key)
        if ssl_context is None:
            node.get_logger().warn('Dashboard is running without TLS; use TLS before remote controller operation.')
        host = str(node.get_parameter('host').value)
        port = int(node.get_parameter('port').value)
        site = web.TCPSite(runner, host, port, ssl_context=ssl_context)
        await site.start()
        log_dashboard_urls(node, 'https' if ssl_context else 'http', host, port)
        broadcaster = asyncio.create_task(broadcast_telemetry(node))
        while not stop.is_set():
            await asyncio.sleep(0.05)
    finally:
        node.begin_shutdown()
        if broadcaster is not None:
            broadcaster.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await broadcaster
        # Keep ROS alive for the stop heartbeat while HTTP clients disconnect.
        await asyncio.sleep(0.15)
        await runner.cleanup()


def main() -> None:
    stop = threading.Event()
    # Keep this handler through interpreter exit: launch may forward SIGINT twice.
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop.set())
    node = executor = spinner = None
    try:
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        node = OperatorGateway()
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        spinner = threading.Thread(target=executor.spin, daemon=True)
        spinner.start()
        asyncio.run(start_server(node, stop))
    finally:
        if executor is not None:
            executor.shutdown()
        if spinner is not None:
            spinner.join()
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
