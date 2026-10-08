#!/usr/bin/env python3
"""ROS operator control: lease arbitration, safety gates and motion coordination."""
from __future__ import annotations
import json
import math
import threading
import time
from typing import Any

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistWithCovarianceStamped
from rclpy.qos import qos_profile_sensor_data
from mavros_msgs.msg import State, RCIn
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, Float32, String
from peaceofmine_operator.rc_safety import RcSafety
from peaceofmine_operator.detector import DetectorTelemetry
from peaceofmine_operator.node_runner import run_node
from peaceofmine_operator.actuator_protocol import command_message, status_dict
from peaceofmine_interfaces.msg import ActuatorCommand, ActuatorStatus

# Authoritative actuator limits. The dashboard reads these from telemetry so
# the browser never carries its own copy.
SWEEP_SPEED_MIN_DEG_S = 5.0
SWEEP_SPEED_MAX_DEG_S = 180.0

# After the drive command stops being valid, keep publishing zeros for this
# long and then go silent, so an idle gateway does not hold the downstream
# twist_consumer watchdog open or fight other cmd_vel publishers.
STOP_TAIL_S = 0.5


class OperatorControl(Node):
    def __init__(self, **kwargs) -> None:
        super().__init__('operator_control', **kwargs)
        self.declare_parameter('safety_simulation', False)
        self.declare_parameter('cmd_vel_topic', 'cmd_vel')
        self.declare_parameter('joy_topic', 'joy')
        self.declare_parameter('odometry_topic', 'odometry/local')
        self.declare_parameter('fixture_sweep_enabled_topic', 'fixture/sweep_enabled')
        self.declare_parameter('fixture_sweep_speed_topic', 'fixture/sweep_speed_deg_s')
        self.declare_parameter('probe_fault_topic', 'probe/fault')
        self.declare_parameter('command_timeout_s', 0.25)
        self.declare_parameter('max_velocity_mps', 0.8)
        self.declare_parameter('max_yaw_rate_rad_s', 1.4)
        self.declare_parameter('probe_motion_speed_limit_mps', 0.03)
        self.declare_parameter('rc_steering_channel', 1)
        self.declare_parameter('rc_throttle_channel', 2)

        self._lock = threading.RLock()
        self._shutting_down = False
        self._rc_safety = RcSafety(bool(self.get_parameter('safety_simulation').value), clock=lambda: time.monotonic())
        self._arm_state = {'connected': False, 'reason': 'Arm driver not running'}
        self._arm_state_at = 0.0
        self._permission_pub = self.create_publisher(Bool, 'operator/actuation_enabled', 1)
        self._arm_command_pub = self.create_publisher(ActuatorCommand, 'arm/command', 10)
        self._probe_command_pub = self.create_publisher(ActuatorCommand, 'probe/command', 10)
        self._probe_state = {}
        self._probe_state_at = 0.0
        self._selected_actuator = 'arm'
        self.create_subscription(ActuatorStatus, 'probe/state', self._probe_state_cb, 10)
        self.create_subscription(State, 'mavros/state', self._state_cb, 10)
        self.create_subscription(RCIn, 'mavros/rc/in', self._rc_cb, 10)
        self.create_subscription(ActuatorStatus, 'arm/state', self._arm_state_cb, 1)
        self._lease = None
        # Do not call this `_clients`: rclpy Node uses that private attribute
        # for ROS service clients while its executor spins.
        self._connected_clients = set()
        self._client_seen = {}
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
        self._detector_telemetry = DetectorTelemetry()
        self._detector_command_pub = self.create_publisher(String, 'detector/command', 10)
        self.create_subscription(String, 'detector/state', self._detector_state_cb, 10)
        self._probe_fault = False
        self._pose_at = 0.
        self._pose = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
        self._speed = 0.0
        self._speed_at = 0.
        self._wheel_stamp = -1

        self._max_velocity = float(self.get_parameter('max_velocity_mps').value)
        self._max_yaw_rate = float(self.get_parameter('max_yaw_rate_rad_s').value)
        self._timeout = float(self.get_parameter('command_timeout_s').value)
        self._probe_speed_limit = float(self.get_parameter('probe_motion_speed_limit_mps').value)

        self._cmd_pub = self.create_publisher(Twist, str(self.get_parameter('cmd_vel_topic').value), 10)
        self._sweep_enabled_pub = self.create_publisher(ActuatorCommand, 'arm/command', 1)
        self._sweep_speed_pub = self.create_publisher(Float32, str(self.get_parameter('fixture_sweep_speed_topic').value), 10)
        self.create_subscription(Odometry, str(self.get_parameter('odometry_topic').value), self._odometry_cb, 10)
        self.create_subscription(TwistWithCovarianceStamped, 'mavros/wheel_odometry/velocity', self._wheel_velocity_cb, qos_profile_sensor_data)
        self.create_subscription(Bool, str(self.get_parameter('probe_fault_topic').value), self._probe_fault_cb, 10)
        self.create_subscription(Joy, str(self.get_parameter('joy_topic').value), self._joy_cb, 10)
        self.create_timer(0.05, self._drive_watchdog)
        self._state_pub = self.create_publisher(String, 'operator/state', 10)
        self._result_pub = self.create_publisher(String, 'operator/result', 10)
        self._adc_command_pub = self.create_publisher(String, 'adc/command', 10)
        self._settings_command_pub = self.create_publisher(String, 'operator/settings/command', 10)
        self.create_subscription(String, 'operator/command', self._transport_command, 50)
        self.create_timer(.05, self.publish_state)

    def _arm_state_cb(self, message):
        try:
            value = status_dict(message)
            if isinstance(value, dict):
                with self._lock:
                    self._arm_state = value
                    self._arm_state_at = time.monotonic()
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
        self._armed = False
        self._permission_pub.publish(Bool(data=False))
        self._stop_motion_locked()

    def _stop_motion_locked(self):
        self._calibrating = False
        self._deadman = False
        self._command = (0.0, 0.0)
        self._sweep_enabled_pub.publish(command_message(dict(action='sweep',enabled=False)))
        self._arm_command_pub.publish(command_message(dict(action='stop')))
        self._probe_command_pub.publish(command_message(dict(action='stop')))

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
        # Compatibility telemetry field: readiness is derived from RC and the
        # control lease, never a second dashboard arming latch.
        self._armed = bool(safety['servo_allowed'] and self._lease is not None and not self._shutting_down)
        if not self._armed:
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
            if not all(math.isfinite(v) for v in (message.pose.pose.position.x, message.pose.pose.position.y, yaw)):
                return
            self._pose_at = time.monotonic()
            self._pose = {'x': message.pose.pose.position.x, 'y': message.pose.pose.position.y, 'yaw': yaw}
            self._speed = message.twist.twist.linear.x
            self._speed_at = time.monotonic() if math.isfinite(self._speed) else 0.

    def _wheel_velocity_cb(self, message):
        stamp = message.header.stamp.sec*1000000000+message.header.stamp.nanosec
        speed = message.twist.twist.linear.x
        with self._lock:
            if stamp <= self._wheel_stamp or not math.isfinite(speed):
                return
            self._wheel_stamp = stamp
            self._speed, self._speed_at = speed, time.monotonic()

    def _detector_state_cb(self, message):
        try:
            value = json.loads(message.data)
            if isinstance(value, dict):
                with self._lock:
                    self._detector_telemetry.update(value)
        except (ValueError, TypeError):
            pass

    def _probe_fault_cb(self, message: Bool) -> None:
        with self._lock:
            self._probe_fault = bool(message.data)

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
            active = (self._armed and self._deadman and safety['allowed'] and not self._calibrating and self._lease is not None
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
            if self._lease is None or not self._armed or self._calibrating or not self._enforce_safety_locked()['allowed']:
                self._joy_prev_a = bool(parsed['a'])
                return
            self._joy_prev_a = bool(parsed['a'])
            self._apply_drive_locked(float(parsed['linear']), float(parsed['angular']), bool(parsed['deadman']))
            flush = True
        if flush:
            self._emit_cmd_vel()

    def _joy_live(self, now: float) -> bool:
        return self._last_joy > 0.0 and now - self._last_joy < 0.4

    def _drive_watchdog(self) -> None:
        with self._lock:
            now = time.monotonic()
            for client, seen in list(self._client_seen.items()):
                if now - seen > .5:
                    self.release(client)
                    self._client_seen.pop(client, None)
            safety = self._enforce_safety_locked()
            self._permission_pub.publish(Bool(data=bool(self._armed and safety['servo_allowed'] and self._lease is not None)))
        self._emit_cmd_vel()

    def _probe_state_cb(self, message):
        try:
            value = status_dict(message)
            if isinstance(value, dict):
                self._probe_state, self._probe_state_at = value, time.monotonic()
        except (ValueError, TypeError):
            pass

    def _actuator_command(self, payload):
        role = payload.get('role', self._selected_actuator)
        if role not in ('arm', 'probe'):
            return {'type': 'error', 'message': 'Select arm or probe'}
        if payload.get('action') == 'select_role':
            self._selected_actuator = role
            return None
        publisher = self._arm_command_pub if role == 'arm' else self._probe_command_pub
        publisher.publish(command_message({**payload, 'role':role}))
        return None

    ## Dashboard protocol ##

    def snapshot(self) -> dict[str, Any]:
        """Telemetry shared by every client, without the per-client lease flag."""
        now = time.monotonic()
        with self._lock:
            publishing = (self._armed and self._deadman and self._rc_safety.snapshot()['allowed'] and not self._calibrating and self._lease is not None
                          and now - self._last_command <= self._timeout)
            joy_live = self._joy_live(now)
            joy = {**self._joy, 'seen': joy_live}
            if self._last_joy:
                joy['age_ms'] = round((now - self._last_joy) * 1000)
            return {
                'type': 'state',
                'safety': self._rc_safety.snapshot(),
                'selected_actuator': self._selected_actuator,
                'drive': {
                    'armed': self._armed,
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
                    'client_count': len(self._connected_clients),
                    'control_owner_present': self._lease is not None,
                    'max_velocity_mps': self._max_velocity,
                    'max_yaw_rate_rad_s': self._max_yaw_rate,
                },
                'robot': {**(self._pose if time.monotonic()-self._pose_at < .5 else dict(x=None, y=None, yaw=None)), 'pose_available':time.monotonic()-self._pose_at < .5, 'speed_mps': self._speed if time.monotonic()-self._speed_at < .5 else None, 'velocity_fresh':time.monotonic()-self._speed_at < .5},

            }

    def connected_clients(self) -> list[str]:
        with self._lock:
            return list(self._connected_clients)

    def owns_lease(self, ws: str) -> bool:
        with self._lock:
            return self._lease == ws

    def connect_client(self, ws: str) -> None:
        with self._lock:
            self._connected_clients.add(ws)

    def release(self, ws: str) -> None:
        dropped = False
        with self._lock:
            self._connected_clients.discard(ws)
            if self._lease == ws:
                dropped = True
                self._disable_actuation_locked()
                self._lease = None
                self._armed = False
                self._deadman = False
                self._command = (0.0, 0.0)
                self._joy_prev_a = False
        if dropped:
            self._emit_cmd_vel()

    def handle_command(self, ws: str, payload: dict[str, Any]) -> dict[str, Any] | None:
        message_type = payload.get('type')
        error: dict[str, Any] | None = None
        flush_drive = False
        with self._lock:
            if self._shutting_down:
                return {'type': 'error', 'message': 'Operator control node is shutting down.'}
            safety = self._enforce_safety_locked()
            if message_type == 'take_control':
                # A new owner must send fresh motion/deadman commands.
                self._disable_actuation_locked()
                self._lease = ws
                self._armed = False
                self._deadman = False
                self._command = (0.0, 0.0)
                self._enforce_safety_locked()
                flush_drive = True
            elif self._lease != ws:
                error = {'type': 'error', 'message': 'Spectator mode: take control before sending commands.'}
            elif message_type in ('calibration_begin', 'calibration_end'):
                self._stop_motion_locked()
                self._calibrating = message_type == 'calibration_begin'
                flush_drive = True
            elif message_type in ('adc', 'settings'):
                publisher = self._adc_command_pub if message_type == 'adc' else self._settings_command_pub
                publisher.publish(String(data=json.dumps(payload)))
            elif message_type == 'probe_zero_contact':
                self._probe_command_pub.publish(command_message(dict(action='zero_contact', request_id=payload.get('request_id'))))
            elif message_type == 'detector_calibrate':
                if abs(self._speed) > self._probe_speed_limit:
                    return {'type': 'error', 'message': 'Stop the vehicle before detector calibration.'}
                if not self._detector_telemetry.snapshot()['fresh']:
                    return {'type': 'error', 'message': 'Fresh Arduino readings required for calibration.'}
                command = {key: payload[key] for key in ('action', 'request_id', 'baseline_adc', 'full_response_adc', 'reference_voltage') if key in payload}
                self._detector_command_pub.publish(String(data=json.dumps(command)))
            elif message_type == 'arm_servo' and payload.get('action') in ('select', 'select_role', 'reconnect', 'capture', 'configure', 'configure_motion', 'enable_multiturn', 'save_probe_extension', 'stop'):
                command = {key: payload[key] for key in ('action', 'request_id', 'servo_id', 'role', 'point', 'minimum', 'center', 'maximum', 'max_speed_deg_s', 'acceleration_deg_s2', 'max_extension_mm') if key in payload}
                return self._actuator_command(command)
            elif message_type == 'drive' and not safety['allowed']:
                return {'type': 'error', 'message': safety['reason']}
            elif message_type not in ('disarm', 'estop') and not safety['servo_allowed']:
                return {'type': 'error', 'message': self._rc_safety.servo_snapshot()['reason']}
            elif message_type == 'arm':
                # Older clients may still select calibration this way; this
                # message cannot override the RC gate.
                self._disable_actuation_locked()
                self._calibrating = payload.get('calibration') is True
                self._enforce_safety_locked()
                flush_drive = True
            elif message_type in ('disarm', 'estop'):
                # STOP stays stopped until control is explicitly taken again.
                self._lease = None
                self._disable_actuation_locked()
                self._armed = False
                self._deadman = False
                self._command = (0.0, 0.0)
                flush_drive = True
            elif not self._armed:
                return {'type':'error', 'message':'Take control and enable motion on the RC.'}
            elif self._calibrating and message_type in ('drive', 'probe_target', 'sweep_enabled', 'sweep_speed'):
                return {'type': 'error', 'message': 'Vehicle and probe motion are blocked during arm calibration.'}
            elif message_type == 'drive':
                # A live ROS pad on the SVEA owns the sticks so the browser
                # Gamepad API is not required on http://<lan-ip>.
                if not self._joy_live(time.monotonic()):
                    self._apply_drive_locked(
                            float(payload.get('linear_x', 0.0)),
                            float(payload.get('angular_z', 0.0)),
                            payload.get('deadman') is True,
                    )
                    flush_drive = True
            elif message_type == 'arm_servo':
                if payload.get('action') in ('jog', 'position', 'home', 'move') and not self._calibrating:
                    # Stop prior work without briefly revoking permission:
                    # that false heartbeat can otherwise race the new jog.
                    self._stop_motion_locked()
                    self._calibrating = True
                    self._enforce_safety_locked()
                    self._stop_until = time.monotonic() + STOP_TAIL_S
                    self._publish_twist(0.0, 0.0)
                command = {key: payload[key] for key in ('action', 'request_id', 'role', 'point', 'held', 'direction', 'position', 'speed_deg_s', 'hold_id', 'load_percent') if key in payload}
                return self._actuator_command(command)
            elif message_type == 'probe_target':
                requested = float(payload.get('depth_mm', 0.0))
                if not math.isfinite(requested):
                    return {'type': 'error', 'message': 'Probe extension must be finite'}
                if abs(self._speed) > self._probe_speed_limit:
                    error = {'type': 'error', 'message': 'Probe motion is blocked while the rover is moving.'}
                elif self._probe_fault:
                    error = {'type': 'error', 'message': 'Probe motion is blocked by a probe fault.'}
                else:
                    probe = self._probe_state.get('probe', {})
                    if time.monotonic() - self._probe_state_at >= .5 or not probe.get('ready'):
                        return {'type': 'error', 'message': probe.get('reason', 'Home and calibrate probe first')}
                    if self._arm_state.get('sweeping'):
                        return {'type': 'error', 'message': 'Stop arm sweep before moving the probe'}
                    if not 0 <= requested <= probe['max_depth_mm']:
                        return {'type': 'error', 'message': 'Target exceeds saved maximum extension'}
                    self._probe_command_pub.publish(command_message(dict(action='target', depth_mm=requested, request_id=payload.get('request_id'))))
            elif message_type == 'sweep_enabled':
                if payload.get('enabled') and not self._rc_safety.simulation:
                    if time.monotonic() - self._arm_state_at >= 1 or not self._arm_state.get('connected'):
                        return {'type': 'error', 'message': 'Select a connected arm servo in Settings first.'}
                    if not self._arm_state.get('calibration'):
                        return {'type': 'error', 'message': 'Record and apply arm limits in Settings before sweeping.'}
                # Publish permission before the sweep request; separate topics may
                # still arrive out of order, so the periodic watchdog renews it.
                self._permission_pub.publish(Bool(data=True))
                self._sweep_enabled_pub.publish(command_message(dict(action='sweep',enabled=bool(payload.get('enabled',False)))))
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

    def _transport_command(self, message):
        client = None
        payload = {}
        try:
            envelope = json.loads(message.data)
            client = envelope['client_id']
            if not isinstance(client, str) or not 1 <= len(client) <= 100:
                raise ValueError('Invalid client ID')
            stamp = envelope['stamp']
            if type(stamp) not in (float, int) or not math.isfinite(stamp) or not 0 <= time.time() - stamp < .5:
                raise ValueError('Expired operator command')
            payload = envelope['payload']
            if not isinstance(payload, dict):
                raise ValueError('Invalid command payload')
            event = envelope.get('event', 'command')
            if event == 'heartbeat':
                self.connect_client(client)
                self._client_seen[client] = time.monotonic()
                return
            if event == 'disconnect':
                self.release(client)
                self._client_seen.pop(client, None)
                return
            if event != 'command' or client not in self._client_seen:
                raise ValueError('Client heartbeat required')
            result = self.handle_command(client, payload)
            if result:
                self._result_pub.publish(String(data=json.dumps(dict(
                    client_id=client, request_id=payload.get('request_id'), **result))))
        except (ValueError, TypeError, KeyError) as exc:
            self._result_pub.publish(String(data=json.dumps(dict(
                type='error', client_id=client, request_id=payload.get('request_id') if isinstance(payload, dict) else None, message=str(exc)))))

    def publish_state(self):
        state = self.snapshot()
        state['control_owner'] = self._lease
        self._state_pub.publish(String(data=json.dumps(state, allow_nan=False)))


def main():
    run_node(OperatorControl)


if __name__ == '__main__':
    main()
