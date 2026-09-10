#!/usr/bin/env python3
"""Narrow browser-to-ROS gateway for the PeaceOfMine operator dashboard.

It deliberately exposes a small protocol instead of a generic ROS bridge:
one browser owns the drive lease, must explicitly arm, and must continuously
hold a deadman input.  Physical RC override and the downstream SVEA watchdog
remain independent safety layers.
"""

from __future__ import annotations

import asyncio
import json
import math
import os
import socket
import ssl
import threading
import time
from pathlib import Path
from typing import Any

try:
    from aiohttp import WSMsgType, web
except ImportError as error:  # pragma: no cover - depends on target image
    raise RuntimeError('aiohttp is required; install the workspace requirements first.') from error

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool, Float32


class OperatorGateway(Node):
    def __init__(self) -> None:
        super().__init__('operator_gateway')
        self.declare_parameter('host', '0.0.0.0')
        self.declare_parameter('port', 8080)
        self.declare_parameter('tls_cert', '')
        self.declare_parameter('tls_key', '')
        self.declare_parameter('cmd_vel_topic', 'cmd_vel')
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
        self.declare_parameter('command_timeout_s', 0.25)
        self.declare_parameter('max_velocity_mps', 0.8)
        self.declare_parameter('max_yaw_rate_rad_s', 1.4)
        self.declare_parameter('probe_motion_speed_limit_mps', 0.03)
        self.declare_parameter('max_probe_depth_mm', 110.0)

        self._lock = threading.Lock()
        self._lease: web.WebSocketResponse | None = None
        # Do not call this `_clients`: rclpy Node uses that private attribute
        # for ROS service clients while its executor spins.
        self._connected_sockets: set[web.WebSocketResponse] = set()
        self._armed = False
        self._deadman = False
        self._command = (0.0, 0.0)
        self._last_command = 0.0
        self._detector = 0.0
        self._probe_depth = 0.0
        self._probe_pressure_ratio = 0.0
        self._fixture_angle_deg = 0.0
        self._fixture_sweep_enabled = False
        self._fixture_sweep_speed = 0.0
        self._detector_history: list[dict[str, float]] = []
        self._last_detector_sample = 0.0
        self._pressure_history: list[dict[str, float]] = []
        self._last_pressure_sample = 0.0
        self._probe_fault = False
        self._probe_target = 0.0
        self._pose = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
        self._speed = 0.0
        self._was_detected = False
        self._events: list[dict[str, float]] = []

        self._max_velocity = float(self.get_parameter('max_velocity_mps').value)
        self._max_yaw_rate = float(self.get_parameter('max_yaw_rate_rad_s').value)
        self._timeout = float(self.get_parameter('command_timeout_s').value)
        self._probe_speed_limit = float(self.get_parameter('probe_motion_speed_limit_mps').value)
        self._max_probe_depth = float(self.get_parameter('max_probe_depth_mm').value)
        self._cmd_pub = self.create_publisher(Twist, str(self.get_parameter('cmd_vel_topic').value), 10)
        self._probe_pub = self.create_publisher(Float32, str(self.get_parameter('probe_target_topic').value), 10)
        self._sweep_enabled_pub = self.create_publisher(Bool, str(self.get_parameter('fixture_sweep_enabled_topic').value), 10)
        self._sweep_speed_pub = self.create_publisher(Float32, str(self.get_parameter('fixture_sweep_speed_topic').value), 10)
        self.create_subscription(Odometry, str(self.get_parameter('odometry_topic').value), self._odometry, 10)
        self.create_subscription(Float32, str(self.get_parameter('detector_signal_topic').value), self._detector_cb, 10)
        self.create_subscription(Float32, str(self.get_parameter('probe_depth_topic').value), self._probe_depth_cb, 10)
        self.create_subscription(Float32, str(self.get_parameter('probe_pressure_topic').value), self._probe_pressure_cb, 10)
        self.create_subscription(Float32, str(self.get_parameter('fixture_angle_topic').value), self._fixture_angle_cb, 10)
        self.create_subscription(Bool, str(self.get_parameter('fixture_sweep_enabled_state_topic').value), self._sweep_enabled_cb, 10)
        self.create_subscription(Float32, str(self.get_parameter('fixture_sweep_speed_state_topic').value), self._sweep_speed_cb, 10)
        self.create_subscription(Bool, str(self.get_parameter('probe_fault_topic').value), self._probe_fault_cb, 10)
        self.create_timer(0.05, self._drive_watchdog)

    def _odometry(self, message: Odometry) -> None:
        quaternion = message.pose.pose.orientation
        yaw = math.atan2(2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
                         1.0 - 2.0 * (quaternion.y ** 2 + quaternion.z ** 2))
        with self._lock:
            self._pose = {'x': message.pose.pose.position.x, 'y': message.pose.pose.position.y, 'yaw': yaw}
            self._speed = message.twist.twist.linear.x

    def _detector_cb(self, message: Float32) -> None:
        now = time.monotonic()
        with self._lock:
            self._detector = max(0.0, min(1.0, float(message.data)))
            detected = self._detector >= 0.65
            if detected and not self._was_detected:
                self._events.append({'x': self._pose['x'], 'y': self._pose['y'], 'signal': self._detector})
                self._events = self._events[-20:]
            self._was_detected = detected
            if now - self._last_detector_sample >= 0.1:
                self._detector_history.append({'t': now, 'angle_deg': self._fixture_angle_deg, 'ratio': self._detector})
                self._detector_history = [sample for sample in self._detector_history if sample['t'] >= now - 6.0]
                self._last_detector_sample = now

    def _probe_depth_cb(self, message: Float32) -> None:
        with self._lock:
            self._probe_depth = float(message.data)

    def _probe_pressure_cb(self, message: Float32) -> None:
        now = time.monotonic()
        with self._lock:
            self._probe_pressure_ratio = max(0.0, min(1.0, float(message.data)))
            # A 5 Hz trace is smooth enough for the operator and bounds each
            # dashboard message to the latest 150 samples (30 seconds).
            if now - self._last_pressure_sample >= 0.2:
                self._pressure_history.append({'t': now, 'ratio': self._probe_pressure_ratio})
                self._pressure_history = [sample for sample in self._pressure_history if sample['t'] >= now - 30.0]
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

    def _drive_watchdog(self) -> None:
        with self._lock:
            active = self._armed and self._deadman and time.monotonic() - self._last_command <= self._timeout
            linear, angular = self._command if active else (0.0, 0.0)
        command = Twist()
        command.linear.x = linear
        command.angular.z = angular
        self._cmd_pub.publish(command)

    def snapshot(self, socket: web.WebSocketResponse) -> dict[str, Any]:
        with self._lock:
            return {
                'type': 'state',
                'drive': {
                    'armed': self._armed,
                    'deadman': self._deadman,
                    'timeout_ms': round(self._timeout * 1000),
                    'client_count': len(self._connected_sockets),
                    'you_control_owner': self._lease is socket,
                    'control_owner_present': self._lease is not None,
                },
                'robot': {**self._pose, 'speed_mps': self._speed},
                'detector': {
                    'signal_ratio': self._detector,
                    'threshold_ratio': 0.65,
                    'detected': self._detector >= 0.65,
                    'fixture_angle_deg': self._fixture_angle_deg,
                    'sweep_enabled': self._fixture_sweep_enabled,
                    'sweep_speed_deg_s': self._fixture_sweep_speed,
                    'history': list(self._detector_history),
                },
                'probe': {
                    'depth_mm': self._probe_depth,
                    'target_mm': self._probe_target,
                    'max_depth_mm': self._max_probe_depth,
                    'pressure_ratio': self._probe_pressure_ratio,
                    'pressure_history': list(self._pressure_history),
                    'fault': self._probe_fault,
                },
                'events': list(self._events),
            }

    def connect_client(self, socket: web.WebSocketResponse) -> None:
        with self._lock:
            self._connected_sockets.add(socket)

    def release(self, socket: web.WebSocketResponse) -> None:
        with self._lock:
            self._connected_sockets.discard(socket)
            if self._lease is socket:
                self._lease = None
                self._armed = False
                self._deadman = False
                self._command = (0.0, 0.0)

    def handle_command(self, socket: web.WebSocketResponse, payload: dict[str, Any]) -> dict[str, Any] | None:
        message_type = payload.get('type')
        with self._lock:
            if message_type == 'take_control':
                # Transferring a lease stops the rover; the new operator must
                # consciously arm again before commands can take effect.
                self._lease = socket
                self._armed = False
                self._deadman = False
                self._command = (0.0, 0.0)
                return None
            if self._lease is not socket:
                return {'type': 'error', 'message': 'Spectator mode: take control before sending commands.'}
            if message_type == 'arm':
                self._armed = True
            elif message_type in ('disarm', 'estop'):
                self._armed = False
                self._deadman = False
                self._command = (0.0, 0.0)
            elif message_type == 'drive':
                linear = float(payload.get('linear_x', 0.0))
                angular = float(payload.get('angular_z', 0.0))
                self._deadman = bool(payload.get('deadman', False))
                self._command = (
                    max(-self._max_velocity, min(self._max_velocity, linear)),
                    max(-self._max_yaw_rate, min(self._max_yaw_rate, angular)),
                )
                self._last_command = time.monotonic()
            elif message_type == 'probe_target':
                requested = float(payload.get('depth_mm', 0.0))
                if abs(self._speed) > self._probe_speed_limit:
                    return {'type': 'error', 'message': 'Probe motion is blocked while the rover is moving.'}
                if self._probe_fault:
                    return {'type': 'error', 'message': 'Probe motion is blocked by a probe fault.'}
                self._probe_target = max(0.0, min(self._max_probe_depth, requested))
                self._probe_pub.publish(Float32(data=self._probe_target))
            elif message_type == 'sweep_enabled':
                self._sweep_enabled_pub.publish(Bool(data=bool(payload.get('enabled', False))))
            elif message_type == 'sweep_speed':
                speed = max(5.0, min(180.0, float(payload.get('deg_s', 60.0))))
                self._sweep_speed_pub.publish(Float32(data=speed))
            else:
                return {'type': 'error', 'message': 'Unsupported operator command.'}
        return None


def dashboard_directory() -> Path:
    return Path(get_package_share_directory('peaceofmine_operator')) / 'dashboard'


def local_ipv4_addresses() -> list[str]:
    """Return non-loopback IPv4 addresses visible from this process."""
    addresses: set[str] = set()
    for result in socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET):
        address = result[4][0]
        if not address.startswith('127.'):
            addresses.add(address)
    return sorted(addresses)


async def start_server(node: OperatorGateway) -> None:
    app = web.Application()
    dashboard = dashboard_directory()
    app.router.add_get('/', lambda _: web.FileResponse(dashboard / 'index.html'))

    # ament's symlink install makes the packaged assets symlinks during
    # development. aiohttp's generic static route refuses symlinks by design,
    # so expose this small, explicit allow-list instead.
    async def asset_handler(request: web.Request) -> web.FileResponse:
        filename = request.match_info['filename']
        if filename not in {'app.js', 'style.css'}:
            raise web.HTTPNotFound()
        return web.FileResponse(dashboard / filename)

    app.router.add_get('/assets/{filename}', asset_handler)

    async def socket_handler(request: web.Request) -> web.WebSocketResponse:
        socket = web.WebSocketResponse(heartbeat=10.0)
        await socket.prepare(request)
        node.connect_client(socket)

        async def telemetry() -> None:
            while not socket.closed:
                await socket.send_json(node.snapshot(socket))
                await asyncio.sleep(0.05)

        telemetry_task = asyncio.create_task(telemetry())
        try:
            async for message in socket:
                if message.type is WSMsgType.TEXT:
                    try:
                        payload = json.loads(message.data)
                        if not isinstance(payload, dict):
                            raise ValueError('Operator command must be an object.')
                        error = node.handle_command(socket, payload)
                        if error:
                            await socket.send_json(error)
                    except (ValueError, TypeError, json.JSONDecodeError) as error:
                        await socket.send_json({'type': 'error', 'message': f'Invalid operator command: {error}'})
        finally:
            telemetry_task.cancel()
            node.release(socket)
        return socket

    app.router.add_get('/ws', socket_handler)
    runner = web.AppRunner(app)
    await runner.setup()
    cert = str(node.get_parameter('tls_cert').value)
    key = str(node.get_parameter('tls_key').value)
    ssl_context = None
    if bool(cert) != bool(key):
        raise RuntimeError('Set both tls_cert and tls_key, or neither.')
    if cert:
        ssl_context = ssl.create_default_context(ssl.Purpose.CLIENT_AUTH)
        ssl_context.load_cert_chain(cert, key)
    elif node.get_logger():
        node.get_logger().warn('Dashboard is running without TLS; use TLS before remote controller operation.')
    site = web.TCPSite(runner, str(node.get_parameter('host').value), int(node.get_parameter('port').value), ssl_context=ssl_context)
    await site.start()
    scheme = 'https' if ssl_context else 'http'
    port = int(node.get_parameter('port').value)
    host = str(node.get_parameter('host').value)
    if host in {'0.0.0.0', '::'}:
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
    else:
        node.get_logger().info(f'Operator dashboard listening on {scheme}://{host}:{port}')
    try:
        while rclpy.ok():
            await asyncio.sleep(0.5)
    finally:
        await runner.cleanup()


def main() -> None:
    rclpy.init()
    node = OperatorGateway()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()
    try:
        asyncio.run(start_server(node))
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
