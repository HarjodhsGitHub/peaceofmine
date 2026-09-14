#!/usr/bin/env python3
"""Browser-to-ROS gateway for the PeaceOfMine operator dashboard.

One browser holds the drive lease, must explicitly arm, and must hold a
deadman input. Physical RC override and the downstream SVEA watchdog remain
independent safety layers.
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
    import aiohttp
    from aiohttp import WSMsgType, web
except ImportError as error:  # pragma: no cover - depends on target image
    raise RuntimeError('aiohttp is required; install the workspace requirements first.') from error

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Bool, Float32

# Authoritative actuator limits. The dashboard reads these from telemetry so
# the browser never carries its own copy.
SWEEP_SPEED_MIN_DEG_S = 5.0
SWEEP_SPEED_MAX_DEG_S = 180.0

# The detector beam trace covers +/-BEAM_HALF_ANGLE_DEG in one-degree bins.
BEAM_HALF_ANGLE_DEG = 45
BEAM_BINS = 2 * BEAM_HALF_ANGLE_DEG + 1

PRESSURE_SAMPLE_PERIOD_S = 0.2
PRESSURE_WINDOW_S = 30.0
PRESSURE_SAMPLES = int(PRESSURE_WINDOW_S / PRESSURE_SAMPLE_PERIOD_S)

TELEMETRY_PERIOD_S = 0.1

# After the drive command stops being valid, keep publishing zeros for this
# long and then go silent, so an idle gateway does not hold the downstream
# twist_consumer watchdog open or fight other cmd_vel publishers.
STOP_TAIL_S = 0.5


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
        self.declare_parameter('detector_threshold_ratio', 0.65)

        self._lock = threading.Lock()
        self._lease: web.WebSocketResponse | None = None
        # Do not call this `_clients`: rclpy Node uses that private attribute
        # for ROS service clients while its executor spins.
        self._connected_sockets: set[web.WebSocketResponse] = set()
        self._armed = False
        self._deadman = False
        self._command = (0.0, 0.0)
        self._last_command = 0.0
        self._stop_until = 0.0
        self._detector = 0.0
        self._probe_depth = 0.0
        self._probe_pressure_ratio = 0.0
        self._fixture_angle_deg = 0.0
        self._fixture_sweep_enabled = False
        self._fixture_sweep_speed = 60.0
        self._beam = [0.0] * BEAM_BINS
        self._pressure_history = [0.0] * PRESSURE_SAMPLES
        self._last_pressure_sample = 0.0
        self._probe_fault = False
        self._probe_target = 0.0
        self._pose = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
        self._speed = 0.0
        self._was_detected = False
        self._detections: list[dict[str, float]] = []
        self._camera_frames = {
            'front': {'last_frame': 0.0, 'window_started': time.monotonic(), 'window_frames': 0, 'fps': 0.0},
            'auxiliary': {'last_frame': 0.0, 'window_started': time.monotonic(), 'window_frames': 0, 'fps': 0.0},
        }

        self._max_velocity = float(self.get_parameter('max_velocity_mps').value)
        self._max_yaw_rate = float(self.get_parameter('max_yaw_rate_rad_s').value)
        self._timeout = float(self.get_parameter('command_timeout_s').value)
        self._probe_speed_limit = float(self.get_parameter('probe_motion_speed_limit_mps').value)
        self._max_probe_depth = float(self.get_parameter('max_probe_depth_mm').value)
        self._detector_threshold = float(self.get_parameter('detector_threshold_ratio').value)

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
        self.create_subscription(CameraInfo, str(self.get_parameter('front_camera_info_topic').value),
                                 lambda _: self._camera_frame_cb('front'), qos_profile_sensor_data)
        self.create_subscription(CameraInfo, str(self.get_parameter('auxiliary_camera_info_topic').value),
                                 lambda _: self._camera_frame_cb('auxiliary'), qos_profile_sensor_data)
        self.create_timer(0.05, self._drive_watchdog)

    ## ROS callbacks ##

    def _odometry_cb(self, message: Odometry) -> None:
        quaternion = message.pose.pose.orientation
        yaw = math.atan2(2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
                         1.0 - 2.0 * (quaternion.y ** 2 + quaternion.z ** 2))
        with self._lock:
            self._pose = {'x': message.pose.pose.position.x, 'y': message.pose.pose.position.y, 'yaw': yaw}
            self._speed = message.twist.twist.linear.x

    def _detector_cb(self, message: Float32) -> None:
        with self._lock:
            self._detector = max(0.0, min(1.0, float(message.data)))
            bin_index = round(self._fixture_angle_deg) + BEAM_HALF_ANGLE_DEG
            if 0 <= bin_index < BEAM_BINS:
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

    def _camera_frame_cb(self, camera: str) -> None:
        now = time.monotonic()
        with self._lock:
            sample = self._camera_frames[camera]
            sample['last_frame'] = now
            sample['window_frames'] += 1
            elapsed = now - sample['window_started']
            if elapsed >= 1.0:
                sample['fps'] = sample['window_frames'] / elapsed
                sample['window_frames'] = 0
                sample['window_started'] = now

    def _drive_watchdog(self) -> None:
        now = time.monotonic()
        with self._lock:
            active = (self._lease is not None
                      and self._armed
                      and self._deadman
                      and now - self._last_command <= self._timeout)
            if active:
                self._stop_until = now + STOP_TAIL_S
                linear, angular = self._command
            elif now < self._stop_until:
                linear, angular = 0.0, 0.0
            else:
                return
        command = Twist()
        command.linear.x = linear
        command.angular.z = angular
        self._cmd_pub.publish(command)

    ## Dashboard protocol ##

    def snapshot(self) -> dict[str, Any]:
        """Telemetry shared by every client, without the per-client lease flag."""
        now = time.monotonic()
        with self._lock:
            cameras = {}
            for name, sample in self._camera_frames.items():
                if sample['last_frame']:
                    cameras[name] = {
                        'frame_age_ms': round((now - sample['last_frame']) * 1000),
                        'fps': round(sample['fps'], 1),
                    }
            return {
                'type': 'state',
                'drive': {
                    'armed': self._armed,
                    'deadman': self._deadman,
                    'timeout_ms': round(self._timeout * 1000),
                    'client_count': len(self._connected_sockets),
                    'control_owner_present': self._lease is not None,
                    'max_velocity_mps': self._max_velocity,
                    'max_yaw_rate_rad_s': self._max_yaw_rate,
                },
                'robot': {**self._pose, 'speed_mps': self._speed},
                'detector': {
                    'signal_ratio': self._detector,
                    'threshold_ratio': self._detector_threshold,
                    'detected': self._detector >= self._detector_threshold,
                    'fixture_angle_deg': self._fixture_angle_deg,
                    'sweep_enabled': self._fixture_sweep_enabled,
                    'sweep_speed_deg_s': self._fixture_sweep_speed,
                    'sweep_speed_min': SWEEP_SPEED_MIN_DEG_S,
                    'sweep_speed_max': SWEEP_SPEED_MAX_DEG_S,
                    'beam_half_angle_deg': BEAM_HALF_ANGLE_DEG,
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
        with self._lock:
            self._connected_sockets.discard(ws)
            if self._lease is ws:
                self._lease = None
                self._armed = False
                self._deadman = False
                self._command = (0.0, 0.0)

    def handle_command(self, ws: web.WebSocketResponse, payload: dict[str, Any]) -> dict[str, Any] | None:
        message_type = payload.get('type')
        with self._lock:
            if message_type == 'take_control':
                # Transferring a lease stops the rover; the new operator must
                # consciously arm again before commands can take effect.
                self._lease = ws
                self._armed = False
                self._deadman = False
                self._command = (0.0, 0.0)
                return None
            if self._lease is not ws:
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
                speed = max(SWEEP_SPEED_MIN_DEG_S,
                            min(SWEEP_SPEED_MAX_DEG_S, float(payload.get('deg_s', 60.0))))
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

    async def index_handler(_: web.Request) -> web.FileResponse:
        return web.FileResponse(dashboard / 'index.html', headers={'Cache-Control': 'no-store'})

    async def asset_handler(request: web.Request) -> web.FileResponse:
        # ament's symlink install makes the packaged assets symlinks during
        # development, and aiohttp's generic static route refuses symlinks by
        # design, so serve this small explicit allow-list instead.
        filename = request.match_info['filename']
        if filename not in {'app.js', 'style.css'}:
            raise web.HTTPNotFound()
        return web.FileResponse(dashboard / filename, headers={'Cache-Control': 'no-store'})

    async def camera_handler(request: web.Request) -> web.StreamResponse:
        topics = {
            'front': str(node.get_parameter('front_camera_topic').value),
            'auxiliary': str(node.get_parameter('auxiliary_camera_topic').value),
        }
        topic = topics.get(request.match_info['camera'])
        if topic is None:
            raise web.HTTPNotFound()
        base_url = str(node.get_parameter('camera_stream_base_url').value).rstrip('/')
        timeout = aiohttp.ClientTimeout(total=None, sock_connect=4.0, sock_read=None)
        try:
            async with aiohttp.ClientSession(timeout=timeout) as session:
                async with session.get(
                    f'{base_url}/stream',
                    params={
                        'topic': topic,
                        'type': 'mjpeg',
                        'quality': '60',
                        'qos_profile': 'default',
                    },
                ) as upstream:
                    if upstream.status != 200:
                        raise web.HTTPBadGateway(text=f'Camera stream returned HTTP {upstream.status}.')
                    response = web.StreamResponse(
                        headers={
                            'Content-Type': upstream.headers.get('Content-Type', 'multipart/x-mixed-replace'),
                            'Cache-Control': 'no-store',
                            'X-Accel-Buffering': 'no',
                        })
                    await response.prepare(request)
                    async for chunk in upstream.content.iter_any():
                        await response.write(chunk)
                    return response
        except (aiohttp.ClientError, asyncio.TimeoutError) as error:
            raise web.HTTPBadGateway(text=f'Camera streaming service unavailable: {error}') from error

    async def socket_handler(request: web.Request) -> web.WebSocketResponse:
        ws = web.WebSocketResponse(heartbeat=10.0)
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
        for ws in sockets:
            if ws.closed:
                continue
            message = {**shared, 'drive': {**shared['drive'], 'you_control_owner': node.owns_lease(ws)}}
            try:
                await ws.send_json(message)
            except (ConnectionResetError, RuntimeError):
                pass


async def start_server(node: OperatorGateway) -> None:
    runner = web.AppRunner(build_app(node))
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
    try:
        while rclpy.ok():
            await asyncio.sleep(0.5)
    finally:
        broadcaster.cancel()
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
