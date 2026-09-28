#!/usr/bin/env python3
"""HTTP/WebSocket frontend for ROS operator topics.

Safety, leases, motion, cameras and sensor acquisition are owned by ROS nodes.
"""

from __future__ import annotations

import asyncio
import signal
import contextlib
import json
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
from rclpy.signals import SignalHandlerOptions
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import MultiThreadedExecutor
from peaceofmine_operator.camera_stream import CameraStreams

from peaceofmine_operator.web_bridge import OperatorWebBridge as OperatorGateway

TELEMETRY_PERIOD_S = .05


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
    async def adc_handler(request):
        try:
            after = max(0, int(request.query.get('after', '0')))
        except ValueError:
            raise web.HTTPBadRequest(text='Invalid sample cursor')
        state = node.adc.snapshot(after, request.query.get('session'))
        if state is None:
            raise web.HTTPServiceUnavailable(text='Waiting for ADC ROS node')
        return web.json_response(state, headers={'Cache-Control': 'no-store'})

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
        if filename not in {'app.js', 'actuator-settings.js', 'adc.js', 'camera-config.js', 'style.css', 'keydrown-1.3.0.js', 'chart-4.4.8.umd.js'}:
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
    app.router.add_get('/api/adc', adc_handler)
    return app


async def broadcast_telemetry(node: OperatorGateway) -> None:
    """Build one snapshot per tick and fan it out to every connected client."""
    while True:
        await asyncio.sleep(TELEMETRY_PERIOD_S)
        sockets = node.connected_sockets()
        if not sockets:
            continue
        for ws, reply in node.replies():
            if not ws.closed:
                with contextlib.suppress(ConnectionError, RuntimeError):
                    await ws.send_json(reply)
        shared = node.snapshot()
        if shared is None:
            if not node.has_control_state():
                continue
            for ws in sockets:
                await ws.close(code=1013, message=b'Operator control ROS node unavailable')
            continue
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
        for sig in (signal.SIGINT, signal.SIGTERM):
            signal.signal(sig, signal.SIG_IGN)
        if executor is not None:
            executor.shutdown()
        if spinner is not None:
            spinner.join()
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
