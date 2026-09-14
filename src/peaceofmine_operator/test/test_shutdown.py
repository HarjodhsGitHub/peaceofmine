"""Real process signals with connected browser and camera clients; no hardware."""
import asyncio
import contextlib
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import textwrap
import unittest

import aiohttp
import rclpy
from sensor_msgs.msg import CameraInfo
from aiohttp import web

SCRIPTS = Path(__file__).parents[1] / 'scripts'


class ShutdownTest(unittest.TestCase):
    def test_gateway_signals_close_live_clients(self):
        for sig in (signal.SIGINT, signal.SIGTERM):
            with self.subTest(signal=sig):
                asyncio.run(self.gateway_shutdown(sig))

    async def gateway_shutdown(self, sig):
        # A real MJPEG upstream that stays open until gateway cleanup closes it.
        async def stream(request):
            response = web.StreamResponse(headers={
                'Content-Type': 'multipart/x-mixed-replace; boundary=frame'})
            await response.prepare(request)
            try:
                while True:
                    await response.write(b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: 3\r\n\r\nabc\r\n')
                    await asyncio.sleep(.05)
            except ConnectionError:
                return response
        rclpy.init()
        node = rclpy.create_node("shutdown_camera")
        publisher = node.create_publisher(CameraInfo, "/shutdown_camera/camera_info", 1)
        app = web.Application()
        app.router.add_get('/stream', stream)
        runner = web.AppRunner(app)
        await runner.setup()
        site = web.TCPSite(runner, '127.0.0.1', 18082)
        await site.start()
        with tempfile.TemporaryFile(mode='w+') as log:
            process = subprocess.Popen([
                sys.executable, str(SCRIPTS / 'operator_gateway.py'), '--ros-args',
                '-p', 'host:=127.0.0.1', '-p', 'port:=18081',
                '-p', 'safety_simulation:=true',
                '-p', 'camera_stream_base_url:=http://127.0.0.1:18082',
            ], stdout=log, stderr=subprocess.STDOUT)
            try:
                async with aiohttp.ClientSession() as session:
                    for _ in range(150):
                        try:
                            ws = await session.ws_connect('http://127.0.0.1:18081/ws')
                            break
                        except aiohttp.ClientError:
                            await asyncio.sleep(.05)
                    else:
                        self.fail('Gateway did not start')
                    await ws.send_json({'type': 'take_control'})
                    await ws.send_json({'type': 'arm'})
                    await ws.send_json({'type': 'drive', 'linear_x': .4, 'deadman': True})
                    async def camera_discovered():
                        async for message in ws:
                            publisher.publish(CameraInfo(width=640, height=480))
                            if message.type == aiohttp.WSMsgType.TEXT:
                                data = message.json()
                                if 'shutdown_camera_camera_info' in data.get('cameras', {}):
                                    return
                        self.fail('WebSocket closed before camera discovery')
                    await asyncio.wait_for(camera_discovered(), 8)
                    publisher.publish(CameraInfo(width=640, height=480))
                    async with session.get('http://127.0.0.1:18081/camera/shutdown_camera_camera_info') as camera:
                        self.assertEqual(camera.status, 200)
                        await camera.content.read(1)
                        process.send_signal(sig)
                        async def drain():
                            async for _ in ws:
                                pass
                        await asyncio.wait_for(drain(), 3)
                        # MJPEG ends by closing its transport.
                        with contextlib.suppress(aiohttp.ClientPayloadError):
                            await asyncio.wait_for(camera.read(), 3)
                    self.assertEqual(await asyncio.to_thread(process.wait, timeout=3), 0)
            finally:
                if process.poll() is None:
                    process.kill()
                    process.wait()
                await runner.cleanup()
                node.destroy_node()
                rclpy.shutdown()
                log.seek(0)
                output = log.read()
                self.assertNotIn('Traceback', output, output)
                self.assertNotIn('context is invalid', output, output)

    def test_servo_signals_without_hardware(self):
        for sig in (signal.SIGINT, signal.SIGTERM):
            with self.subTest(signal=sig), tempfile.TemporaryFile(mode='w+') as log:
                # Signal only after initialization; the timer proves spin has begun.
                code = ('import importlib.util, os, signal; '
                        f's = importlib.util.spec_from_file_location("arm", {str(SCRIPTS / "arm_servo_node.py")!r}); '
                        'm = importlib.util.module_from_spec(s); s.loader.exec_module(m); '
                        'original = m.ArmServoNode.__init__; '
                        f'm.ArmServoNode.__init__ = lambda self: (original(self), self.create_timer(.1, lambda: os.kill(os.getpid(), {int(sig)}))) and None; '
                        'm.main()')
                result = subprocess.run([sys.executable, '-c', code], stdout=log, stderr=subprocess.STDOUT, timeout=8)
                log.seek(0)
                self.assertEqual(result.returncode, 0, log.read())

    def test_drive_signals_publish_neutral_before_ros_shutdown(self):
        script = SCRIPTS.parents[1] / 'svea_examples/scripts/twist_consumer.py'
        for sig in (signal.SIGINT, signal.SIGTERM):
            with self.subTest(signal=sig):
                code = textwrap.dedent(f"""
                    import importlib.util, os
                    spec = importlib.util.spec_from_file_location('drive', {str(script)!r})
                    module = importlib.util.module_from_spec(spec)
                    spec.loader.exec_module(module)
                    original = module.twist_consumer.on_shutdown
                    def run(self):
                        self.actuation.send_control(.2, .4)
                        self.actuation.loop()
                        os.kill(os.getpid(), {int(sig)})
                    def shutdown(self):
                        assert self.context.ok()
                        original(self)
                        assert self.actuation.velocity_percent == 0
                        assert self.actuation.steering_percent == 0
                        print('neutral published with live ROS context')
                    module.twist_consumer.run = run
                    module.twist_consumer.on_shutdown = shutdown
                    module.twist_consumer.main()
                """)
                result = subprocess.run([sys.executable, '-c', code], capture_output=True, text=True, timeout=8)
                self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
                self.assertIn('neutral published with live ROS context', result.stdout)
                self.assertNotIn('context is invalid', result.stderr)
