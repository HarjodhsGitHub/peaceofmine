"""MJPEG fanout and recovery checks; no ROS or camera hardware required."""
import asyncio
import importlib.util
from pathlib import Path
import unittest

try:
    import aiohttp
    from aiohttp import web
    from aiohttp.test_utils import TestServer, TestClient
except ImportError:
    aiohttp = None

if aiohttp:
    spec = importlib.util.spec_from_file_location('camera_stream', Path(__file__).resolve().parents[1] / 'peaceofmine_operator/camera_stream.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)


@unittest.skipUnless(aiohttp, 'aiohttp required')
class CameraStreamTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.requests = 0
        self.release = asyncio.Event()
        self.count = 0

        async def stream(request):
            self.requests += 1
            self.assertEqual(request.query['qos_profile'], 'sensor_data')
            response = web.StreamResponse(headers={'Content-Type': 'multipart/x-mixed-replace; boundary=camera'})
            await response.prepare(request)
            try:
                for i in range(40):
                    frame = str(i).encode()
                    await response.write(b'--camera\r\nContent-Type: image/jpeg\r\nContent-Length: ' + str(len(frame)).encode() + b'\r\n\r\n' + frame + b'\r\n')
                    self.count = i
                    await asyncio.sleep(.002)
                await self.release.wait()
            except ConnectionResetError:
                pass
            return response

        app = web.Application()
        app.router.add_get('/stream', stream)
        self.upstream = TestServer(app)
        await self.upstream.start_server()
        self.streams = module.CameraStreams(str(self.upstream.make_url('/')))
        proxy = web.Application()
        async def camera(request):
            return await self.streams.serve(request, '/image_raw')
        proxy.router.add_get('/camera', camera)
        self.client = TestClient(TestServer(proxy))
        await self.client.start_server()

    async def asyncTearDown(self):
        self.release.set()
        await self.streams.close(None)
        await self.client.close()
        await self.upstream.close()

    async def test_two_viewers_share_upstream(self):
        first, second = await asyncio.gather(self.client.get('/camera'), self.client.get('/camera'))
        for response in (first, second):
            self.assertEqual(response.status, 200)
            self.assertEqual(response.headers['Cache-Control'], 'no-store')
            reader = aiohttp.MultipartReader.from_response(response)
            part = await reader.next()
            self.assertTrue((await part.read()).isdigit())
        self.assertEqual(self.requests, 1)
        first.close()
        second.close()

    async def test_slow_consumer_keeps_only_latest_frame(self):
        queue = asyncio.Queue(maxsize=1)
        task = asyncio.create_task(self.streams._read('/image_raw', {queue}))
        try:
            async with asyncio.timeout(3):
                while self.count < 39 or queue.empty() or queue._queue[0] != b'39':
                    await asyncio.sleep(.005)
            self.assertEqual(queue.qsize(), 1)
            self.assertEqual(queue.get_nowait(), b'39')
        finally:
            task.cancel()
            await asyncio.gather(task, return_exceptions=True)

    async def test_unavailable_upstream_returns_502(self):
        await self.upstream.close()
        response = await self.client.get('/camera')
        self.assertEqual(response.status, 502)
        self.assertEqual(self.streams.streams, {})
