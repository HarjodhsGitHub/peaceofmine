"""Share one MJPEG encoder per camera, retaining only the newest frame."""

import asyncio
import contextlib

import aiohttp
from aiohttp import web


class CameraStreams:
    def __init__(self, base_url):
        self.base_url = base_url.rstrip('/')
        self.streams = {}

    async def _read(self, topic, queues):
        timeout = aiohttp.ClientTimeout(total=None, sock_connect=4, sock_read=4)
        try:
            async with aiohttp.ClientSession(timeout=timeout) as session:
                async with session.get(self.base_url + '/stream', params={
                    'topic': topic, 'type': 'mjpeg', 'quality': '60',
                    'qos_profile': 'sensor_data',
                }) as upstream:
                    upstream.raise_for_status()
                    reader = aiohttp.MultipartReader.from_response(upstream)
                    while True:
                        part = await reader.next()
                        if part is None:
                            break
                        frame = bytes(await part.read())
                        for queue in tuple(queues):
                            if queue.full():
                                queue.get_nowait()
                            queue.put_nowait(frame)
        except (aiohttp.ClientError, asyncio.TimeoutError, ValueError, AssertionError):
            pass  # Close viewers; their retry starts a fresh upstream connection.
        finally:
            for queue in tuple(queues):
                if queue.full():
                    queue.get_nowait()
                queue.put_nowait(None)

    async def serve(self, request, topic):
        if topic not in self.streams:
            queues = set()
            task = asyncio.create_task(self._read(topic, queues))
            self.streams[topic] = (queues, task)
        queues, task = self.streams[topic]
        queue = asyncio.Queue(maxsize=1)
        queues.add(queue)
        response = None
        try:
            frame = await asyncio.wait_for(queue.get(), timeout=5)
            if frame is None:
                raise web.HTTPBadGateway(text='Camera stream unavailable. Please retry.')
            response = web.StreamResponse(headers={
                'Content-Type': 'multipart/x-mixed-replace; boundary=frame',
                'Cache-Control': 'no-store', 'X-Accel-Buffering': 'no',
            })
            await response.prepare(request)
            while frame is not None:
                packet = (b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: '
                          + str(len(frame)).encode() + b'\r\n\r\n' + frame + b'\r\n')
                # A stalled viewer must reconnect, not watch a growing backlog.
                await asyncio.wait_for(response.write(packet), timeout=0.5)
                frame = await asyncio.wait_for(queue.get(), timeout=5)
            if request.transport:
                request.transport.close()
            return response
        except (asyncio.TimeoutError, ConnectionError):
            if response is None:
                raise web.HTTPBadGateway(text='Camera stream timed out.')
            if request.transport:
                request.transport.close()
            return response
        finally:
            queues.discard(queue)
            if not queues:
                self.streams.pop(topic, None)
                task.cancel()
                with contextlib.suppress(asyncio.CancelledError):
                    await task

    async def close(self, _app):
        tasks = [task for _, task in self.streams.values()]
        for task in tasks:
            task.cancel()
        await asyncio.gather(*tasks, return_exceptions=True)
