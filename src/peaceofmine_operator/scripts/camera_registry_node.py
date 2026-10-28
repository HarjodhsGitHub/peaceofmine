#!/usr/bin/env python3
"""Discover ROS camera metadata without subscribing to full image frames."""
import json
import re
import threading
import time
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import String
from peaceofmine_operator.node_runner import run_node


class CameraRegistry(Node):
    def __init__(self):
        super().__init__('camera_registry')
        self._lock = threading.RLock()
        self._camera_frames = {}
        self._camera_subscriptions = {}
        self.publisher = self.create_publisher(String, 'cameras/state', 10)
        self.create_timer(1.0, self._discover_cameras)
        self.create_timer(.1, self.publish_state)
        self._discover_cameras()

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


    def snapshot(self):
        now = time.monotonic()
        with self._lock:
            return {name: dict(label=s['label'], topic=s['topic'],
                         frame_age_ms=round((now-s['last_frame'])*1000),
                         fps=round(s['fps'],1), width=s.get('width',0), height=s.get('height',0))
                    for name,s in self._camera_frames.items() if s['last_frame']}

    def publish_state(self):
        self.publisher.publish(String(data=json.dumps(self.snapshot())))


if __name__ == '__main__':
    run_node(CameraRegistry)
