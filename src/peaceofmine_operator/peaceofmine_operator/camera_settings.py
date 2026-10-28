"""Portable robot camera defaults; browser-owned device IDs never enter this file."""
import copy
import math

DEFAULTS = {
    'forward': {'source': 'virtual', 'rotation': 0},
    'auxiliary': {'source': 'off', 'rotation': 0},
    'capture': {
        'forward': {'device': 'auto', 'width': 640, 'height': 480, 'framerate': 30.0, 'pixel_format': 'mjpeg2rgb'},
        'auxiliary': {'device': 'auto', 'width': 640, 'height': 480, 'framerate': 30.0, 'pixel_format': 'yuyv2rgb'},
    },
}


def validate(value):
    if not isinstance(value, dict) or set(value) != set(DEFAULTS):
        raise ValueError('Expected forward, auxiliary and capture camera settings')
    for slot in ('forward', 'auxiliary'):
        view = value[slot]
        if not isinstance(view, dict) or set(view) != {'source', 'rotation'}:
            raise ValueError('Invalid camera view')
        source = view['source']
        if (not isinstance(source, str) or len(source) > 250
                or (source not in ('off', 'virtual') and not source.startswith('raspberry:'))):
            raise ValueError('Laptop camera IDs are local to the browser; choose a ROS camera, virtual or off')
        if type(view['rotation']) is not int or view['rotation'] not in (0, 90, 180, 270):
            raise ValueError('Camera rotation must be 0, 90, 180 or 270')
    capture = value['capture']
    if not isinstance(capture, dict) or set(capture) != {'forward', 'auxiliary'}:
        raise ValueError('Both capture configurations are required')
    for c in capture.values():
        if not isinstance(c, dict) or set(c) != {'device', 'width', 'height', 'framerate', 'pixel_format'}:
            raise ValueError('Invalid capture settings')
        if not isinstance(c['device'], str) or len(c['device']) > 250 or (c['device'] != 'auto' and not c['device'].startswith('/dev/')):
            raise ValueError('Camera device must be auto or an absolute /dev path')
        if any(type(c[k]) is not int or not 16 <= c[k] <= 4096 for k in ('width', 'height')):
            raise ValueError('Camera dimensions must be 16..4096 pixels')
        if type(c['framerate']) not in (int, float) or not math.isfinite(c['framerate']) or not 1 <= c['framerate'] <= 120:
            raise ValueError('Camera frame rate must be 1..120')
        if c['pixel_format'] not in ('mjpeg2rgb', 'yuyv2rgb', 'uyvy2rgb', 'rgb8', 'mono8'):
            raise ValueError('Unsupported camera pixel format')
    return copy.deepcopy(value)
