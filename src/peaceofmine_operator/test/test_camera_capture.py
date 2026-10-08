"""Camera startup consumes shared settings, without opening a physical camera."""
import copy
import importlib.util
import json
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

try:
    import pyudev
    from ament_index_python.packages import get_package_prefix
except ImportError:
    pyudev = None


@unittest.skipIf(pyudev is None, 'ROS camera environment required')
class CameraCaptureTest(unittest.TestCase):
    def test_capture_settings_reach_camera_driver(self):
        from peaceofmine_operator.camera_settings import DEFAULTS
        spec=importlib.util.spec_from_file_location('camera_auto',Path(__file__).parents[1]/'scripts/usb_camera_auto.py')
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/'operator_config.json'
            cameras=copy.deepcopy(DEFAULTS)
            cameras['capture']['forward'].update(device='/dev/v4l/by-id/camera',width=800,framerate=20.0)
            path.write_text(json.dumps({'cameras':cameras}))
            with patch.dict(os.environ,{'POM_CONFIG_FILE':str(path),'POM_CAMERA_INDEX':'0'}), \
                    patch.object(module,'resolve_device',return_value='/dev/video4'), \
                    patch.object(module,'get_package_prefix',return_value='/opt/ros/jazzy'), \
                    patch.object(module.os,'execv') as execute:
                module.main()
                args=execute.call_args.args[1]
                self.assertIn('image_width:=800',args)
                self.assertIn('framerate:=20.0',args)
                self.assertIn('video_device:=/dev/video4',args)
                self.assertEqual(os.environ['POM_CAMERA_DEVICE'],'/dev/v4l/by-id/camera')
