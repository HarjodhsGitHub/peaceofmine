import copy
import unittest
from peaceofmine_operator.camera_settings import DEFAULTS, validate


class CameraSettingsTest(unittest.TestCase):
    def test_rejects_private_device_ids_and_invalid_capture(self):
        self.assertEqual(validate(DEFAULTS), DEFAULTS)
        for path, value in [(('forward','source'),'laptop-device-id'), (('forward','rotation'),45),
                            (('capture','forward','framerate'),float('nan')),
                            (('capture','forward','width'),True), (('capture','forward','device'),'/tmp/not-a-camera')]:
            setting=copy.deepcopy(DEFAULTS)
            target=setting
            for key in path[:-1]: target=target[key]
            target[path[-1]]=value
            with self.subTest(path=path), self.assertRaises(ValueError): validate(setting)
