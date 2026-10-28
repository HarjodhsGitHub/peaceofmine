import json
import tempfile
from pathlib import Path
import unittest
import rclpy
from rclpy.parameter import Parameter
from peaceofmine_interfaces.srv import SaveSettings
from peaceofmine_operator import configuration
from test_arm_node import module


class SettingsServiceTest(unittest.TestCase):
    def test_revision_conflicts_and_invalid_documents_preserve_saved_settings(self):
        rclpy.init()
        with tempfile.TemporaryDirectory() as tmp:
            path=str(Path(tmp)/'operator_config.json')
            node=module('operator_settings_node').OperatorSettings(parameter_overrides=[Parameter('config_file',value=path)])
            try:
                request=SaveSettings.Request(section='arm_motion',value_json=json.dumps(dict(max_speed_deg_s=10.,acceleration_deg_s2=34.332)),expected_revision=0)
                result=node.save(request,SaveSettings.Response())
                self.assertTrue(result.success,result.message)
                self.assertEqual(result.revision,1)
                original=Path(path).read_text()
                self.assertFalse(node.save(request,SaveSettings.Response()).success)
                for value in ('[]','null','{"max_speed_deg_s":0}','{broken'):
                    request.value_json=value
                    request.expected_revision=1
                    self.assertFalse(node.save(request,SaveSettings.Response()).success)
                    self.assertEqual(Path(path).read_text(),original)
                self.assertEqual(configuration.load(path)['section_revisions']['arm_motion'],1)
            finally:
                node.destroy_node()
                rclpy.shutdown()
