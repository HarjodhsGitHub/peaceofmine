#!/usr/bin/env python3
"""Shared operator preferences over ROS; hardware nodes own their own sections."""
import copy
import json
import math
from peaceofmine_interfaces.srv import SaveSettings
from peaceofmine_operator.adc_acquisition import OperatorADC
from peaceofmine_operator.servo_motor import limits
from peaceofmine_operator.probe_calibration import validate as validate_probe
from rclpy.node import Node
from std_msgs.msg import String
from peaceofmine_operator import configuration, camera_settings
from peaceofmine_operator.node_runner import run_node


class OperatorSettings(Node):
    def __init__(self, **kwargs):
        super().__init__('operator_settings', **kwargs)
        self.path = self.declare_parameter('config_file', configuration.default_path()).value
        self.state_pub = self.create_publisher(String, 'operator/settings', 10)
        self.result_pub = self.create_publisher(String, 'operator/settings/result', 10)
        self.create_subscription(String, 'operator/settings/command', self.command, 10)
        self.create_timer(.5, self.publish_state)
        self.error = None
        self.saved = False
        self.document = {}
        self.cameras = copy.deepcopy(camera_settings.DEFAULTS)
        self.create_service(SaveSettings, 'operator/settings/save', self.save)
        self.reload()

    def save(self, request, response):
        try:
            value = json.loads(request.value_json)
            if not isinstance(value,dict):
                raise ValueError('Expected settings object')
            storage_section=request.section
            simulation=storage_section in ('arm_simulation','probe_simulation','arm_motion_simulation','probe_motion_simulation')
            section=storage_section.removesuffix('_simulation') if simulation else storage_section
            if section == 'cameras':
                value = camera_settings.validate(value)
            elif section in ('adc', 'adc_simulation'):
                config, routes, trigger = OperatorADC.validate(value)
                value = dict(config=config, routes=routes, probe_trigger=trigger)
            elif section == 'probe':
                value = validate_probe(value)
            elif section == 'arm':
                ident = value.get('servo_id')
                if type(ident) is not int or not 0 <= ident <= 252:
                    raise ValueError('Invalid servo ID')
                value = dict(servo_id=ident, **limits(**{k:value[k] for k in ('minimum','center','maximum')}))
            elif section in ('arm_motion', 'probe_motion'):
                speed, acceleration = value.get('max_speed_deg_s'), value.get('acceleration_deg_s2')
                if not all(type(v) in (int,float) and math.isfinite(v) for v in (speed,acceleration)) or not .684 <= speed <= 699.732 or not 8.583 <= acceleration <= 2180.082:
                    raise ValueError('Invalid motion limits')
                value = dict(max_speed_deg_s=speed, acceleration_deg_s2=acceleration)
            elif section == 'metal_detector':
                baseline, full, reference = (value.get(k) for k in ('baseline_adc','full_response_adc','reference_voltage'))
                if not all(type(v) in (int,float) and math.isfinite(v) for v in (baseline,full,reference)) or not (0 <= baseline <= 255 and 0 <= full <= 255 and abs(full-baseline) >= 1) or not 0 < reference <= 5.5:
                    raise ValueError('Invalid detector calibration')
            else:
                raise ValueError('Unknown settings section')
            if section in ('arm','probe'):
                other='probe' if section=='arm' else 'arm'
                if value['servo_id']==configuration.actuator_settings(self.path,simulation).get(other,{}).get('servo_id',2 if other=='probe' else 1):
                    raise ValueError('Servo is already assigned to the other actuator')
            configuration.save_section(self.path, storage_section, value, request.expected_revision)
            response.success, response.message = True, 'Saved; owner will report applied state'
        except (ValueError, TypeError, KeyError, OSError) as exc:
            response.success, response.message = False, str(exc)
        try:
            response.revision = configuration.load(self.path).get('revision', 0)
        except (ValueError,OSError):
            response.revision = 0
        return response

    def reload(self):
        try:
            saved = configuration.load(self.path)
            self.document = saved
            self.saved = 'cameras' in saved
            self.cameras = camera_settings.validate(saved.get('cameras', camera_settings.DEFAULTS))
            self.error = None
        except (OSError, ValueError, TypeError) as exc:
            self.error = str(exc)

    def command(self, message):
        request_id = None
        try:
            command = json.loads(message.data)
            if not isinstance(command, dict):
                raise ValueError('Expected settings command')
            request_id = command.get('request_id')
            if not isinstance(request_id, str) or not 1 <= len(request_id) <= 100:
                raise ValueError('Invalid request ID')
            if command.get('section') != 'cameras':
                raise ValueError('Only shared camera settings are managed here')
            value = camera_settings.validate(command.get('value'))
            configuration.save_section(self.path, 'cameras', value)
            self.cameras = value
            self.error = None
            result = dict(ok=True, message='Camera defaults saved. Restart the camera nodes to apply capture changes.')
        except (OSError, ValueError, TypeError) as exc:
            result = dict(ok=False, message=str(exc))
        self.result_pub.publish(String(data=json.dumps(dict(request_id=request_id, **result))))
        self.publish_state()

    def publish_state(self):
        self.reload()
        self.state_pub.publish(String(data=json.dumps(dict(cameras=self.cameras, saved=self.saved, error=self.error,
                                                          config_file=self.path, revision=self.document.get('revision',0),
                                                          schema_version=self.document.get('schema_version',1),
                                                          camera_capture_apply_mode='restart'))))


if __name__ == '__main__':
    run_node(OperatorSettings)
