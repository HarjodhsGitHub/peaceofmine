#!/usr/bin/env python3
"""Interpret the shared acquisition's A3 samples; never open another I²C driver."""
import json
from rclpy.node import Node
from std_msgs.msg import String, Float32
from peaceofmine_interfaces.msg import AdcSamples
from peaceofmine_operator import configuration
from peaceofmine_operator.adc_detector import ADCDetector, validate_calibration
from peaceofmine_operator.settings_client import SettingsClient
from peaceofmine_operator.node_runner import run_node


class ADCDetectorNode(Node):
    def __init__(self):
        super().__init__('adc_detector')
        self.simulation = self.declare_parameter('simulation', False).value
        self.path = self.declare_parameter('config_file', '').value or configuration.default_path()
        self.section = 'metal_detector_adc_simulation' if self.simulation else 'metal_detector_adc'
        saved = configuration.load(self.path).get(self.section)
        if self.simulation and saved is None:
            saved = dict(baseline_volts=1.3, trigger_volts=1.5)
        self.sensor = ADCDetector(saved)
        self.settings = SettingsClient(self, self.path)
        self.result = None
        self.pub = self.create_publisher(String, 'detector/state', 10)
        self.ratio = self.create_publisher(Float32, 'detector/signal_ratio', 10)
        self.create_subscription(AdcSamples, 'adc/samples', self.samples, 10)
        self.create_subscription(String, 'detector/command', self.command, 10)
        self.create_timer(.1, self.publish)

    def samples(self, message):
        samples = [dict(mux=s.channel, seq=s.sequence, time=s.stamp.sec+s.stamp.nanosec/1e9,
                        volts=s.volts, clipped=s.raw in (-32768, 32767)) for s in message.samples]
        self.sensor.update(samples, message.valid, message.session)
        value = self.sensor.snapshot(include_history=False)['signal_ratio']
        if value is not None:
            self.ratio.publish(Float32(data=value))

    def command(self, message):
        request_id = None
        try:
            command = json.loads(message.data)
            request_id = command.get('request_id')
            if not isinstance(request_id, str) or not 1 <= len(request_id) <= 100:
                raise ValueError('Invalid request ID')
            if not self.sensor.snapshot(include_history=False)['fresh']:
                raise ValueError('Fresh A3 readings required')
            action = command.get('action')
            if action == 'apply':
                calibration = dict(baseline_volts=command.get('baseline_adc'), trigger_volts=command.get('full_response_adc'))
            elif action == 'zero':
                if not self.sensor.calibration:
                    raise ValueError('Set a new voltage trigger before zeroing')
                recent = [v for at,v,_ in self.sensor.samples if self.sensor.clock()-at < 10]
                if len(recent) < 5:
                    raise ValueError('Wait for five fresh A3 readings')
                calibration = {**self.sensor.calibration, 'baseline_volts':sum(recent)/len(recent)}
            elif action == 'reset':
                def cleared(ok, message):
                    if ok: self.sensor.calibration = None
                    self.result = dict(request_id=request_id, ok=ok, message=message)
                self.settings.save('metal_detector_adc', {'clear':True}, cleared)
                return
            else:
                raise ValueError('Unknown calibration action')
            calibration = validate_calibration(calibration)
            def complete(ok, message):
                if ok:
                    self.sensor.calibration = calibration
                self.result = dict(request_id=request_id, ok=ok, message=message)
            self.settings.save('metal_detector_adc', calibration, complete)
        except (ValueError, TypeError, AttributeError) as exc:
            self.result = dict(request_id=request_id, ok=False, message=str(exc))

    def publish(self):
        value = self.sensor.snapshot(include_history=False)
        value['command_result'] = self.result
        self.pub.publish(String(data=json.dumps(value, allow_nan=False)))


def main():
    run_node(ADCDetectorNode)


if __name__ == '__main__':
    main()
