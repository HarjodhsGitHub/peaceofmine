#!/usr/bin/env python3
"""ADS1115 acquisition and timestamped ROS publication; probe owns contact policy."""
import json
import uuid
from pathlib import Path

from peaceofmine_operator.node_runner import run_node
from rclpy.node import Node
from std_msgs.msg import Float64, Int16, String
from peaceofmine_interfaces.msg import AdcSamples, AdcSample
from builtin_interfaces.msg import Time
from peaceofmine_operator.settings_client import SettingsClient

from peaceofmine_operator.adc_acquisition import OperatorADC
from peaceofmine_operator import configuration

INPUT_TOPICS = ('a0_a1', 'a0_a3', 'a1_a3', 'a2_a3', 'a0', 'a1', 'a2', 'a3')


class ADS1115Node(Node):
    def __init__(self):
        super().__init__('ads1115')
        demo = self.declare_parameter('simulation', False).value
        path = self.declare_parameter('settings_path', '').value
        config_file = self.declare_parameter('config_file', '').value or configuration.default_path()
        # Import the earlier standalone operator ADC file once, without deleting it.
        legacy = Path(path).expanduser() if path else Path('.operator') / ('ads1115-sim.json' if demo else 'ads1115.json')
        section = 'adc_simulation' if demo else 'adc'
        saved = configuration.load(config_file)
        legacy_value = None
        if section not in saved and legacy.exists():
            value = json.loads(legacy.read_text())
            config, routes, trigger = OperatorADC.validate(value)
            legacy_value = dict(config=config,routes=routes,probe_trigger=trigger)
            # Legacy file is read-only; persist through the settings service on the next save.
        self.adc = OperatorADC(config_file, demo=demo)
        if legacy_value:
            self.adc.command(dict(action='settings',settings=legacy_value),persist=False)
        self.legacy_value = legacy_value
        self.settings = SettingsClient(self, config_file)
        self.config_file = config_file
        self.config_revision = saved.get('section_revisions',{}).get(section,0)
        self.samples_pub = self.create_publisher(AdcSamples, 'adc/samples', 10)
        self.session = uuid.uuid4().hex
        self.cursor = 0
        self.state_pub = self.create_publisher(String, 'adc/state', 10)
        self.result_pub = self.create_publisher(String, 'adc/command_result', 10)
        self.input_pubs = [(
            self.create_publisher(Int16, f'adc/{name}/raw', 10),
            self.create_publisher(Float64, f'adc/{name}/volts', 10),
            self.create_publisher(Float64, f'adc/{name}/scaled', 10),
        ) for name in INPUT_TOPICS]
        self.create_subscription(String, 'adc/command', self.command, 10)
        self.create_timer(.05, self.publish_state)
        self.create_timer(.5, self.reload_settings)

    def reload_settings(self):
        try:
            if self.legacy_value and not self.settings.pending and self.settings.client.service_is_ready():
                value,self.legacy_value=self.legacy_value,None
                self.settings.save(self.adc.section,value,lambda ok,message:None)
            saved=configuration.load(self.config_file)
            revision=saved.get('section_revisions',{}).get(self.adc.section,0)
            if revision != self.config_revision and self.adc.section in saved:
                self.adc.command(dict(action='settings',settings=saved[self.adc.section]),persist=False)
                self.config_revision=revision
        except (OSError,ValueError,TypeError) as exc:
            self.adc.load_error=str(exc)

    def command(self, message):
        request_id = None
        try:
            value = json.loads(message.data)
            if not isinstance(value, dict):
                raise ValueError('Expected ADC command object')
            request_id = value.get('request_id')
            if not isinstance(request_id, str) or not 1 <= len(request_id) <= 100:
                raise ValueError('Expected request_id of 1..100 characters')
            if value.get('action') == 'settings':
                OperatorADC.validate(value.get('settings'))
                def complete(ok, message):
                    if ok:
                        self.adc.command(value, persist=False)
                        self.config_revision = configuration.load(self.config_file).get('section_revisions',{}).get(self.adc.section,0)
                    self.result_pub.publish(String(data=json.dumps(dict(request_id=request_id, ok=ok, message=message))))
                self.settings.save(self.adc.section, value['settings'], complete)
                return
            self.adc.command(value, persist=False)
            result = dict(ok=True)
        except (ValueError, TypeError, OSError, TimeoutError) as exc:
            result = dict(ok=False, message=str(exc))
        self.result_pub.publish(String(data=json.dumps(dict(request_id=request_id, **result))))

    def publish_state(self):
        state = self.adc.snapshot(self.cursor)
        state['session'] = self.session
        state['saved_config_revision'] = self.config_revision
        if state['samples']:
            self.cursor = state['samples'][-1]['seq']
        # Every acquired sample, with epoch timestamp and sequence, travels in
        # adc/state. Scalar topics publish the newest reading per input per tick.
        latest = {sample['mux']: sample for sample in state['samples']}
        for mux, sample in latest.items():
            raw, volts, scaled = self.input_pubs[mux]
            raw.publish(Int16(data=sample['raw']))
            volts.publish(Float64(data=sample['volts']))
            scaled.publish(Float64(data=sample['scaled']))
        packet = AdcSamples(session=self.session, config_revision=self.config_revision,
            valid=bool(state['running'] and state['connected'] and state['applied']==state['revision']))
        packet.header.stamp = self.get_clock().now().to_msg()
        for sample in state['samples']:
            sec = int(sample['time'])
            packet.samples.append(AdcSample(stamp=Time(sec=sec, nanosec=int((sample['time']-sec)*1e9)),
                sequence=sample['seq'], channel=sample['mux'], raw=sample['raw'],
                volts=float(sample['volts']), scaled=float(sample['scaled'])))
        self.samples_pub.publish(packet)
        self.state_pub.publish(String(data=json.dumps(state, allow_nan=False)))

    def destroy_node(self):
        self.adc.close()
        return super().destroy_node()


def main():
    run_node(ADS1115Node)


if __name__ == '__main__':
    main()
