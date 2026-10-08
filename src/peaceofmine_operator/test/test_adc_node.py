"""Real ROS topics and gateway integration, with simulated ADC only."""
import importlib.util
import json
from pathlib import Path
import tempfile
import time
import unittest

try:
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from std_msgs.msg import Bool, Float64, Int16, String
except ImportError:
    rclpy = None


def script(name):
    path = Path(__file__).resolve().parents[1] / 'scripts' / (name + '.py')
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@unittest.skipIf(rclpy is None, 'ROS environment required')
class ADCNodeTest(unittest.TestCase):
    def test_topics_commands_contact_and_gateway(self):
        with tempfile.TemporaryDirectory() as tmp:
            rclpy.init(args=['--ros-args', '-p', 'simulation:=true', '-p', 'safety_simulation:=true',
                            '-p', f'config_file:={tmp}/operator_config.json', '-r', '__ns:=/adc_test'])
            node = script('ads1115_node').ADS1115Node()
            gateway = script('operator_gateway').OperatorGateway()
            control = script('operator_control_node').OperatorControl()
            probe = script('probe_controller_node').ProbeController()
            settings_node = script('operator_settings_node').OperatorSettings()
            observer = rclpy.create_node('adc_observer')
            executor = SingleThreadedExecutor()
            for n in (node, gateway, control, observer, probe, settings_node):
                executor.add_node(n)
            raw, volts, scaled, contact, fresh = [], [], [], [], []
            subscriptions = [observer.create_subscription(kind, topic, lambda m, out=out: out.append(m.data), 10)
                             for kind, topic, out in [(Int16, 'adc/a0/raw', raw), (Float64, 'adc/a0/volts', volts),
                                 (Float64, 'adc/a0/scaled', scaled)]]

            from peaceofmine_interfaces.msg import ContactState
            observer.create_subscription(ContactState, 'probe/contact', lambda m:(fresh.append(m.valid),contact.append(m.detected)),10)

            def spin(seconds):
                end = time.monotonic() + seconds
                while time.monotonic() < end:
                    executor.spin_once(timeout_sec=.01)

            try:
                spin(.4)
                state = gateway.adc.snapshot()
                self.assertTrue(state['demo'])
                self.assertTrue(state['running'])
                ws = object()
                gateway.connect_client(ws)
                spin(.2)
                gateway.handle_command(ws, {'type': 'take_control'})
                spin(.2)
                settings = {key: state[key] for key in ('config', 'routes', 'probe_trigger')}
                settings['probe_trigger'].update(enabled=True, level=.1)
                gateway.handle_command(ws, dict(type='adc', action='settings', settings=settings))
                spin(.3)
                self.assertTrue(gateway.replies()[-1][1]['ok'])
                gateway.handle_command(ws, dict(type='adc', action='run', running=True))
                spin(1.2)
                self.assertTrue(gateway.replies()[-1][1]['ok'])
                self.assertTrue(raw and volts and scaled)
                self.assertTrue(fresh[-1])
                self.assertTrue(contact[-1])
                state = gateway.adc.snapshot()
                self.assertGreater(len(state['samples']), 4)
                self.assertTrue(state['probe_contact']['detected'])
                self.assertEqual(gateway.adc.snapshot(state['sequence'])['samples'], [])
                gateway.handle_command(ws, dict(type='adc', action='run', running=False))
                spin(.7)
                self.assertTrue(gateway.replies()[-1][1]['ok'])
                self.assertFalse(fresh[-1])
                self.assertFalse(contact[-1])
            finally:
                executor.shutdown()
                for n in (node, gateway, control, observer, probe, settings_node):
                    n.destroy_node()
                rclpy.shutdown()
