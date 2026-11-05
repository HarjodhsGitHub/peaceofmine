"""Downstream pump watchdog, independent of the operator control process."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import Mock
from rclpy.time import Time
from std_msgs.msg import Bool


class PumpWatchdogTest(unittest.TestCase):
    def test_pump_turns_off_when_control_heartbeat_stops(self):
        path = Path(__file__).parents[2] / 'svea_examples/scripts/twist_consumer.py'
        spec = importlib.util.spec_from_file_location('twist_watchdog', path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        now = [Time(nanoseconds=1_000_000_000)]
        consumer = SimpleNamespace(get_clock=lambda: SimpleNamespace(now=lambda: now[0]),
            actuation=Mock(), _pump_until=None, _last_msg=now[0], cmd_timeout=.6,
            _velocity=0., _steering=0., steering_sign=1., velocity_sign=1.)
        module.twist_consumer.pump_cb.callback(consumer, Bool(data=True))
        consumer.actuation.send_xtr.assert_called_with(xtr1=100.)
        now[0] = Time(nanoseconds=1_300_000_000)
        module.twist_consumer.loop.callback(consumer)
        consumer.actuation.send_xtr.assert_called_with(xtr1=0.)
        self.assertIsNone(consumer._pump_until)
