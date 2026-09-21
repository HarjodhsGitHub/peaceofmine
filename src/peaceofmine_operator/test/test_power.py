import json
import unittest
from types import SimpleNamespace as Msg
from peaceofmine_operator.power import PowerTelemetry


class PowerTest(unittest.TestCase):
    def setUp(self):
        self.now = 10
        self.power = PowerTelemetry(clock=lambda: self.now)

    def battery(self, **values):
        data = dict(present=True, voltage=12.0, current=-2.0, percentage=.5,
                    temperature=float('nan'), cell_voltage=[4.0, 3.95, 4.05])
        data.update(values)
        return Msg(**data)

    def test_battery_sign_units_and_unknown_values(self):
        self.power.update_battery(self.battery())
        data = self.power.snapshot()['battery']
        self.assertEqual(data['power_w'], 24)
        self.assertEqual(data['current_a'], 2)
        self.assertEqual(data['remaining_pct'], 50)
        self.assertIsNone(data['temperature_c'])
        self.assertAlmostEqual(data['cell_delta_v'], .1)
        self.power.update_battery(self.battery(current=1))
        self.assertEqual(self.power.snapshot()['battery']['power_w'], -12)
        self.power.update_battery(self.battery(current=float('nan'), percentage=-1))
        self.assertIsNone(self.power.snapshot()['battery']['power_w'])
        self.assertIsNone(self.power.snapshot()['battery']['remaining_pct'])
        json.dumps(self.power.snapshot(), allow_nan=False)

    def test_stale_and_missing_are_not_live_zero(self):
        self.assertFalse(self.power.snapshot()['battery']['available'])
        self.power.update_battery(self.battery())
        self.now += 4
        self.assertTrue(self.power.snapshot()['battery']['stale'])
        self.now += 60
        self.assertEqual(self.power.snapshot()['history'], [])

    def test_esc_online_status_and_unknown_voltage(self):
        motor = Msg(timestamp=10, esc_voltage=12., esc_current=3., esc_temperature=40., esc_rpm=1000, failures=0)
        self.power.update_esc(Msg(esc=[motor, motor], esc_count=2, esc_online_flags=1))
        motors = self.power.snapshot()['esc']['motors']
        self.assertEqual(motors[0]['power_w'], 36)
        self.assertIsNone(motors[1]['power_w'])
        motor.esc_voltage = 0
        self.power.update_esc(Msg(esc=[motor], esc_count=1, esc_online_flags=1))
        self.assertIsNone(self.power.snapshot()['esc']['motors'][0]['power_w'])
