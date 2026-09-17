"""Normalize measured power telemetry; unavailable values remain JSON null."""
import math
import time
from collections import deque


def finite(value):
    return float(value) if isinstance(value, (int, float)) and math.isfinite(value) else None


class PowerTelemetry:
    def __init__(self, clock=time.monotonic):
        self.clock = clock
        self.battery = None
        self.esc = None
        self.history = deque(maxlen=120)

    def update_battery(self, msg):
        now = self.clock()
        voltage = finite(msg.voltage)
        voltage = voltage if voltage is not None and voltage > 0 else None
        current = finite(msg.current)
        # sensor_msgs/BatteryState uses negative current for discharge.
        draw = -current if current is not None else None
        watts = voltage * draw if voltage is not None and draw is not None else None
        fraction = finite(msg.percentage)
        cells = [float(v) for v in msg.cell_voltage if math.isfinite(v) and v > 0]
        self.battery = dict(at=now, present=bool(msg.present), voltage_v=voltage,
                            current_a=draw, power_w=watts,
                            remaining_pct=fraction * 100 if fraction is not None and 0 <= fraction <= 1 else None,
                            temperature_c=finite(msg.temperature),
                            cell_min_v=min(cells) if cells else None,
                            cell_delta_v=max(cells) - min(cells) if len(cells) > 1 else None)
        if msg.present and watts is not None and (not self.history or now - self.history[-1][0] >= .5):
            self.history.append((now, watts))

    def update_esc(self, msg):
        rows = []
        for i, report in enumerate(msg.esc[:msg.esc_count]):
            online = bool(msg.esc_online_flags & (1 << i)) and report.timestamp > 0
            voltage = finite(report.esc_voltage) if online else None
            voltage = voltage if voltage is not None and voltage > 0 else None
            # Reports with no voltage cannot establish supported current/power.
            current = finite(report.esc_current) if voltage is not None else None
            rows.append(dict(id=i + 1, online=online, voltage_v=voltage, current_a=current,
                             power_w=voltage * current if current is not None else None,
                             temperature_c=finite(report.esc_temperature) if online else None,
                             rpm=report.esc_rpm if online else None,
                             faults=int(report.failures) if online else None))
        self.esc = dict(at=self.clock(), motors=rows)

    def snapshot(self):
        now = self.clock()
        def sample(value):
            if value is None:
                return dict(available=False, stale=False)
            return {k: v for k, v in value.items() if k != 'at'} | dict(
                available=True, stale=now - value['at'] > 3,
                age_ms=round((now - value['at']) * 1000))
        return dict(battery=sample(self.battery), esc=sample(self.esc),
                    history=[dict(age_s=now - t, watts=w) for t, w in self.history if now - t <= 60])
