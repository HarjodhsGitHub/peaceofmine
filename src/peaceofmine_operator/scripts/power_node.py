#!/usr/bin/env python3
"""Publish battery and ESC power telemetry independently of browser clients."""
import json
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String
from peaceofmine_operator.power import PowerTelemetry
from peaceofmine_operator.node_runner import run_node


class PowerNode(Node):
    def __init__(self):
        super().__init__('operator_power')
        self.telemetry = PowerTelemetry()
        self.publisher = self.create_publisher(String, 'power/state', 10)
        self.create_subscription(BatteryState, 'mavros/battery', self.telemetry.update_battery, qos_profile_sensor_data)
        try:
            from px4_msgs.msg import EscStatus
        except ImportError:
            pass
        else:
            self.create_subscription(EscStatus, 'px4/uorb/esc_status', self.telemetry.update_esc, qos_profile_sensor_data)
        self.create_timer(.1, self.publish_state)

    def publish_state(self):
        self.publisher.publish(String(data=json.dumps(self.telemetry.snapshot(), allow_nan=False)))


if __name__ == '__main__':
    run_node(PowerNode)
