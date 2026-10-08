#!/usr/bin/env python3
"""ZED-F9P telemetry and optional NTRIP corrections for the operator dashboard."""
import json
import os
import threading
from rclpy.node import Node
from std_msgs.msg import String
from peaceofmine_operator.gnss import GnssState, serial_reader, ntrip_reader
from peaceofmine_operator.node_runner import run_node


class GNSSNode(Node):
    def __init__(self):
        super().__init__('operator_gnss')
        self.simulation = self.declare_parameter('simulation', False).value
        device = self.declare_parameter('device', '/dev/ttyAMA0').value
        baud = self.declare_parameter('baud', 115200).value
        env_file = self.declare_parameter('ntrip_env_file', '').value
        self.state = GnssState(device, baud)
        self.threads = []
        self.publisher = self.create_publisher(String, 'gnss/state', 10)
        if not self.simulation:
            if env_file:
                from dotenv import load_dotenv
                load_dotenv(env_file)
            username, password = os.getenv('NTRIP_USERNAME', ''), os.getenv('NTRIP_PASSWORD', '')
            ntrip_port = int(os.getenv('NTRIP_PORT', '80'))
            self.start(serial_reader, self.state)
            if username and password:
                self.start(ntrip_reader, self.state, os.getenv('NTRIP_HOST', 'nrtk-swepos.lm.se'),
                           ntrip_port, os.getenv('NTRIP_MOUNTPOINT', 'MSM_GNSS'), username, password)
            else:
                self.state.set_ntrip('Credentials not configured')
        self.create_timer(.2, self.publish)

    def start(self, target, *args):
        thread = threading.Thread(target=target, args=args, daemon=True)
        self.threads.append(thread)
        thread.start()

    def publish(self):
        if self.simulation:
            self.state.update(latitude=59.35, longitude=18.07, fix_quality=1,
                              fix_label='GPS FIX', satellites=12, altitude_m=30.0,
                              horizontal_accuracy_m=1.5, accuracy_source='Simulation', hdop=1.1)
        value = self.state.snapshot()
        value['simulation'] = self.simulation
        value.pop('raw_log', None)
        value.pop('event_log', None)
        self.publisher.publish(String(data=json.dumps(value, allow_nan=False)))

    def destroy_node(self):
        self.state.stop.set()
        for thread in self.threads:
            thread.join(timeout=2.5)
        return super().destroy_node()


def main():
    run_node(GNSSNode)


if __name__ == '__main__':
    main()
