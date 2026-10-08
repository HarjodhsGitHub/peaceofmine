#!/usr/bin/env python3
"""Sole ZED-F9P UART owner; GNSS health is independent of vehicle authority."""
import json
import math
import os
import threading
import time

from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import String
from peaceofmine_operator.gnss import GnssState, serial_reader, ntrip_reader
from peaceofmine_operator.node_runner import run_node


def presentation(state, now=None):
    """Expire individual fields so retained coordinates never look like a live fix."""
    now = time.time() if now is None else now
    value = state.snapshot()
    ages = value.get('field_age_s', {})
    for key in ('latitude', 'longitude', 'altitude_m', 'horizontal_accuracy_m',
                'hdop', 'vdop', 'pdop', 'speed_mps', 'course_deg', 'satellites'):
        if ages.get(key, float('inf')) > 3:
            value[key] = None
    value['fresh'] = bool(value['connected'] and value['age_s'] is not None
                          and value['age_s'] <= 3 and value['latitude'] is not None
                          and value['longitude'] is not None and ages.get('fix_quality', 999) <= 3)
    value['stale'] = not value['fresh']
    if not value['fresh']:
        value['fix_quality'] = 0
        value['fix_label'] = 'UNAVAILABLE'
    return value


class GNSSNode(Node):
    def __init__(self):
        super().__init__('gnss')
        self.simulation = self.declare_parameter('simulation', False).value
        device = self.declare_parameter('device', '/dev/ttyAMA0').value
        baud = self.declare_parameter('baud', 115200).value
        self.frame = self.declare_parameter('frame_id', 'gnss_link').value
        self.state = GnssState(device, baud)
        self.threads = []
        self.state_pub = self.create_publisher(String, 'gnss/state', 10)
        self.fix_pub = self.create_publisher(NavSatFix, 'gnss/fix', 10)
        if not self.simulation:
            self.start(serial_reader, self.state)
            username, password = os.getenv('NTRIP_USERNAME', ''), os.getenv('NTRIP_PASSWORD', '')
            if username and password:
                self.start(ntrip_reader, self.state, os.getenv('NTRIP_HOST', 'nrtk-swepos.lm.se'),
                           int(os.getenv('NTRIP_PORT', '80')), os.getenv('NTRIP_MOUNTPOINT', 'MSM_GNSS'),
                           username, password)
        self.create_timer(.5, self.publish)

    def start(self, target, *args):
        thread = threading.Thread(target=target, args=args, daemon=True)
        self.threads.append(thread)
        thread.start()

    def publish(self):
        if self.simulation:
            self.state.update(latitude=59.347, longitude=18.073, altitude_m=20.,
                              fix_quality=4, fix_label='RTK FIXED', fix_mode='3D', satellites=18,
                              horizontal_accuracy_m=.02, accuracy_source='simulated', speed_mps=0.,
                              course_deg=None, last_message='Simulated GNSS · no physical UART')
        value = presentation(self.state)
        value['simulation'] = self.simulation
        self.state_pub.publish(String(data=json.dumps(value, allow_nan=False)))
        fix = NavSatFix()
        # Receive time is the timestamp available from this NMEA/UBX backend.
        from builtin_interfaces.msg import Time
        received = self.state.field_updated.get('latitude', time.time())
        sec = int(received)
        fix.header.stamp = Time(sec=sec, nanosec=int((received-sec)*1e9))
        fix.header.frame_id = self.frame
        fix.status.service = NavSatStatus.SERVICE_GPS
        valid = value['fresh'] and value['fix_quality'] > 0
        fix.status.status = NavSatStatus.STATUS_FIX if valid else NavSatStatus.STATUS_NO_FIX
        fix.latitude = float(value['latitude']) if valid else math.nan
        fix.longitude = float(value['longitude']) if valid else math.nan
        fix.altitude = float(value['altitude_m']) if valid and value['altitude_m'] is not None else math.nan
        # hAcc is a horizontal uncertainty radius, not per-axis 1-sigma variance.
        # Publish unknown covariance rather than inventing a statistical model.
        fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
        self.fix_pub.publish(fix)

    def destroy_node(self):
        self.state.stop_event.set()
        with self.state.lock:
            if self.state.port:
                self.state.port.close()
        for thread in self.threads:
            thread.join(timeout=2)
        return super().destroy_node()


def main():
    run_node(GNSSNode)


if __name__ == '__main__':
    main()
