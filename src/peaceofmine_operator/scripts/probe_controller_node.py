#!/usr/bin/env python3
"""Probe homing, depth and contact policy over typed driver telemetry."""
import math
import time
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistWithCovarianceStamped
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool, Float32
from peaceofmine_interfaces.msg import AdcSamples, ContactState
from peaceofmine_operator.actuator_controller import ActuatorController
from peaceofmine_operator.adc_contact import AdcContact
from peaceofmine_operator.probe_contact import ProbeContact
from peaceofmine_operator import configuration, probe_calibration
from peaceofmine_operator.node_runner import run_node


class ProbeController(ActuatorController):
    role = 'probe'
    supported_actions = ('home','target')

    def __init__(self, **kwargs):
        self.home_position = None
        self.home_status = 'Not homed'
        self.motion = None
        self.home_hold = None
        self.target = 0.
        self.started = 0.
        self.fault = False
        self.speed = 0.
        self.odometry_at = 0.
        self.history = []
        super().__init__(**kwargs)
        self.extension = probe_calibration.load(self.config_file,self.simulation)
        self.apply_probe_motion(self.saved.get('probe_motion',dict(max_speed_deg_s=self.motion_speed*.684,acceleration_deg_s2=34.332)))
        self.speed_limit = self.declare_parameter('probe_motion_speed_limit_mps', .03).value
        self.contact = AdcContact()
        self.load_contact = ProbeContact()
        self.hold_target = True
        self.adc_revision = 0
        self.reload_contact()
        self.depth_pub = self.create_publisher(Float32, 'probe/depth_mm', 10)
        self.pressure_pub = self.create_publisher(Float32, 'probe/pressure_ratio', 10)
        self.contact_pub = self.create_publisher(ContactState, 'probe/contact', 1)
        velocity_topic = self.declare_parameter('velocity_topic','').value
        odometry_topic = self.declare_parameter('odometry_topic','odometry/local').value
        self.velocity_stamp = -1
        if velocity_topic:
            self.create_subscription(TwistWithCovarianceStamped, velocity_topic, self.odometry_cb, qos_profile_sensor_data)
        else:
            self.create_subscription(Odometry, odometry_topic, self.odometry_cb, qos_profile_sensor_data)
        self.create_subscription(Bool, 'probe/fault', lambda msg:setattr(self, 'fault', msg.data), 1)
        self.create_subscription(AdcSamples, 'adc/samples', self.adc_cb, 10)
        self.create_timer(1., self.reload_contact)

    def apply_probe_motion(self, value):
        speed, acceleration=value.get('max_speed_deg_s'),value.get('acceleration_deg_s2')
        if not all(type(v) in (int,float) and math.isfinite(v) for v in (speed,acceleration)) or not .684 <= speed <= 699.732 or not 8.583 <= acceleration <= 2180.082:
            raise ValueError('Invalid probe motion limits')
        self.motion_speed=max(1,math.floor(speed/.684+1e-9))
        self.acceleration=max(1,math.floor(acceleration/8.583+1e-9))

    def apply_saved_settings(self):
        self.extension=probe_calibration.load(self.config_file,self.simulation)
        if 'probe_motion' in self.saved:
            self.apply_probe_motion(self.saved['probe_motion'])

    def reload_contact(self):
        try:
            saved = configuration.load(self.config_file)
        except (ValueError,OSError) as exc:
            self.contact.received=0.
            self.reason=str(exc)
            return
        from peaceofmine_operator.lean_migration import migrated
        saved = migrated(saved, self.simulation)
        adc = saved.get('adc_simulation' if self.simulation else 'adc', {})
        config = adc.get('config', {})
        scan = sum(c['enabled'] for c in config.get('channels', []))/config.get('rate',128)
        self.contact.configure(adc.get('probe_trigger', self.contact.config),
            max(.5, config.get('interval_ms',0)/1000+3*scan))
        revision=saved.get('section_revisions',{}).get('adc_simulation' if self.simulation else 'adc',0)
        if revision != self.adc_revision:
            self.contact.received=0.
        self.adc_revision=revision

    def adc_cb(self, msg):
        if not msg.valid:
            self.contact.received=0.
            return
        if msg.config_revision != self.adc_revision:
            self.reload_contact()
        if msg.config_revision != self.adc_revision:
            return
        for sample in msg.samples:
            age=time.time()-(sample.stamp.sec+sample.stamp.nanosec/1e9)
            if sample.channel == self.contact.config['mux'] and 0 <= age <= self.contact.max_age:
                self.contact.sample(sample.scaled, sample.sequence, msg.session)

    def odometry_cb(self, msg):
        stamp = msg.header.stamp.sec*1000000000+msg.header.stamp.nanosec
        if stamp <= self.velocity_stamp:
            return
        self.velocity_stamp = stamp
        twist = msg.twist.twist
        values = (twist.linear.x, twist.linear.y, twist.linear.z, twist.angular.x, twist.angular.y, twist.angular.z)
        self.speed = max(abs(v) for v in values) if all(math.isfinite(v) for v in values) else math.inf
        self.odometry_at = time.monotonic()

    def stationary(self):
        return time.monotonic()-self.odometry_at < .5 and self.speed <= self.speed_limit

    def interlock(self):
        return super().interlock() and not self.fault

    def calibration(self):
        return None

    def on_disconnect(self):
        self.home_position = None
        self.home_status = 'Home required after driver loss'

    def stop(self, reason='Stopped; torque released', success=False):
        if self.motion == 'home' and not success:
            self.home_position = None
            self.home_status = 'Homing interrupted; zero not set'
        self.motion = None
        super().stop(reason, success)

    def start_home(self, hold, threshold):
        if hold == self.home_hold:
            if self.motion == 'home':
                self.command_at = time.monotonic()
            return
        if type(threshold) not in (int,float) or not math.isfinite(threshold) or not 10 <= threshold <= 60:
            raise ValueError('Home load threshold must be 10..60 percent')
        self.start(self.telemetry.maximum, self.telemetry.minimum, self.telemetry.maximum, load=threshold)
        self.home_hold, self.home_position = hold, None
        self.motion, self.started = 'home', time.monotonic()
        self.home_status, self.lifecycle = 'Seeking top; release to stop', 'HOMING'

    def start_target(self, depth, hold=True):
        calibration = self.extension
        if self.home_position is None or not calibration or calibration['servo_id'] != self.ident:
            raise ValueError('Home and calibrate probe before extension')
        if type(depth) not in (int,float) or not math.isfinite(depth) or not 0 <= depth <= calibration['max_extension_mm']:
            raise ValueError('Probe depth outside calibrated range')
        low, high = self.home_position-calibration['travel_ticks'], self.home_position
        if not self.telemetry.minimum <= low < high <= self.telemetry.maximum:
            raise ValueError('Probe travel exceeds encoder range')
        target = round(high-depth/calibration['max_extension_mm']*calibration['travel_ticks'])
        self.start(target, low, high)
        self.target, self.motion, self.started = depth, 'target', time.monotonic()
        self.hold_target = hold

    def special_command(self, action, command):
        if action == 'home':
            if command.get('held') is not True or not self.interlock():
                raise ValueError('Homing requires held command, RC permission and no fault')
            hold = command.get('hold_id')
            if not isinstance(hold,str) or not hold:
                raise ValueError('Home requires a unique hold identifier')
            self.start_home(hold, command.get('load_percent'))
        elif action == 'target':
            self.start_target(command.get('depth_mm'))
        elif action == 'save_probe_extension':
            if self.operation or self.telemetry.torque or self.home_position is None:
                raise ValueError('Home and stop before saving maximum extension')
            value = probe_calibration.validate(dict(servo_id=self.ident, max_extension_mm=command.get('max_extension_mm'),
                travel_ticks=self.home_position-self.telemetry.position))
            self.save(command, 'probe', value, lambda:setattr(self,'extension',value))
            return True
        elif action == 'zero_contact':
            if not self.interlock():
                raise ValueError('Contact zero requires RC permission and no fault')
            self.load_contact.zero(time.monotonic())
            self.reason = 'Motor contact baseline zeroed for this session'
        elif action == 'configure_motion':
            if self.operation or self.telemetry.torque:
                raise ValueError('Stop before changing probe limits')
            old=self.motion_speed,self.acceleration
            self.apply_probe_motion(command)
            self.motion_speed,self.acceleration=old
            value={k:command[k] for k in ('max_speed_deg_s','acceleration_deg_s2')}
            self.save(command,'probe_motion',value,lambda:self.apply_probe_motion(value))
            return True
        elif action in ('configure','capture'):
            raise ValueError('Use probe homing and extension calibration')
        else:
            return False
        self.reply(command, True)
        return True

    def start_action(self, request):
        if request.operation == 'home':
            import uuid
            self.start_home(uuid.uuid4().hex, request.load_percent)
        else:
            self.start_target(request.target, hold=False)

    def operation_finished(self, outcome, message):
        if self.motion == 'home' and outcome == 'load_stop':
            position = self.telemetry.position
            self.stop('Home reached; torque released', success=True)
            self.home_position, self.home_status = position, 'Homed; verify top visually'
        else:
            self.stop(message or outcome)

    def update_behavior(self):
        if not self.operation:
            return
        if self.motion and time.monotonic()-self.started >= 60:
            self.stop('Probe operation timed out')
        elif self.motion == 'target':
            self.command_at = time.monotonic()
            if self.telemetry.operation_id == self.operation.operation_id and abs(self.telemetry.position-self.operation.target) <= 3:
                if self.hold_target:
                    self.lifecycle, self.reason = 'HOLDING', 'Holding requested depth; stop releases torque'
                else:
                    self.stop('Probe target reached; torque released', success=True)
        elif self.motion == 'home':
            if self.action_handle:
                self.command_at = time.monotonic()
            if self.telemetry.position >= self.telemetry.maximum-3:
                self.stop('Encoder limit reached without confirmed home contact')

    def behavior_state(self):
        now = time.monotonic()
        ready = bool(self.connected() and self.extension and self.extension['servo_id']==self.ident and self.home_position is not None)
        depth = (self.home_position-self.telemetry.position)*self.extension['max_extension_mm']/self.extension['travel_ticks'] if ready else None
        if depth is not None:
            self.depth_pub.publish(Float32(data=depth))
        ratio = min(1., abs(self.telemetry.load_percent)/100) if self.connected() else None
        if ratio is not None:
            self.pressure_pub.publish(Float32(data=ratio))
            self.history.append((now,ratio))
        self.history = [(t,v) for t,v in self.history if now-t<=30][-700:]
        contact = self.contact.snapshot()
        msg = ContactState(valid=contact['fresh'], detected=contact['detected'] is True,
            channel=contact['mux'], value=float(contact['value'] or 0), threshold=float(contact['level']),
            reason='Fresh' if contact['fresh'] else 'Contact input disabled or stale')
        msg.header.stamp = self.get_clock().now().to_msg()
        self.contact_pub.publish(msg)
        self.load_contact.update(dict(connected=self.connected(), active_role='probe',
            sample_time=self.driver_at, serial_port=self.driver.device if self.driver else '',servo_id=self.ident,
            load_percent=self.telemetry.load_percent if self.telemetry else None, current_a=self.telemetry.current_a if self.telemetry else None,
            position=self.telemetry.position if self.telemetry else None, goal_position=self.telemetry.goal if self.telemetry else None,
            torque=bool(self.telemetry and self.telemetry.torque), probe=dict(holding=self.lifecycle=='HOLDING')), now)
        return dict(sweep_acceleration_deg_s2=self.acceleration*8.583, home_position=self.home_position, home_status=self.home_status, homing=self.motion=='home',
            probe=dict(ready=ready, homed=self.home_position is not None, calibrated=bool(self.extension),
                max_depth_mm=self.extension['max_extension_mm'] if self.extension else None,
                depth_mm=depth, target_mm=self.target, active=self.motion=='target', moving=self.motion=='target' and self.lifecycle!='HOLDING', holding=self.lifecycle=='HOLDING',
                reason=self.reason, fault=self.fault, pressure_ratio=ratio, pressure_window_s=30,
                pressure_samples=[dict(x=t-now,y=v*100) for t,v in self.history],
                **self.load_contact.snapshot(now), adc_contact=contact['detected'], adc_contact_fresh=contact['fresh'], adc_contact_state=contact))


def main():
    run_node(ProbeController)


if __name__ == '__main__':
    main()
