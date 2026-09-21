#!/usr/bin/env python3
"""Pi-owned servo output: PX4 safety and gateway lease checked independently."""
import json
import math
import signal
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import rclpy
from mavros_msgs.msg import State, RCIn
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Bool, String, Float32
from peaceofmine_operator.arm_servo import ArbotiX, ArmServo, FirmwareMotionFault, ServoAlarm
from peaceofmine_operator.rc_safety import RcSafety


class ArmServoNode(Node):
    def __init__(self, **kwargs):
        super().__init__('arm_servo', **kwargs)
        for key, value in dict(serial_port='auto', servo_id=-1,
                               probe_servo_id=2, probe_minimum=-1, probe_center=-1, probe_maximum=-1,
                               minimum=-1, center=-1, maximum=-1, speed=20,
                               sweep_endpoint_tolerance_deg=2.0,
                               sweep_acceleration_deg_s2=40.0).items():
            self.declare_parameter(key, value)
        requested_speed = self.get_parameter('speed').value
        if requested_speed < 1:
            raise ValueError('Servo speed must be positive; zero means unlimited')
        self.motion_speed = min(1023, requested_speed)
        if requested_speed > 1023:
            self.get_logger().warning(f'Servo speed {requested_speed} exceeds firmware limit; using 1023')
        self.active_role = 'arm'
        self.role_ids = {role: self.get_parameter(key).value for role, key in
                         (('arm', 'servo_id'), ('probe', 'probe_servo_id'))}
        self.role_servos = {}
        self.saved_role_calibration = {}
        tolerance = float(self.get_parameter('sweep_endpoint_tolerance_deg').value)
        if not math.isfinite(tolerance) or not 0 < tolerance <= 10:
            raise ValueError('Sweep endpoint tolerance must be within 0..10 degrees')
        self.sweep_endpoint_tolerance = max(1, math.ceil(tolerance * 4096 / 360))
        acceleration = float(self.get_parameter('sweep_acceleration_deg_s2').value)
        if not math.isfinite(acceleration) or not 8.583 <= acceleration <= 254 * 8.583:
            raise ValueError('Sweep acceleration must be within 8.583..2180.082 degrees/s^2')
        self.sweep_acceleration = max(1, math.floor(acceleration / 8.583))
        self.safety = RcSafety()  # No hardware simulation bypass.
        self.sweeping = False
        self.sweep_speed = 5.0
        self.permission = False
        self.permission_at = self.command_at = 0.0
        self.bus = self.servo = None
        self.discovery_pool = ThreadPoolExecutor(max_workers=1)
        self.discovery = None
        self.discovery_started = False
        self.next_discovery_at = 0.0
        self.discovery_stop = threading.Event()
        self.discovered_servos = []
        self.device = ''
        self.reason = 'Looking for the arm adapter'
        self.connection_error = ''
        self.last_poll = 0.0
        self.state = {}
        self.last_command = {}
        self.create_subscription(State, 'mavros/state', self.state_cb, 10)
        self.create_subscription(RCIn, 'mavros/rc/in', self.rc_cb, 10)
        self.create_subscription(Bool, 'operator/actuation_enabled', self.permission_cb, 1)
        self.create_subscription(String, 'arm/command', self.command_cb, 1)
        self.create_subscription(Bool, 'fixture/sweep_enabled', self.sweep_cb, 1)
        self.create_subscription(Float32, 'fixture/sweep_speed_deg_s', self.sweep_speed_cb, 1)
        self.speed_pub = self.create_publisher(Float32, 'fixture/sweep_speed_deg_s_state', 1)
        self.angle_pub = self.create_publisher(Float32, 'fixture/angle_deg', 1)
        self.sweep_pub = self.create_publisher(Bool, 'fixture/sweep_enabled_state', 1)
        self.publisher = self.create_publisher(String, 'arm/state', 1)
        self.create_timer(.05, self.tick)

    def discover(self, configured):
        # This worker exclusively owns the port until its result is adopted by
        # tick(). Never scan concurrently with an active servo or motion writes.
        candidates = ([configured] if configured not in ('', 'auto') else
                      sorted(str(p) for p in Path('/dev/serial/by-id').glob('usb-FTDI*')))
        errors = []
        for device in candidates:
            bus = None
            try:
                if self.discovery_stop.is_set():
                    return None
                bus = ArbotiX(device, 1000000)
                found = []
                for ident in range(253):
                    if self.discovery_stop.is_set():
                        bus.close()
                        return None
                    try:
                        if bus.read(ident, 0) == 310:
                            found.append(ident)
                    except RuntimeError:
                        continue
                if found:
                    return bus, device, found
                errors.append(f'{device}: no MX-64 servos found')
            except Exception as error:
                errors.append(f'{device}: {error}')
            if bus:
                bus.close()
        raise RuntimeError('; '.join(errors) or 'No FTDI arm adapter found; connect it and restart the arm driver')

    def poll_discovery(self):
        if self.bus:
            return
        if self.discovery is not None:
            if not self.discovery.done():
                return
            future, self.discovery = self.discovery, None
            self.discovery_started = True
            result = future.result()
            if result is None:
                return
            self.bus, self.device, self.discovered_servos = result
            self.connection_error = ''
            self.get_logger().info(f'Arm discovery: {self.device}, servo IDs {self.discovered_servos}, 1000000 baud')
            ident = self.role_ids[self.active_role]
            if ident != -1:
                try:
                    self.select(ident)
                except ValueError as error:
                    self.reason = str(error)
                    self.get_logger().warning(self.reason)
            else:
                self.reason = 'Scan complete; select a detected servo'
        elif not self.discovery_started and time.monotonic() >= self.next_discovery_at:
            self.discovery_started = True
            self.reason = 'Scanning arm adapter and servo IDs…'
            self.discovery = self.discovery_pool.submit(
                self.discover, str(self.get_parameter('serial_port').value))

    def allowed(self):
        now = time.monotonic()
        return (self.safety.servo_snapshot()['allowed'] and self.permission
                and now - self.permission_at < .3 and now - self.command_at < .25)

    def stop(self):
        self.command_at = 0.0
        self.sweeping = False
        if self.servo and (self.servo.torque or self.servo.target is not None):
            self.servo.stop()

    def sweep_speed_cb(self, message):
        if not math.isfinite(message.data) or not self.permission or not self.safety.servo_snapshot()['allowed']:
            return
        # MX-64 Protocol 1.0 speed units are approximately 0.114 rpm.
        self.sweep_speed = max(.684, min(float(message.data), self.motion_speed * .684))

    def sweep_cb(self, message):
        try:
            if not message.data:
                self.stop()
                self.reason = 'Sweep stopped'
                return
            safety = self.safety.servo_snapshot()
            if not safety['allowed']:
                raise ValueError(safety['reason'])
            if not self.permission or time.monotonic() - self.permission_at >= .3:
                raise ValueError('Sweep blocked: waiting for control-owner permission')
            if self.active_role != 'arm':
                self.select_role('arm')
            if not self.servo:
                raise ValueError('Select an arm servo in Settings')
            if not self.servo.calibration:
                raise ValueError('Record and apply minimum, center and maximum before sweeping')
            self.sweeping = True
            self.reason = 'Sweeping between calibrated limits'
        except ValueError as error:
            self.stop()
            self.reason = str(error)
            self.get_logger().warning(self.reason)
        except Exception as error:
            self.fail(error)

    def state_cb(self, message):
        self.safety.update_state(message)
        self.enforce_safety()

    def rc_cb(self, message):
        self.safety.update_rc(message)
        self.enforce_safety()

    def enforce_safety(self):
        try:
            if not self.safety.servo_snapshot()['allowed']:
                self.stop()
        except Exception as error:
            self.safety.samples.clear()
            self.fail(error)

    def permission_cb(self, message):
        self.permission, self.permission_at = bool(message.data), time.monotonic()
        if not self.permission:
            try:
                self.stop()
            except Exception as error:
                self.fail(error)

    def fail(self, error):
        self.sweeping = False
        self.command_at = 0.0
        if isinstance(error, ServoAlarm) and self.servo and self.bus:
            try:
                try:
                    self.servo.stop()
                except ServoAlarm as alarm:
                    if alarm.ident != self.servo.ident or alarm.address != 24:
                        raise
                try:
                    torque = self.bus.read(self.servo.ident, 24, 1)
                except ServoAlarm as alarm:
                    if alarm.ident != self.servo.ident or alarm.address != 24 or len(alarm.data) != 1:
                        raise
                    torque = alarm.data[0]
                if torque != 0:
                    raise RuntimeError('Servo alarm stop did not confirm torque-off')
                self.servo.torque = False
                self.state.update(torque=False, commanded_torque=False, target=None)
                self.connection_error = ''
                reason = f'{error}; torque off, connection retained. Check servo load before retrying.'
                if reason != self.reason:
                    self.get_logger().warning(reason)
                self.reason = reason
                return
            except Exception as stop_error:
                error = RuntimeError(f'{error}; stop verification failed: {stop_error}')
        if isinstance(error, FirmwareMotionFault) and self.servo and self.bus:
            try:
                self.servo.stop()
                if self.bus.read(self.servo.ident, 24, 1) != 0:
                    raise RuntimeError('Controller stop did not confirm torque-off')
                self.state = self.servo.snapshot()
                self.connection_error = ''
                self.reason = f'{error}; stopped, connection retained. Start a new move to retry.'
                self.get_logger().warning(self.reason)
                return
            except Exception as stop_error:
                error = RuntimeError(f'{error}; stop verification failed: {stop_error}')
        self.reason = str(error)
        self.connection_error = self.reason
        self.get_logger().error(f'Arm driver: {self.reason}')
        self.command_at = 0.0
        self.remember_calibration()
        if self.servo:
            try:
                self.servo.stop()
            except Exception:
                pass
        if self.bus:
            self.bus.close()
        self.bus = self.servo = None
        self.role_servos.clear()
        self.discovered_servos = []
        self.state = {}
        self.permission = False
        self.permission_at = 0.0
        self.discovery_started = False
        self.next_discovery_at = time.monotonic() + 2.0

    def select_role(self, role):
        if role not in self.role_ids:
            raise ValueError('Select arm or probe')
        if self.servo and (self.servo.torque or self.servo.target is not None):
            raise ValueError('Stop the servo before switching arm/probe')
        if self.servo:
            self.role_servos[self.active_role] = self.servo
        self.stop()
        self.active_role = role
        self.servo = None
        self.state = {}
        ident = self.role_ids[role]
        if ident in self.discovered_servos:
            self.select(ident)
        else:
            self.reason = f'{role.title()} servo ID {ident} is not connected; connect it and restart discovery'

    def reconnect(self):
        if self.discovery is not None:
            raise ValueError('Servo discovery is already running')
        if self.sweeping or (self.servo and (self.servo.torque or self.servo.target is not None)):
            raise ValueError('Stop the servo before reconnecting')
        self.stop()
        self.remember_calibration()
        if self.bus:
            self.bus.close()
        self.bus = self.servo = None
        self.role_servos.clear()
        self.state = {}
        self.discovered_servos = []
        self.active_role = 'arm'
        self.permission = False
        self.permission_at = self.command_at = 0.0
        self.connection_error = ''
        self.discovery_pool.shutdown(wait=False)
        self.discovery_pool = ThreadPoolExecutor(max_workers=1)
        self.discovery_started = False
        self.reason = 'Reconnecting; motion remains stopped'
        self.next_discovery_at = 0.0

    def remember_calibration(self):
        servos = dict(self.role_servos)
        if self.servo:
            servos[self.active_role] = self.servo
        for role, servo in servos.items():
            self.saved_role_calibration[role] = (servo.ident,
                dict(servo.calibration) if servo.calibration else None, dict(servo.captured))

    def select(self, ident):
        if ident not in self.discovered_servos:
            raise ValueError('Select a detected servo ID')
        other = 'probe' if self.active_role == 'arm' else 'arm'
        if ident == self.role_ids[other]:
            raise ValueError(f'Servo ID {ident} is assigned to {other}; select that role to calibrate it')
        self.stop()
        prefix = '' if self.active_role == 'arm' else 'probe_'
        calibration = {key: self.get_parameter(prefix + key).value for key in ('minimum', 'center', 'maximum')}
        if ident != self.role_ids[self.active_role]:
            calibration = None
        if calibration is not None and all(value == -1 for value in calibration.values()):
            calibration = None
        saved = self.saved_role_calibration.get(self.active_role)
        if saved and saved[0] == ident:
            calibration = saved[1]
        servo = self.role_servos.get(self.active_role)
        if servo is None or servo.ident != ident:
            servo = ArmServo(self.bus, ident, calibration, self.motion_speed)
            if saved and saved[0] == ident:
                servo.captured = dict(saved[2])
        self.servo = servo
        self.role_ids[self.active_role] = ident
        self.role_servos[self.active_role] = servo
        self.state = servo.snapshot()
        self.reason = 'Torque off; ready to record supported arm positions'

    def command_cb(self, message):
        command = {}
        success = True
        try:
            command = json.loads(message.data)
            action = command.get('action')
            if action == 'stop':
                self.stop()
                if self.servo:
                    self.reason = 'Stopped; torque released'
                return
            if action == 'reconnect':
                self.reconnect()
                return
            if not self.bus:
                raise ValueError('Arm serial port is not connected')
            if action == 'select_role':
                self.select_role(command.get('role'))
                return
            if action == 'select':
                if self.servo and (self.servo.torque or self.servo.target is not None):
                    raise ValueError('Stop the arm before selecting a servo')
                self.select(command.get('servo_id'))
                self.reason = f'Servo {self.servo.ident} selected; live position ready'
                return
            if not self.servo:
                raise ValueError('Select a servo ID first')
            if action == 'configure_motion':
                if self.servo.torque or self.servo.target is not None or self.sweeping:
                    raise ValueError('Stop before changing motion limits')
                speed = command.get('max_speed_deg_s')
                acceleration = command.get('acceleration_deg_s2')
                if (isinstance(speed, bool) or not isinstance(speed, (int, float))
                        or not math.isfinite(speed) or not .684 <= speed <= 1023 * .684):
                    raise ValueError('Maximum speed must be within 0.684..699.732 degrees/s')
                if (isinstance(acceleration, bool) or not isinstance(acceleration, (int, float))
                        or not math.isfinite(acceleration) or not 8.583 <= acceleration <= 254 * 8.583):
                    raise ValueError('Acceleration must be within 8.583..2180.082 degrees/s^2')
                self.motion_speed = max(1, math.floor(speed / .684 + 1e-9))
                self.sweep_acceleration = max(1, math.floor(acceleration / 8.583 + 1e-9))
                self.sweep_speed = min(self.sweep_speed, self.motion_speed * .684)
                self.reason = 'Motion limits applied; export launch settings to keep after restart'
            elif action in ('capture', 'configure'):
                if self.servo.torque or self.servo.target is not None:
                    raise ValueError('Release the move button before recording or applying limits')
                if action == 'capture':
                    position = self.servo.capture(command.get('point'))
                    self.reason = f'Recorded {command.get("point")}: {position * 360 / 4096:.1f} degrees'
                else:
                    self.servo.configure({key: command.get(key) for key in ('minimum', 'center', 'maximum')})
                    self.reason = 'Limits applied; copy launch settings to keep them after restart'
            elif action in ('move', 'jog', 'position'):
                self.sweeping = False
                # Never store a request without website permission and PX4 status 4.
                if (command.get('held') is not True or not self.permission
                        or time.monotonic() - self.permission_at >= .3
                        or not self.safety.servo_snapshot()['allowed']):
                    self.stop()
                    raise ValueError('Movement blocked: enable calibration and require fresh PX4 status 4')
                self.servo.speed = self.motion_speed
                self.servo.max_step = 16
                if action == 'jog':
                    self.servo.request_jog(command.get('direction'), command.get('speed_deg_s', 2.0))
                    self.reason = 'Jogging slowly; release to stop'
                elif action == 'position':
                    self.servo.request_position(command.get('position'))
                    self.reason = 'Moving to slider position'
                else:
                    self.servo.request(command.get('point'))
                    self.reason = f'Moving toward {command.get("point")}; release to stop'
                self.command_at = time.monotonic()
            else:
                raise ValueError('Unknown arm command')
        except (ValueError, TypeError, AttributeError) as error:
            success = False
            self.reason = str(error)
            try:
                self.stop()
            except Exception as stop_error:
                self.fail(stop_error)
        except Exception as error:
            success = False
            self.fail(error)
        finally:
            if isinstance(command, dict) and command.get('request_id'):
                self.last_command = dict(request_id=command['request_id'], success=success, message=self.reason)
            if self.servo:
                self.state.update(position=self.servo.position, torque=self.servo.torque,
                                  captured=dict(self.servo.captured), calibration=self.servo.calibration)

    def tick(self):
        now = time.monotonic()
        try:
            self.poll_discovery()
            if self.servo:
                if self.sweeping:
                    if (not self.permission or now - self.permission_at >= .3 or not self.safety.servo_snapshot()['allowed']):
                        self.stop()
                    else:
                        speed = max(1, min(self.motion_speed, round(self.sweep_speed / .684)))
                        self.servo.request_sweep('maximum', speed, self.sweep_acceleration,
                                                 self.sweep_endpoint_tolerance)
                        self.command_at = now
                self.servo.step(self.allowed)
                # step already reads position during motion; do not wait for
                # slower voltage/current diagnostics to publish the arm angle.
                if self.servo.target is None:
                    self.servo.position = self.bus.read(self.servo.ident, 36)
                if now - self.last_poll > .2:
                    self.last_poll = now
                    self.state = self.servo.snapshot()
                self.state['position'] = self.servo.position
                if self.active_role == 'arm' and self.servo.calibration:
                    angle = (self.servo.position - self.servo.calibration['center']) * 360.0 / 4096
                    self.angle_pub.publish(Float32(data=angle))
            self.sweep_pub.publish(Bool(data=self.sweeping))
            self.speed_pub.publish(Float32(data=self.sweep_speed))
            data = {**self.state, 'reason': self.connection_error or self.reason, 'last_command': self.last_command,
                    'sweeping': self.sweeping,
                    'active_role': self.active_role, 'role_ids': dict(self.role_ids),
                    'arm_calibration': self.role_servos['arm'].calibration if 'arm' in self.role_servos else None,
                    'motion_speed_limit': self.motion_speed,
                    'telemetry_read_retries': getattr(self.bus, 'read_retries', 0),
                    'sweep_acceleration_deg_s2': self.sweep_acceleration * 8.583,
                    'sweep_endpoint_tolerance_deg': self.get_parameter('sweep_endpoint_tolerance_deg').value,
                    'connected': self.servo is not None, 'serial_connected': self.bus is not None,
                    'serial_port': self.device, 'discovered_servos': self.discovered_servos,
                    'scanning': self.discovery is not None, 'safety': self.safety.servo_snapshot()}
            self.publisher.publish(String(data=json.dumps(data)))
        except Exception as error:
            self.fail(error)
            self.publisher.publish(String(data=json.dumps({**self.state,
                'connected': self.servo is not None, 'serial_connected': self.bus is not None,
                'sweeping': False, 'discovered_servos': self.discovered_servos,
                'reason': self.reason, 'last_command': self.last_command})))

    def destroy_node(self):
        self.discovery_stop.set()
        self.discovery_pool.shutdown(wait=True)
        if self.discovery is not None and not self.discovery.cancelled():
            try:
                result = self.discovery.result()
                if result:
                    result[0].close()
            except Exception:
                pass
            self.discovery = None
        try:
            self.stop()
        finally:
            if self.bus:
                self.bus.close()
            super().destroy_node()


def main():
    stop = threading.Event()
    # Keep this handler through interpreter exit: launch may forward SIGINT twice.
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop.set())
    node = None
    try:
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        node = ArmServoNode()
        while not stop.is_set():
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        try:
            if node is not None:
                node.destroy_node()  # Torque off and serial close while ROS is still valid.
        finally:
            rclpy.try_shutdown()


if __name__ == '__main__':
    main()
