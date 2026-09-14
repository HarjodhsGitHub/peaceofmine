#!/usr/bin/env python3
"""Pi-owned servo output: PX4 safety and gateway lease checked independently."""
import json
import math
import signal
import threading
import time

import rclpy
from mavros_msgs.msg import State
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Bool, String, Float32
from peaceofmine_operator.arm_servo import ArbotiX, ArmServo
from peaceofmine_operator.rc_safety import RcSafety


class ArmServoNode(Node):
    def __init__(self):
        super().__init__('arm_servo')
        for key, value in dict(serial_port='', servo_id=-1, bus_baud=1000000,
                               minimum=-1, center=-1, maximum=-1, speed=20).items():
            self.declare_parameter(key, value)
        self.safety = RcSafety()  # No hardware simulation bypass.
        self.sweeping = False
        self.sweep_speed = 5.0
        self.sweep_direction = 1
        self.permission = False
        self.permission_at = self.command_at = 0.0
        self.bus = self.servo = None
        self.reason = 'Set serial_port and select the servo ID; torque remains off'
        self.last_retry = self.last_poll = 0.0
        self.state = {}
        self.last_command = {}
        self.create_subscription(State, 'mavros/state', self.state_cb, 10)
        self.create_subscription(Bool, 'operator/actuation_enabled', self.permission_cb, 1)
        self.create_subscription(String, 'arm/command', self.command_cb, 1)
        self.create_subscription(Bool, 'fixture/sweep_enabled', self.sweep_cb, 1)
        self.create_subscription(Float32, 'fixture/sweep_speed_deg_s', self.sweep_speed_cb, 1)
        self.speed_pub = self.create_publisher(Float32, 'fixture/sweep_speed_deg_s_state', 1)
        self.angle_pub = self.create_publisher(Float32, 'fixture/angle_deg', 1)
        self.sweep_pub = self.create_publisher(Bool, 'fixture/sweep_enabled_state', 1)
        self.publisher = self.create_publisher(String, 'arm/state', 1)
        self.create_timer(.05, self.tick)

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
        self.sweep_speed = max(1.0, min(float(message.data), self.get_parameter('speed').value * .684, 28.0))

    def sweep_cb(self, message):
        if (not message.data or not self.permission or time.monotonic() - self.permission_at >= .3
                or not self.safety.servo_snapshot()['allowed'] or not self.servo or not self.servo.calibration):
            try:
                self.stop()
            except Exception as error:
                self.fail(error)
            return
        self.sweeping = True
        self.sweep_direction = 1

    def state_cb(self, message):
        self.safety.update_state(message)
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
        self.reason = str(error)
        self.command_at = 0.0
        if self.servo:
            try:
                self.servo.stop()
            except Exception:
                pass
        if self.bus:
            self.bus.close()
        self.bus = self.servo = None
        self.state = {}

    def select(self, ident):
        self.stop()
        calibration = {key: self.get_parameter(key).value for key in ('minimum', 'center', 'maximum')}
        if all(value == -1 for value in calibration.values()):
            calibration = None
        servo = ArmServo(self.bus, ident, calibration, self.get_parameter('speed').value)
        self.servo = servo
        self.reason = 'Torque off; ready to record supported arm positions'

    def command_cb(self, message):
        command = {}
        success = True
        try:
            command = json.loads(message.data)
            action = command.get('action')
            if action == 'stop':
                self.stop()
                self.reason = 'Stopped; torque released'
                return
            if not self.bus:
                raise ValueError('Arm serial port is not connected')
            if action == 'select':
                if self.permission:
                    raise ValueError('Disarm before selecting a servo')
                self.select(command.get('servo_id'))
                self.reason = f'Servo {self.servo.ident} selected; live position ready'
                return
            if not self.servo:
                raise ValueError('Select a servo ID first')
            if action in ('capture', 'configure'):
                if self.servo.torque or self.servo.target is not None:
                    raise ValueError('Release the move button before recording or applying limits')
                if action == 'capture':
                    position = self.servo.capture(command.get('point'))
                    self.reason = f'Recorded {command.get("point")}: {position * 360 / 4096:.1f} degrees'
                else:
                    self.servo.configure({key: command.get(key) for key in ('minimum', 'center', 'maximum')})
                    self.reason = 'Limits applied; copy launch settings to keep them after restart'
            elif action in ('move', 'jog'):
                self.sweeping = False
                # Never store a request without website permission and PX4 status 4.
                if (command.get('held') is not True or not self.permission
                        or time.monotonic() - self.permission_at >= .3
                        or not self.safety.servo_snapshot()['allowed']):
                    self.stop()
                    raise ValueError('Movement blocked: enable calibration and require fresh PX4 status 4')
                self.servo.speed = self.get_parameter('speed').value
                self.servo.max_step = 16
                if action == 'jog':
                    self.servo.request_jog(command.get('direction'))
                    self.reason = 'Jogging slowly; release to stop'
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
            device = str(self.get_parameter('serial_port').value)
            if not self.bus and device and now - self.last_retry > 2:
                self.last_retry = now
                self.bus = ArbotiX(device, self.get_parameter('bus_baud').value)
                ident = self.get_parameter('servo_id').value
                if ident != -1:
                    self.select(ident)
                else:
                    self.reason = 'Adapter connected; select the servo ID to read its position'
            if self.servo:
                if self.sweeping:
                    if (not self.permission or now - self.permission_at >= .3 or not self.safety.servo_snapshot()['allowed']):
                        self.stop()
                    else:
                        point = 'maximum' if self.sweep_direction > 0 else 'minimum'
                        if abs(self.servo.position - self.servo.calibration[point]) <= 8:
                            self.sweep_direction *= -1
                            point = 'maximum' if self.sweep_direction > 0 else 'minimum'
                        self.servo.speed = max(1, min(self.get_parameter('speed').value, round(self.sweep_speed / .684)))
                        self.servo.max_step = max(1, min(16, round(self.sweep_speed * 4096 / 360 * .05)))
                        self.servo.request(point)
                        self.command_at = now
                self.servo.step(self.allowed)
                if now - self.last_poll > .2:
                    self.last_poll = now
                    self.state = self.servo.snapshot()
                    if self.servo.calibration:
                        angle = (self.servo.position - self.servo.calibration['center']) * 360.0 / 4096
                        self.angle_pub.publish(Float32(data=angle))
            self.sweep_pub.publish(Bool(data=self.sweeping))
            self.speed_pub.publish(Float32(data=self.sweep_speed))
            data = {**self.state, 'reason': self.reason, 'last_command': self.last_command,
                    'connected': self.servo is not None, 'serial_connected': self.bus is not None,
                    'serial_port': device, 'safety': self.safety.servo_snapshot()}
            self.publisher.publish(String(data=json.dumps(data)))
        except Exception as error:
            self.fail(error)
            self.publisher.publish(String(data=json.dumps(dict(connected=False, reason=self.reason, last_command=self.last_command))))

    def destroy_node(self):
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
