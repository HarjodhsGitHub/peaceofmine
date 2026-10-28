#!/usr/bin/env python3
"""Arm-specific calibration and sweep behavior; no serial access."""
import math
import time
from std_msgs.msg import Bool, Float32
from peaceofmine_interfaces.msg import ServoCommand
from peaceofmine_operator.actuator_controller import ActuatorController
from peaceofmine_operator.node_runner import run_node


class ArmController(ActuatorController):
    role = 'arm'
    supported_actions = ('sweep', 'position')

    def __init__(self, **kwargs):
        self.sweeping = False
        self.sweep_speed = 5.
        self.auto_position = False
        self.started = 0.
        super().__init__(**kwargs)
        self.tolerance_deg = self.declare_parameter('sweep_endpoint_tolerance_deg', 2.).value
        self.offset = self.declare_parameter('fixture_angle_offset_deg', 0.).value
        acceleration = self.declare_parameter('sweep_acceleration_deg_s2', 34.332).value
        self.apply_motion(self.saved.get('arm_motion', dict(max_speed_deg_s=self.motion_speed*.684, acceleration_deg_s2=acceleration)))
        self.angle_pub = self.create_publisher(Float32, 'fixture/angle_deg', 10)
        self.sweep_pub = self.create_publisher(Bool, 'fixture/sweep_enabled_state', 1)
        self.speed_pub = self.create_publisher(Float32, 'fixture/sweep_speed_deg_s_state', 1)
        self.create_subscription(Float32, 'fixture/sweep_speed_deg_s', self.speed_cb, 1)

    def apply_saved_settings(self):
        if 'arm_motion' in self.saved:
            self.apply_motion(self.saved['arm_motion'])

    def apply_motion(self, value):
        speed, acceleration = value.get('max_speed_deg_s'), value.get('acceleration_deg_s2')
        if not all(type(v) in (int,float) and math.isfinite(v) for v in (speed,acceleration)) or not .684 <= speed <= 699.732 or not 8.583 <= acceleration <= 2180.082:
            raise ValueError('Invalid motion limits')
        self.motion_speed = max(1, math.floor(speed/.684+1e-9))
        self.acceleration = max(1, math.floor(acceleration/8.583+1e-9))
        self.sweep_speed = min(self.sweep_speed, speed)

    def stop(self, reason='Stopped; torque released', success=False):
        self.sweeping = self.auto_position = False
        super().stop(reason, success)

    def speed_cb(self, msg):
        if self.permitted() and math.isfinite(msg.data):
            self.sweep_speed = max(.684, min(msg.data, self.motion_speed*.684))
            if self.sweeping and self.operation:
                self.operation.speed = max(1, min(self.motion_speed, round(self.sweep_speed/.684)))

    def sweep_cb(self, msg):
        try:
            if not msg.data:
                self.stop()
            elif not self.sweeping:
                calibration = self.calibration()
                if not calibration:
                    raise ValueError('Calibrate arm before sweeping')
                self.start(calibration['maximum'], calibration['minimum'], calibration['maximum'],
                    speed=max(1, round(self.sweep_speed/.684)), mode=ServoCommand.SWEEP,
                    tolerance=max(1, math.ceil(self.tolerance_deg*4096/360)))
                self.sweeping = True
                self.lifecycle = 'SWEEPING'
        except Exception as exc:
            self.stop(str(exc))

    def special_command(self, action, command):
        if action == 'sweep':
            self.sweep_cb(Bool(data=command.get('enabled') is True))
            self.reply(command,not command.get('enabled') or self.sweeping)
            return True
        if action != 'configure_motion':
            return False
        if self.operation or self.telemetry.torque:
            raise ValueError('Stop before changing motion limits')
        # Validate without applying before persistence succeeds.
        old = self.motion_speed, self.acceleration, self.sweep_speed
        self.apply_motion(command)
        self.motion_speed, self.acceleration, self.sweep_speed = old
        value = {k:command[k] for k in ('max_speed_deg_s','acceleration_deg_s2')}
        self.save(command, 'arm_motion', value, lambda: self.apply_motion(value))
        return True

    def start_action(self, request):
        if request.operation == 'sweep':
            if request.speed > 0:
                self.speed_cb(Float32(data=request.speed))
            self.sweep_cb(Bool(data=True))
            if not self.sweeping:
                raise ValueError(self.reason)
        else:
            calibration = self.calibration()
            if not calibration or not math.isfinite(request.target):
                raise ValueError('Calibrated position required')
            target = round(request.target)
            if not calibration['minimum'] <= target <= calibration['maximum']:
                raise ValueError('Target outside calibrated arm range')
            self.start(target, calibration['minimum'], calibration['maximum'])
            self.auto_position, self.started = True, time.monotonic()

    def update_behavior(self):
        if self.sweeping or self.auto_position:
            self.command_at = time.monotonic()
        if self.auto_position and self.operation:
            if time.monotonic()-self.started > 60:
                self.stop('Position timeout')
            elif self.telemetry.operation_id == self.operation.operation_id and abs(self.telemetry.position-self.operation.target) <= 3:
                self.stop('Position reached', success=True)

    def behavior_state(self):
        calibration = self.calibration()
        if self.connected() and calibration:
            self.angle_pub.publish(Float32(data=(self.telemetry.position-calibration['center'])*360/4096+self.offset))
        self.sweep_pub.publish(Bool(data=self.sweeping))
        self.speed_pub.publish(Float32(data=self.sweep_speed))
        return dict(sweeping=self.sweeping, sweep_acceleration_deg_s2=self.acceleration*8.583,
                    sweep_endpoint_tolerance_deg=self.tolerance_deg, arm_calibration=calibration)


def main():
    run_node(ArmController)


if __name__ == '__main__':
    main()
