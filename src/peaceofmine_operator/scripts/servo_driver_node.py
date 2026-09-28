#!/usr/bin/env python3
"""Own the serial bus, bounded motor execution, telemetry and independent watchdog."""
import math
import threading
import time
import uuid
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from rclpy.node import Node
from mavros_msgs.msg import State, RCIn
from std_msgs.msg import Bool, Float32
from peaceofmine_interfaces.msg import ServoCommand, ServoState, ServoTelemetry
from peaceofmine_interfaces.srv import ServoConfigure
from peaceofmine_operator import configuration
from peaceofmine_operator.servo_transport import ArbotiX
from peaceofmine_operator.servo_runtime import ServoRuntime
from peaceofmine_operator.simulated_servo_bus import SimulatedServoBus
from peaceofmine_operator.rc_safety import RcSafety
from peaceofmine_operator.node_runner import run_node


class ServoDriver(Node):
    def __init__(self, **kwargs):
        super().__init__('servo_driver', **kwargs)
        self.simulation = self.declare_parameter('simulation', False).value
        self.path = self.declare_parameter('config_file', configuration.default_path()).value
        self.declare_parameter('serial_port', 'auto')
        self.arm_id=self.declare_parameter('servo_id',-1).value
        self.probe_id=self.declare_parameter('probe_servo_id',-1).value
        self.default_speed = self.declare_parameter('speed', 116).value
        self.probe_speed = self.declare_parameter('probe_speed', 73).value
        self.arm_acceleration=self.declare_parameter('sweep_acceleration_deg_s2',34.332).value
        self.runtime = None
        self.session, self.challenge = uuid.uuid4().hex, 0
        self.challenge_times = {}
        self.device, self.reason = '', 'Looking for servo adapter'
        self.permission, self.permission_at = False, 0.0
        self.safety = RcSafety(simulation=self.simulation)
        self.config = configuration.actuator_settings(self.path,self.simulation)
        self.pool = ThreadPoolExecutor(max_workers=1)
        self.discovery = None
        self.cancel = threading.Event()
        self.next_scan = 0.0
        self.publisher = self.create_publisher(ServoState, 'servo/state', 1)
        self.create_subscription(ServoCommand, 'servo/command', self.command, 1)
        self.create_subscription(Bool, 'operator/actuation_enabled', self.permission_cb, 1)
        self.create_subscription(State, 'mavros/state', self.state_cb, 1)
        self.create_subscription(RCIn, 'mavros/rc/in', self.rc_cb, 1)
        self.create_service(ServoConfigure, 'servo/configure', self.configure)
        if self.simulation:
            self.create_subscription(Float32,'simulation/probe_load_ratio',self.simulated_load,1)
        self.create_timer(.05, self.tick)
        self.create_timer(1., self.reload)

    def simulated_load(self, message):
        if self.runtime and math.isfinite(message.data):
            self.runtime.bus.external_load_ratio=max(0.,min(1.,message.data))

    def reload(self):
        # Read-only snapshot; operator_settings is the sole runtime writer.
        try:
            proposed = configuration.actuator_settings(self.path,self.simulation)
            if proposed != self.config:
                changed = any(proposed.get(k) != self.config.get(k) for k in ('arm','probe','arm_motion','probe_motion'))
                if self.runtime and changed:
                    self.runtime.stop('aborted', 'Configuration changed; start again')
                self.config = proposed
        except (ValueError, OSError) as exc:
            self.reason = str(exc)
            if self.runtime:
                self.runtime.stop('aborted', 'Configuration unavailable')

    def limits(self, ident):
        arm = self.arm_id if self.arm_id >= 0 else self.config.get('arm', {}).get('servo_id', 1)
        motion = self.config.get('arm_motion' if ident == arm else 'probe_motion', {})
        speed = motion.get('max_speed_deg_s', (self.default_speed if ident == arm else self.probe_speed)*.684)
        acceleration = motion.get('acceleration_deg_s2', self.arm_acceleration if ident==arm else 34.332)
        if not all(type(x) in (float, int) and math.isfinite(x) for x in (speed, acceleration)):
            raise ValueError('Invalid saved motor limits')
        return max(1, min(1023, math.floor(speed/.684+1e-9))), max(1, min(254, math.floor(acceleration/8.583+1e-9)))

    def allowed(self):
        return self.permission and time.monotonic()-self.permission_at < .3 and self.safety.servo_snapshot()['allowed']

    def enforce(self):
        if self.runtime:
            try:
                self.runtime.enforce()
            except Exception as exc:
                self.disconnect(exc)

    def permission_cb(self, message):
        self.permission, self.permission_at = message.data, time.monotonic()
        self.enforce()

    def state_cb(self, message):
        self.safety.update_state(message)
        self.enforce()

    def rc_cb(self, message):
        self.safety.update_rc(message)
        self.enforce()

    def disconnect(self, error):
        runtime, self.runtime = self.runtime, None
        try:
            if runtime:
                runtime.close()
        except Exception:
            pass
        self.reason = str(error)
        self.session, self.challenge = uuid.uuid4().hex, 0
        self.challenge_times.clear()
        self.next_scan = time.monotonic()+2

    def discover(self):
        if self.simulation:
            arm = self.arm_id if self.arm_id >= 0 else self.config.get('arm', {}).get('servo_id', 1)
            probe = self.probe_id if self.probe_id >= 0 else self.config.get('probe', {}).get('servo_id', 2)
            return SimulatedServoBus(arm, probe), 'simulation', [arm, probe]
        configured = self.get_parameter('serial_port').value
        paths = [configured] if configured not in ('', 'auto') else sorted(str(p) for p in Path('/dev/serial/by-id').glob('usb-FTDI*'))
        errors = []
        for path in paths:
            bus = None
            try:
                if self.cancel.is_set():
                    return None
                bus = ArbotiX(path, 1000000)
                ids = []
                for ident in range(253):
                    if self.cancel.is_set():
                        bus.close()
                        return None
                    try:
                        if bus.read(ident, 0) == 310:
                            ids.append(ident)
                    except RuntimeError:
                        pass
                if ids:
                    return bus, path, ids
            except Exception as exc:
                errors.append(str(exc))
            if bus:
                bus.close()
        raise RuntimeError('; '.join(errors) or 'No supported servo adapter found')

    def command(self, message):
        try:
            if not self.runtime or message.session != self.session:
                raise ValueError('Driver session changed')
            if message.mode != ServoCommand.STOP:
                issued = self.challenge_times.get(message.challenge)
                if issued is None or time.monotonic()-issued >= .25:
                    raise ValueError('Expired driver challenge')
            self.runtime.submit(message)
        except ValueError as exc:
            self.reason = str(exc)
            if self.runtime and message.servo_id in self.runtime.motors:
                active = self.runtime.active
                if active and active.operation_id == message.operation_id and active.owner == message.owner:
                    self.runtime.stop('aborted', str(exc))
                elif not active or active.servo_id != message.servo_id:
                    self.runtime.outcomes[message.servo_id] = message.operation_id, 'rejected', str(exc)
        except Exception as exc:
            self.disconnect(exc)

    def configure(self, request, response):
        try:
            if not self.runtime or request.session != self.session:
                raise ValueError('Driver session changed')
            if self.runtime.active:
                raise ValueError('Stop all actuators before configuration')
            if request.operation == ServoConfigure.Request.RECONNECT:
                self.disconnect('Reconnect requested')
            elif request.operation == ServoConfigure.Request.MULTITURN:
                if request.servo_id not in self.runtime.motors:
                    raise ValueError('Servo unavailable')
                self.runtime.bus.enable_multiturn(request.servo_id)
                self.disconnect('Mode changed; reconnect and home again')
            else:
                raise ValueError('Unsupported configuration operation')
            response.success, response.message = True, 'Configuration applied'
        except Exception as exc:
            response.success, response.message = False, str(exc)
        return response

    def tick(self):
        try:
            if self.discovery and self.discovery.done():
                future, self.discovery = self.discovery, None
                result = future.result()
                if result:
                    bus, self.device, ids = result
                    try:
                        self.runtime = ServoRuntime(bus, ids, self.allowed, self.limits)
                    except Exception:
                        bus.close()
                        raise
                    self.session = uuid.uuid4().hex
                    self.reason = 'Connected'
            if not self.runtime and not self.discovery and time.monotonic() >= self.next_scan:
                self.discovery = self.pool.submit(self.discover)
            if self.runtime:
                self.runtime.tick()
        except Exception as exc:
            self.disconnect(exc)
        self.challenge += 1
        now = time.monotonic()
        self.challenge_times = {k:v for k,v in self.challenge_times.items() if now-v < .25}
        self.challenge_times[self.challenge] = now
        message = ServoState(session=self.session, challenge=self.challenge, connected=self.runtime is not None,
                             device=self.device, error=self.reason, config_revision=self.config.get('revision',0))
        message.header.stamp = self.get_clock().now().to_msg()
        if self.runtime:
            message.active_owner = self.runtime.active.owner if self.runtime.active else ''
            for ident, sample in self.runtime.samples.items():
                op, outcome, error = self.runtime.outcomes[ident]
                message.servos.append(ServoTelemetry(servo_id=ident, connected=True,
                    position=sample['position'], goal=sample['goal_position'], minimum=sample['position_minimum'],
                    maximum=sample['position_maximum'], multiturn=sample['position_mode']=='multi-turn',
                    torque=self.runtime.motors[ident].torque,
                    speed_deg_s=float(sample['speed_deg_s']), load_percent=float(sample['load_percent']),
                    current_a=float(sample['current_a']), voltage=float(sample['voltage']),
                    temperature_c=float(sample['temperature_c']), torque_limit_percent=float(sample['torque_limit_percent']),
                    speed_limit=self.runtime.motors[ident].speed, acceleration=self.runtime.motors[ident].acceleration,
                    operation_id=op, outcome=outcome, error=error, firmware=sample['firmware'],
                    max_torque_percent=float(sample['max_torque_percent'])))
        self.publisher.publish(message)

    def destroy_node(self):
        self.cancel.set()
        self.pool.shutdown(wait=True)
        if self.discovery:
            try:
                result = self.discovery.result()
                if result:
                    result[0].close()
            except Exception:
                pass
        self.disconnect('Shutdown')
        super().destroy_node()


def main():
    run_node(ServoDriver)


if __name__ == '__main__':
    main()
