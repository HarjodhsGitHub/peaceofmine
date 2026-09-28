"""Nonblocking actuator lifecycle over typed servo telemetry and bounded commands.

JSON topics here are compatibility adapters for the existing web protocol. The
motor contract and long-running ROS operations are typed.
"""
import json
import math
import time
import uuid
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.task import Future
from mavros_msgs.msg import State, RCIn
from std_msgs.msg import Bool, String
from peaceofmine_interfaces.msg import ServoCommand, ServoState
from peaceofmine_interfaces.srv import ServoConfigure
from peaceofmine_interfaces.action import ActuatorOperation
from . import configuration
from .rc_safety import RcSafety
from .settings_client import SettingsClient
from .servo_motor import limits
from .actuator_protocol import command_dict
from peaceofmine_interfaces.msg import ActuatorCommand, ActuatorStatus


class ActuatorController(Node):
    role = ''

    def __init__(self, **kwargs):
        super().__init__(self.role+'_controller', **kwargs)
        self.config_file = self.declare_parameter('config_file', configuration.default_path()).value
        self.simulation = self.declare_parameter('simulation', False).value
        self.saved = configuration.actuator_settings(self.config_file,self.simulation)
        self.safety = RcSafety(simulation=self.simulation)
        requested = self.declare_parameter('servo_id', -1).value
        self.ident = self.saved.get(self.role, {}).get('servo_id', 1 if self.role == 'arm' else 2) if requested == -1 else requested
        self.motion_speed = self.declare_parameter('speed', 116 if self.role == 'arm' else 73).value
        self.acceleration = 4
        self.launch_limits = {key:self.declare_parameter(key, -1).value for key in ('minimum','center','maximum')}
        self.owner = self.role+'-'+uuid.uuid4().hex
        self.driver = None
        self.telemetry = None
        self.driver_at = self.permission_at = self.command_at = 0.
        self.permission = False
        self.operation = None
        self.sequence = 0
        self.reason = 'Waiting for driver telemetry'
        self.lifecycle = 'DISCONNECTED'
        self.captured = {}
        self.last_command = {}
        self.state = {}
        self.last_outcome = None
        self.held_key = None
        self.retired_holds = set()
        self.action_handle = self.action_done = None
        self.action_reserved = False
        self.settings = SettingsClient(self, self.config_file)
        self.publisher = self.create_publisher(String, self.role+'/web_state', 1)
        self.status_pub = self.create_publisher(ActuatorStatus, self.role+'/state', 1)
        self.result_pub = self.create_publisher(String, self.role+'/result', 10)
        self.motor_pub = self.create_publisher(ServoCommand, 'servo/command', 1)
        self.configure_client = self.create_client(ServoConfigure, 'servo/configure')
        self.create_subscription(ActuatorCommand, self.role+'/command', self.command_cb, 1)
        self.create_subscription(ServoState, 'servo/state', self.driver_cb, 1)
        self.create_subscription(Bool, 'operator/actuation_enabled', self.permission_cb, 1)
        self.create_subscription(State, 'mavros/state', self.state_cb, 1)
        self.create_subscription(RCIn, 'mavros/rc/in', self.rc_cb, 1)
        self.actions = ActionServer(self, ActuatorOperation, self.role+'/execute',
            self.execute_action, goal_callback=self.goal_callback, cancel_callback=lambda _: CancelResponse.ACCEPT)
        self.create_timer(.05, self.tick)
        self.create_timer(.5, self.reload_settings)

    def reload_settings(self):
        try:
            saved=configuration.actuator_settings(self.config_file,self.simulation)
            keys=(self.role,self.role+'_motion')
            if any(saved.get(k)!=self.saved.get(k) for k in keys):
                self.stop('Configuration changed; start again')
                previous=self.ident
                self.saved=saved
                if self.get_parameter('servo_id').value == -1:
                    self.ident=saved.get(self.role,{}).get('servo_id',self.ident)
                if previous!=self.ident:
                    self.telemetry=None
                    self.on_disconnect()
                self.apply_saved_settings()
        except (OSError,ValueError,TypeError) as exc:
            self.stop('Configuration unavailable: '+str(exc))

    def apply_saved_settings(self):
        pass

    def calibration(self):
        value = self.saved.get(self.role, {})
        if value.get('servo_id') == self.ident and all(k in value for k in self.launch_limits):
            return {k:value[k] for k in self.launch_limits}
        return self.launch_limits if all(v >= 0 for v in self.launch_limits.values()) else None

    def connected(self):
        return bool(self.driver and self.driver.connected and self.telemetry and time.monotonic()-self.driver_at < .3)

    def permitted(self):
        return self.permission and time.monotonic()-self.permission_at < .3 and self.safety.servo_snapshot()['allowed']

    def interlock(self):
        return self.permitted() and self.connected()

    def permission_cb(self, msg):
        self.permission, self.permission_at = msg.data, time.monotonic()
        if self.operation and not self.permitted():
            self.stop('Permission lost', success=False)

    def state_cb(self, msg):
        self.safety.update_state(msg)
        if self.operation and not self.permitted():
            self.stop('PX4 interlock', success=False)

    def rc_cb(self, msg):
        self.safety.update_rc(msg)
        if self.operation and not self.permitted():
            self.stop('RC interlock', success=False)

    def driver_cb(self, msg):
        was_connected=self.connected()
        if self.driver and (self.driver.session != msg.session or not msg.connected):
            self.stop('Driver disconnected or restarted', success=False)
            self.on_disconnect()
        self.driver, self.driver_at = msg, time.monotonic()
        self.telemetry = next((s for s in msg.servos if s.servo_id == self.ident), None)
        if self.connected() and not was_connected:
            self.lifecycle,self.reason='IDLE','Connected; torque off'
        elif not msg.connected:
            self.reason=msg.error or 'Driver disconnected'

    def on_disconnect(self):
        pass

    def complete_action(self, success, message):
        if self.action_done and not self.action_done.done():
            self.action_done.set_result((success, message))

    def stop(self, reason='Stopped; torque released', success=False):
        command, self.operation = self.operation, None
        if command:
            command.mode = ServoCommand.STOP
            self.motor_pub.publish(command)
        if self.held_key:
            self.retired_holds.add(self.held_key)
            self.held_key = None
        self.reason = reason
        self.lifecycle = 'IDLE' if self.connected() else 'DISCONNECTED'
        self.complete_action(success, reason)

    def start(self, target, low, high, speed=None, mode=ServoCommand.POSITION, load=0., lead=114, tolerance=23):
        if not self.interlock():
            raise ValueError('Motion requires fresh telemetry, permission and behavior interlocks')
        if self.driver.config_revision < self.saved.get('revision',0):
            raise ValueError('Driver is applying saved settings; retry shortly')
        if self.operation:
            self.stop('Replaced by a new request')
        self.last_outcome = None
        self.operation = ServoCommand(session=self.driver.session, owner=self.owner,
            operation_id=uuid.uuid4().hex, servo_id=self.ident, mode=mode,
            target=int(target), minimum=int(low), maximum=int(high),
            speed=int(min(self.motion_speed, speed or self.motion_speed)), acceleration=int(self.acceleration),
            lead=int(lead), tolerance=int(tolerance), load_stop_percent=float(load))
        self.command_at = time.monotonic()
        self.lifecycle = 'MOVING'
        self.reason = 'Motion requested'

    def save(self, command, section, value, applied):
        self.stop('Saving configuration')
        def complete(ok, message):
            if ok:
                self.saved = configuration.actuator_settings(self.config_file,self.simulation)
                applied()
            self.reason = message if not ok else 'Saved; driver applies limits on its next configuration update'
            self.reply(command, ok)
        self.settings.save(section, value, complete)

    def reply(self, command, ok):
        if command.get('request_id'):
            self.last_command = dict(request_id=command['request_id'], success=ok, message=self.reason)
            self.result_pub.publish(String(data=json.dumps(dict(request_id=command['request_id'], ok=ok, message=self.reason))))

    def configure_hardware(self, command, operation):
        if self.operation or (self.telemetry and self.telemetry.torque):
            raise ValueError('Stop before changing hardware configuration')
        if not self.driver or not self.configure_client.service_is_ready():
            raise ValueError('Driver configuration service unavailable')
        request = ServoConfigure.Request(session=self.driver.session, servo_id=self.ident, operation=operation)
        def completed(future):
            try:
                result = future.result()
                self.reason = result.message
                self.reply(command, result.success)
            except Exception as exc:
                self.reason = str(exc)
                self.reply(command, False)
        self.configure_client.call_async(request).add_done_callback(completed)

    def command_cb(self, msg):
        command = {}
        try:
            command = command_dict(msg)
            if not isinstance(command, dict) or command.get('role', self.role) != self.role:
                raise ValueError('Wrong actuator command')
            action = command.get('action')
            if not hasattr(msg,'data') and action != 'stop':
                age=time.time()-(msg.stamp.sec+msg.stamp.nanosec/1e9)
                if not 0 <= age < .25:
                    raise ValueError('Expired actuator command')
            if action == 'stop':
                self.stop()
                # Explicit release allows a later fresh manual gesture.
                self.retired_holds.clear()
            elif action == 'select_role':
                pass
            elif action == 'reconnect':
                self.configure_hardware(command, ServoConfigure.Request.RECONNECT)
                return
            elif action == 'enable_multiturn':
                self.configure_hardware(command, ServoConfigure.Request.MULTITURN)
                return
            elif action == 'select':
                ident = command.get('servo_id')
                if self.operation or not self.driver or ident not in [s.servo_id for s in self.driver.servos]:
                    raise ValueError('Stop and select a connected servo')
                other = 'probe' if self.role == 'arm' else 'arm'
                if ident == self.saved.get(other, {}).get('servo_id', 2 if other=='probe' else 1):
                    raise ValueError('Servo belongs to the other actuator')
                self.ident = ident
                self.telemetry = None
                self.on_disconnect()
            elif not self.connected():
                raise ValueError('Servo telemetry unavailable')
            elif self.special_command(action, command):
                return
            elif action == 'capture' and self.role == 'arm':
                point = command.get('point')
                if self.operation or self.telemetry.torque or point not in ('minimum','center','maximum'):
                    raise ValueError('Stop before capturing a valid calibration point')
                self.captured[point] = self.telemetry.position
                self.reason = 'Position captured'
            elif action == 'configure' and self.role == 'arm':
                value = limits(**{k:command.get(k) for k in ('minimum','center','maximum')}, low=self.telemetry.minimum, high=self.telemetry.maximum)
                self.save(command, 'arm', dict(servo_id=self.ident, **value), lambda: None)
                return
            elif action in ('move','jog','position'):
                if command.get('held') is not True:
                    raise ValueError('Held command required')
                key = command.get('hold_id') or action
                if key in self.retired_holds:
                    raise ValueError('Release and start again after interrupted motion')
                if not self.interlock():
                    raise ValueError('Motion interlock unavailable')
                low, high = self.telemetry.minimum, self.telemetry.maximum
                speed = self.motion_speed
                if action == 'jog':
                    if command.get('direction') not in (-1, 1):
                        raise ValueError('Invalid jog direction')
                    requested = command.get('speed_deg_s', 2.)
                    if type(requested) not in (int,float) or not math.isfinite(requested) or requested <= 0:
                        raise ValueError('Invalid jog speed')
                    speed = max(1, min(speed, int(requested/.684)))
                    target = low if command['direction'] < 0 else high
                elif action == 'move':
                    calibration = self.calibration()
                    if not calibration or command.get('point') not in calibration:
                        raise ValueError('Calibration required')
                    low, high = calibration['minimum'], calibration['maximum']
                    target = calibration[command['point']]
                else:
                    target = command.get('position')
                if type(target) is not int or not low <= target <= high:
                    raise ValueError('Position outside supported range')
                if not self.operation or self.held_key != key:
                    self.before_manual()
                    self.start(target, low, high, speed)
                else:
                    self.operation.target, self.operation.speed = target, speed
                self.held_key, self.command_at = key, time.monotonic()
            else:
                raise ValueError('Unsupported actuator command')
            self.reply(command, True)
        except Exception as exc:
            self.stop(str(exc), success=False)
            self.reply(command if isinstance(command, dict) else {}, False)

    def before_manual(self):
        self.stop()

    def special_command(self, action, command):
        return False

    def update_behavior(self):
        pass

    def behavior_state(self):
        return {}

    def tick(self):
        if not self.connected():
            if self.operation:
                self.stop('Driver telemetry stale', success=False)
            self.on_disconnect()
        if self.operation:
            if not self.interlock():
                self.stop('Motion interlock lost', success=False)
            else:
                telemetry = self.telemetry
                if telemetry.operation_id == self.operation.operation_id and telemetry.outcome in ('aborted','rejected','load_stop','stopped'):
                    self.last_outcome = telemetry.outcome
                    self.operation_finished(telemetry.outcome, telemetry.error)
                self.update_behavior()
                if self.operation:
                    if time.monotonic()-self.command_at >= .25:
                        self.stop('Held command expired; release and start again')
                    else:
                        self.sequence += 1
                        self.operation.sequence, self.operation.challenge = self.sequence, self.driver.challenge
                        self.motor_pub.publish(self.operation)
        if self.action_handle:
            if self.action_handle.is_cancel_requested:
                self.stop('Operation cancelled')
            elif self.action_handle.is_active:
                feedback = ActuatorOperation.Feedback(state=self.lifecycle,
                    position=float(self.telemetry.position if self.telemetry else 0), message=self.reason)
                self.action_handle.publish_feedback(feedback)
        t = self.telemetry
        self.state = dict(connected=self.connected(), servo_id=self.ident, active_role=self.role,
            saved_config_revision=self.saved.get('revision',0), driver_config_revision=self.driver.config_revision if self.driver else None,
            reason=self.reason, lifecycle=self.lifecycle, position=t.position if t else None,
            sample_time=self.driver_at, goal_position=t.goal if t else None,
            model=310 if t else None, firmware=t.firmware if t else None,
            speed_limit_deg_s=t.speed_limit*.684 if t else None,
            max_torque_percent=t.max_torque_percent if t else None,
            moving=bool(t and t.torque and abs(t.goal-t.position)>3),
            torque=t.torque if t else False, commanded_torque=bool(self.operation),
            target=self.operation.target if self.operation else None,
            position_minimum=t.minimum if t else None, position_maximum=t.maximum if t else None,
            position_mode='multi-turn' if t and t.multiturn else 'joint',
            speed_deg_s=t.speed_deg_s if t else None, load_percent=t.load_percent if t else None,
            current_a=t.current_a if t else None, temperature_c=t.temperature_c if t else None,
            voltage=t.voltage if t else None, torque_limit_percent=t.torque_limit_percent if t else None,
            motion_speed_limit=self.motion_speed, calibration=self.calibration(), captured=self.captured,
            last_command=self.last_command, serial_connected=bool(self.driver and self.driver.connected),
            serial_port=self.driver.device if self.driver else '',
            discovered_servos=[s.servo_id for s in self.driver.servos] if self.driver else [],
            safety=self.safety.servo_snapshot(), **self.behavior_state())
        self.publisher.publish(String(data=json.dumps(self.state, allow_nan=False)))
        calibration=self.calibration()
        probe=self.state.get('probe',{})
        status=ActuatorStatus(role=self.role,connected=self.connected(),lifecycle=self.lifecycle,reason=self.reason,
            servo_id=self.ident,sweeping=bool(self.state.get('sweeping')),motion_speed_limit=self.motion_speed,calibrated=bool(calibration),
            minimum=calibration['minimum'] if calibration else 0,center=calibration['center'] if calibration else 0,
            maximum=calibration['maximum'] if calibration else 0,homed=bool(probe.get('homed')),
            probe_ready=bool(probe.get('ready')),probe_active=bool(probe.get('active')),probe_holding=bool(probe.get('holding')),
            depth_mm=float(probe.get('depth_mm') or 0),target_mm=float(probe.get('target_mm') or 0),
            max_depth_mm=float(probe.get('max_depth_mm') or 0))
        status.header.stamp=self.get_clock().now().to_msg()
        if t:status.telemetry=t
        self.status_pub.publish(status)

    def operation_finished(self, outcome, message):
        self.stop(message or outcome, success=outcome=='load_stop')

    def goal_callback(self, request):
        if self.action_reserved or self.operation or not self.interlock() or request.operation not in self.supported_actions:
            return GoalResponse.REJECT
        self.action_reserved = True
        return GoalResponse.ACCEPT

    async def execute_action(self, handle):
        self.action_handle = handle
        self.action_done = Future()
        try:
            self.start_action(handle.request)
            success, message = await self.action_done
            if handle.is_cancel_requested:
                handle.canceled()
            elif success:
                handle.succeed()
            else:
                handle.abort()
            return ActuatorOperation.Result(success=success, message=message)
        except Exception as exc:
            self.stop(str(exc))
            handle.abort()
            return ActuatorOperation.Result(success=False, message=str(exc))
        finally:
            self.action_handle = self.action_done = None
            self.action_reserved = False

    def destroy_node(self):
        self.stop('Controller shutdown')
        self.actions.destroy()
        super().destroy_node()
