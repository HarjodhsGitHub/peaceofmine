"""Web client transport over ROS topics; no actuator or sensor control logic."""
from collections import deque
import copy
import json
import threading
import time
import uuid

from rclpy.node import Node
from std_msgs.msg import String
from .adc_telemetry import ADCTelemetry


class OperatorWebBridge(Node):
    def __init__(self):
        super().__init__('operator_web')
        for name, value in dict(host='0.0.0.0', port=8080, tls_cert='', tls_key='',
                                camera_stream_base_url='http://127.0.0.1:8081').items():
            self.declare_parameter(name, value)
        self._lock = threading.RLock()
        self._connected_sockets = {}
        self._state = None
        self._state_at = 0
        self._camera_frames = {}
        self._cameras_at = 0
        self._power = {}
        self._power_at = 0
        self._detector = {}
        self._detector_at = 0.0
        self._actuators = {}
        self._actuator_times = {}
        self._settings = None
        self._settings_at = 0
        self._requests = {}
        self._replies = deque(maxlen=256)
        self._shutting_down = False
        self.adc = ADCTelemetry()
        self.publisher = self.create_publisher(String, 'operator/command', 50)
        for topic, callback in (
                ('detector/telemetry', self._detector_state),
                ('arm/web_state', lambda value: self._actuator_state('arm', value)),
                ('probe/web_state', lambda value: self._actuator_state('probe', value)),
                ('arm/result', self._device_result), ('probe/result', self._device_result),
                ('operator/state', self._control_state), ('operator/result', self._control_result),
                ('adc/state', self._adc_state), ('adc/command_result', self._device_result),
                ('cameras/state', self._camera_state), ('power/state', self._power_state),
                ('operator/settings', self._settings_state), ('operator/settings/result', self._device_result)):
            self.create_subscription(String, topic, self._json_callback(callback), 10)
        self.create_timer(.1, self._heartbeat)

    def _json_callback(self, callback):
        def receive(message):
            try:
                value = json.loads(message.data)
                if not isinstance(value, dict):
                    raise ValueError('Expected object')
                with self._lock:
                    callback(value)
            except (ValueError, TypeError, KeyError):
                self.get_logger().warning('Invalid operator telemetry')
        return receive

    def _detector_state(self, value):
        self._detector, self._detector_at = value, time.monotonic()

    def _actuator_state(self, role, value):
        self._actuators[role], self._actuator_times[role] = value, time.monotonic()

    def _control_state(self, value):
        self._state, self._state_at = value, time.monotonic()

    def _camera_state(self, value):
        self._camera_frames, self._cameras_at = value, time.monotonic()

    def _power_state(self, value):
        self._power, self._power_at = value, time.monotonic()

    def _settings_state(self, value):
        self._settings, self._settings_at = value, time.monotonic()

    def _adc_state(self, value):
        probe = self._actuators.get('probe', {}).get('probe', {})
        if time.monotonic()-self._actuator_times.get('probe',0) < .5:
            value['probe_contact'] = probe.get('adc_contact_state', value.get('probe_contact'))
        self.adc.update(value)

    def _device_result(self, value):
        request = self._requests.pop(value.get('request_id'), None)
        if request:
            ws, kind, _ = request
            if kind == 'actuator' and value.get('ok'):
                return
            self._replies.append((ws, {**value, 'type': 'error' if kind == 'actuator' else kind + '_result'}))

    def _control_result(self, value):
        if value.get('request_id') in self._requests:
            self._device_result({**value, 'ok': False})
            return
        ws = next((ws for ws, client in self._connected_sockets.items() if client == value.get('client_id')), None)
        if ws is not None:
            self._replies.append((ws, value))

    def _send(self, client, payload, event='command'):
        self.publisher.publish(String(data=json.dumps(dict(
            client_id=client, payload=payload, event=event, stamp=time.time()), allow_nan=False)))

    def _heartbeat(self):
        with self._lock:
            if not self._shutting_down:
                for client in self._connected_sockets.values():
                    self._send(client, {}, 'heartbeat')
            for key, (ws, kind, started) in list(self._requests.items()):
                if time.monotonic() - started > 3:
                    self._requests.pop(key)
                    self._replies.append((ws, dict(type='error' if kind == 'actuator' else kind+'_result', ok=False,
                        message='ROS node did not acknowledge the request. Check device status.')))

    def connect_client(self, ws):
        with self._lock:
            self._connected_sockets[ws] = uuid.uuid4().hex
            self._send(self._connected_sockets[ws], {}, 'heartbeat')

    def release(self, ws):
        with self._lock:
            client = self._connected_sockets.pop(ws, None)
            if client is not None:
                self._send(client, {}, 'disconnect')
            for key, value in list(self._requests.items()):
                if value[0] is ws:
                    self._requests.pop(key)

    def connected_sockets(self):
        with self._lock:
            return list(self._connected_sockets)

    def owns_lease(self, ws):
        with self._lock:
            client = self._connected_sockets.get(ws)
            return bool(client and self._state and time.monotonic()-self._state_at < .5
                        and self._state.get('control_owner') == client)

    def handle_command(self, ws, payload):
        with self._lock:
            if self._shutting_down or ws not in self._connected_sockets:
                return dict(type='error', message='Operator connection is closed.')
            if not self._state or time.monotonic() - self._state_at >= .5:
                return dict(type='error', message='Operator control ROS node unavailable.')
            payload = dict(payload)
            kind = payload.get('type')
            if kind in ('probe_target', 'probe_zero_contact'):
                kind = 'actuator'
            if kind in ('adc', 'settings', 'actuator'):
                if len(self._requests) >= 8:
                    return dict(type='error' if kind == 'actuator' else kind+'_result', ok=False, message='Configuration requests are pending.')
                request_id = uuid.uuid4().hex
                payload['request_id'] = request_id
                self._requests[request_id] = (ws, kind, time.monotonic())
            self._send(self._connected_sockets[ws], payload)

    def replies(self):
        with self._lock:
            values = list(self._replies)
            self._replies.clear()
            return values

    def has_control_state(self):
        with self._lock:
            return self._state is not None

    def snapshot(self):
        with self._lock:
            if self._state is None or time.monotonic() - self._state_at >= .5:
                return None
            value = copy.deepcopy(self._state)
            value.pop('control_owner', None)
            if time.monotonic()-self._detector_at < .5:
                value.update(copy.deepcopy(self._detector))
            else:
                value.update(detector={'fresh': False, 'sensor': {'fresh': False}, 'beam': [], 'signal_ratio': 0, 'threshold_ratio': 1, 'fixture_angle_deg': 0, 'sweep_enabled': False, 'sweep_speed_deg_s': 0, 'sweep_speed_min': .684, 'sweep_speed_max': 180, 'beam_min_angle_deg': -45, 'beam_max_angle_deg': 45, 'beam_half_angle_deg': 45}, detections=[])
            selected = value.pop('selected_actuator', 'arm')
            actuators = {role: copy.deepcopy(self._actuators.get(role, {}))
                         if time.monotonic()-self._actuator_times.get(role, 0) < .5
                         else {'connected': False, 'reason': role.title() + ' controller unavailable'}
                         for role in ('arm', 'probe')}
            value['actuators'] = actuators
            value['arm_servo'] = {**actuators[selected], 'active_role': selected,
                'role_ids': {role: state.get('servo_id') for role, state in actuators.items()},
                'arm_calibration': actuators['arm'].get('arm_calibration'),
                'probe': actuators['probe'].get('probe', {'ready': False})}
            if 'probe' in actuators['probe']:
                value['probe'] = copy.deepcopy(actuators['probe']['probe'])
            else:
                value['probe'] = {'depth_mm': None, 'target_mm': 0, 'max_depth_mm': None, 'pressure_ratio': None, 'pressure_history': [], 'pressure_window_s': 30, 'fault': False, **value.get('probe', {}), 'ready': False,
                    'reason': actuators['probe'].get('reason', 'Probe unavailable')}

            value['cameras'] = copy.deepcopy(self._camera_frames) if time.monotonic()-self._cameras_at < 1 else {}
            value['power'] = copy.deepcopy(self._power) if time.monotonic()-self._power_at < 1 else {}
            value['settings'] = copy.deepcopy(self._settings) if time.monotonic()-self._settings_at < 2 else None
            return value

    def begin_shutdown(self):
        with self._lock:
            self._shutting_down = True
            for client in self._connected_sockets.values():
                self._send(client, {}, 'disconnect')
