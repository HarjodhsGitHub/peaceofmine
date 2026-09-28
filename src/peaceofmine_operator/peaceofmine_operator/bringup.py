"""Start and stop the operator subsystems as child launches for the Setup panel.

Commands come only from the fixed SUBSYSTEMS table and launch settings; no
browser-supplied text reaches a command line.
"""
import collections
import ctypes
import os
import signal
import subprocess
import threading
import time
from glob import glob
from pathlib import Path

DEFAULTS = dict(
    name='self',
    lli_serial_device='/dev/serial/by-id/usb-SVEA_PX4_AUTOPILOT_0-if00',
    lli_baud_rate='921600',
    max_velocity='0.8',
    use_joy='false',
    use_front_camera='true',
    use_auxiliary_camera='false',
    front_camera_device='auto',
    auxiliary_camera_device='auto',
    camera_width='640',
    camera_height='480',
    front_camera_framerate='30.0',
    auxiliary_camera_framerate='30.0',
    front_camera_pixel_format='mjpeg2rgb',
    auxiliary_camera_pixel_format='yuyv2rgb',
    camera_stream_port='8081',
    calibration_file='',
    sensor_serial_port='/dev/serial/by-id/usb-Arduino_Srl_Arduino_Uno_7543134333435170D1E2-if00',
    sensor_baud_rate='115200',
    sensor_baseline_adc='0.0',
    sensor_full_response_adc='255.0',
    sensor_reference_voltage='5.0',
    arm_serial_port='auto',
    arm_servo_id='1',
    arm_minimum='-1',
    arm_center='-1',
    arm_maximum='-1',
    arm_speed='116',
    arm_sweep_endpoint_tolerance_deg='4.0',
    arm_sweep_acceleration_deg_s2='34.332',
    probe_servo_id='2',
    probe_minimum='-1',
    probe_center='-1',
    probe_maximum='-1',
)

# Insertion order is the "Start everything" order: the car link first, so PX4
# safety status exists before any servo can be granted permission.
SUBSYSTEMS = {
    'vehicle': dict(label='Car link (PX4)', launch='operator_drive.launch.xml',
                    args=('name', 'lli_serial_device', 'lli_baud_rate', 'max_velocity', 'use_joy')),
    'cameras': dict(label='Cameras', launch='operator_cameras.launch.xml',
                    args=('name', 'use_front_camera', 'use_auxiliary_camera', 'front_camera_device',
                          'auxiliary_camera_device', 'camera_width', 'camera_height',
                          'front_camera_framerate', 'auxiliary_camera_framerate',
                          'front_camera_pixel_format', 'auxiliary_camera_pixel_format',
                          'camera_stream_port')),
    'detector': dict(label='Metal detector', launch='operator_detector.launch.xml',
                     args=('name', 'calibration_file', 'sensor_serial_port', 'sensor_baud_rate',
                           'sensor_baseline_adc', 'sensor_full_response_adc', 'sensor_reference_voltage')),
    'arm': dict(label='Arm and probe servos', launch='operator_arm.launch.xml',
                args=('name', 'calibration_file', 'arm_serial_port', 'arm_servo_id', 'arm_minimum',
                      'arm_center', 'arm_maximum', 'arm_speed', 'arm_sweep_endpoint_tolerance_deg',
                      'arm_sweep_acceleration_deg_s2', 'probe_servo_id', 'probe_minimum',
                      'probe_center', 'probe_maximum')),
}

LOG_LINES = 50
STARTUP_S = 3.0
STOP_TIMEOUTS_S = ((signal.SIGINT, 8.0), (signal.SIGTERM, 4.0), (signal.SIGKILL, 2.0))
GROUP_SWEEP_S = 1.0


def launch_command(name, settings):
    subsystem = SUBSYSTEMS[name]
    return ['ros2', 'launch', 'peaceofmine_operator', subsystem['launch'],
            *(f'{key}:={settings[key]}' for key in subsystem['args'] if settings.get(key, '') != '')]


def _path_check(path, missing):
    return dict(found=Path(path).exists(), detail=path if Path(path).exists() else missing)


def _glob_check(pattern, missing):
    matches = sorted(glob(pattern))
    return dict(found=bool(matches), detail=', '.join(Path(match).name for match in matches) or missing)


def device_check(name, settings):
    """Report whether the hardware a subsystem needs is plugged in."""
    if name == 'vehicle':
        return _path_check(settings['lli_serial_device'], 'PX4 USB not found')
    if name == 'detector':
        return _path_check(settings['sensor_serial_port'], 'Arduino not found')
    if name == 'arm':
        port = settings['arm_serial_port']
        if port not in ('', 'auto'):
            return _path_check(port, 'Servo adapter not found')
        return _glob_check('/dev/serial/by-id/usb-FTDI*', 'FTDI servo adapter not found')
    if name == 'cameras':
        device = settings['front_camera_device']
        if device not in ('', 'auto'):
            return _path_check(device, 'Camera not found')
        return _glob_check('/dev/v4l/by-id/*video-index0', 'No USB camera found')
    raise ValueError(f'Unknown subsystem {name!r}')


try:
    _PRCTL = ctypes.CDLL(None, use_errno=True).prctl
except (OSError, AttributeError):
    _PRCTL = None
DEVICE_CHECK_PERIOD_S = 1.0


def _die_with_parent():
    """Ask Linux to stop the child launch if the gateway itself is killed."""
    if _PRCTL is not None:
        _PRCTL(1, int(signal.SIGTERM))  # PR_SET_PDEATHSIG


class _Child:
    def __init__(self):
        self.process = None
        self.started = 0.0
        self.stopping = False
        self.failed = False
        self.exit_code = None
        self.log = collections.deque(maxlen=LOG_LINES)


class Bringup:
    def __init__(self, settings=None, commands=None, checks=None, clock=time.monotonic):
        self.settings = {**DEFAULTS, **(settings or {})}
        self._commands = commands or {}
        self._checks = checks or device_check
        self._clock = clock
        self._lock = threading.Lock()
        self._children = {name: _Child() for name in SUBSYSTEMS}
        self._devices = None
        self._devices_at = 0.0

    def _child(self, name):
        if name not in SUBSYSTEMS:
            raise ValueError(f'Unknown subsystem {name!r}')
        return self._children[name]

    def command(self, name):
        self._child(name)
        return list(self._commands[name]) if name in self._commands else launch_command(name, self.settings)

    def running(self, name):
        with self._lock:
            process = self._child(name).process
            return process is not None and process.poll() is None

    def start(self, name):
        child = self._child(name)
        with self._lock:
            if child.process is not None and child.process.poll() is None:
                return
            command = self.command(name)
            child.log.clear()
            child.log.append('$ ' + ' '.join(command))
            child.failed = child.stopping = False
            child.exit_code = None
            try:
                child.process = subprocess.Popen(
                    command, stdin=subprocess.DEVNULL, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                    text=True, bufsize=1, start_new_session=True, preexec_fn=_die_with_parent)
            except OSError as error:
                child.process = None
                child.failed = True
                child.log.append(f'Could not start: {error}')
                return
            child.started = self._clock()
            process = child.process
        threading.Thread(target=self._read, args=(child, process), daemon=True).start()

    def _read(self, child, process):
        for line in process.stdout:
            with self._lock:
                child.log.append(line.rstrip())
        process.stdout.close()
        code = process.wait()
        with self._lock:
            if child.process is process:
                child.exit_code = code
                child.failed = not child.stopping
                child.log.append(f'Exited with code {code}')

    def stop(self, name, wait=False):
        child = self._child(name)
        with self._lock:
            process = child.process
            if process is None or process.poll() is not None:
                return
            child.stopping = True
        thread = threading.Thread(target=self._terminate, args=(process,), daemon=True)
        thread.start()
        if wait:
            thread.join()

    @staticmethod
    def _terminate(process):
        for sig, timeout in STOP_TIMEOUTS_S:
            try:
                os.killpg(process.pid, sig)
            except ProcessLookupError:
                return
            try:
                process.wait(timeout=timeout)
                break
            except subprocess.TimeoutExpired:
                continue
        # A leader that exits on SIGINT can leave group members that ignore it.
        for sig in (signal.SIGTERM, signal.SIGKILL):
            try:
                os.killpg(process.pid, sig)
            except ProcessLookupError:
                return
            deadline = time.monotonic() + GROUP_SWEEP_S
            while time.monotonic() < deadline:
                try:
                    os.killpg(process.pid, 0)
                except ProcessLookupError:
                    return
                time.sleep(.05)

    def restart(self, name, wait=False):
        self._child(name)

        def cycle():
            self.stop(name, wait=True)
            self.start(name)

        thread = threading.Thread(target=cycle, daemon=True)
        thread.start()
        if wait:
            thread.join()

    def devices(self):
        now = self._clock()
        if self._devices is None or now - self._devices_at >= DEVICE_CHECK_PERIOD_S:
            self._devices = {name: self._checks(name, self.settings) for name in SUBSYSTEMS}
            self._devices_at = now
        return self._devices

    def start_all(self):
        """Start every subsystem whose hardware is present; report the skipped ones."""
        self._devices = None
        devices = self.devices()
        skipped = []
        for name in SUBSYSTEMS:
            if devices[name]['found']:
                self.start(name)
            elif not self.running(name):
                skipped.append(name)
        return skipped

    def stop_all(self, wait=False):
        for name in reversed(SUBSYSTEMS):
            self.stop(name, wait=wait)

    def snapshot(self):
        now = self._clock()
        devices = self.devices()
        subsystems = {}
        with self._lock:
            for name, child in self._children.items():
                alive = child.process is not None and child.process.poll() is None
                if alive:
                    state = 'stopping' if child.stopping else (
                        'starting' if now - child.started < STARTUP_S else 'running')
                else:
                    state = 'failed' if child.failed else 'stopped'
                subsystems[name] = dict(label=SUBSYSTEMS[name]['label'], state=state,
                                        exit_code=child.exit_code, device=devices[name],
                                        log=list(child.log))
        return dict(enabled=True, order=list(SUBSYSTEMS), subsystems=subsystems)
