"""Run the actual dashboard scripts in Chromium with synthetic input devices."""
import pathlib
import shutil
import subprocess
import tempfile
import unittest

DASHBOARD = pathlib.Path(__file__).resolve().parents[1] / 'dashboard'


@unittest.skipUnless(shutil.which('chromium'), 'Chromium required')
class BrowserInputs(unittest.TestCase):
    def test_inputs_and_stop_gates(self):
        html = (DASHBOARD / 'index.html').read_text()
        import re
        html = re.sub(r'<script src="/assets/[^"]+"></script>', '', html)
        html = html.replace('<link rel="stylesheet" href="/assets/style.css">', '<style>' + (DASHBOARD / 'style.css').read_text() + '</style>')
        setup = '''
window.requestAnimationFrame = () => 0;
window.setInterval = () => 0;
window.sent = [];
window.WebSocket = class { static OPEN = 1; readyState = 1;
  send(data) { sent.push(JSON.parse(data)); } };
Object.defineProperty(document, 'hasFocus', {value: () => true});
Object.defineProperty(document, 'hidden', {value: false});
'''
        checks = '''
const assert = (condition, label) => { if (!condition) throw Error(label); };
const latestDrive = () => sent.filter(m => m.type === 'drive').at(-1);
state.connected = true;
state.power = {battery: {available: true, present: true, stale: false, power_w: 24, current_a: 2, voltage_v: 12, remaining_pct: 50},
  esc: {available: false}, history: [{age_s: 2, watts: 24}]};
updatePower();
assert($('battery-watts').textContent === '24.0 W' && $('battery-charge').textContent === '50 %', 'power metrics render');
assert($('esc-status').textContent === 'No telemetry', 'missing ESC does not invent readings');
state.power.battery.stale = true; updatePower();
assert($('battery-watts').textContent === '—' && $('battery-level').hidden, 'stale power is hidden');
state.connected = false; updatePower();
assert($('power-status').textContent === 'DISCONNECTED', 'power follows connection status');
state.arm_servo = {discovered_servos: [1, 2], serial_connected: true};
updateArmSettings();
assert($('arm-servo-id').options.length === 3, 'detected servo options');
assert($('arm-servo-id').value === '', 'discovery never chooses an actuator');
$('arm-servo-id').value = '2';
updateArmSettings();
assert($('arm-servo-id').value === '2', 'telemetry preserves pending selection');
state.arm_servo = {discovered_servos: [], scanning: true};
updateArmSettings();
assert($('arm-servo-id').value === '' && $('arm-select-id').disabled, 'lost adapter clears selection');
const pad = {index: 0, id: 'Test wheel', mapping: '', axes: [0, 1, 1], buttons: [{pressed: true, value: 1}]};
const mapping = {steering: 0, throttle: 1, brake: 2, deadman: 0, deadzone: .05, invertThrottle: true, invertBrake: true};
assert(readWheel(pad, mapping).linear === 0, 'released pedals');
pad.axes[1] = -1;
assert(readWheel(pad, mapping).linear === MAX_LINEAR_MPS, 'forward pedal');
pad.axes[1] = 1; pad.axes[2] = -1;
assert(readWheel(pad, mapping).linear === -MAX_LINEAR_MPS, 'reverse pedal');
assert(!readWheel(pad, {...mapping, steering: 9}).deadman, 'missing axis');
assert(!readWheel(pad, {...mapping, throttle: 2}).deadman, 'combined pedals rejected');
pad.axes = [0.01, 1, 1];
assert(readWheel(pad, mapping).angular === 0, 'center deadzone');
Object.defineProperty(navigator, 'getGamepads', {value: () => [pad], configurable: true});
selectedPadIndex = 0; selectedPadId = pad.id; preferences.input = 'wheel';
state.safety = {allowed: true, servo_allowed: true, mode: 'simulation'};
state.connected = true; state.drive.armed = true; state.drive.you_control_owner = true;
$('settings').click();
assert(sent.some(m => m.type === 'estop'), 'settings disarms');
lastDriveSent = -1000; inputLoop();
assert(latestDrive().deadman === false && latestDrive().linear_x === 0, 'settings blocks drive');
const beforeSettingsClose = sent.length;
$('settings-dialog').querySelector('button[value="close"]').click();
const closeMessages = sent.slice(beforeSettingsClose);
assert(closeMessages.some(m => m.type === 'arm_servo' && m.action === 'stop'), 'closing settings releases arm');
assert(closeMessages.some(m => m.type === 'estop'), 'closing settings exits calibration');
preferences.input = 'controller'; lastDriveSent = -1000; inputLoop();
assert(!latestDrive().deadman, 'unmapped wheel has no trigger drive');
preferences.input = 'wheel';
Object.defineProperty(navigator, 'getGamepads', {value: () => []});
lastDriveSent = -1000; inputLoop();
assert(!latestDrive().deadman && latestDrive().linear_x === 0, 'disconnect stops');
lastInput.linear = .8; keys.add('KeyW'); window.dispatchEvent(new Event('blur'));
assert(lastInput.linear === 0 && keys.size === 0, 'blur clears input');
setConnection(true, 'Raspberry Pi connected');
assert($('connection-overlay').classList.contains('hidden'), 'connected overlay hidden');
setConnection(false, 'Raspberry Pi disconnected');
assert(!$('connection-overlay').classList.contains('hidden'), 'disconnect overlay visible');
preferences.cameras.forward = 'virtual'; preferences.cameras.auxiliary = 'off';
updateCameraVisibility();
assert(!$('forward-view').classList.contains('hidden'), 'virtual camera visible');
assert($('forward-video').classList.contains('hidden'), 'unused browser camera hidden');
document.querySelector('[data-settings-tab="cameras"]').click();
assert($('settings-cameras').classList.contains('active'), 'camera tab opens');
assert(getComputedStyle($('settings-cameras')).backgroundColor === 'rgba(0, 0, 0, 0)', 'settings panel does not inherit alert badge color');
assert(getComputedStyle($('forward-network-camera')).objectFit === 'contain', 'camera preserves full field of view');
assert(!$('settings-input').classList.contains('active'), 'control tab closes');
state.cameras = {front: {label: 'Front', topic: '/camera_front/camera_info'}, auxiliary: {label: 'Auxiliary', topic: '/camera_auxiliary/camera_info'}};
updateNetworkCameraOptions();
assert(preferences.cameras.forward === 'virtual', 'discovery preserves virtual selection');
assert([...$('forward-camera-source').options].some(option => option.value === 'raspberry:front'), 'Raspberry Pi front camera listed');
assert([...$('forward-camera-source').options].some(option => option.value === 'raspberry:auxiliary'), 'Raspberry Pi auxiliary camera listed');
localCameras = [{deviceId: 'local-1', label: 'Laptop'}];
preferences.cameras.forward = 'local-1';
rebuildCameraOptions();
state.cameras.extra = {label: 'Extra', topic: '/extra/camera_info'};
updateNetworkCameraOptions();
assert($('forward-camera-source').value === 'local-1', 'discovery preserves laptop selection');
rebuildCameraOptions(); rebuildCameraOptions();
assert([...$('forward-camera-source').options].filter(o => o.value === 'local-1').length === 1, 'no duplicate cameras');
preferences.cameras.forward = 'raspberry:saved'; rebuildCameraOptions();
assert($('forward-camera-source').value === 'raspberry:saved', 'missing saved camera remains selected');
keys.clear(); keys.add('KeyA'); assert(readKeyboard().angular > 0, 'A turns left in ROS');
keys.clear(); keys.add('KeyD'); assert(readKeyboard().angular < 0, 'D turns right in ROS');
keys.clear();
state.connected = true; state.drive.you_control_owner = true;
$('probe-slider').value = 17; $('probe-slider').dispatchEvent(new Event('input'));
$('probe-slider').value = 18; $('probe-slider').dispatchEvent(new Event('change'));
assert(sent.at(-1).depth_mm === 18, 'slider release sends final value');
state.drive.you_control_owner = false;
const count = sent.length; stopInput();
assert(sent.length === count, 'spectator settings do not send drive errors');
const oldSocket = socket;
state.drive.you_control_owner = true; state.drive.armed = true;
connect();
assert(!state.drive.you_control_owner && !state.drive.armed, 'retry clears lease immediately');
oldSocket.onmessage({data: JSON.stringify({type: 'state', drive: {armed: true, you_control_owner: true}})});
assert(!state.drive.you_control_owner, 'superseded socket cannot restore lease');
let resolveCapture;
Object.defineProperty(navigator, 'mediaDevices', {configurable: true, value: {
  getUserMedia: () => new Promise(resolve => {resolveCapture = resolve;})
}});
preferences.cameras.forward = 'pending-camera';
const pendingCapture = startCamera('forward', 'pending-camera');
preferences.cameras.forward = 'virtual';
await startCamera('forward', 'virtual');
let stoppedTracks = 0;
resolveCapture({getTracks: () => [{stop: () => stoppedTracks++}]});
await pendingCapture;
assert(stoppedTracks === 1 && cameraStreams.forward === null, 'late capture is stopped after switching');
assert(cameraSources.forward === 'virtual', 'late capture cannot replace selected source');
let rejectCapture;
navigator.mediaDevices.getUserMedia = () => new Promise((resolve, reject) => {rejectCapture = reject;});
preferences.cameras.forward = 'rejected-camera';
const rejectedCapture = startCamera('forward', 'rejected-camera');
preferences.cameras.forward = 'virtual';
await startCamera('forward', 'virtual');
rejectCapture(Error('permission denied'));
await rejectedCapture;
assert(cameraErrors.forward === '', 'superseded camera errors are ignored');
const xbox = {index: 1, id: 'Xbox 360', mapping: '', timestamp: 1,
  axes: [-1, 0, -1, 0, 0, 1], buttons: Array.from({length: 17}, () => ({value: 0, pressed: false}))};
Object.defineProperty(navigator, 'getGamepads', {configurable: true, value: () => [xbox]});
preferences.input = 'controller'; preferences.mapping = 'auto';
state.connected = true; state.drive.you_control_owner = true; state.drive.armed = true;
lastDriveSent = -1000; inputLoop();
assert(latestDrive().linear_x === .8 && latestDrive().angular_z > 0 && latestDrive().deadman, 'raw Xbox automatically selected and drives');
$('settings').click();
xbox.buttons[0] = {pressed: true, value: 1}; xbox.timestamp++;
state.drive.armed = false;
const beforeSettings = sent.length;
lastDriveSent = -1000; inputLoop();
assert(!sent.slice(beforeSettings).some(m => m.type === 'arm' || (m.type === 'drive' && m.deadman)), 'settings blocks Xbox drive and A arming');
$('settings-dialog').close();
state.drive.armed = true; xbox.timestamp++; lastDriveSent = -1000; inputLoop();
Object.defineProperty(navigator, 'getGamepads', {configurable: true, value: () => []});
lastDriveSent = -1000; inputLoop();
assert(!latestDrive().deadman && latestDrive().linear_x === 0, 'active Xbox disconnect stops drive');
runTests(dashboardSource);
assert(!$('settings-connection') && !document.querySelector('[data-settings-tab="connection"]'), 'unused Connection tab removed');
preferences.cameras.forward = 'raspberry:front';
state.cameras.front = {label: 'Front', topic: '/front/camera_info', width: 1280, height: 720, fps: 29.8, frame_age_ms: 20};
state.connected = true; lastTelemetryAt = performance.now();
$('settings-dialog').showModal();
document.querySelector('[data-settings-tab="cameras"]').click();
renderCameraSettings();
assert($('forward-dimensions').textContent === '1280 × 720 px', 'camera resolution shown');
assert($('forward-fps').textContent.includes('29.8'), 'source FPS shown');
assert($('forward-source-detail').textContent === '/front/camera_info', 'source topic shown');
preferences.cameraRotation.forward = 0;
$('forward-rotate').click();
assert(preferences.cameraRotation.forward === 90 && $('forward-network-camera').style.transform.includes('90deg'), 'rotate applies to main view');
assert(JSON.parse(localStorage.getItem('peaceofmine.operator.preferences')).cameraRotation.forward === 90, 'rotation persists');
for (let i = 0; i < 3; i++) $('forward-rotate').click();
assert(preferences.cameraRotation.forward === 0, 'rotation wraps at 360');
preferences.cameras.auxiliary = 'off'; renderCameraSettings();
assert($('auxiliary-preview-state').textContent === 'Off' && $('auxiliary-dimensions').textContent === '—', 'off camera has no invented stats');
$('settings-dialog').close();
preferences.input = 'keyboard'; cameraFocused = true;
const keyEvent = (type, code, keyCode) => document.dispatchEvent(new KeyboardEvent(type, {code, keyCode, bubbles: true}));
keyEvent('keydown', 'KeyW', 87); keyEvent('keydown', 'KeyA', 65);
assert(keys.has('KeyW') && keys.has('KeyA') && !keys.has('ShiftLeft'), 'WASD works without Shift');
resetKeyboardRamp(0);
let ramped = readKeyboard(); rampKeyboard(ramped, 16);
assert(ramped.linear > 0 && ramped.linear < .1 && ramped.angular > 0 && ramped.angular < .2, 'WASD starts gradually with correct yaw');
const runRamp = hz => { resetKeyboardRamp(0); let value; for (let i = 1; i <= hz / 2; i++) { value = readKeyboard(); rampKeyboard(value, i * 1000 / hz); } return value; };
const at30 = runRamp(30), at60 = runRamp(60);
assert(Math.abs(at30.linear - at60.linear) < .00001 && Math.abs(at30.angular - at60.angular) < .00001, 'ramp independent of frame rate');
keyEvent('keyup', 'KeyW', 87); keyEvent('keyup', 'KeyA', 65);
ramped = readKeyboard(); rampKeyboard(ramped, 516);
assert(ramped.linear === 0 && ramped.angular === 0, 'releasing WASD stops without a separate deadman');

state.safety = {allowed: false, mode: 'kill', reason: 'PX4 kill'};
state.connected = true; state.drive.you_control_owner = true; state.drive.armed = false;
updateHud();
assert($('rc-safety').textContent === 'RC KILL', 'physical safety indicator');
assert(!$('arm') && !$('estop') && $('probe-slider').disabled && $('sweep-toggle').disabled && $('sweep-speed').disabled, 'RC kill gates actuation without UI arm buttons');
state.arm_servo = {connected: true, servo_id: 2, discovered_servos: [1, 2], serial_port: '/dev/serial/by-id/arm', position: 2000, torque: false,
  calibration: {minimum: 1500, center: 2000, maximum: 2500}};
updateArmSettings();
assert($('arm-launch-xml').value.includes('arm_servo_id" default="2') && $('arm-launch-xml').value.includes('arm_minimum" default="1500'), 'calibration exports launch XML');
assert($('arm-move-center').disabled, 'calibration cannot move under PX4 kill');
state.arm_servo.serial_connected = true;
state.drive.you_control_owner = false;
updateArmSettings();
assert($('arm-select-id').disabled && !$('arm-take-control').hidden, 'calibration exposes control requirement');
assert($('arm-calibration-access').textContent.includes('Take control'), 'disabled selection explains next step');
state.drive.you_control_owner = true;
updateArmSettings();
assert(!$('arm-select-id').disabled && $('arm-take-control').hidden, 'selection enabled for disarmed owner');
state.arm_servo.torque = true;
updateArmSettings();
assert($('arm-select-id').disabled && $('arm-calibration-access').textContent.includes('Stop'), 'moving servo cannot change ID');
state.arm_servo.torque = false;
state.drive.armed = false;
state.drive.calibrating = false;
state.safety = {allowed: false, servo_allowed: true, mode: 'override'};
state.arm_servo.calibration = null;
updateArmSettings();
assert(!$('arm-calibration-enable').disabled, 'jog setup available before limits exist');
$('arm-select-id').click();
const selection = sent.at(-1);
assert(selection.request_id && $('arm-feedback').textContent.includes('Selecting'), 'selection gives immediate pending feedback');
state.arm_servo.last_command = {request_id: selection.request_id, success: true, message: 'Servo 2 selected'};
updateArmSettings();
assert($('arm-feedback').textContent === 'Servo 2 selected', 'driver acknowledgement shown');
state.drive.armed = true; state.drive.calibrating = true;
updateArmSettings();
assert(!$('arm-jog-left').disabled && !$('arm-jog-right').disabled, 'uncalibrated jog enabled in RC override');
$('arm-jog-left').onkeydown({key: ' ', repeat: false, preventDefault() {}});
assert(sent.at(-1).action === 'jog' && sent.at(-1).direction === -1, 'hold sends left jog');
$('arm-jog-left').onkeyup({key: ' '});
assert(sent.at(-1).action === 'stop', 'jog release sends stop');
updateArmSettings();
$('arm-jog-speed').value = '6';
updateArmSettings();
$('arm-position-slider').value = '2030'; $('arm-position-slider').oninput();
assert(sent.at(-1).action === 'position' && sent.at(-1).position === 2030, 'slider directly sends target');
assert(!('speed_deg_s' in sent.at(-1)) && !('travel_deg' in sent.at(-1)), 'slider independent of jog speed');
assert(!$('arm-position-slider').disabled, 'slider remains adjustable during movement');
assert(Number($('arm-position-slider').value) === 2000, 'slider mirrors actual position while moving');
$('arm-position-slider').onpointerdown();
$('arm-position-slider').value = '2040'; $('arm-position-slider').oninput();
state.arm_servo.position = 2010; updateArmSettings();
assert(Number($('arm-position-slider').value) === 2040, 'telemetry does not fight pointer drag');
$('arm-position-slider').onpointerup();
assert(Number($('arm-position-slider').value) === 2010 && armSliderTarget === 2040, 'release follows measurement without losing target');
state.arm_servo.position = 2040; state.arm_servo.goal_position = 2040; state.arm_servo.torque = true;
updateArmSettings();
assert(sent.at(-1).action === 'stop' && !armSliderMoving, 'slider stops on arrival');
state.arm_servo.torque = false;
$('arm-position-slider').value = '2040'; $('arm-position-slider').oninput();
state.safety.servo_allowed = false; updateArmSettings();
assert(sent.at(-1).action === 'stop' && !armSliderMoving, 'permission loss cancels slider movement');
state.safety.servo_allowed = true;
assert($('arm-servo-position').textContent.includes('°'), 'position shown as degrees');
state.arm_servo.load_percent = 12.5; state.arm_servo.torque_limit_percent = 50;
state.arm_servo.current_a = .123; state.arm_servo.speed_deg_s = 2;
updateArmSettings();
assert($('arm-load').textContent === '12.5 %' && $('arm-torque-limit').textContent === '50.0 %', 'load and torque limit displayed separately');
assert($('arm-current').textContent === '0.123 A', 'current telemetry displayed');
$('settings-dialog').showModal();
const beforeCalibrationLoop = sent.length;
inputLoop();
assert(!sent.slice(beforeCalibrationLoop).some(m => m.type === 'drive'), 'calibration does not send background drive commands');
state.drive.calibrating = false;
state.drive.you_control_owner = true;
state.safety = {allowed: false, servo_allowed: true, mode: 'override'};
state.drive.rc = {fresh: true, steering: .5, throttle: -.7, channels: [1750, 1150, 1500, 1500, 1500], steering_channel: 1, throttle_channel: 2};
updateHud();
assert(!$('sweep-toggle').disabled, 'RC override permits sweep without website arming');
assert($('controller').textContent === 'Physical RC transmitter' && $('pad-raw').textContent.includes('1750'), 'RC inputs visualized');
state.detector.sensor = {available: true, fresh: true, packet: {v: 1, seq: 2, uptime_ms: 100, amplitude_adc: 20},
  baseline_adc: 20, full_response_adc: 150, reference_voltage: 5, rate_hz: 20, age_ms: 10,
  history: [{age_s: 1, adc: 20}, {age_s: .05, adc: 150}]};
state.robot.speed_mps = 0;
document.querySelector('[data-settings-tab="detector"]').click();
updateMetalSettings();
assert($('metal-voltage').textContent === '0.392 V', 'ADC peak converted to volts');
assert(!$('metal-zero').disabled && !$('settings-detector').hidden, 'detector tab and zero available');
$('metal-zero').click();
assert(sent.at(-1).type === 'detector_calibrate' && sent.at(-1).action === 'zero', 'zero reaches gateway');
metalPending = null;
state.drive.you_control_owner = false; updateMetalSettings();
assert($('metal-zero').disabled && $('metal-apply').disabled, 'spectator cannot calibrate');
state.detector.sensor.fresh = false; updateMetalSettings();
assert($('metal-voltage').textContent === '-- V', 'stale voltage hidden');
document.body.textContent = 'BROWSER TESTS PASSED';
'''
        import json
        source = (DASHBOARD / 'app.js').read_text()
        gamepad_checks = (DASHBOARD.parent / 'test/test_gamepad.cjs').read_text().split("if (typeof require")[0]
        script = setup + (DASHBOARD / 'keydrown-1.3.0.js').read_text() + source + gamepad_checks + '\nconst dashboardSource = ' + json.dumps(source) + ';\n'
        html += '<script>(async () => {try {' + script + checks + "} catch(e) {document.body.textContent = 'TEST FAILED: ' + e.stack;}})();</script>"
        with tempfile.TemporaryDirectory() as tmp:
            page = pathlib.Path(tmp) / 'test.html'
            page.write_text(html)
            result = subprocess.run(['chromium', '--headless', '--no-sandbox', '--disable-gpu',
                                     '--user-data-dir=' + tmp + '/profile', '--dump-dom', page.as_uri()],
                                    capture_output=True, text=True, timeout=60)
        self.assertIn('BROWSER TESTS PASSED</body>', result.stdout, result.stdout + result.stderr[-2000:])


if __name__ == '__main__':
    unittest.main()
