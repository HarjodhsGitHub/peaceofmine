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
        html = html.replace('<script src="/assets/app.js"></script>', '')
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
state.connected = true; state.drive.armed = true; state.drive.you_control_owner = true;
$('settings').click();
assert(sent.some(m => m.type === 'estop'), 'settings disarms');
lastDriveSent = -1000; inputLoop();
assert(sent.at(-1).deadman === false && sent.at(-1).linear_x === 0, 'settings blocks drive');
$('settings-dialog').close();
preferences.input = 'controller'; lastDriveSent = -1000; inputLoop();
assert(!sent.at(-1).deadman, 'nonstandard pad cannot drive standard profile');
preferences.input = 'wheel';
Object.defineProperty(navigator, 'getGamepads', {value: () => []});
lastDriveSent = -1000; inputLoop();
assert(!sent.at(-1).deadman && sent.at(-1).linear_x === 0, 'disconnect stops');
smoothedDrive.linear = .8; keys.add('KeyW'); window.dispatchEvent(new Event('blur'));
assert(smoothedDrive.linear === 0 && keys.size === 0, 'blur clears input');
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
document.body.textContent = 'BROWSER TESTS PASSED';
'''
        script = setup + (DASHBOARD / 'app.js').read_text()
        html += '<script>(async () => {try {' + script + checks + "} catch(e) {document.body.textContent = 'TEST FAILED: ' + e.stack;}})();</script>"
        with tempfile.TemporaryDirectory() as tmp:
            page = pathlib.Path(tmp) / 'test.html'
            page.write_text(html)
            result = subprocess.run(['chromium', '--headless', '--no-sandbox', '--disable-gpu',
                                     '--user-data-dir=' + tmp + '/profile', '--dump-dom', page.as_uri()],
                                    capture_output=True, text=True, timeout=30)
        self.assertIn('BROWSER TESTS PASSED</body>', result.stdout, result.stdout + result.stderr[-2000:])


if __name__ == '__main__':
    unittest.main()
