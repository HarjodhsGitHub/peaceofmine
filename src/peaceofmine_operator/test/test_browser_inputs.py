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
state.drive.you_control_owner = true;
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
assert(!$('settings-input').classList.contains('active'), 'control tab closes');
assert([...$('forward-camera-source').options].some(option => option.value === 'raspberry:front'), 'Raspberry Pi front camera listed');
assert([...$('forward-camera-source').options].some(option => option.value === 'raspberry:auxiliary'), 'Raspberry Pi auxiliary camera listed');
document.body.textContent = 'BROWSER TESTS PASSED';
'''
        script = setup + (DASHBOARD / 'app.js').read_text()
        html += '<script>try {' + script + checks + "} catch(e) {document.body.textContent = 'TEST FAILED: ' + e.stack;}</script>"
        with tempfile.TemporaryDirectory() as tmp:
            page = pathlib.Path(tmp) / 'test.html'
            page.write_text(html)
            result = subprocess.run(['chromium', '--headless', '--no-sandbox', '--disable-gpu',
                                     '--user-data-dir=' + tmp + '/profile', '--dump-dom', page.as_uri()],
                                    capture_output=True, text=True, timeout=30)
        self.assertIn('BROWSER TESTS PASSED</body>', result.stdout, result.stdout + result.stderr[-2000:])


if __name__ == '__main__':
    unittest.main()
