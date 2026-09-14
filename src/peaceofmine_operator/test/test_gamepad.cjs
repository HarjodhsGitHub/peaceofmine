// Run: node src/peaceofmine_operator/test/test_gamepad.cjs
function runTests(source) {
  const inputFunctions = source.slice(source.indexOf('function buttonValue('),
    source.indexOf('function readKeyboard('));
  const check = new Function(`${inputFunctions}
    const preferences = {mapping: 'auto', sensitivity: 100};
    const triggerSeen = {lt: false, rt: false};
    let lastPadId = null;
    const MAX_LINEAR_MPS = 0.8;
    const MAX_ANGULAR_RAD_S = 1.4;
    let gamepad = {id: 'Xbox 360', mapping: 'standard', axes: [0, 0, 1, 0],
      buttons: Array.from({length: 17}, () => ({value: 0, pressed: false}))};
    const target = {};
    function expect(ok, message) { if (!ok) throw new Error(message); }
    readGamepad(target);
    expect(target.linear === 0 && !target.deadman, 'Right stick must not drive');
    gamepad.axes = [-1, 0, 0, 0];
    gamepad.buttons[7] = {value: 1, pressed: true};
    readGamepad(target);
    expect(target.linear === 0.8 && target.deadman, 'RT must drive forward');
    expect(target.angular > 0, 'Left stick must produce left ROS yaw');
    gamepad.buttons[7] = {value: 0, pressed: false};
    gamepad.buttons[6] = {value: 1, pressed: true};
    readGamepad(target);
    expect(target.linear === -0.8, 'LT must reverse');
    preferences.mapping = 'xbox360';
    gamepad.id = 'Raw Xbox 360';
    gamepad.axes = [0, 0, -1, 0, 0, -1];
    readGamepad(target);
    expect(target.linear === 0 && !target.deadman, 'Back must not act as LT');
    gamepad.axes[5] = 1;
    readGamepad(target);
    expect(target.linear === 0.8, 'Raw RT must drive forward');
    gamepad.axes[2] = 1;
    gamepad.axes[5] = -1;
    readGamepad(target);
    expect(target.linear === -0.8, 'Raw LT must reverse');
  `);
  check();
  return 'Browser gamepad regression checks passed';
}

if (typeof require !== 'undefined' && require.main === module) {
  const fs = require('node:fs');
  const path = require('node:path');
  console.log(runTests(fs.readFileSync(path.join(__dirname, '../dashboard/app.js'), 'utf8')));
}
module.exports = runTests;
