const state = {
  connected: false,
  armed: false,
  deadman: false,
  controller: null,
  drive: {armed: false, deadman: false, client_count: 0, you_control_owner: false, control_owner_present: false},
  robot: {x: 0, y: 0, yaw: 0, speed_mps: 0},
  detector: {signal_ratio: 0, threshold_ratio: .65, fixture_angle_deg: 0, sweep_enabled: false, sweep_speed_deg_s: 60, history: [], detected: false},
  probe: {depth_mm: 0, target_mm: 0, max_depth_mm: 110, pressure_ratio: 0, fault: false},
  events: [],
};

const $ = id => document.getElementById(id);
const forward = $('forward-view');
const map = $('map-view');
let socket;
let lastProbeSent = 0;
const keys = new Set();
const preferences = JSON.parse(localStorage.getItem('peaceofmine.operator.preferences') || '{"input":"controller","sensitivity":100}');
let cameraLocked = false;
const smoothedDrive = {linear: 0, angular: 0, updatedAt: performance.now()};

function send(message) {
  if (socket?.readyState === WebSocket.OPEN) socket.send(JSON.stringify(message));
}

function setConnection(connected, label) {
  state.connected = connected;
  $('connection').textContent = label;
  $('connection-dot').style.background = connected ? '#37b98e' : '#d36458';
  $('arm').disabled = !connected;
  $('estop').disabled = !connected;
  $('take-control').disabled = !connected;
}

function connect() {
  const scheme = location.protocol === 'https:' ? 'wss' : 'ws';
  socket = new WebSocket(`${scheme}://${location.host}/ws`);
  socket.onopen = () => setConnection(true, 'Dashboard linked');
  socket.onclose = () => {
    state.armed = false;
    setConnection(false, 'Connection lost — rover stopped');
    setTimeout(connect, 1200);
  };
  socket.onerror = () => socket.close();
  socket.onmessage = event => {
    const message = JSON.parse(event.data);
    if (message.type === 'error') {
      $('camera-notice').textContent = message.message;
      return;
    }
    if (message.type !== 'state') return;
    Object.assign(state, message);
    updateHud();
  };
}

function updateHud() {
  const {drive, robot, detector, probe} = state;
  const owner = drive.you_control_owner;
  $('client-count').textContent = `${drive.client_count} ${drive.client_count === 1 ? 'viewer' : 'viewers'}`;
  $('control-owner').textContent = owner ? 'YOU CONTROL' : (drive.control_owner_present ? 'SPECTATING' : 'UNCLAIMED');
  $('control-owner').className = `badge ${owner ? 'safe' : 'neutral'}`;
  $('take-control').textContent = drive.control_owner_present ? 'Steal control' : 'Take control';
  $('take-control').disabled = !state.connected || owner;
  $('arm').disabled = !state.connected || !owner;
  $('estop').disabled = !state.connected || !owner;
  $('probe-slider').disabled = !state.connected || !owner;
  $('speed').textContent = `${Math.abs(robot.speed_mps).toFixed(2)} m/s`;
  const signalPercent = detector.signal_ratio * 100;
  $('signal').textContent = Math.round(signalPercent);
  $('meter-fill').style.width = `${signalPercent}%`;
  $('threshold').style.left = `${detector.threshold_ratio * 100}%`;
  $('detector-detail').textContent = `Threshold ${Math.round(detector.threshold_ratio * 100)}% · fixture ${detector.fixture_angle_deg.toFixed(0)}°`;
  $('sweep-toggle').textContent = detector.sweep_enabled ? 'Stop sweep' : 'Start sweep';
  $('sweep-toggle').disabled = !state.connected || !owner;
  $('sweep-speed').disabled = !state.connected || !owner;
  if (document.activeElement !== $('sweep-speed')) $('sweep-speed').value = detector.sweep_speed_deg_s;
  $('sweep-speed-value').textContent = `${Math.round(detector.sweep_speed_deg_s)}°/s`;
  $('detector-status').textContent = detector.detected ? 'DETECTION' : 'CLEAR';
  $('detector-status').className = `badge ${detector.detected ? 'active' : 'safe'}`;
  $('probe-depth').textContent = `${Math.round(probe.depth_mm)} mm`;
  $('probe-target').textContent = `${Math.round(probe.target_mm)} mm`;
  $('probe-pressure').textContent = `${Math.round(probe.pressure_ratio * 100)}%`;
  $('probe-slider').max = probe.max_depth_mm;
  if (document.activeElement !== $('probe-slider')) $('probe-slider').value = probe.target_mm;
  $('probe-arm').style.height = `${Math.max(0, probe.depth_mm / probe.max_depth_mm * 3.1)}rem`;
  $('probe-status').textContent = probe.fault ? 'FAULT' : probe.depth_mm > 2 ? 'DEPLOYED' : 'STOWED';
  $('probe-status').className = `badge ${probe.fault ? 'active' : probe.depth_mm > 2 ? 'safe' : 'neutral'}`;
  $('arm-state').textContent = !owner ? 'SPECTATOR' : (drive.armed ? (drive.deadman ? 'DRIVING' : 'ARMED') : 'DISARMED');
  $('arm-state').className = `badge ${drive.armed ? 'active' : 'neutral'}`;
  $('arm').textContent = drive.armed ? 'Armed' : 'Arm control';
  $('camera-notice').textContent = !owner ? 'Spectator mode · take control to arm or actuate.' : drive.armed
    ? (drive.deadman ? 'Deadman held · command stream active.' : 'Armed · hold the right bumper to drive.')
    : 'Disarmed · arm only after the lane is clear.';
  const heading = ((robot.yaw * 180 / Math.PI % 360) + 360) % 360;
  $('position').textContent = `X ${robot.x.toFixed(1)} · Y ${robot.y.toFixed(1)} · heading ${heading.toFixed(0)}°`;
}

function gamepadLoop() {
  const pads = navigator.getGamepads ? [...navigator.getGamepads()].filter(Boolean) : [];
  state.controller = pads[0] ?? null;
  let targetLinear = 0;
  let targetAngular = 0;
  let deadman = false;
  if (state.controller && preferences.input === 'controller') {
    const pad = state.controller;
    $('controller').textContent = `${pad.id.replace(/\([^)]*\)/g, '').trim()} · ${pad.mapping || 'custom mapping'}`;
    // Standard mapping: left stick steers; RT/LT set throttle; RB is the deadman.
    const steering = responseCurve(deadzone(pad.axes[0] ?? 0));
    const throttle = responseCurve(buttonValue(pad.buttons[7]) - buttonValue(pad.buttons[6]));
    targetLinear = throttle * .8;
    targetAngular = steering * 1.4 * preferences.sensitivity / 100;
    deadman = Boolean(pad.buttons[5]?.pressed);
  } else {
    $('controller').textContent = preferences.input === 'keyboard'
      ? (cameraLocked ? 'WASD camera input locked · hold Shift to drive' : 'Click the forward camera to lock WASD input')
      : 'No controller connected';
  }
  if (preferences.input === 'keyboard' && cameraLocked) {
    const linear = (keys.has('KeyW') ? 1 : 0) - (keys.has('KeyS') ? 1 : 0);
    const angular = (keys.has('KeyD') ? 1 : 0) - (keys.has('KeyA') ? 1 : 0);
    targetLinear = linear * .8;
    targetAngular = angular * 1.4 * preferences.sensitivity / 100;
    deadman = keys.has('ShiftLeft') || keys.has('ShiftRight');
  }
  const now = performance.now();
  const dt = Math.min(.1, (now - smoothedDrive.updatedAt) / 1000);
  smoothedDrive.updatedAt = now;
  const smoothing = 1 - Math.exp(-dt / .12);
  smoothedDrive.linear += (targetLinear - smoothedDrive.linear) * smoothing;
  smoothedDrive.angular += (targetAngular - smoothedDrive.angular) * smoothing;
  if (state.drive.you_control_owner) send({type: 'drive', linear_x: smoothedDrive.linear, angular_z: smoothedDrive.angular, deadman});
  requestAnimationFrame(gamepadLoop);
}

function buttonValue(button) { return button?.value ?? 0; }
function deadzone(value) { return Math.abs(value) < .12 ? 0 : value; }
function responseCurve(value) { return Math.sign(value) * Math.abs(value) ** 1.65; }

$('take-control').onclick = () => send({type: 'take_control'});
$('arm').onclick = () => send({type: state.drive.armed ? 'disarm' : 'arm'});
$('estop').onclick = () => send({type: 'estop'});
$('probe-slider').oninput = event => {
  const now = performance.now();
  if (now - lastProbeSent < 70) return;
  lastProbeSent = now;
  send({type: 'probe_target', depth_mm: Number(event.target.value)});
};
$('sweep-toggle').onclick = () => send({type: 'sweep_enabled', enabled: !state.detector.sweep_enabled});
$('sweep-speed').oninput = event => send({type: 'sweep_speed', deg_s: Number(event.target.value)});

function savePreferences() {
  localStorage.setItem('peaceofmine.operator.preferences', JSON.stringify(preferences));
  $('control-source').value = preferences.input;
  $('steering-sensitivity').value = preferences.sensitivity;
  $('sensitivity-value').textContent = `${preferences.sensitivity}%`;
}

$('settings').onclick = () => $('settings-dialog').showModal();
$('control-source').onchange = event => { preferences.input = event.target.value; savePreferences(); };
$('steering-sensitivity').oninput = event => { preferences.sensitivity = Number(event.target.value); savePreferences(); };
savePreferences();

forward.onclick = () => {
  if (preferences.input !== 'keyboard') return;
  forward.focus();
  forward.requestPointerLock?.();
  cameraLocked = true;
  $('input-lock').textContent = 'WASD INPUT LOCKED · ESC TO RELEASE';
  $('input-lock').classList.add('locked');
};
document.addEventListener('pointerlockchange', () => {
  cameraLocked = document.pointerLockElement === forward;
  if (!cameraLocked) {
    keys.clear();
    $('input-lock').textContent = 'CLICK CAMERA FOR INPUT';
    $('input-lock').classList.remove('locked');
  }
});
window.addEventListener('keydown', event => {
  if (preferences.input !== 'keyboard' || !cameraLocked) return;
  if (['KeyW', 'KeyA', 'KeyS', 'KeyD', 'ShiftLeft', 'ShiftRight'].includes(event.code)) {
    event.preventDefault(); keys.add(event.code);
  }
});
window.addEventListener('keyup', event => keys.delete(event.code));

function resize(canvas) {
  const ratio = window.devicePixelRatio || 1;
  const width = Math.floor(canvas.clientWidth * ratio);
  const height = Math.floor(canvas.clientHeight * ratio);
  if (canvas.width !== width || canvas.height !== height) {
    canvas.width = width;
    canvas.height = height;
  }
  const context = canvas.getContext('2d');
  context.setTransform(ratio, 0, 0, ratio, 0, 0);
  return context;
}

function worldToCamera(x, y) {
  const dx = x - state.robot.x;
  const dy = y - state.robot.y;
  const yaw = state.robot.yaw;
  return {
    forward: Math.sin(yaw) * dx + Math.cos(yaw) * dy,
    side: Math.cos(yaw) * dx - Math.sin(yaw) * dy,
  };
}

function project(point, width, height) {
  if (point.forward < .35) return null;
  const horizon = height * .31;
  const scale = height * .72;
  return {x: width / 2 + point.side / point.forward * scale, y: horizon + scale / point.forward};
}

function drawForwardView() {
  const context = resize(forward);
  const width = forward.clientWidth;
  const height = forward.clientHeight;
  const horizon = height * .31;
  context.clearRect(0, 0, width, height);
  context.fillStyle = '#4f8995'; context.fillRect(0, 0, width, horizon);
  context.fillStyle = '#31492c'; context.fillRect(0, horizon, width, height - horizon);

  context.strokeStyle = '#7b8e64'; context.lineWidth = 1;
  for (let distance = 1; distance < 26; distance += 1) {
    const left = project(worldToCamera(-5, state.robot.y + distance), width, height);
    const right = project(worldToCamera(5, state.robot.y + distance), width, height);
    if (left && right) { context.beginPath(); context.moveTo(left.x, left.y); context.lineTo(right.x, right.y); context.stroke(); }
  }

  context.strokeStyle = '#e2d29a'; context.lineWidth = 3;
  for (const laneX of [-2, 2]) {
    const start = project(worldToCamera(laneX, state.robot.y + .4), width, height);
    const end = project(worldToCamera(laneX, state.robot.y + 28), width, height);
    if (start && end) { context.beginPath(); context.moveTo(start.x, start.y); context.lineTo(end.x, end.y); context.stroke(); }
  }

  // The two simulated targets are deliberately not drawn until their detector event exists.
  state.events.forEach(event => {
    const point = project(worldToCamera(event.x, event.y), width, height);
    if (!point) return;
    context.fillStyle = '#d75b4e'; context.beginPath(); context.arc(point.x, point.y - 8, 7, 0, Math.PI * 2); context.fill();
    context.fillStyle = '#fff4e6'; context.font = '700 11px system-ui'; context.fillText('DETECTION', point.x + 10, point.y - 6);
  });

  context.fillStyle = '#18282a'; context.beginPath(); context.moveTo(width * .32, height); context.lineTo(width * .68, height); context.lineTo(width * .59, height * .84); context.lineTo(width * .41, height * .84); context.closePath(); context.fill();
  requestAnimationFrame(drawForwardView);
}

function drawMap() {
  const context = resize(map);
  const width = map.clientWidth;
  const height = map.clientHeight;
  const scale = Math.min(width / 12, height / 24);
  const toScreen = (x, y) => ({x: width / 2 + x * scale, y: height - (y + 2) * scale});
  context.clearRect(0, 0, width, height);
  context.fillStyle = '#101f20'; context.fillRect(0, 0, width, height);
  context.strokeStyle = '#244244'; context.lineWidth = 1;
  for (let x = -6; x <= 6; x++) { const a = toScreen(x, -2), b = toScreen(x, 22); context.beginPath(); context.moveTo(a.x, a.y); context.lineTo(b.x, b.y); context.stroke(); }
  for (let y = -2; y <= 22; y++) { const a = toScreen(-6, y), b = toScreen(6, y); context.beginPath(); context.moveTo(a.x, a.y); context.lineTo(b.x, b.y); context.stroke(); }
  context.fillStyle = '#1e4d43'; const laneA = toScreen(-2, -2), laneB = toScreen(2, 22); context.fillRect(laneA.x, laneB.y, laneB.x - laneA.x, laneA.y - laneB.y);
  context.strokeStyle = '#d4ce8a'; context.lineWidth = 2; context.strokeRect(laneA.x, laneB.y, laneB.x - laneA.x, laneA.y - laneB.y);
  state.events.forEach(event => { const p = toScreen(event.x, event.y); context.fillStyle = '#df6251'; context.beginPath(); context.arc(p.x, p.y, 6, 0, Math.PI * 2); context.fill(); });
  const p = toScreen(state.robot.x, state.robot.y); context.save(); context.translate(p.x, p.y); context.rotate(state.robot.yaw); context.fillStyle = '#dff3df'; context.beginPath(); context.moveTo(0, -10); context.lineTo(7, 8); context.lineTo(-7, 8); context.closePath(); context.fill(); context.restore();
  requestAnimationFrame(drawMap);
}

function drawPressureHistory() {
  const canvas = $('pressure-history');
  const context = resize(canvas);
  const width = canvas.clientWidth, height = canvas.clientHeight;
  const samples = state.probe.pressure_history || [];
  context.clearRect(0, 0, width, height);
  context.strokeStyle = '#26434a'; context.lineWidth = 1;
  for (let n = 1; n < 4; n++) { const y = n * height / 4; context.beginPath(); context.moveTo(0, y); context.lineTo(width, y); context.stroke(); }
  if (samples.length > 1) {
    const newest = samples.at(-1).t, floor = newest - 30;
    const max = 1;
    context.strokeStyle = '#57d3a8'; context.lineWidth = 2; context.beginPath();
    samples.forEach((sample, index) => {
      const x = Math.max(0, (sample.t - floor) / 30) * width;
      const y = height - sample.ratio / max * (height - 5) - 2;
      index ? context.lineTo(x, y) : context.moveTo(x, y);
    });
    context.stroke();
  }
  requestAnimationFrame(drawPressureHistory);
}

function drawRadar() {
  const canvas = $('radar-view');
  const context = resize(canvas);
  const width = canvas.clientWidth, height = canvas.clientHeight;
  const padding = 10;
  const cx = width / 2, cy = height - padding, radius = Math.max(1, Math.min(width * .42, height - 2 * padding));
  context.clearRect(0, 0, width, height);
  context.strokeStyle = '#265148'; context.lineWidth = 1;
  for (const factor of [.33, .66, 1]) { context.beginPath(); context.arc(cx, cy, radius * factor, Math.PI * 1.25, Math.PI * 1.75); context.stroke(); }
  context.beginPath(); context.moveTo(cx, cy); context.lineTo(cx - radius * .7, cy - radius * .7); context.moveTo(cx, cy); context.lineTo(cx + radius * .7, cy - radius * .7); context.stroke();
  // A continuous heat band: the latest measured signal for every servo angle
  // is interpolated across the 90° scan, instead of showing discrete samples.
  const bins = Array.from({length: 91}, () => []);
  for (const sample of state.detector.history || []) {
    const bin = Math.round(Math.max(-45, Math.min(45, sample.angle_deg))) + 45;
    bins[bin].push(sample.ratio);
  }
  const values = bins.map(samples => samples.length ? samples.reduce((sum, value) => sum + value, 0) / samples.length : null);
  let previous = 0;
  for (let index = 0; index < values.length; index++) {
    if (values[index] !== null) previous = values[index];
    else values[index] = previous;
  }
  let next = 0;
  for (let index = values.length - 1; index >= 0; index--) {
    if (bins[index].length) next = values[index];
    else if (!index || values[index] === 0) values[index] = next;
  }
  for (let index = 0; index < 90; index++) {
    const ratio = (values[index] + values[index + 1]) / 2;
    const start = -Math.PI * .75 + index / 90 * Math.PI / 2;
    context.strokeStyle = `hsl(${(1 - ratio) * 120} 78% 55%)`;
    context.lineWidth = 8;
    context.beginPath(); context.arc(cx, cy, radius * .84, start, start + Math.PI / 90 + .012); context.stroke();
  }
  // The beam angle is telemetry from the fixture, not a locally invented animation.
  const angle = -Math.PI / 2 + state.detector.fixture_angle_deg * Math.PI / 180;
  context.strokeStyle = '#4de0a8'; context.lineWidth = 2; context.beginPath(); context.moveTo(cx, cy); context.lineTo(cx + Math.cos(angle) * radius, cy + Math.sin(angle) * radius); context.stroke();
  if (state.detector.detected) { context.fillStyle = '#e5b54a'; context.beginPath(); context.arc(cx, cy - radius * .58, 4 + state.detector.signal_ratio * 5, 0, Math.PI * 2); context.fill(); }
  requestAnimationFrame(drawRadar);
}

window.addEventListener('gamepadconnected', () => $('camera-notice').textContent = 'Controller found. Arm only after the lane is clear.');
window.addEventListener('resize', () => { /* Canvas resize happens on the next animation frame. */ });
setConnection(false, 'Connecting');
connect();
gamepadLoop();
drawForwardView();
drawMap();
drawPressureHistory();
drawRadar();
