// The search lane runs along world +X, matching the ROS convention where a
// rover at yaw 0 faces +X. Cross-lane offsets are world Y.
const LANE_START_M = -2;
const LANE_END_M = 22;
const LANE_HALF_WIDTH_M = 2;
const CAMERA_RANGE_M = 26;

const DRIVE_PERIOD_MS = 50;
const LATENCY_PERIOD_MS = 1000;
const LATENCY_HISTORY_LENGTH = 30;
const TELEMETRY_TIMEOUT_MS = 3000;
const MAX_LINEAR_MPS = 0.8;
const MAX_ANGULAR_RAD_S = 1.4;
const state = {
  connected: false,
  drive: {
    armed: false,
    deadman: false,
    client_count: 0,
    you_control_owner: false,
    control_owner_present: false,
  },
  robot: {x: 0, y: 0, yaw: 0, speed_mps: 0},
  detector: {
    signal_ratio: 0,
    threshold_ratio: 0.65,
    detected: false,
    fixture_angle_deg: 0,
    sweep_enabled: false,
    sweep_speed_deg_s: 60,
    sweep_speed_min: 5,
    sweep_speed_max: 180,
    beam_half_angle_deg: 45,
    beam: [],
  },
  probe: {
    depth_mm: 0,
    target_mm: 0,
    max_depth_mm: 110,
    pressure_ratio: 0,
    pressure_history: [],
    fault: false,
  },
  detections: [],
  cameras: {},
};

const $ = id => document.getElementById(id);
const forward = $('forward-view');
const map = $('map-view');
let preferences;
try { preferences = JSON.parse(localStorage.getItem('peaceofmine.operator.preferences')); } catch {}
preferences = preferences && typeof preferences === 'object' ? preferences : {};
if (!['controller', 'keyboard', 'wheel'].includes(preferences.input)) preferences.input = 'controller';
if (!Number.isFinite(preferences.sensitivity)) preferences.sensitivity = 100;
preferences.sensitivity = Math.max(50, Math.min(150, preferences.sensitivity));
preferences.wheel = Object.assign({steering: 0, throttle: 1, brake: 2, deadman: 0,
  invertSteering: false, invertThrottle: true, invertBrake: true, deadzone: 0.05}, preferences.wheel);
preferences.cameras = Object.assign({forward: 'virtual', auxiliary: 'off'}, preferences.cameras);
let selectedPadIndex = null;
let selectedPadId = null;

const keys = new Set();
const smoothedDrive = {linear: 0, angular: 0, updatedAt: performance.now()};
let socket;
let gamepad = null;
let cameraFocused = false;
let lastDriveSent = 0;
let noticeExpiry = 0;
let latencyProbeId = 0;
let latencyProbeStarted = 0;
let linkLatencyMs = null;
let lastTelemetryAt = 0;
let reconnectTimer = null;
const latencyHistory = [];
const cameraStreams = {forward: null, auxiliary: null};
let networkCameraSignature = '';

function throttle(periodMs, action) {
  let last = 0;
  return value => {
    const now = performance.now();
    if (now - last < periodMs) return;
    last = now;
    action(value);
  };
}

/* ------------------------------------------------------------ Transport --- */

function send(message) {
  if (socket?.readyState === WebSocket.OPEN) socket.send(JSON.stringify(message));
}

function setConnection(connected, label, detail = '') {
  state.connected = connected;
  $('connection').textContent = label;
  $('connection-dot').style.background = connected ? '#37b98e' : '#d36458';
  $('connection-overlay').classList.toggle('hidden', connected);
  $('connection-title').textContent = connected ? 'Connected' : label;
  $('connection-detail').textContent = detail || 'The operator gateway is unavailable. Start the PeaceOfMine stack on the Raspberry Pi; this page will reconnect automatically.';
  $('settings-connection-state').textContent = connected ? 'Connected to Raspberry Pi telemetry' : label;
  updateHud();
}

function drawLatencyGraph() {
  const canvas = $('latency-graph');
  const context = canvas.getContext('2d');
  const width = canvas.width;
  const height = canvas.height;
  context.clearRect(0, 0, width, height);
  if (latencyHistory.length < 2) return;

  const maximum = Math.max(100, ...latencyHistory);
  const sampleSpan = Math.max(1, latencyHistory.length - 1);
  context.beginPath();
  latencyHistory.forEach((latency, index) => {
    const x = index / sampleSpan * width;
    const y = height - Math.min(latency / maximum, 1) * (height - 3) - 1;
    if (index === 0) context.moveTo(x, y);
    else context.lineTo(x, y);
  });
  context.strokeStyle = '#55c9a1';
  context.lineWidth = 2;
  context.stroke();
}

function measureLatency() {
  if (socket?.readyState !== WebSocket.OPEN || latencyProbeStarted) return;
  latencyProbeId += 1;
  latencyProbeStarted = performance.now();
  send({type: 'latency_ping', probe_id: latencyProbeId});
}

function recordLatency(probeId) {
  if (probeId !== latencyProbeId || !latencyProbeStarted) return;
  const latency = Math.round(performance.now() - latencyProbeStarted);
  linkLatencyMs = latency;
  latencyProbeStarted = 0;
  latencyHistory.push(latency);
  if (latencyHistory.length > LATENCY_HISTORY_LENGTH) latencyHistory.shift();
  $('latency-value').textContent = `${latency} ms`;
  drawLatencyGraph();
}

function connect() {
  clearTimeout(reconnectTimer);
  if (socket && socket.readyState < WebSocket.CLOSING) socket.close();
  const scheme = location.protocol === 'https:' ? 'wss' : 'ws';
  const connectingSocket = new WebSocket(`${scheme}://${location.host}/ws`);
  socket = connectingSocket;
  lastTelemetryAt = 0;
  setConnection(false, 'Connecting to operator stack', 'The dashboard is loaded. Waiting for telemetry from the Raspberry Pi gateway.');

  connectingSocket.onopen = () => {
    $('connection').textContent = 'Gateway linked · waiting for telemetry';
    measureLatency();
  };

  connectingSocket.onclose = () => {
    if (socket !== connectingSocket) return;
    latencyProbeStarted = 0;
    linkLatencyMs = null;
    latencyHistory.length = 0;
    $('latency-value').textContent = '-- ms';
    drawLatencyGraph();
    state.drive.armed = false;
    state.drive.deadman = false;
    state.drive.you_control_owner = false;
    stopLocalInput();
    setConnection(false, 'Raspberry Pi disconnected', 'The gateway stopped responding or the network link was lost. The rover command is stopped. Retrying automatically…');
    reconnectTimer = setTimeout(connect, 1200);
  };

  connectingSocket.onerror = () => connectingSocket.close();

  connectingSocket.onmessage = event => {
    const message = JSON.parse(event.data);
    if (message.type === 'latency_pong') {
      recordLatency(message.probe_id);
      return;
    }
    if (message.type === 'error') {
      showNotice(message.message);
      return;
    }
    if (message.type !== 'state') return;
    lastTelemetryAt = performance.now();
    if (!state.connected) setConnection(true, 'Raspberry Pi connected');
    Object.assign(state, message);
    updateNetworkCameraOptions();
    updateCameraLatency();
    updateHud();
  };
}

setInterval(measureLatency, LATENCY_PERIOD_MS);
setInterval(() => {
  if (socket?.readyState === WebSocket.OPEN && lastTelemetryAt &&
      performance.now() - lastTelemetryAt > TELEMETRY_TIMEOUT_MS) {
    setConnection(false, 'Telemetry stalled', 'The gateway connection is open, but rover telemetry stopped. Controls are disabled while reconnecting.');
    socket.close();
  }
}, 500);

$('retry-connection').onclick = connect;
$('settings-retry-connection').onclick = connect;

// The camera notice is rewritten on every telemetry tick, so rejected
// commands need their own element to stay readable.
function showNotice(text) {
  const time = new Date().toLocaleTimeString([], {hour: '2-digit', minute: '2-digit', second: '2-digit'});
  $('notice').textContent = `${time} · ${text}`;
  $('notice').classList.remove('hidden');
  noticeExpiry = performance.now() + 6000;
}

/* ------------------------------------------------------------------ HUD --- */

function setRange(input, min, max, step) {
  if (Number(input.min) !== min) input.min = min;
  if (Number(input.max) !== max) input.max = max;
  if (Number(input.step) !== step) input.step = step;
}

function updateHud() {
  const {drive, robot, detector, probe} = state;
  const owner = state.connected && drive.you_control_owner;

  $('client-count').textContent = `${drive.client_count} ${drive.client_count === 1 ? 'viewer' : 'viewers'}`;
  $('control-owner').textContent = owner ? 'YOU CONTROL' : (drive.control_owner_present ? 'SPECTATING' : 'UNCLAIMED');
  $('control-owner').className = `badge ${owner ? 'safe' : 'neutral'}`;
  $('take-control').textContent = drive.control_owner_present ? 'Steal control' : 'Take control';

  $('take-control').disabled = !state.connected || owner;
  $('arm').disabled = !owner || drive.armed;
  $('estop').disabled = !owner;
  $('probe-slider').disabled = !owner;
  $('sweep-toggle').disabled = !owner;
  $('sweep-speed').disabled = !owner;

  $('speed').textContent = `${Math.abs(robot.speed_mps).toFixed(2)} m/s`;

  const signalPercent = detector.signal_ratio * 100;
  $('signal').textContent = Math.round(signalPercent);
  $('meter-fill').style.width = `${signalPercent}%`;
  $('threshold').style.left = `${detector.threshold_ratio * 100}%`;
  $('detector-detail').textContent =
    `Threshold ${Math.round(detector.threshold_ratio * 100)}% · fixture ${detector.fixture_angle_deg.toFixed(0)}°`;
  $('detector-status').textContent = detector.detected ? 'DETECTION' : 'CLEAR';
  $('detector-status').className = `badge ${detector.detected ? 'active' : 'safe'}`;

  $('sweep-toggle').textContent = detector.sweep_enabled ? 'Stop sweep' : 'Start sweep';
  setRange($('sweep-speed'), detector.sweep_speed_min, detector.sweep_speed_max, 5);
  if (document.activeElement !== $('sweep-speed')) $('sweep-speed').value = detector.sweep_speed_deg_s;
  $('sweep-speed-value').textContent = `${Math.round(detector.sweep_speed_deg_s)}°/s`;

  $('probe-depth').textContent = `${Math.round(probe.depth_mm)} mm`;
  $('probe-target').textContent = `${Math.round(probe.target_mm)} mm`;
  $('probe-pressure').textContent = `${Math.round(probe.pressure_ratio * 100)}%`;
  setRange($('probe-slider'), 0, probe.max_depth_mm, 1);
  if (document.activeElement !== $('probe-slider')) $('probe-slider').value = probe.target_mm;
  $('probe-arm').style.height = `${probe.depth_mm / probe.max_depth_mm * 2.4}rem`;
  $('probe-status').textContent = probe.fault ? 'FAULT' : probe.depth_mm > 2 ? 'DEPLOYED' : 'STOWED';
  $('probe-status').className = `badge ${probe.fault ? 'active' : probe.depth_mm > 2 ? 'safe' : 'neutral'}`;

  $('arm-state').textContent = !owner ? 'SPECTATOR' : (drive.armed ? (drive.deadman ? 'DRIVING' : 'ARMED') : 'DISARMED');
  $('arm-state').className = `badge ${drive.armed ? 'active' : 'neutral'}`;
  $('camera-notice').textContent = !owner
    ? 'Spectator mode · take control to arm or actuate.'
    : drive.armed
      ? (drive.deadman ? 'Deadman held · command stream active.' : `Armed · ${deadmanHint()}`)
      : 'Disarmed · arm only after the lane is clear.';

  const heading = ((robot.yaw * 180 / Math.PI % 360) + 360) % 360;
  $('position').textContent = `X ${robot.x.toFixed(1)} · Y ${robot.y.toFixed(1)} · heading ${heading.toFixed(0)}°`;

  updateInputHint();
}

function deadmanHint() {
  return preferences.input === 'keyboard' ? 'Hold Shift to drive.' : preferences.input === 'wheel' ? `Hold wheel button ${preferences.wheel.deadman} to drive.` : 'Hold the right bumper to drive.';
}

function updateInputHint() {
  $('drive-help').textContent = `${deadmanHint()} Releasing it stops the rover.`;
  const keyboard = preferences.input === 'keyboard';
  $('input-lock').classList.toggle('hidden', !keyboard);
  $('input-lock').classList.toggle('locked', keyboard && cameraFocused);
  $('input-lock').textContent = cameraFocused ? 'WASD INPUT ACTIVE' : 'CLICK CAMERA FOR INPUT';

  if (gamepad && !keyboard) {
    $('controller').textContent = `${gamepad.id.replace(/\([^)]*\)/g, '').trim()} · ${gamepad.mapping || 'custom mapping'}`;
  } else if (keyboard) {
    $('controller').textContent = cameraFocused
      ? 'WASD active · hold Shift to drive'
      : 'Click the forward camera to use WASD';
  } else {
    $('controller').textContent = 'No controller connected';
  }
}

/* ---------------------------------------------------------------- Input --- */

function buttonValue(button) {
  return button?.value ?? 0;
}

function deadzone(value) {
  return Math.abs(value) < 0.12 ? 0 : value;
}

function responseCurve(value) {
  return Math.sign(value) * Math.abs(value) ** 1.65;
}

function readGamepad() {
  const pad = gamepad;
  // Browser standard mapping: left stick steers, LT/RT set throttle, RB is the deadman.
  const steering = responseCurve(deadzone(pad.axes[0] ?? 0));
  const throttle = responseCurve(buttonValue(pad.buttons[7]) - buttonValue(pad.buttons[6]));
  return {
    linear: throttle * MAX_LINEAR_MPS,
    angular: steering * MAX_ANGULAR_RAD_S * preferences.sensitivity / 100,
    deadman: Boolean(pad.buttons[5]?.pressed),
  };
}

function readKeyboard() {
  const linear = (keys.has('KeyW') ? 1 : 0) - (keys.has('KeyS') ? 1 : 0);
  const angular = (keys.has('KeyD') ? 1 : 0) - (keys.has('KeyA') ? 1 : 0);
  return {
    linear: linear * MAX_LINEAR_MPS,
    angular: angular * MAX_ANGULAR_RAD_S * preferences.sensitivity / 100,
    deadman: keys.has('ShiftLeft') || keys.has('ShiftRight'),
  };
}

function inputLoop() {
  const pads = visiblePads();
  gamepad = pads.find(pad => pad.index === selectedPadIndex && pad.id === selectedPadId) ?? null;

  let target = {linear: 0, angular: 0, deadman: false};
  if (preferences.input === 'keyboard' && cameraFocused) {
    target = readKeyboard();
  } else if (preferences.input === 'controller' && gamepad?.mapping === 'standard') {
    target = readGamepad();
  } else if (preferences.input === 'wheel' && gamepad) {
    target = readWheel(gamepad, preferences.wheel);
  }
  renderInputs(pads, target);
  if ($('settings-dialog').open || document.hidden || !document.hasFocus()) target = {linear: 0, angular: 0, deadman: false};
  if (!target.deadman) {
    target.linear = target.angular = 0;
    smoothedDrive.linear = smoothedDrive.angular = 0;
  }

  const now = performance.now();
  const dt = Math.min(0.1, (now - smoothedDrive.updatedAt) / 1000);
  smoothedDrive.updatedAt = now;
  const smoothing = 1 - Math.exp(-dt / 0.12);
  smoothedDrive.linear += (target.linear - smoothedDrive.linear) * smoothing;
  smoothedDrive.angular += (target.angular - smoothedDrive.angular) * smoothing;

  if (state.drive.you_control_owner && now - lastDriveSent >= DRIVE_PERIOD_MS) {
    lastDriveSent = now;
    send({
      type: 'drive',
      linear_x: smoothedDrive.linear,
      angular_z: smoothedDrive.angular,
      deadman: target.deadman,
    });
  }

  if (noticeExpiry && now > noticeExpiry) {
    noticeExpiry = 0;
    $('notice').classList.add('hidden');
  }

  requestAnimationFrame(inputLoop);
}

function stopLocalInput() {
  keys.clear();
  smoothedDrive.linear = smoothedDrive.angular = 0;
}

$('take-control').onclick = () => send({type: 'take_control'});
$('arm').onclick = () => send({type: 'arm'});
$('estop').onclick = () => send({type: 'estop'});

$('probe-slider').oninput = throttle(70, event => send({type: 'probe_target', depth_mm: Number(event.target.value)}));
$('sweep-speed').oninput = throttle(70, event => send({type: 'sweep_speed', deg_s: Number(event.target.value)}));
$('sweep-toggle').onclick = () => send({type: 'sweep_enabled', enabled: !state.detector.sweep_enabled});

function savePreferences() {
  try { localStorage.setItem('peaceofmine.operator.preferences', JSON.stringify(preferences)); } catch {}
  $('control-source').value = preferences.input;
  $('steering-sensitivity').value = preferences.sensitivity;
  $('sensitivity-value').textContent = `${preferences.sensitivity}%`;
  updateInputHint();
}

$('settings').onclick = () => {
  stopInput();
  $('settings-dialog').showModal();
};
$('offline-settings').onclick = $('settings').onclick;
$('control-source').onchange = event => {
  stopInput();
  preferences.input = event.target.value;
  savePreferences();
};
$('steering-sensitivity').oninput = event => {
  preferences.sensitivity = Number(event.target.value);
  savePreferences();
};

// The canvas carries tabindex="0", so a click focuses it and WASD is captured
// only while it holds focus.
function activateKeyboardInput() {
  cameraFocused = true;
  updateInputHint();
}

function deactivateKeyboardInput() {
  cameraFocused = false;
  keys.clear();
  updateInputHint();
}

for (const preview of [forward, $('forward-video'), $('forward-network-camera')]) {
  preview.addEventListener('focus', activateKeyboardInput);
  preview.addEventListener('blur', deactivateKeyboardInput);
}

window.addEventListener('keydown', event => {
  if (preferences.input !== 'keyboard' || !(cameraFocused || document.activeElement === $('keyboard-test'))) return;
  if (['KeyW', 'KeyA', 'KeyS', 'KeyD', 'ShiftLeft', 'ShiftRight'].includes(event.code)) {
    event.preventDefault();
    keys.add(event.code);
  }
});

window.addEventListener('keyup', event => keys.delete(event.code));

window.addEventListener('gamepadconnected', () => {
  $('camera-notice').textContent = 'Controller found. Arm only after the lane is clear.';
});

/* --------------------------------------------------------------- Canvas --- */

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

// World to rover body frame: forward is along the rover's heading, side is to
// its right, which is the direction screen x grows in project().
function worldToCamera(x, y) {
  const dx = x - state.robot.x;
  const dy = y - state.robot.y;
  const yaw = state.robot.yaw;
  return {
    forward: Math.cos(yaw) * dx + Math.sin(yaw) * dy,
    side: Math.sin(yaw) * dx - Math.cos(yaw) * dy,
  };
}

function project(point, width, height) {
  if (point.forward < 0.35) return null;
  const horizon = height * 0.31;
  const scale = height * 0.72;
  return {
    x: width / 2 + point.side / point.forward * scale,
    y: horizon + scale / point.forward,
  };
}

function drawSegment(context, from, to) {
  if (!from || !to) return;
  context.beginPath();
  context.moveTo(from.x, from.y);
  context.lineTo(to.x, to.y);
  context.stroke();
}

function drawForwardView() {
  const context = resize(forward);
  const width = forward.clientWidth;
  const height = forward.clientHeight;
  const horizon = height * 0.31;
  const {x: roverX} = state.robot;

  context.clearRect(0, 0, width, height);
  context.fillStyle = '#4f8995';
  context.fillRect(0, 0, width, horizon);
  context.fillStyle = '#31492c';
  context.fillRect(0, horizon, width, height - horizon);

  // Cross-lane rungs every metre ahead of the rover along +X.
  context.strokeStyle = '#7b8e64';
  context.lineWidth = 1;
  for (let distance = 1; distance < CAMERA_RANGE_M; distance += 1) {
    const left = project(worldToCamera(roverX + distance, 5), width, height);
    const right = project(worldToCamera(roverX + distance, -5), width, height);
    drawSegment(context, left, right);
  }

  // Lane edges at constant Y, running along +X.
  context.strokeStyle = '#e2d29a';
  context.lineWidth = 3;
  for (const laneY of [-LANE_HALF_WIDTH_M, LANE_HALF_WIDTH_M]) {
    const start = project(worldToCamera(roverX + 0.4, laneY), width, height);
    const end = project(worldToCamera(roverX + 28, laneY), width, height);
    drawSegment(context, start, end);
  }

  for (const detection of state.detections) {
    const point = project(worldToCamera(detection.x, detection.y), width, height);
    if (!point) continue;
    context.strokeStyle = '#d75b4e';
    context.lineWidth = 2;
    context.beginPath();
    context.arc(point.x, point.y - 8, 7, 0, Math.PI * 2);
    context.stroke();
    context.fillStyle = '#fff4e6';
    context.font = '700 11px system-ui';
    context.fillText('MARKED', point.x + 10, point.y - 6);
  }

  // Rover nose in the near field.
  context.fillStyle = '#18282a';
  context.beginPath();
  context.moveTo(width * 0.32, height);
  context.lineTo(width * 0.68, height);
  context.lineTo(width * 0.59, height * 0.84);
  context.lineTo(width * 0.41, height * 0.84);
  context.closePath();
  context.fill();

  requestAnimationFrame(drawForwardView);
}

function drawMap() {
  const context = resize(map);
  const width = map.clientWidth;
  const height = map.clientHeight;
  const crossHalf = 6;
  const scale = Math.min(width / (LANE_END_M - LANE_START_M), height / (2 * crossHalf));
  // +X runs left to right, +Y runs up the screen.
  const toScreen = (x, y) => ({x: (x - LANE_START_M) * scale, y: height / 2 - y * scale});

  context.clearRect(0, 0, width, height);
  context.fillStyle = '#101f20';
  context.fillRect(0, 0, width, height);

  context.strokeStyle = '#244244';
  context.lineWidth = 1;
  for (let x = LANE_START_M; x <= LANE_END_M; x += 1) {
    drawSegment(context, toScreen(x, -crossHalf), toScreen(x, crossHalf));
  }
  for (let y = -crossHalf; y <= crossHalf; y += 1) {
    drawSegment(context, toScreen(LANE_START_M, y), toScreen(LANE_END_M, y));
  }

  const laneTopLeft = toScreen(LANE_START_M, LANE_HALF_WIDTH_M);
  const laneBottomRight = toScreen(LANE_END_M, -LANE_HALF_WIDTH_M);
  const laneWidth = laneBottomRight.x - laneTopLeft.x;
  const laneHeight = laneBottomRight.y - laneTopLeft.y;
  context.fillStyle = '#1e4d43';
  context.fillRect(laneTopLeft.x, laneTopLeft.y, laneWidth, laneHeight);
  context.strokeStyle = '#d4ce8a';
  context.lineWidth = 2;
  context.strokeRect(laneTopLeft.x, laneTopLeft.y, laneWidth, laneHeight);

  // Detections mark where the rover stood when the signal crossed threshold.
  for (const detection of state.detections) {
    const point = toScreen(detection.x, detection.y);
    context.strokeStyle = '#df6251';
    context.lineWidth = 2;
    context.beginPath();
    context.arc(point.x, point.y, 5, 0, Math.PI * 2);
    context.stroke();
  }

  const rover = toScreen(state.robot.x, state.robot.y);
  context.save();
  context.translate(rover.x, rover.y);
  // Screen y is flipped relative to world y, so a counter-clockwise world yaw
  // is a clockwise canvas rotation.
  context.rotate(-state.robot.yaw);
  context.fillStyle = '#dff3df';
  context.beginPath();
  context.moveTo(10, 0);
  context.lineTo(-8, 7);
  context.lineTo(-8, -7);
  context.closePath();
  context.fill();
  context.restore();

  requestAnimationFrame(drawMap);
}

function drawPressureHistory() {
  const canvas = $('pressure-history');
  const context = resize(canvas);
  const width = canvas.clientWidth;
  const height = canvas.clientHeight;
  const samples = state.probe.pressure_history;

  context.clearRect(0, 0, width, height);
  context.strokeStyle = '#26434a';
  context.lineWidth = 1;
  for (let line = 1; line < 4; line++) {
    const y = line * height / 4;
    drawSegment(context, {x: 0, y}, {x: width, y});
  }

  if (samples.length > 1) {
    context.strokeStyle = '#57d3a8';
    context.lineWidth = 2;
    context.beginPath();
    samples.forEach((ratio, index) => {
      const x = index / (samples.length - 1) * width;
      const y = height - ratio * (height - 5) - 2;
      if (index) context.lineTo(x, y);
      else context.moveTo(x, y);
    });
    context.stroke();
  }

  requestAnimationFrame(drawPressureHistory);
}

// Fixture angles are counter-clockwise in the world, which is counter-clockwise
// on screen too once the canvas y flip is accounted for.
function beamScreenAngle(degrees) {
  return -Math.PI / 2 - degrees * Math.PI / 180;
}

function drawRadar() {
  const canvas = $('radar-view');
  const context = resize(canvas);
  const width = canvas.clientWidth;
  const height = canvas.clientHeight;
  const padding = 10;
  const cx = width / 2;
  const cy = height - padding;
  const radius = Math.max(1, Math.min(width * 0.42, height - 2 * padding));
  const {beam, beam_half_angle_deg: half} = state.detector;

  context.clearRect(0, 0, width, height);
  context.strokeStyle = '#265148';
  context.lineWidth = 1;
  for (const factor of [0.33, 0.66, 1]) {
    context.beginPath();
    context.arc(cx, cy, radius * factor, beamScreenAngle(half), beamScreenAngle(-half));
    context.stroke();
  }
  for (const edge of [half, -half]) {
    const angle = beamScreenAngle(edge);
    drawSegment(context, {x: cx, y: cy}, {x: cx + Math.cos(angle) * radius, y: cy + Math.sin(angle) * radius});
  }

  // One arc per degree of the gateway's beam trace: green is quiet, red is hot.
  context.lineWidth = 8;
  for (let index = 0; index < beam.length - 1; index++) {
    const ratio = (beam[index] + beam[index + 1]) / 2;
    const from = beamScreenAngle(-half + index + 1);
    const to = beamScreenAngle(-half + index);
    context.strokeStyle = `hsl(${(1 - ratio) * 120} 78% 55%)`;
    context.beginPath();
    context.arc(cx, cy, radius * 0.84, from, to + 0.012);
    context.stroke();
  }

  const angle = beamScreenAngle(state.detector.fixture_angle_deg);
  context.strokeStyle = '#4de0a8';
  context.lineWidth = 2;
  drawSegment(context, {x: cx, y: cy}, {x: cx + Math.cos(angle) * radius, y: cy + Math.sin(angle) * radius});

  if (state.detector.detected) {
    context.fillStyle = '#e5b54a';
    context.beginPath();
    context.arc(cx, cy - radius * 0.58, 4 + state.detector.signal_ratio * 5, 0, Math.PI * 2);
    context.fill();
  }

  requestAnimationFrame(drawRadar);
}

// Browser input acquisition and diagnostics. No device output reports are sent.
function visiblePads() {
  try { return navigator.getGamepads ? Array.from(navigator.getGamepads()).filter(Boolean) : []; }
  catch { return []; }
}

function readWheel(pad, mapping) {
  const zero = {linear: 0, angular: 0, deadman: false};
  const indices = ['steering', 'throttle', 'brake'];
  if (!indices.every(key => Number.isInteger(mapping[key]) && mapping[key] >= 0 &&
      Number.isFinite(pad.axes[mapping[key]])) ||
      new Set(indices.map(key => mapping[key])).size !== 3 ||
      !Number.isInteger(mapping.deadman) || mapping.deadman < 0 || !pad.buttons[mapping.deadman] ||
      !Number.isFinite(mapping.deadzone) || mapping.deadzone < 0 || mapping.deadzone >= 1) return zero;
  const pedal = (key, inverted) => Math.max(0, Math.min(1, (pad.axes[mapping[key]] * (inverted ? -1 : 1) + 1) / 2));
  const raw = pad.axes[mapping.steering] * (mapping.invertSteering ? -1 : 1);
  const steering = Math.sign(raw) * Math.max(0, Math.min(1, (Math.abs(raw) - mapping.deadzone) / (1 - mapping.deadzone)));
  return {linear: (pedal('throttle', mapping.invertThrottle) - pedal('brake', mapping.invertBrake)) * MAX_LINEAR_MPS,
    angular: steering * MAX_ANGULAR_RAD_S * preferences.sensitivity / 100,
    deadman: Boolean(pad.buttons[mapping.deadman].pressed)};
}

function stopInput() {
  stopLocalInput();
  send({type: 'drive', linear_x: 0, angular_z: 0, deadman: false});
  if (state.drive.you_control_owner) send({type: 'estop'});
}

function stopCameraStream(slot) {
  cameraStreams[slot]?.getTracks().forEach(track => track.stop());
  cameraStreams[slot] = null;
  const video = slot === 'forward' ? $('forward-video') : $('aux-video');
  video.srcObject = null;
  const networkImage = slot === 'forward' ? $('forward-network-camera') : $('aux-network-camera');
  networkImage.removeAttribute('src');
}

async function startCamera(slot, source) {
  stopCameraStream(slot);
  if (source === 'virtual' || source === 'off') {
    updateCameraVisibility();
    return;
  }
  if (source.startsWith('raspberry:')) {
    const camera = source.split(':')[1];
    const networkImage = slot === 'forward' ? $('forward-network-camera') : $('aux-network-camera');
    networkImage.src = `/camera/${camera}?t=${Date.now()}`;
    networkImage.onerror = () => {
      $('camera-source-status').textContent = 'The Raspberry Pi camera stream is unavailable. Check the ROS launch log.';
    };
    networkImage.onload = () => {
      $('camera-source-status').textContent = 'Raspberry Pi camera stream connected.';
    };
    updateCameraVisibility();
    return;
  }
  try {
    const stream = await navigator.mediaDevices.getUserMedia({
      audio: false,
      video: {deviceId: {exact: source}},
    });
    cameraStreams[slot] = stream;
    const video = slot === 'forward' ? $('forward-video') : $('aux-video');
    video.srcObject = stream;
    await video.play();
    $('camera-source-status').textContent = 'Camera previews are live on this browser.';
  } catch (error) {
    $('camera-source-status').textContent = `Could not open camera: ${error.message}`;
  }
  updateCameraVisibility();
}

function updateCameraVisibility() {
  const forwardIsNetwork = preferences.cameras.forward.startsWith('raspberry:');
  const forwardIsVirtual = preferences.cameras.forward === 'virtual' ||
    (!forwardIsNetwork && !cameraStreams.forward);
  forward.classList.toggle('hidden', !forwardIsVirtual);
  $('forward-video').classList.toggle('hidden', forwardIsVirtual || forwardIsNetwork);
  $('forward-network-camera').classList.toggle('hidden', !forwardIsNetwork);
  $('forward-camera-label').textContent = forwardIsVirtual ? 'VIRTUAL FORWARD CAMERA' : forwardIsNetwork ? 'RASPBERRY PI CAMERA' : 'LOCAL CAMERA';
  $('forward-camera-latency').classList.toggle('hidden', !forwardIsNetwork);
  const auxiliaryIsNetwork = preferences.cameras.auxiliary.startsWith('raspberry:');
  const auxiliaryIsLocal = preferences.cameras.auxiliary !== 'off' && Boolean(cameraStreams.auxiliary);
  $('aux-video').classList.toggle('hidden', !auxiliaryIsLocal || auxiliaryIsNetwork);
  $('aux-network-camera').classList.toggle('hidden', !auxiliaryIsNetwork);
  $('aux-camera-label').classList.toggle('hidden', !auxiliaryIsNetwork && !auxiliaryIsLocal);
  $('aux-camera-latency').classList.toggle('hidden', !auxiliaryIsNetwork);
  updateCameraLatency();
}

function updateCameraLatency() {
  const render = (element, camera) => {
    if (!camera) {
      element.textContent = `NO FRAMES · LINK ${linkLatencyMs ?? '--'} ms`;
      element.classList.add('stale');
      return;
    }
    const age = Math.max(0, Math.round(camera.frame_age_ms));
    const fps = Number(camera.fps || 0).toFixed(1);
    element.textContent = `${fps} FPS · ROS ${age} ms · LINK ${linkLatencyMs ?? '--'} ms`;
    element.classList.toggle('stale', age > 500);
  };
  const selected = source => source.startsWith('raspberry:') ? state.cameras?.[source.slice(10)] : null;
  render($('forward-camera-latency'), selected(preferences.cameras.forward));
  render($('aux-camera-latency'), selected(preferences.cameras.auxiliary));
}

function updateNetworkCameraOptions() {
  const networkCameras = Object.entries(state.cameras || {});
  const signature = networkCameras.map(([id, camera]) => `${id}:${camera.topic}`).join('|');
  if (signature === networkCameraSignature) return;
  networkCameraSignature = signature;

  const selections = [
    [$('forward-camera-source'), 'virtual', 'Virtual forward camera'],
    [$('aux-camera-source'), 'off', 'Off'],
  ];
  for (const [select, defaultValue, defaultLabel] of selections) {
    const current = select.value || (select === $('forward-camera-source') ? preferences.cameras.forward : preferences.cameras.auxiliary);
    select.replaceChildren(new Option(defaultLabel, defaultValue));
    Object.entries(state.cameras || {}).forEach(([id, camera]) =>
      select.add(new Option(`Raspberry Pi · ${camera.label}`, `raspberry:${id}`)));
    const firstNetworkSource = networkCameras.length ? `raspberry:${networkCameras[0][0]}` : defaultValue;
    const shouldAutoSelect = select === $('forward-camera-source')
      && preferences.cameras.forward === 'virtual'
      && networkCameras.length > 0;
    select.value = shouldAutoSelect ? firstNetworkSource
      : [...select.options].some(option => option.value === current) ? current : defaultValue;
  }
  const forward = $('forward-camera-source').value;
  const auxiliary = $('aux-camera-source').value;
  if (preferences.cameras.forward !== forward || preferences.cameras.auxiliary !== auxiliary) {
    preferences.cameras.forward = forward;
    preferences.cameras.auxiliary = auxiliary;
    savePreferences();
    startCamera('forward', forward);
    startCamera('auxiliary', auxiliary);
  }
}

async function refreshCameraDevices(requestPermission = false) {
  const selections = [
    [$('forward-camera-source'), 'virtual', 'Virtual forward camera'],
    [$('aux-camera-source'), 'off', 'Off'],
  ];
  updateNetworkCameraOptions();
  if (!navigator.mediaDevices?.enumerateDevices) {
    const restore = (select, source, fallback) => {
      select.value = source.startsWith('raspberry:') ? source : fallback;
      return select.value;
    };
    preferences.cameras.forward = restore($('forward-camera-source'), preferences.cameras.forward, 'virtual');
    preferences.cameras.auxiliary = restore($('aux-camera-source'), preferences.cameras.auxiliary, 'off');
    $('camera-source-status').textContent = `${Object.keys(state.cameras || {}).length} live Raspberry Pi camera source(s). Laptop camera access needs HTTPS or localhost.`;
    savePreferences();
    if (preferences.cameras.forward.startsWith('raspberry:')) await startCamera('forward', preferences.cameras.forward);
    if (preferences.cameras.auxiliary.startsWith('raspberry:')) await startCamera('auxiliary', preferences.cameras.auxiliary);
    return;
  }
  let permissionStream = null;
  try {
    if (requestPermission) permissionStream = await navigator.mediaDevices.getUserMedia({video: true, audio: false});
    const cameras = (await navigator.mediaDevices.enumerateDevices()).filter(device => device.kind === 'videoinput');
    for (const [select, defaultValue, defaultLabel] of selections) {
      cameras.forEach((camera, index) => select.add(new Option(camera.label || `Camera ${index + 1}`, camera.deviceId)));
    }
    const knownSource = source => source.startsWith('raspberry:') || cameras.some(camera => camera.deviceId === source);
    $('forward-camera-source').value = knownSource(preferences.cameras.forward)
      ? preferences.cameras.forward : 'virtual';
    $('aux-camera-source').value = knownSource(preferences.cameras.auxiliary)
      ? preferences.cameras.auxiliary : 'off';
    preferences.cameras.forward = $('forward-camera-source').value;
    preferences.cameras.auxiliary = $('aux-camera-source').value;
    $('camera-source-status').textContent = `${Object.keys(state.cameras || {}).length} live Raspberry Pi camera source(s) · ${cameras.length} laptop camera${cameras.length === 1 ? '' : 's'} available.`;
    savePreferences();
    if (preferences.cameras.forward.startsWith('raspberry:') ||
        (cameras.some(camera => camera.label) && preferences.cameras.forward !== 'virtual')) {
      await startCamera('forward', preferences.cameras.forward);
    }
    if (preferences.cameras.auxiliary.startsWith('raspberry:') ||
        (cameras.some(camera => camera.label) && preferences.cameras.auxiliary !== 'off')) {
      await startCamera('auxiliary', preferences.cameras.auxiliary);
    }
  } catch (error) {
    $('camera-source-status').textContent = `Camera permission failed: ${error.message}`;
  } finally {
    permissionStream?.getTracks().forEach(track => track.stop());
  }
}

function initializeInputs() {
  document.querySelectorAll('[data-settings-tab]').forEach(tab => {
    tab.onclick = () => {
      document.querySelectorAll('[data-settings-tab]').forEach(candidate => {
        const active = candidate === tab;
        candidate.classList.toggle('active', active);
        candidate.setAttribute('aria-selected', String(active));
      });
      document.querySelectorAll('[data-settings-panel]').forEach(panel => {
        const active = panel.dataset.settingsPanel === tab.dataset.settingsTab;
        panel.classList.toggle('active', active);
        panel.hidden = !active;
      });
    };
  });
  for (const [key, label, checkbox] of [
    ['steering', 'Steering axis'], ['throttle', 'Forward pedal axis'], ['brake', 'Reverse pedal axis'],
    ['deadman', 'Deadman button'], ['deadzone', 'Steering deadzone'],
    ['invertSteering', 'Invert steering', true], ['invertThrottle', 'Invert forward pedal', true],
    ['invertBrake', 'Invert reverse pedal', true],
  ]) {
    const wrapper = document.createElement('label');
    wrapper.textContent = label;
    const input = document.createElement('input');
    input.type = checkbox ? 'checkbox' : 'number';
    if (checkbox) input.checked = Boolean(preferences.wheel[key]);
    else {
      input.min = 0; input.max = key === 'deadzone' ? 0.9 : 63;
      input.step = key === 'deadzone' ? 0.01 : 1;
      input.value = preferences.wheel[key];
    }
    input.onchange = () => {
      stopInput();
      preferences.wheel[key] = checkbox ? input.checked : input.value === '' ? null : Number(input.value);
      savePreferences();
    };
    wrapper.append(input); $('wheel-mapping').append(wrapper);
  }
  for (const code of ['KeyW', 'KeyA', 'KeyS', 'KeyD', 'ShiftLeft', 'ShiftRight']) {
    const key = document.createElement('kbd'); key.dataset.code = code;
    key.textContent = code.replace('Key', ''); $('key-preview').append(key);
  }
  $('input-device').onchange = event => {
    stopInput();
    const pad = visiblePads().find(p => String(p.index) === event.target.value);
    selectedPadIndex = pad?.index ?? null; selectedPadId = pad?.id ?? null;
  };
  $('keyboard-test').onblur = () => keys.clear();
  $('settings-dialog').addEventListener('close', stopInput);
  window.addEventListener('blur', stopInput);
  document.addEventListener('visibilitychange', () => { if (document.hidden) stopInput(); });
  $('refresh-cameras').onclick = () => refreshCameraDevices(true);
  $('forward-camera-source').onchange = async event => {
    preferences.cameras.forward = event.target.value;
    savePreferences();
    await startCamera('forward', event.target.value);
  };
  $('aux-camera-source').onchange = async event => {
    preferences.cameras.auxiliary = event.target.value;
    savePreferences();
    await startCamera('auxiliary', event.target.value);
  };
  navigator.mediaDevices?.addEventListener?.('devicechange', () => refreshCameraDevices(false));
  refreshCameraDevices(false);
}

let deviceSignature = '';
let lastPreview = 0;
function renderInputs(pads, target) {
  updateInputHint();
  if (!$('settings-dialog').open || performance.now() - lastPreview < 50) return;
  lastPreview = performance.now();
  const keyboard = preferences.input === 'keyboard';
  $('device-settings').classList.toggle('hidden', keyboard);
  $('wheel-settings').classList.toggle('hidden', preferences.input !== 'wheel');
  $('keyboard-test').classList.toggle('hidden', !keyboard);
  $('key-preview').classList.toggle('hidden', !keyboard);
  $('browser-status').textContent = `Secure context: ${window.isSecureContext ? 'yes' : 'no — open HTTPS or localhost'} · Gamepad API: ${navigator.getGamepads ? 'available' : 'unavailable'} · ${pads.length} device(s) visible`;
  const signature = JSON.stringify(pads.map(p => [p.index, p.id]));
  if (signature !== deviceSignature) {
    deviceSignature = signature;
    $('input-device').replaceChildren(new Option('Select a connected device', ''));
    pads.forEach(p => $('input-device').add(new Option(`${p.index}: ${p.id}`, String(p.index))));
    $('input-device').value = gamepad ? String(gamepad.index) : '';
  }
  $('device-info').textContent = gamepad
    ? `${gamepad.id} · ${gamepad.mapping || 'non-standard'} · ${gamepad.axes.length} axes · ${gamepad.buttons.length} buttons${preferences.input === 'controller' && gamepad.mapping !== 'standard' ? ' · Choose Steering wheel and map inputs; standard driving disabled.' : ''}`
    : 'No device selected. Press a device button, then select it above.';
  document.querySelectorAll('[data-code]').forEach(el => el.classList.toggle('pressed', keys.has(el.dataset.code)));
  const preview = keyboard ? readKeyboard() : target;
  $('steering-preview').setAttribute('transform', `rotate(${preview.angular / MAX_ANGULAR_RAD_S * 100} 80 80)`);
  $('mapped-input').textContent = `Forward / reverse: ${preview.linear.toFixed(3)} m/s\nSteering: ${preview.angular.toFixed(3)} rad/s\nDeadman: ${preview.deadman ? 'HELD' : 'released'}\nDrive output: stopped in Settings`;
  const axes = keyboard ? [] : gamepad?.axes ?? [];
  const buttons = keyboard ? [] : gamepad?.buttons ?? [];
  $('raw-axes').replaceChildren(...axes.map((value, index) => {
    const label = document.createElement('label'); label.textContent = `Axis ${index}  ${value.toFixed(3)}`;
    const meter = document.createElement('meter'); meter.min = -1; meter.max = 1; meter.value = value;
    label.append(meter); return label;
  }));
  $('raw-buttons').replaceChildren(...buttons.map((button, index) => {
    const item = document.createElement('span'); item.className = button.pressed ? 'pressed' : '';
    item.textContent = `B${index} ${button.value.toFixed(2)}`; return item;
  }));
}

initializeInputs();
savePreferences();
updateCameraVisibility();
setConnection(false, 'Connecting to operator stack');
connect();
inputLoop();
drawForwardView();
drawMap();
drawPressureHistory();
drawRadar();
