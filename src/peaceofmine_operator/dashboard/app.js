// The search lane runs along world +X, matching the ROS convention where a
// rover at yaw 0 faces +X. Cross-lane offsets are world Y.
const LANE_START_M = -2;
const LANE_END_M = 22;
const LANE_HALF_WIDTH_M = 2;
const CAMERA_RANGE_M = 26;

const DRIVE_PERIOD_MS = 50;
const LATENCY_PERIOD_MS = 1000;
const LATENCY_HISTORY_LENGTH = 30;
const MAX_LINEAR_MPS = 0.8;
const MAX_ANGULAR_RAD_S = 1.4;

const state = {
  connected: false,
  drive: {
    armed: false,
    deadman: false,
    publishing: false,
    cmd_linear_x: 0,
    cmd_angular_z: 0,
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
};

const $ = id => document.getElementById(id);
const forward = $('forward-view');
const map = $('map-view');
const defaultPreferences = {input: 'controller', sensitivity: 100, mapping: 'auto'};
const preferences = {
  ...defaultPreferences,
  ...JSON.parse(localStorage.getItem('peaceofmine.operator.preferences') || '{}'),
};
if (preferences.input !== 'keyboard' && preferences.input !== 'controller') {
  preferences.input = 'controller';
}
if (preferences.mapping !== 'auto' && preferences.mapping !== 'standard' && preferences.mapping !== 'xbox360') {
  preferences.mapping = 'auto';
}
preferences.sensitivity = Math.max(50, Math.min(150, Number(preferences.sensitivity) || 100));

const keys = new Set();
const smoothedDrive = {linear: 0, angular: 0, updatedAt: performance.now()};
const triggerSeen = {lt: false, rt: false};
const lastInput = {
  steer: 0,
  throttle: 0,
  deadman: false,
  mapping: '',
  stickX: 0,
  stickY: 0,
  lt: 0,
  rt: 0,
  lb: false,
  rb: false,
  a: false,
  b: false,
  x: false,
  y: false,
};
let socket;
let gamepad = null;
let lastPadId = null;
let prevA = false;
let cameraFocused = false;
let lastDriveSent = 0;
let noticeExpiry = 0;
let latencyProbeId = 0;
let latencyProbeStarted = 0;
const latencyHistory = [];

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

function setConnection(connected, label) {
  state.connected = connected;
  $('connection').textContent = label;
  $('connection-dot').style.background = connected ? '#37b98e' : '#d36458';
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
  latencyProbeStarted = 0;
  latencyHistory.push(latency);
  if (latencyHistory.length > LATENCY_HISTORY_LENGTH) latencyHistory.shift();
  $('latency-value').textContent = `${latency} ms`;
  drawLatencyGraph();
}

function connect() {
  const scheme = location.protocol === 'https:' ? 'wss' : 'ws';
  socket = new WebSocket(`${scheme}://${location.host}/ws`);

  socket.onopen = () => {
    setConnection(true, 'Dashboard linked');
    measureLatency();
  };

  socket.onclose = () => {
    latencyProbeStarted = 0;
    latencyHistory.length = 0;
    $('latency-value').textContent = '-- ms';
    drawLatencyGraph();
    state.drive.armed = false;
    state.drive.deadman = false;
    state.drive.you_control_owner = false;
    setConnection(false, 'Connection lost — rover stopped');
    setTimeout(connect, 1200);
  };

  socket.onerror = () => socket.close();

  socket.onmessage = event => {
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
    Object.assign(state, message);
    updateHud();
  };
}

setInterval(measureLatency, LATENCY_PERIOD_MS);

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
      ? (drive.deadman ? 'Deadman held · command stream active.' : 'Armed · hold the right bumper to drive.')
      : 'Disarmed · arm only after the lane is clear.';

  const heading = ((robot.yaw * 180 / Math.PI % 360) + 360) % 360;
  $('position').textContent = `X ${robot.x.toFixed(1)} · Y ${robot.y.toFixed(1)} · heading ${heading.toFixed(0)}°`;

  updateInputHint();
}

function padDisplayName(pad) {
  return pad.id.replace(/\([^)]*\)/g, '').replace(/\s+/g, ' ').trim();
}

function mappingLabel(kind) {
  if (kind === 'xbox360') return 'Xbox 360 / Linux';
  if (kind === 'standard') return 'standard gamepad';
  return kind || 'unknown';
}

function setLit(id, on, extraClass) {
  const el = $(id);
  if (!el) return;
  el.classList.toggle('lit', Boolean(on));
  if (extraClass) el.classList.toggle(extraClass, Boolean(on));
}

function paintTrigger(id, value) {
  const el = $(id);
  if (!el) return;
  const active = value > 0.08;
  el.classList.toggle('lit', active);
  el.style.fill = active ? `rgba(42, 165, 134, ${0.35 + value * 0.65})` : '';
}

function gamepadApiBlocked() {
  return typeof navigator.getGamepads !== 'function'
    || (typeof window.isSecureContext === 'boolean' && !window.isSecureContext);
}

function driveBlockMessage() {
  const owner = state.connected && state.drive.you_control_owner;
  if (preferences.input === 'controller' && gamepadApiBlocked()) {
    return {
      text: 'Gamepad API blocked. Open http://localhost:8080 on this computer (not the LAN IP) or use HTTPS.',
      kind: 'blocked',
    };
  }
  if (preferences.input === 'keyboard' && !cameraFocused) {
    return {text: 'Click the forward camera, then hold Shift and WASD.', kind: 'blocked'};
  }
  if (preferences.input === 'controller' && !gamepad) {
    return {text: 'No pad seen. Plug in the Xbox 360 and press a button.', kind: 'blocked'};
  }
  if (!owner) return {text: 'Spectator · take control. Pad input stays local until then.', kind: 'blocked'};
  if (!state.drive.armed) return {text: 'Disarmed · press Arm or A, then use RT/LT.', kind: 'blocked'};
  if (!lastInput.deadman) return {text: 'Armed · pull RT/LT or hold a bumper to publish cmd_vel.', kind: 'blocked'};
  if (!state.drive.publishing) {
    return {text: 'Commands leaving the browser · waiting for ROS cmd_vel…', kind: 'blocked'};
  }
  return {
    text: `ROS cmd_vel ${state.drive.cmd_linear_x.toFixed(2)} m/s · ${state.drive.cmd_angular_z.toFixed(2)} rad/s`,
    kind: 'ready',
  };
}

function updateInputHint() {
  const keyboard = preferences.input === 'keyboard';
  $('input-lock')?.classList.toggle('hidden', !keyboard);
  $('input-lock')?.classList.toggle('locked', keyboard && cameraFocused);
  if ($('input-lock')) {
    $('input-lock').textContent = cameraFocused ? 'WASD INPUT ACTIVE' : 'CLICK CAMERA FOR INPUT';
  }

  if (gamepadApiBlocked() && !keyboard) {
    $('controller').textContent = 'Controller blocked by the browser';
    $('controller-mapping').textContent =
      'Use http://localhost:8080 on this computer. http://<ip>:8080 is not a secure context, so Chrome hides the Gamepad API.';
  } else if (keyboard) {
    $('controller').textContent = cameraFocused
      ? 'WASD active · hold Shift to drive'
      : 'Click the forward camera to use WASD';
    $('controller-mapping').textContent = 'WASD steer/throttle · Shift is the deadman (shown as RB).';
  } else if (gamepad) {
    const kind = resolveMapping(gamepad);
    $('controller').textContent = padDisplayName(gamepad);
    $('controller-mapping').textContent =
      `${mappingLabel(kind)} · LS steer · LT/RT throttle · LB/RB deadman · A arm`;
  } else {
    $('controller').textContent = 'No controller connected';
    $('controller-mapping').textContent =
      'Plug the Xbox 360 into this computer and press any button so the browser can see it.';
  }

  if ($('pad-raw')) {
    if (gamepad) {
      const axes = [...gamepad.axes].map(value => Number(value).toFixed(2)).join(' ');
      const buttons = [...gamepad.buttons].map(button => (button.pressed ? '1' : '0')).join('');
      $('pad-raw').textContent = `${gamepad.mapping || 'no-mapping'} · axes ${axes} · btns ${buttons}`;
    } else {
      $('pad-raw').textContent = gamepadApiBlocked()
        ? 'Gamepad API unavailable in this origin.'
        : `Pads seen: ${(navigator.getGamepads ? [...navigator.getGamepads()].filter(Boolean).length : 0)}. Press a button.`;
    }
  }

  $('pad-ls')?.setAttribute('transform', `translate(${lastInput.stickX * 14}, ${lastInput.stickY * 14})`);
  $('pad-ls')?.classList.toggle('lit', Math.hypot(lastInput.stickX, lastInput.stickY) > 0.12);
  paintTrigger('pad-lt', lastInput.lt);
  paintTrigger('pad-rt', lastInput.rt);
  setLit('pad-lb', lastInput.lb, 'deadman');
  setLit('pad-rb', lastInput.rb, 'deadman');
  setLit('pad-a', lastInput.a);
  setLit('pad-b', lastInput.b);
  setLit('pad-x', lastInput.x);
  setLit('pad-y', lastInput.y);

  const block = driveBlockMessage();
  if ($('drive-block')) {
    $('drive-block').textContent = block.text;
    $('drive-block').className = block.kind;
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

function looksLikeXbox(id) {
  return /xbox|x-box|xinput/i.test(id);
}

function resolveMapping(pad) {
  const override = preferences.mapping;
  if (override === 'standard' || override === 'xbox360') return override;
  if (pad.mapping === 'standard') return 'standard';
  if (looksLikeXbox(pad.id)) return 'xbox360';
  return 'standard';
}

function syncTriggerState(pad) {
  if (!pad || pad.id !== lastPadId) {
    triggerSeen.lt = false;
    triggerSeen.rt = false;
    lastPadId = pad?.id ?? null;
  }
}

// Linux xpad often reports LT/RT as -1..1, but the axis stays at 0 until the
// first press. Treat that unused 0 as released; after motion, map to 0..1.
function axisTrigger(key, value) {
  if (value == null || Number.isNaN(value)) return 0;
  if (!triggerSeen[key]) {
    if (value === 0) return 0;
    triggerSeen[key] = true;
  }
  return Math.max(0, Math.min(1, (value + 1) / 2));
}

function readTriggers(pad, kind) {
  const stdLt = buttonValue(pad.buttons[6]);
  const stdRt = buttonValue(pad.buttons[7]);
  const axisLt = axisTrigger('lt', pad.axes[2]);
  const axisRt = axisTrigger('rt', pad.axes[5]);
  if (kind === 'standard') {
    if (stdLt + stdRt < 0.05 && axisLt + axisRt > 0.2 && pad.mapping !== 'standard') {
      return {lt: axisLt, rt: axisRt};
    }
    return {lt: stdLt, rt: stdRt};
  }
  if (stdLt + stdRt > 0.2 && axisLt + axisRt < 0.05) return {lt: stdLt, rt: stdRt};
  return {lt: axisLt, rt: axisRt};
}

function readGamepad() {
  const pad = gamepad;
  const kind = resolveMapping(pad);
  syncTriggerState(pad);
  const stickX = pad.axes[0] ?? 0;
  const stickY = pad.axes[1] ?? 0;
  const steering = responseCurve(deadzone(stickX));
  const {lt: leftTrigger, rt: rightTrigger} = readTriggers(pad, kind);
  const throttle = responseCurve(rightTrigger - leftTrigger);
  const lb = Boolean(pad.buttons[4]?.pressed);
  const rb = Boolean(pad.buttons[5]?.pressed);
  return {
    linear: throttle * MAX_LINEAR_MPS,
    angular: steering * MAX_ANGULAR_RAD_S * preferences.sensitivity / 100,
    deadman: lb || rb || Math.abs(throttle) > 0.12,
    steer: steering,
    throttle,
    mapping: kind,
    stickX,
    stickY,
    lt: leftTrigger,
    rt: rightTrigger,
    lb,
    rb,
    a: Boolean(pad.buttons[0]?.pressed),
    b: Boolean(pad.buttons[1]?.pressed),
    x: Boolean(pad.buttons[2]?.pressed),
    y: Boolean(pad.buttons[3]?.pressed),
  };
}

function readKeyboard() {
  const throttle = (keys.has('KeyW') ? 1 : 0) - (keys.has('KeyS') ? 1 : 0);
  const steer = (keys.has('KeyD') ? 1 : 0) - (keys.has('KeyA') ? 1 : 0);
  const deadman = keys.has('ShiftLeft') || keys.has('ShiftRight');
  return {
    linear: throttle * MAX_LINEAR_MPS,
    angular: steer * MAX_ANGULAR_RAD_S * preferences.sensitivity / 100,
    deadman,
    steer,
    throttle,
    mapping: 'keyboard',
    stickX: steer,
    stickY: -throttle,
    lt: keys.has('KeyS') ? 1 : 0,
    rt: keys.has('KeyW') ? 1 : 0,
    lb: false,
    rb: deadman,
    a: false,
    b: false,
    x: false,
    y: false,
  };
}

function padActivity(pad) {
  const button = pad.buttons.reduce((sum, item) => sum + (item.pressed ? 1 : item.value || 0), 0);
  const axes = pad.axes.reduce((sum, value) => sum + (Math.abs(value) > 0.2 ? Math.abs(value) : 0), 0);
  return button + axes;
}

function pickGamepad(pads) {
  if (!pads.length) return null;
  const ranked = [...pads].sort((left, right) => padActivity(right) - padActivity(left));
  if (padActivity(ranked[0]) > 0.2) return ranked[0];
  if (lastPadId) {
    const previous = pads.find(pad => pad.id === lastPadId);
    if (previous) return previous;
  }
  return ranked.find(pad => looksLikeXbox(pad.id)) || ranked[0];
}

function maybeArmFromPad(target) {
  if (target.a && !prevA && state.drive.you_control_owner && !state.drive.armed) {
    send({type: 'arm'});
  }
  prevA = Boolean(target.a);
}

function copyInput(target) {
  lastInput.steer = target.steer ?? 0;
  lastInput.throttle = target.throttle ?? 0;
  lastInput.deadman = Boolean(target.deadman);
  lastInput.mapping = target.mapping || '';
  lastInput.stickX = target.stickX ?? 0;
  lastInput.stickY = target.stickY ?? 0;
  lastInput.lt = target.lt ?? 0;
  lastInput.rt = target.rt ?? 0;
  lastInput.lb = Boolean(target.lb);
  lastInput.rb = Boolean(target.rb);
  lastInput.a = Boolean(target.a);
  lastInput.b = Boolean(target.b);
  lastInput.x = Boolean(target.x);
  lastInput.y = Boolean(target.y);
}

function inputLoop() {
  const pads = navigator.getGamepads ? [...navigator.getGamepads()].filter(Boolean) : [];
  gamepad = pickGamepad(pads);

  let target = {
    linear: 0, angular: 0, deadman: false, steer: 0, throttle: 0, mapping: '',
    stickX: 0, stickY: 0, lt: 0, rt: 0, lb: false, rb: false, a: false, b: false, x: false, y: false,
  };
  if (preferences.input === 'keyboard' && cameraFocused) {
    target = readKeyboard();
  } else if (preferences.input === 'controller' && gamepad) {
    target = readGamepad();
    maybeArmFromPad(target);
  } else {
    syncTriggerState(null);
    prevA = false;
  }

  copyInput(target);
  updateInputHint();

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

$('take-control').onclick = () => send({type: 'take_control'});
$('arm').onclick = () => send({type: 'arm'});
$('estop').onclick = () => send({type: 'estop'});

$('probe-slider').oninput = throttle(70, event => send({type: 'probe_target', depth_mm: Number(event.target.value)}));
$('sweep-speed').oninput = throttle(70, event => send({type: 'sweep_speed', deg_s: Number(event.target.value)}));
$('sweep-toggle').onclick = () => send({type: 'sweep_enabled', enabled: !state.detector.sweep_enabled});

function savePreferences() {
  localStorage.setItem('peaceofmine.operator.preferences', JSON.stringify(preferences));
  $('control-source').value = preferences.input;
  $('pad-mapping').value = preferences.mapping;
  $('steering-sensitivity').value = preferences.sensitivity;
  $('sensitivity-value').textContent = `${preferences.sensitivity}%`;
  $('pad-mapping').disabled = preferences.input === 'keyboard';
  updateInputHint();
}

$('settings').onclick = () => $('settings-dialog').showModal();
$('control-source').onchange = event => {
  preferences.input = event.target.value;
  savePreferences();
};
$('steering-sensitivity').oninput = event => {
  preferences.sensitivity = Number(event.target.value);
  savePreferences();
};
$('pad-mapping').onchange = event => {
  preferences.mapping = event.target.value;
  triggerSeen.lt = false;
  triggerSeen.rt = false;
  savePreferences();
};

// The canvas carries tabindex="0", so a click focuses it and WASD is captured
// only while it holds focus.
forward.addEventListener('focus', () => {
  cameraFocused = true;
  updateInputHint();
});

forward.addEventListener('blur', () => {
  cameraFocused = false;
  keys.clear();
  updateInputHint();
});

window.addEventListener('keydown', event => {
  if (preferences.input !== 'keyboard' || !cameraFocused) return;
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

try { savePreferences(); } catch (error) { console.error(error); }
setConnection(false, 'Connecting');
connect();
inputLoop();
drawForwardView();
drawMap();
drawPressureHistory();
drawRadar();
