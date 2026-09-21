// The search lane runs along world +X, matching the ROS convention where a
// rover at yaw 0 faces +X. Cross-lane offsets are world Y.
const LANE_START_M = -2;
const LANE_END_M = 22;
const LANE_HALF_WIDTH_M = 2;
const CAMERA_RANGE_M = 26;

const DRIVE_KEEPALIVE_MS = 50;
const DRIVE_MIN_INTERVAL_MS = 8;
const PAD_UI_PERIOD_MS = 50;
const LATENCY_PERIOD_MS = 1000;
const LATENCY_HISTORY_LENGTH = 30;
const TELEMETRY_TIMEOUT_MS = 3000;
const MAX_LINEAR_MPS = 0.8;
const MAX_ANGULAR_RAD_S = 1.4;
const DRIVE_EPSILON = 1e-4;

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
    command_source: 'none',
    joy: {seen: false},
  },
  robot: {x: 0, y: 0, yaw: 0, speed_mps: 0},
  detector: {
    signal_ratio: 0,
    threshold_ratio: 1.0,
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
  safety: {allowed: false, mode: 'unknown', reason: 'Waiting for PX4 safety status'},
  arm_servo: {},
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
for (const slot of ['forward', 'auxiliary']) {
  if (typeof preferences.cameras[slot] !== 'string' || !preferences.cameras[slot]) {
    preferences.cameras[slot] = slot === 'forward' ? 'virtual' : 'off';
  }
}
preferences.cameraRotation = Object.assign({forward: 0, auxiliary: 0}, preferences.cameraRotation);
for (const slot of ['forward', 'auxiliary']) {
  if (![0, 90, 180, 270].includes(preferences.cameraRotation[slot])) preferences.cameraRotation[slot] = 0;
}
const keyboardRamp = {throttle: 0, steer: 0, updatedAt: performance.now()};
let selectedPadIndex = null;
let selectedPadId = null;

const keys = new Set();
const activeSliders = new Set();
if (!['auto', 'standard', 'xbox360'].includes(preferences.mapping)) preferences.mapping = 'auto';
const triggerSeen = {lt: false, rt: false};
const lastInput = {
  linear: 0,
  angular: 0,
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
const lastSentDrive = {linear: 0, angular: 0, deadman: false};
const padEls = {
  controller: $('controller'),
  mapping: $('controller-mapping'),
  raw: $('pad-raw'),
  ls: $('pad-ls'),
  lt: $('pad-lt'),
  rt: $('pad-rt'),
  lb: $('pad-lb'),
  rb: $('pad-rb'),
  a: $('pad-a'),
  b: $('pad-b'),
  x: $('pad-x'),
  y: $('pad-y'),
  driveBlock: $('drive-block'),
  inputLock: $('input-lock'),
};
let socket;
let gamepad = null;
let lastPadId = null;
let lastPadTimestamp = -1;
let prevA = false;
let cameraFocused = false;
let lastDriveSent = 0;
let lastPadUi = 0;
let noticeExpiry = 0;
let latencyProbeId = 0;
let latencyProbeStarted = 0;
let linkLatencyMs = null;
let lastTelemetryAt = 0;
let reconnectTimer = null;
const latencyHistory = [];
const cameraStreams = {forward: null, auxiliary: null};
let networkCameraSignature = '';
let localCameras = [];
let cameraDeviceRefresh = 0;
const cameraGeneration = {forward: 0, auxiliary: 0};
const cameraSources = {forward: null, auxiliary: null};
const cameraRetries = {forward: null, auxiliary: null};
const cameraErrors = {forward: '', auxiliary: ''};

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

function sendDrive(linear, angular, deadman) {
  if (socket?.readyState !== WebSocket.OPEN) return;
  socket.send(
    `{"type":"drive","linear_x":${linear},"angular_z":${angular},"deadman":${deadman}}`,
  );
}

function setConnection(connected, label, detail = '') {
  state.connected = connected;
  $('connection').textContent = label;
  $('connection-dot').style.background = connected ? '#37b98e' : '#d36458';
  $('connection-overlay').classList.toggle('hidden', connected);
  $('connection-title').textContent = connected ? 'Connected' : label;
  $('connection-detail').textContent = detail || 'The operator gateway is unavailable. Start the PeaceOfMine stack on the Raspberry Pi; this page will reconnect automatically.';
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
  if (socket?.readyState !== WebSocket.OPEN) return;
  if (latencyProbeStarted && performance.now() - latencyProbeStarted < TELEMETRY_TIMEOUT_MS) return;
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
  stopInput();
  state.drive.armed = state.drive.deadman = state.drive.you_control_owner = false;
  latencyProbeStarted = 0;
  if (socket && socket.readyState < WebSocket.CLOSING) socket.close();
  const scheme = location.protocol === 'https:' ? 'wss' : 'ws';
  const connectingSocket = new WebSocket(`${scheme}://${location.host}/ws`);
  socket = connectingSocket;
  lastTelemetryAt = performance.now();
  setConnection(false, 'Connecting to operator stack', 'The dashboard is loaded. Waiting for telemetry from the Raspberry Pi gateway.');

  connectingSocket.onopen = () => {
    if (socket !== connectingSocket) return;
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
    if (socket !== connectingSocket) return;
    let message;
    try { message = JSON.parse(event.data); } catch { return; }
    if (!message || typeof message !== 'object') return;
    if (message.type === 'latency_pong') {
      recordLatency(message.probe_id);
      return;
    }
    if (message.type === 'error') {
      showNotice(message.message);
      if (metalPending) { metalPending = null; metalFeedback = message.message; }
      if (armPending) { armPending = null; setArmFeedback(message.message, 'error'); }
      return;
    }
    if (message.type !== 'state') return;
    lastTelemetryAt = performance.now();
    if (!state.connected) setConnection(true, 'Raspberry Pi connected');
    Object.assign(state, message);
    state.connected = true;
    recordDetectorAngle(lastTelemetryAt);
    updateNetworkCameraOptions();
    updateCameraLatency();
    updateHud();
  };
}

setInterval(measureLatency, LATENCY_PERIOD_MS);
setInterval(() => {
  if (socket && socket.readyState < WebSocket.CLOSING &&
      performance.now() - lastTelemetryAt > TELEMETRY_TIMEOUT_MS) {
    setConnection(false, 'Telemetry stalled', 'The gateway connection is open, but rover telemetry stopped. Controls are disabled while reconnecting.');
    socket.close();
  }
}, 500);

$('retry-connection').onclick = connect;

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

function updatePower() {
  const power = state.power || {};
  const battery = power.battery || {};
  const live = state.connected && battery.available && !battery.stale && battery.present;
  const fmt = (value, unit, digits = 1) => Number.isFinite(value) ? `${value.toFixed(digits)} ${unit}` : '—';
  const text = (id, value) => { $(id).textContent = value; };
  text('power-status', !state.connected ? 'DISCONNECTED' : battery.stale ? 'STALE' : live ? 'LIVE' : 'NO BATTERY DATA');
  $('power-status').className = `badge ${live ? 'safe' : 'neutral'}`;
  text('battery-watts', live ? fmt(battery.power_w, 'W') : '—');
  text('battery-charge', live ? fmt(battery.remaining_pct, '%', 0) : '—');
  text('battery-voltage', live ? fmt(battery.voltage_v, 'V', 2) : '—');
  text('battery-current', live ? fmt(battery.current_a, 'A', 2) : '—');
  text('battery-temperature', live ? fmt(battery.temperature_c, '°C') : '—');
  text('battery-cells', live && Number.isFinite(battery.cell_delta_v) ? fmt(battery.cell_delta_v * 1000, 'mV', 0) : '—');
  $('battery-level').hidden = !live || !Number.isFinite(battery.remaining_pct);
  if (live && Number.isFinite(battery.remaining_pct)) $('battery-level').value = battery.remaining_pct;
  const history = live ? (power.history || []).filter(p => Number.isFinite(p.watts) && p.age_s <= 60) : [];
  const low = Math.min(0, ...history.map(p => p.watts)), high = Math.max(1, ...history.map(p => p.watts));
  $('power-history-line').setAttribute('points', history.map(p => `${300 * (1 - p.age_s / 60)},${35 - 32 * (p.watts - low) / (high - low)}`).join(' '));
  const esc = power.esc || {};
  const escLive = state.connected && esc.available && !esc.stale;
  text('esc-status', !state.connected ? 'Disconnected' : esc.stale ? 'Stale' : escLive ? 'Live' : 'No telemetry');
  const rows = $('esc-readings');
  rows.replaceChildren();
  if (escLive && esc.motors?.length) {
    esc.motors.forEach(motor => {
      const row = document.createElement('div');
      row.textContent = motor.online ? `ESC ${motor.id} · ${fmt(motor.power_w, 'W')} · ${fmt(motor.current_a, 'A')} · ${fmt(motor.temperature_c, '°C')} · ${fmt(motor.rpm, 'rpm', 0)}${motor.faults ? ' · FAULT' : ''}` : `ESC ${motor.id} · Offline`;
      rows.append(row);
    });
  } else rows.textContent = esc.stale ? 'ESC readings are stale' : 'No ESC measurements received';
  const arm = state.arm_servo || {};
  text('power-arm', state.connected && arm.connected ? `ID ${arm.servo_id} · ${fmt(arm.voltage, 'V')} · ${fmt(arm.current_a, 'A', 3)} · ${fmt(arm.temperature_c, '°C', 0)}` : 'No telemetry');
}

function updateHud() {
  updatePower();
  updateArmSettings();
  updateMetalSettings();
  const {drive, robot, detector, probe} = state;
  const owner = state.connected && drive.you_control_owner;

  $('client-count').textContent = `${drive.client_count} ${drive.client_count === 1 ? 'viewer' : 'viewers'}`;
  $('control-owner').textContent = owner ? 'YOU CONTROL' : (drive.control_owner_present ? 'SPECTATING' : 'UNCLAIMED');
  $('control-owner').className = `badge ${owner ? 'safe' : 'neutral'}`;
  $('take-control').textContent = drive.control_owner_present ? 'Steal control' : 'Take control';

  $('take-control').disabled = !state.connected || owner;
  const safe = state.safety?.allowed === true;
  const servoSafe = state.safety?.servo_allowed === true;
  $('rc-safety').textContent = ({ros: 'RC ROS MODE', override: 'RC OVERRIDE', kill: 'RC KILL', rc_lost: 'RC LOST', ros_disarmed: 'RC DISARMED', simulation: 'SIMULATION'})[state.safety?.mode] || 'RC UNKNOWN · LOCKED';
  if (servoSafe && !safe) $('rc-safety').textContent += ' · SERVOS READY';
  $('rc-safety').title = state.safety?.reason || 'Waiting for PX4 safety status';
  $('rc-safety').className = `badge ${safe ? 'ready' : 'blocked'}`;
  $('probe-slider').disabled = !owner || !servoSafe;
  $('sweep-toggle').disabled = !owner || !servoSafe;
  $('sweep-speed').disabled = !owner || !servoSafe;

  $('speed').textContent = `${Math.abs(robot.speed_mps).toFixed(2)} m/s`;

  const signalPercent = detector.signal_ratio * 100;
  $('signal').textContent = detector.fresh === false ? '--' : Math.round(signalPercent);
  const rawAdc = detector.sensor?.packet?.amplitude_adc;
  $('detector-raw').textContent = detector.fresh !== false && detector.sensor?.fresh === true && Number.isFinite(rawAdc)
    ? `${rawAdc} ADC` : '-- ADC';
  $('meter-fill').style.width = `${detector.fresh === false ? 0 : signalPercent}%`;
  $('meter-fill').style.background = detectorColor(detector.signal_ratio);
  $('threshold').style.left = `${detector.threshold_ratio * 100}%`;
  $('detector-detail').textContent =
    `Threshold ${Math.round(detector.threshold_ratio * 100)}% · fixture ${detector.fixture_angle_deg.toFixed(0)}°`;
  $('detector-status').textContent = detector.fresh === false ? 'NO DATA' : detector.detected ? 'DETECTION' : 'CLEAR';
  $('detector-status').className = `badge ${detector.fresh === false ? 'neutral' : detector.detected ? 'active' : 'safe'}`;

  $('sweep-toggle').textContent = detector.sweep_enabled ? 'Stop sweep' : 'Start sweep';
  $('sweep-status').textContent = detector.sweep_enabled ? 'Sweeping'
    : !owner ? 'Take control to sweep'
    : !servoSafe ? state.safety?.reason || 'RC blocks servo movement'
    : state.arm_servo?.reason || (state.safety?.simulated ? 'Ready' : 'Waiting for arm driver');
  setRange($('sweep-speed'), detector.sweep_speed_min, detector.sweep_speed_max, 'any');
  if (!activeSliders.has('sweep-speed')) $('sweep-speed').value = detector.sweep_speed_deg_s;
  $('sweep-speed-value').textContent = `${Math.round(detector.sweep_speed_deg_s)}°/s`;

  $('probe-depth').textContent = `${Math.round(probe.depth_mm)} mm`;
  $('probe-target').textContent = `${Math.round(probe.target_mm)} mm`;
  $('probe-pressure').textContent = `${Math.round(probe.pressure_ratio * 100)}%`;
  setRange($('probe-slider'), 0, probe.max_depth_mm, 1);
  if (!activeSliders.has('probe-slider')) $('probe-slider').value = probe.target_mm;
  $('probe-arm').style.height = `${probe.depth_mm / probe.max_depth_mm * 2.4}rem`;
  $('probe-status').textContent = probe.fault ? 'FAULT' : probe.depth_mm > 2 ? 'DEPLOYED' : 'STOWED';
  $('probe-status').className = `badge ${probe.fault ? 'active' : probe.depth_mm > 2 ? 'safe' : 'neutral'}`;

  $('arm-state').textContent = state.safety?.mode === 'override' ? 'RC CONTROL' : !owner ? 'SPECTATOR' : safe ? (drive.publishing ? 'ROS CONTROL' : 'READY') : 'RC LOCKED';
  $('arm-state').className = `badge ${owner && drive.armed ? 'active' : 'neutral'}`;
  $('camera-notice').textContent = state.safety?.mode === 'override' ? 'Remote control drives the vehicle.'
    : !owner ? 'Spectator mode' : safe ? 'RC permits ROS driving' : state.safety?.reason || 'Waiting for RC';

  const heading = ((robot.yaw * 180 / Math.PI % 360) + 360) % 360;
  $('position').textContent = `X ${robot.x.toFixed(1)} · Y ${robot.y.toFixed(1)} · heading ${heading.toFixed(0)}°`;

  updateInputHint();
}

function deadmanHint() {
  return 'RC switch controls drive authority.';
}

function padDisplayName(pad) {
  return pad.id.replace(/\([^)]*\)/g, '').replace(/\s+/g, ' ').trim();
}

function mappingLabel(kind) {
  if (kind === 'xbox360') return 'Xbox 360 / Linux';
  if (kind === 'standard') return 'standard gamepad';
  return kind || 'unknown';
}

function setLitEl(el, on, extraClass) {
  if (!el) return;
  el.classList.toggle('lit', Boolean(on));
  if (extraClass) el.classList.toggle(extraClass, Boolean(on));
}

function paintTriggerEl(el, value) {
  if (!el) return;
  const active = value > 0.08;
  el.classList.toggle('lit', active);
  el.style.fill = active ? `rgba(42, 165, 134, ${0.35 + value * 0.65})` : '';
}

function gamepadApiBlocked() {
  return typeof navigator.getGamepads !== 'function'
    || (typeof window.isSecureContext === 'boolean' && !window.isSecureContext);
}

function rosJoyLive() {
  return Boolean(state.drive.joy?.seen);
}

function driveBlockMessage() {
  if (state.safety?.mode === 'override') return {text: state.drive.rc?.fresh ? 'Remote control active' : 'RC input stale', kind: 'ready'};
  const owner = state.connected && state.drive.you_control_owner;
  const padReady = Boolean(gamepad) || rosJoyLive();
  if (preferences.input === 'controller' && gamepadApiBlocked() && !rosJoyLive()) {
    return {
      text: 'Browser Gamepad API blocked. Plug the Xbox into the SVEA for ROS /joy, or open http://localhost:8080 / HTTPS.',
      kind: 'blocked',
    };
  }
  if (preferences.input === 'keyboard' && !cameraFocused) {
    return {text: 'Click the forward camera for WASD.', kind: 'blocked'};
  }
  if (preferences.input === 'controller' && !padReady) {
    return {text: 'No pad seen. Plug the Xbox into this computer or the SVEA USB, then press a button.', kind: 'blocked'};
  }
  if (!state.safety?.allowed) return {text: state.safety?.reason || 'Waiting for PX4 safety status', kind: 'blocked'};
  if (!owner) return {text: 'Spectator · take control. Pad input stays local until then.', kind: 'blocked'};
  if (!state.drive.publishing) {
    return {text: 'Commands leaving the browser · waiting for ROS cmd_vel…', kind: 'blocked'};
  }
  return {
    text: `ROS cmd_vel ${state.drive.cmd_linear_x.toFixed(2)} m/s · ${state.drive.cmd_angular_z.toFixed(2)} rad/s`,
    kind: 'ready',
  };
}

function setText(el, text) {
  if (el && el.textContent !== text) el.textContent = text;
}

function updateInputHint() {
  $('drive-help').textContent = deadmanHint();
  const keyboard = preferences.input === 'keyboard';
  const lock = padEls.inputLock;
  if (lock) {
    lock.classList.toggle('hidden', !keyboard);
    lock.classList.toggle('locked', keyboard && cameraFocused);
    setText(lock, cameraFocused ? 'WASD INPUT ACTIVE' : 'CLICK CAMERA FOR INPUT');
  }

  const rcOverride = state.safety?.mode === 'override';
  const rc = state.drive.rc || {};
  if (rcOverride) {
    setText(padEls.controller, 'Physical RC transmitter');
    setText(padEls.mapping, `Steering CH${rc.steering_channel ?? 1} · throttle CH${rc.throttle_channel ?? 2}`);
  } else if (rosJoyLive() && !keyboard && !gamepad) {
    setText(padEls.controller, 'Xbox via ROS /joy (SVEA USB)');
    setText(padEls.mapping, 'LS steer · LT/RT throttle');
  } else if (gamepadApiBlocked() && !keyboard) {
    setText(padEls.controller, 'Browser Gamepad API blocked');
    setText(padEls.mapping,
      'Open http://localhost:8080 on this computer, or plug the Xbox into the SVEA so ROS /joy can drive.');
  } else if (keyboard) {
    setText(padEls.controller, cameraFocused
      ? 'WASD active'
      : 'Click the forward camera to use WASD');
    setText(padEls.mapping, 'WASD steer/throttle');
  } else if (gamepad) {
    setText(padEls.controller, padDisplayName(gamepad));
    setText(padEls.mapping,
      `${mappingLabel(resolveMapping(gamepad))} · LS steer · LT/RT throttle`);
  } else {
    setText(padEls.controller, 'No controller connected');
    setText(padEls.mapping,
      'Plug the Xbox 360 into this computer and press any button so the browser can see it.');
  }

  if (padEls.raw) {
    if (rcOverride) {
      setText(padEls.raw, rc.fresh ? `RC PWM: ${(rc.channels || []).join(' ')}` : 'RC input stale');
    } else if (gamepad) {
      let axes = '';
      for (let i = 0; i < gamepad.axes.length; i++) {
        if (i) axes += ' ';
        axes += Number(gamepad.axes[i]).toFixed(2);
      }
      let buttons = '';
      for (let i = 0; i < gamepad.buttons.length; i++) buttons += gamepad.buttons[i].pressed ? '1' : '0';
      setText(padEls.raw, `${gamepad.mapping || 'no-mapping'} · axes ${axes} · btns ${buttons}`);
    } else if (rosJoyLive()) {
      const joy = state.drive.joy;
      setText(padEls.raw,
        `ROS /joy · age ${joy.age_ms ?? '--'} ms · LT ${Number(joy.lt || 0).toFixed(2)} RT ${Number(joy.rt || 0).toFixed(2)}`);
    } else {
      let seen = 0;
      if (navigator.getGamepads) {
        const pads = navigator.getGamepads();
        for (let i = 0; i < pads.length; i++) if (pads[i]) seen += 1;
      }
      setText(padEls.raw, gamepadApiBlocked()
        ? 'Gamepad API unavailable in this origin. Waiting for ROS /joy…'
        : `Pads seen: ${seen}. Press a button.`);
    }
  }

  const visual = rcOverride ? {
    stickX: rc.fresh ? rc.steering : 0, stickY: 0,
    lt: rc.fresh ? Math.max(0, -rc.throttle) : 0, rt: rc.fresh ? Math.max(0, rc.throttle) : 0,
  } : (!gamepad && rosJoyLive()) ? {
    stickX: state.drive.joy.stick_x || 0,
    stickY: state.drive.joy.stick_y || 0,
    lt: state.drive.joy.lt || 0,
    rt: state.drive.joy.rt || 0,
    lb: Boolean(state.drive.joy.lb),
    rb: Boolean(state.drive.joy.rb),
    a: Boolean(state.drive.joy.a),
    b: Boolean(state.drive.joy.b),
    x: Boolean(state.drive.joy.x),
    y: Boolean(state.drive.joy.y),
  } : lastInput;

  if (padEls.ls) {
    padEls.ls.setAttribute('transform', `translate(${visual.stickX * 14}, ${visual.stickY * 14})`);
    padEls.ls.classList.toggle('lit', Math.hypot(visual.stickX, visual.stickY) > 0.12);
  }
  paintTriggerEl(padEls.lt, visual.lt);
  paintTriggerEl(padEls.rt, visual.rt);
  setLitEl(padEls.lb, visual.lb, 'deadman');
  setLitEl(padEls.rb, visual.rb, 'deadman');
  setLitEl(padEls.a, visual.a);
  setLitEl(padEls.b, visual.b);
  setLitEl(padEls.x, visual.x);
  setLitEl(padEls.y, visual.y);

  const block = driveBlockMessage();
  if (padEls.driveBlock) {
    setText(padEls.driveBlock, block.text);
    if (padEls.driveBlock.className !== block.kind) padEls.driveBlock.className = block.kind;
  }
}

/* ---------------------------------------------------------------- Input --- */

function buttonValue(button) {
  return button?.value ?? 0;
}

function deadzone(value) {
  return Math.abs(value) < 0.12 ? 0 : value;
}

function finite(value) {
  return Number.isFinite(value) ? value : 0;
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

function readTriggers(pad, kind, target) {
  if (kind === 'standard') {
    // Standard axes 2/3 are the right stick, never trigger fallbacks.
    target.lt = buttonValue(pad.buttons[6]);
    target.rt = buttonValue(pad.buttons[7]);
    return;
  }
  // Raw Xbox buttons 6/7 are Back/Start, not LT/RT.
  target.lt = axisTrigger('lt', pad.axes[2]);
  target.rt = axisTrigger('rt', pad.axes[5]);
}

function zeroInput(target) {
  target.linear = 0;
  target.angular = 0;
  target.deadman = false;
  target.steer = 0;
  target.throttle = 0;
  target.mapping = '';
  target.stickX = 0;
  target.stickY = 0;
  target.lt = 0;
  target.rt = 0;
  target.lb = false;
  target.rb = false;
  target.a = false;
  target.b = false;
  target.x = false;
  target.y = false;
}

function readGamepad(target) {
  const pad = gamepad;
  const kind = resolveMapping(pad);
  syncTriggerState(pad);
  const stickX = pad.axes[0] ?? 0;
  const stickY = pad.axes[1] ?? 0;
  const steering = deadzone(stickX);
  readTriggers(pad, kind, target);
  const throttle = target.rt - target.lt;
  const lb = Boolean(pad.buttons[4]?.pressed);
  const rb = Boolean(pad.buttons[5]?.pressed);
  target.linear = throttle * MAX_LINEAR_MPS;
  // Browser axes are positive right; ROS yaw is positive left.
  target.angular = -steering * MAX_ANGULAR_RAD_S * preferences.sensitivity / 100;
  target.deadman = Math.abs(throttle) > 0 || steering !== 0;
  target.steer = steering;
  target.throttle = throttle;
  target.mapping = kind;
  target.stickX = stickX;
  target.stickY = stickY;
  target.lb = lb;
  target.rb = rb;
  target.a = Boolean(pad.buttons[0]?.pressed);
  target.b = Boolean(pad.buttons[1]?.pressed);
  target.x = Boolean(pad.buttons[2]?.pressed);
  target.y = Boolean(pad.buttons[3]?.pressed);
}

function readKeyboard(target = {}) {
  const throttle = (keys.has('KeyW') ? 1 : 0) - (keys.has('KeyS') ? 1 : 0);
  const steer = (keys.has('KeyD') ? 1 : 0) - (keys.has('KeyA') ? 1 : 0);
  const deadman = throttle !== 0 || steer !== 0;
  target.linear = throttle * MAX_LINEAR_MPS;
  target.angular = -steer * MAX_ANGULAR_RAD_S * preferences.sensitivity / 100;
  target.deadman = deadman;
  target.steer = steer;
  target.throttle = throttle;
  target.mapping = 'keyboard';
  target.stickX = steer;
  target.stickY = -throttle;
  target.lt = keys.has('KeyS') ? 1 : 0;
  target.rt = keys.has('KeyW') ? 1 : 0;
  target.lb = false;
  target.rb = false;
  target.a = false;
  target.b = false;
  target.x = false;
  target.y = false;
  return target;
}

function padActivity(pad) {
  let sum = 0;
  const buttons = pad.buttons;
  for (let i = 0; i < buttons.length; i++) {
    const item = buttons[i];
    sum += item.pressed ? 1 : (item.value || 0);
  }
  const axes = pad.axes;
  for (let i = 0; i < axes.length; i++) {
    const value = Math.abs(axes[i]);
    if (value > 0.2) sum += value;
  }
  return sum;
}

function pickGamepad() {
  const pads = visiblePads();
  if (!pads) return null;
  let best = null;
  let bestScore = -1;
  let previous = null;
  let xbox = null;
  for (let i = 0; i < pads.length; i++) {
    const pad = pads[i];
    if (!pad) continue;
    const score = padActivity(pad);
    if (score > bestScore) {
      best = pad;
      bestScore = score;
    }
    if (lastPadId && pad.id === lastPadId) previous = pad;
    if (!xbox && looksLikeXbox(pad.id)) xbox = pad;
  }
  if (best && bestScore > 0.2) return best;
  return previous || xbox || best;
}

function sendDriveIfNeeded(target, now) {
  if (!state.drive.you_control_owner) return;
  const linear = Math.round(finite(target.linear) * 1e4) / 1e4;
  const angular = Math.round(finite(target.angular) * 1e4) / 1e4;
  const deadman = Boolean(target.deadman);
  const changed =
    Math.abs(linear - lastSentDrive.linear) > DRIVE_EPSILON
    || Math.abs(angular - lastSentDrive.angular) > DRIVE_EPSILON
    || deadman !== lastSentDrive.deadman;
  const idle = !deadman && Math.abs(linear) <= DRIVE_EPSILON && Math.abs(angular) <= DRIVE_EPSILON;
  const since = now - lastDriveSent;
  if (changed) {
    if (since < DRIVE_MIN_INTERVAL_MS) return;
  } else if (idle || since < DRIVE_KEEPALIVE_MS) {
    return;
  }
  lastDriveSent = now;
  lastSentDrive.linear = linear;
  lastSentDrive.angular = angular;
  lastSentDrive.deadman = deadman;
  sendDrive(linear, angular, deadman);
}

function inputLoop() {
  requestAnimationFrame(inputLoop);
  const now = performance.now();
  const pads = visiblePads();
  gamepad = pads.find(pad => pad.index === selectedPadIndex && pad.id === selectedPadId) ?? (preferences.input === 'controller' ? pickGamepad() : null);

  const keyboardPreview = $('settings-dialog').open && document.activeElement === $('keyboard-test');
  if (preferences.input === 'keyboard' && (cameraFocused || keyboardPreview)) {
    readKeyboard(lastInput);
    rampKeyboard(lastInput, now);
  } else if (preferences.input === 'controller' && gamepad) {
    const stamp = gamepad.timestamp || 0;
    if (!stamp || stamp !== lastPadTimestamp || gamepad.id !== lastPadId) {
      lastPadTimestamp = stamp;
      readGamepad(lastInput);
    }
  } else if (preferences.input === 'wheel' && gamepad) {
    zeroInput(lastInput);
    Object.assign(lastInput, readWheel(gamepad, preferences.wheel), {mapping: 'wheel'});
  } else {
    zeroInput(lastInput);
    syncTriggerState(null);
    prevA = false;
    lastPadTimestamp = -1;
  }

  renderInputs(pads, lastInput);
  const canDrive = state.connected && state.safety?.allowed && state.drive.you_control_owner
    && !$('settings-dialog').open && !document.hidden && document.hasFocus();
  if (!canDrive && !keyboardPreview) resetKeyboardRamp(now);
  // Opening Settings already sends a stop. Do not flood calibration with rejected drive commands.
  if (!$('settings-dialog').open) sendDriveIfNeeded(canDrive && lastInput.deadman ? lastInput : {linear: 0, angular: 0, deadman: false}, now);

  if (now - lastPadUi >= PAD_UI_PERIOD_MS) {
    lastPadUi = now;
    updateInputHint();
  }

  if (noticeExpiry && now > noticeExpiry) {
    noticeExpiry = 0;
    $('notice').classList.add('hidden');
  }
}

function stopLocalInput() {
  keys.clear();
  resetKeyboardRamp();
  zeroInput(lastInput);
  lastPadTimestamp = -1;
  activeSliders.clear();
}

$('take-control').onclick = () => send({type: 'take_control'});

for (const [id, type, field] of [['probe-slider', 'probe_target', 'depth_mm'], ['sweep-speed', 'sweep_speed', 'deg_s']]) {
  const commit = event => {
    if (state.connected && state.drive.you_control_owner) send({type, [field]: Number(event.target.value)});
  };
  $(id).addEventListener('pointerdown', () => activeSliders.add(id));
  $(id).addEventListener('keydown', () => activeSliders.add(id));
  $(id).addEventListener('blur', () => activeSliders.delete(id));
  $(id).addEventListener('pointercancel', () => activeSliders.delete(id));
  $(id).oninput = throttle(70, commit);
  // Always deliver the released value, even inside the throttle window.
  $(id).onchange = event => { activeSliders.delete(id); commit(event); };
}
$('sweep-toggle').onclick = () => send({type: 'sweep_enabled', enabled: !state.detector.sweep_enabled});

function savePreferences() {
  try { localStorage.setItem('peaceofmine.operator.preferences', JSON.stringify(preferences)); } catch {}
  $('control-source').value = preferences.input;
  $('pad-mapping').value = preferences.mapping;
  $('steering-sensitivity').value = preferences.sensitivity;
  $('sensitivity-value').textContent = `${preferences.sensitivity}%`;
  $('pad-mapping').disabled = preferences.input !== 'controller';
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
  lastPadTimestamp = -1;
  savePreferences();
};
$('pad-mapping').onchange = event => {
  stopInput();
  preferences.mapping = event.target.value;
  triggerSeen.lt = false;
  triggerSeen.rt = false;
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
  stopInput();
  updateInputHint();
}

for (const preview of [forward, $('forward-video'), $('forward-network-camera')]) {
  preview.addEventListener('focus', activateKeyboardInput);
  preview.addEventListener('blur', deactivateKeyboardInput);
}

// Keydrown tracks held keys without relying on OS key-repeat timing.
for (const [name, code] of [['W', 'KeyW'], ['A', 'KeyA'], ['S', 'KeyS'], ['D', 'KeyD']]) {
  kd[name].press(event => {
    if (preferences.input !== 'keyboard' || !(cameraFocused || document.activeElement === $('keyboard-test'))) return;
    if (event?.repeat) return;
    event?.preventDefault();
    keys.add(code);
  });
  kd[name].up(() => {
    keys.delete(code);
    if (name === 'SHIFT') {
      resetKeyboardRamp();
      if (preferences.input === 'keyboard' && state.drive.you_control_owner) sendDrive(0, 0, false);
    }
  });
}

function resetKeyboardRamp(now = performance.now()) {
  keyboardRamp.throttle = keyboardRamp.steer = 0;
  keyboardRamp.updatedAt = now;
}

function rampKeyboard(target, now) {
  const dt = Math.max(0, Math.min(.05, (now - keyboardRamp.updatedAt) / 1000));
  keyboardRamp.updatedAt = now;
  if (!target.deadman) {
    resetKeyboardRamp(now);
    target.linear = target.angular = 0;
    return;
  }
  const approach = (value, desired, rise, fall) => {
    const slowing = desired === 0 || (value !== 0 && Math.sign(value) !== Math.sign(desired));
    const step = (slowing ? fall : rise) * dt;
    return value + Math.max(-step, Math.min(step, desired - value));
  };
  keyboardRamp.throttle = approach(keyboardRamp.throttle, target.throttle, 1.6, 2.8);
  keyboardRamp.steer = approach(keyboardRamp.steer, target.steer, 3.2, 4.8);
  target.linear = keyboardRamp.throttle * MAX_LINEAR_MPS;
  target.angular = -keyboardRamp.steer * MAX_ANGULAR_RAD_S * preferences.sensitivity / 100;
}

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
  if (forward.classList.contains('hidden')) return;
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

}

function drawMap() {
  const context = resize(map);
  const width = map.clientWidth;
  const height = map.clientHeight;
  const crossHalf = 6;
  const scale = Math.min(width / (LANE_END_M - LANE_START_M), height / (2 * crossHalf));
  // +X runs left to right, +Y runs up the screen.
  const centreX = (LANE_START_M + LANE_END_M) / 2;
  const toScreen = (x, y) => ({x: width / 2 + (x - centreX) * scale, y: height / 2 - y * scale});
  const leftX = Math.floor(centreX - width / (2 * scale));
  const rightX = Math.ceil(centreX + width / (2 * scale));

  context.clearRect(0, 0, width, height);
  context.fillStyle = '#101f20';
  context.fillRect(0, 0, width, height);

  context.strokeStyle = '#244244';
  context.lineWidth = 1;
  for (let x = leftX; x <= rightX; x += 1) {
    drawSegment(context, toScreen(x, -crossHalf), toScreen(x, crossHalf));
  }
  for (let y = -crossHalf; y <= crossHalf; y += 1) {
    drawSegment(context, toScreen(leftX, y), toScreen(rightX, y));
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

}

// Fixture angles are counter-clockwise in the world, which is counter-clockwise
// on screen too once the canvas y flip is accounted for.
function beamScreenAngle(degrees) {
  return -Math.PI / 2 - degrees * Math.PI / 180;
}

function radarGeometry(width, height, minimum, maximum) {
  const angles = [minimum, maximum, 0];
  for (let angle = Math.ceil(minimum / 90) * 90; angle <= maximum; angle += 90) angles.push(angle);
  const xs = [0, ...angles.map(angle => Math.cos(beamScreenAngle(angle)))];
  const ys = [0, ...angles.map(angle => Math.sin(beamScreenAngle(angle)))];
  const left = Math.min(...xs), right = Math.max(...xs);
  const top = Math.min(...ys), bottom = Math.max(...ys);
  const radius = Math.max(1, Math.min((width - 20) / (right - left || 1), (height - 20) / (bottom - top || 1)));
  return {radius, cx: (width - radius * (right + left)) / 2,
    cy: (height - radius * (bottom + top)) / 2};
}

function drawRadar() {
  const canvas = $('radar-view');
  const context = resize(canvas);
  const width = canvas.clientWidth;
  const height = canvas.clientHeight;
  const {beam, beam_half_angle_deg: half} = state.detector;
  const minimum = state.detector.beam_min_angle_deg ?? -half;
  const maximum = state.detector.beam_max_angle_deg ?? half;
  const {cx, cy, radius} = radarGeometry(width, height, minimum, maximum);

  context.clearRect(0, 0, width, height);
  context.strokeStyle = '#265148';
  context.lineWidth = 1;
  for (const factor of [0.33, 0.66, 1]) {
    context.beginPath();
    context.arc(cx, cy, radius * factor, beamScreenAngle(maximum), beamScreenAngle(minimum));
    context.stroke();
  }
  for (const edge of [minimum, 0, maximum]) {
    const angle = beamScreenAngle(edge);
    drawSegment(context, {x: cx, y: cy}, {x: cx + Math.cos(angle) * radius, y: cy + Math.sin(angle) * radius});
  }

  // Trace bins span the calibrated sector, including asymmetric endpoints.
  context.lineWidth = 8;
  for (let index = 0; index < beam.length - 1; index++) {
    const ratio = (beam[index] + beam[index + 1]) / 2;
    const step = (maximum - minimum) / (beam.length - 1);
    const from = beamScreenAngle(minimum + (index + 1) * step);
    const to = beamScreenAngle(minimum + index * step);
    context.strokeStyle = detectorColor(ratio);
    context.beginPath();
    context.arc(cx, cy, radius * 0.84, from, to);
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
      !Number.isFinite(mapping.deadzone) || mapping.deadzone < 0 || mapping.deadzone >= 1) return zero;
  const pedal = (key, inverted) => Math.max(0, Math.min(1, (pad.axes[mapping[key]] * (inverted ? -1 : 1) + 1) / 2));
  const raw = pad.axes[mapping.steering] * (mapping.invertSteering ? -1 : 1);
  const steering = Math.sign(raw) * Math.max(0, Math.min(1, (Math.abs(raw) - mapping.deadzone) / (1 - mapping.deadzone)));
  return {linear: (pedal('throttle', mapping.invertThrottle) - pedal('brake', mapping.invertBrake)) * MAX_LINEAR_MPS,
    angular: -steering * MAX_ANGULAR_RAD_S * preferences.sensitivity / 100,
    deadman: true};
}

function stopInput() {
  stopLocalInput();
  if (state.drive.you_control_owner) {
    send({type: 'drive', linear_x: 0, angular_z: 0, deadman: false});
    send({type: 'estop'});
  }
}

function stopCameraStream(slot) {
  cameraGeneration[slot] += 1;
  clearTimeout(cameraRetries[slot]);
  cameraRetries[slot] = null;
  cameraSources[slot] = null;
  cameraStreams[slot]?.getTracks().forEach(track => track.stop());
  cameraStreams[slot] = null;
  const video = slot === 'forward' ? $('forward-video') : $('aux-video');
  video.srcObject = null;
  const networkImage = slot === 'forward' ? $('forward-network-camera') : $('aux-network-camera');
  networkImage.onload = networkImage.onerror = null;
  networkImage.removeAttribute('src');
}

async function startCamera(slot, source, force = false) {
  if (!force && cameraSources[slot] === source) return;
  stopCameraStream(slot);
  cameraSources[slot] = source;
  cameraErrors[slot] = '';
  const generation = cameraGeneration[slot];
  const current = () => generation === cameraGeneration[slot];
  updateCameraVisibility();
  if (source === 'virtual' || source === 'off') return;
  if (source.startsWith('raspberry:')) {
    const networkImage = slot === 'forward' ? $('forward-network-camera') : $('aux-network-camera');
    networkImage.onerror = () => {
      if (!current()) return;
      cameraErrors[slot] = 'STREAM OFFLINE · retrying';
      $('camera-source-status').textContent = 'Camera stream unavailable. Retrying automatically…';
      updateCameraLatency();
      clearTimeout(cameraRetries[slot]);
      cameraRetries[slot] = setTimeout(() => startCamera(slot, source, true), 1500);
    };
    networkImage.onload = () => {
      if (!current()) return;
      cameraErrors[slot] = '';
      updateCameraLatency();
    };
    networkImage.src = `/camera/${encodeURIComponent(source.slice(10))}?t=${Date.now()}`;
    return;
  }
  try {
    if (!navigator.mediaDevices?.getUserMedia) throw Error('Laptop cameras need HTTPS or localhost.');
    const stream = await navigator.mediaDevices.getUserMedia({
      audio: false,
      video: {deviceId: {exact: source}, width: {ideal: 640}, height: {ideal: 480}, frameRate: {ideal: 30}},
    });
    if (!current()) {
      stream.getTracks().forEach(track => track.stop());
      return;
    }
    cameraStreams[slot] = stream;
    const video = slot === 'forward' ? $('forward-video') : $('aux-video');
    video.srcObject = stream;
    await video.play();
    if (!current()) return;
    stream.getVideoTracks().forEach(track => track.addEventListener('ended', () => {
      if (current()) startCamera(slot, source, true);
    }));
    $('camera-source-status').textContent = 'Camera previews are live on this browser.';
  } catch (error) {
    if (!current()) return;
    stopCameraStream(slot);
    cameraErrors[slot] = 'LOCAL CAMERA UNAVAILABLE';
    $('camera-source-status').textContent = `Could not open camera: ${error.message}`;
  }
  updateCameraVisibility();
}

function updateCameraVisibility() {
  const forwardIsNetwork = preferences.cameras.forward.startsWith('raspberry:');
  const forwardIsVirtual = preferences.cameras.forward === 'virtual';
  forward.classList.toggle('hidden', !forwardIsVirtual);
  $('forward-video').classList.toggle('hidden', forwardIsVirtual || forwardIsNetwork);
  $('forward-network-camera').classList.toggle('hidden', !forwardIsNetwork);
  $('forward-camera-label').textContent = forwardIsVirtual ? 'VIRTUAL FORWARD CAMERA' : forwardIsNetwork ? 'RASPBERRY PI CAMERA' : 'LOCAL CAMERA';
  $('forward-camera-latency').classList.toggle('hidden', !forwardIsNetwork && !cameraErrors.forward);
  const auxiliaryIsNetwork = preferences.cameras.auxiliary.startsWith('raspberry:');
  const auxiliaryIsLocal = preferences.cameras.auxiliary !== 'off' && Boolean(cameraStreams.auxiliary);
  $('aux-video').classList.toggle('hidden', !auxiliaryIsLocal || auxiliaryIsNetwork);
  $('aux-network-camera').classList.toggle('hidden', !auxiliaryIsNetwork);
  $('aux-camera-label').classList.toggle('hidden', !auxiliaryIsNetwork && !auxiliaryIsLocal);
  $('aux-camera-latency').classList.toggle('hidden', !auxiliaryIsNetwork && !cameraErrors.auxiliary);
  updateCameraLatency();
  applyCameraRotations();
}

function updateCameraLatency() {
  const render = (element, camera, slot) => {
    if (cameraErrors[slot]) {
      element.textContent = cameraErrors[slot];
      element.classList.add('stale');
      return;
    }
    if (!camera) {
      element.textContent = `NO FRAMES · LINK ${linkLatencyMs ?? '--'} ms`;
      element.classList.add('stale');
      return;
    }
    const age = Math.max(0, Math.round(camera.frame_age_ms));
    const fps = Number(camera.fps || 0).toFixed(1);
    element.textContent = `${fps} SOURCE FPS · SOURCE AGE ${age} ms · RTT ${linkLatencyMs ?? '--'} ms`;
    element.classList.toggle('stale', age > 500);
  };
  const selected = source => source.startsWith('raspberry:') ? state.cameras?.[source.slice(10)] : null;
  render($('forward-camera-latency'), selected(preferences.cameras.forward), 'forward');
  render($('aux-camera-latency'), selected(preferences.cameras.auxiliary), 'auxiliary');
}

function rebuildCameraOptions() {
  for (const [slot, id, fallback, label] of [
    ['forward', 'forward-camera-source', 'virtual', 'Virtual forward camera'],
    ['auxiliary', 'aux-camera-source', 'off', 'Off'],
  ]) {
    const select = $(id);
    const selected = preferences.cameras[slot];
    select.replaceChildren(new Option(label, fallback));
    Object.entries(state.cameras || {}).forEach(([id, camera]) =>
      select.add(new Option(`Raspberry Pi · ${camera.label}`, `raspberry:${id}`)));
    localCameras.filter(camera => camera.deviceId).forEach((camera, index) =>
      select.add(new Option(camera.label || `Laptop camera ${index + 1}`, camera.deviceId)));
    if (![...select.options].some(option => option.value === selected)) {
      select.add(new Option(`Unavailable · ${selected.startsWith('raspberry:') ? selected.slice(10) : 'saved laptop camera'}`, selected));
    }
    select.value = selected;
  }
}

function updateNetworkCameraOptions() {
  const signature = JSON.stringify(Object.entries(state.cameras || {}).map(([id, camera]) => [id, camera.topic, camera.label]));
  if (signature === networkCameraSignature) return;
  networkCameraSignature = signature;
  rebuildCameraOptions();
  for (const slot of ['forward', 'auxiliary']) {
    if (preferences.cameras[slot].startsWith('raspberry:')) startCamera(slot, preferences.cameras[slot]);
  }
}

async function refreshCameraDevices(requestPermission = false) {
  const refresh = ++cameraDeviceRefresh;
  rebuildCameraOptions();
  if (!navigator.mediaDevices?.enumerateDevices) {
    $('refresh-cameras').disabled = true;
    $('camera-source-status').textContent = 'Raspberry Pi cameras work here. Laptop cameras need HTTPS or localhost.';
    return;
  }
  let permissionStream;
  try {
    if (requestPermission) {
      permissionStream = await navigator.mediaDevices.getUserMedia({video: true, audio: false});
      // Release the temporary capture before opening selected cameras.
      permissionStream.getTracks().forEach(track => track.stop());
    }
    const devices = await navigator.mediaDevices.enumerateDevices();
    if (refresh !== cameraDeviceRefresh) return;
    localCameras = devices.filter(device => device.kind === 'videoinput');
    rebuildCameraOptions();
    $('camera-source-status').textContent = `${Object.keys(state.cameras || {}).length} Raspberry Pi camera source(s) · ${localCameras.length} laptop camera(s).`;
    for (const slot of ['forward', 'auxiliary']) {
      const source = preferences.cameras[slot];
      if (localCameras.some(camera => camera.deviceId === source && camera.label)) await startCamera(slot, source);
    }
  } catch (error) {
    if (refresh === cameraDeviceRefresh) $('camera-source-status').textContent = `Camera permission failed: ${error.message}`;
  } finally {
    permissionStream?.getTracks().forEach(track => track.stop());
  }
}

function selectedCameraElement(slot) {
  const source = preferences.cameras[slot];
  if (source === 'off') return null;
  if (source === 'virtual') return forward;
  return $(slot === 'forward'
    ? (source.startsWith('raspberry:') ? 'forward-network-camera' : 'forward-video')
    : (source.startsWith('raspberry:') ? 'aux-network-camera' : 'aux-video'));
}

function applyCameraRotations() {
  for (const slot of ['forward', 'auxiliary']) {
    const angle = preferences.cameraRotation[slot];
    const elements = slot === 'forward' ? [forward, $('forward-video'), $('forward-network-camera')]
      : [$('aux-video'), $('aux-network-camera')];
    for (const element of elements) {
      const w = element.clientWidth, h = element.clientHeight;
      const scale = angle % 180 && w && h ? Math.min(w / h, h / w) : 1;
      element.style.transform = `rotate(${angle}deg) scale(${scale})`;
    }
    $(slot + '-rotation').textContent = `${angle}°`;
    $(slot + '-rotate').disabled = preferences.cameras[slot] === 'off';
  }
}

function cameraDetails(slot) {
  const source = preferences.cameras[slot];
  const element = selectedCameraElement(slot);
  const network = source.startsWith('raspberry:');
  const camera = network ? state.cameras?.[source.slice(10)] : null;
  const track = cameraStreams[slot]?.getVideoTracks()[0];
  const settings = track?.getSettings?.() || {};
  const virtual = source === 'virtual';
  const width = element?.videoWidth || element?.naturalWidth || (virtual ? element.width : 0) || camera?.width || settings.width || 0;
  const height = element?.videoHeight || element?.naturalHeight || (virtual ? element.height : 0) || camera?.height || settings.height || 0;
  const age = camera ? Math.round(camera.frame_age_ms + Math.max(0, performance.now() - lastTelemetryAt)) : null;
  let status = source === 'off' ? 'Off' : virtual ? 'Simulation' : 'Waiting';
  if (network && camera && state.connected) status = age > 1000 ? 'Stale' : 'Live';
  if (!network && track?.readyState === 'live') status = 'Live';
  if (network && !state.connected) status = 'Disconnected';
  if (cameraErrors[slot]) status = 'Unavailable';
  const drawable = virtual || (network ? Boolean(element?.naturalWidth) : element?.readyState >= 2);
  return {element, width, height, status, drawable,
    fps: network ? (camera ? `${Number(camera.fps).toFixed(1)} fps (source)` : '—')
      : virtual ? '30 fps target' : settings.frameRate ? `${Number(settings.frameRate).toFixed(1)} fps (configured)` : '—',
    age: age === null ? '—' : `${age} ms`,
    kind: network ? 'Raspberry Pi · MJPEG' : virtual ? 'Virtual camera' : source === 'off' ? 'None' : 'Browser camera',
    detail: network ? camera?.topic || source.slice(10) : virtual ? 'Rendered simulation · no physical camera' : track?.label || (source === 'off' ? 'Select a camera to enable the inset.' : 'Waiting for camera permission or a connected device.'),
  };
}

function renderCameraSettings() {
  if (!$('settings-dialog').open || $('settings-cameras').hidden) return;
  for (const slot of ['forward', 'auxiliary']) {
    const details = cameraDetails(slot);
    $(slot + '-preview-state').textContent = details.status;
    $(slot + '-preview-state').dataset.state = details.status.toLowerCase();
    $(slot + '-dimensions').textContent = details.width && details.height ? `${details.width} × ${details.height} px` : '—';
    $(slot + '-fps').textContent = details.fps;
    $(slot + '-age').textContent = details.age;
    $(slot + '-source-kind').textContent = details.kind;
    $(slot + '-source-detail').textContent = details.detail;
    $(slot + '-camera-retry').disabled = ['off', 'virtual'].includes(preferences.cameras[slot]);
    const canvas = $(slot + '-settings-preview');
    const ctx = canvas.getContext('2d');
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    const empty = $(slot + '-preview-empty');
    empty.textContent = cameraErrors[slot] || (details.status === 'Off' ? 'Inset camera is off' : `${details.status} · waiting for frames`);
    empty.hidden = details.drawable && !['Unavailable', 'Disconnected', 'Stale'].includes(details.status);
    if (!empty.hidden || !details.width || !details.height) continue;
    const angle = preferences.cameraRotation[slot];
    const sideways = angle % 180 !== 0;
    const scale = Math.min(canvas.width / (sideways ? details.height : details.width), canvas.height / (sideways ? details.width : details.height));
    ctx.save();
    ctx.translate(canvas.width / 2, canvas.height / 2);
    ctx.rotate(angle * Math.PI / 180);
    try {
      ctx.drawImage(details.element, -details.width * scale / 2, -details.height * scale / 2, details.width * scale, details.height * scale);
    } catch { empty.hidden = false; }
    ctx.restore();
  }
}

function initializeCameraSettings() {
  for (const slot of ['forward', 'auxiliary']) {
    $(slot + '-rotate').onclick = () => {
      preferences.cameraRotation[slot] = (preferences.cameraRotation[slot] + 90) % 360;
      savePreferences();
      applyCameraRotations();
      renderCameraSettings();
    };
    $(slot + '-camera-retry').onclick = () => startCamera(slot, preferences.cameras[slot], true);
  }
  new ResizeObserver(applyCameraRotations).observe($('camera-panel'));
  let lastFrame = 0;
  function previewLoop(now) {
    if (!document.hidden && now - lastFrame >= 100) {
      lastFrame = now;
      renderCameraSettings();
    }
    requestAnimationFrame(previewLoop);
  }
  requestAnimationFrame(previewLoop);
}

let armHoldTimer = null;
let armSliderMoving = false;
let armSliderDragging = false;
let armSliderTarget = null;
let armPending = null;
let armRequestSequence = 0;
function setArmFeedback(text, kind = 'pending') {
  $('arm-feedback').textContent = text;
  $('arm-feedback').dataset.kind = kind;
}
function armCommand(action, values = {}, feedback = true) {
  const payload = {type: 'arm_servo', action, ...values};
  if (feedback) {
    payload.request_id = `${Date.now()}-${++armRequestSequence}`;
    armPending = {id: payload.request_id, at: performance.now()};
    setArmFeedback(({select: 'Selecting servo…', capture: 'Recording position…', configure: 'Applying limits…', stop: 'Stopping…', jog: 'Starting jog…', position: 'Moving to slider target…', move: 'Starting movement…'})[action] || 'Sending command…');
  }
  send(payload);
}
function armControlCommand(payload, confirmation, predicate) {
  armPending = {at: performance.now(), predicate, confirmation};
  setArmFeedback('Waiting for dashboard confirmation…');
  send(payload);
}
function armAngle(position) {
  return Number.isFinite(position) ? `${(position * 360 / 4096).toFixed(1)}°` : '—';
}
function stopArmMove() {
  armSliderMoving = false;
  if (armHoldTimer !== null) {
    clearInterval(armHoldTimer);
    armHoldTimer = null;
    armCommand('stop');
  }
}
function updateArmSettings() {
  const servo = state.arm_servo || {};
  const servoSelect = $('arm-servo-id');
  const role = servo.active_role || 'arm';
  const roles = servo.role_ids || {arm: servo.servo_id, probe: 2};
  const roleSelect = $('arm-servo-role');
  roleSelect.value = role;
  const ids = servo.discovered_servos || [];
  for (const option of roleSelect.options) {
    const id = roles[option.value];
    option.textContent = `${option.value === 'arm' ? 'Arm' : 'Probe'} servo${id >= 0 ? ` · ID ${id}` : ''}${ids.includes(id) ? '' : ' · Not connected'}`;
  }
  const signature = JSON.stringify([ids, Boolean(servo.scanning), role, servo.servo_id]);
  if (servoSelect.dataset.discovery !== signature) {
    const previous = servoSelect.value;
    servoSelect.replaceChildren(new Option(servo.scanning ? 'Scanning for servos…' : 'Select a detected servo', ''));
    ids.forEach(id => servoSelect.add(new Option(`Servo ${id}`, String(id))));
    servoSelect.value = ids.includes(servo.servo_id) ? String(servo.servo_id)
      : ids.includes(Number(previous)) && previous !== '' ? previous : '';
    servoSelect.dataset.discovery = signature;
  }
  const owner = state.connected && state.drive.you_control_owner;
  roleSelect.disabled = !owner || !servo.serial_connected || Boolean(servo.torque) || armHoldTimer !== null;
  $('arm-reconnect').disabled = !owner || Boolean(servo.scanning || servo.torque || servo.sweeping) || armHoldTimer !== null;
  if (armPending) {
    const acknowledgement = servo.last_command;
    if (armPending.id && acknowledgement?.request_id === armPending.id) {
      setArmFeedback(acknowledgement.message, acknowledgement.success ? 'success' : 'error');
      armPending = null;
    } else if (armPending.predicate?.()) {
      setArmFeedback(armPending.confirmation, 'success'); armPending = null;
    } else if (performance.now() - armPending.at > 4000) {
      setArmFeedback('No confirmation received. Check the driver connection and retry.', 'error'); armPending = null;
    }
  }
  const ready = owner && state.safety?.servo_allowed && servo.connected;
  const slider = $('arm-position-slider');
  if (servo.connected) {
    slider.min = servo.eeprom_minimum ?? 0;
    slider.max = servo.eeprom_maximum ?? 4095;
    if (!armSliderDragging) slider.value = servo.position;
  }
  slider.disabled = !ready || !state.drive.calibrating || (armHoldTimer !== null && !armSliderMoving);
  if (armSliderMoving && !armSliderDragging && servo.torque && Math.abs(servo.position - armSliderTarget) <= 3
      && servo.goal_position === armSliderTarget) stopArmMove();
  $('arm-position-label').textContent = servo.connected ? armAngle(Number(slider.value)) : '—';
  $('arm-jog-speed').disabled = armHoldTimer !== null && !armSliderMoving;

  $('arm-servo-status').textContent = servo.reason || 'Arm driver not running';
  $('arm-servo-position').textContent = servo.connected ? `${armAngle(servo.position)} (${servo.position ?? '—'} ticks)` : servo.serial_connected ? 'Adapter connected · no servo selected' : 'Disconnected';
  $('arm-servo-details').textContent = servo.connected ? `Servo ${servo.servo_id} · ${servo.serial_port || 'serial adapter'}` : servo.serial_connected ? 'Select the ID of the arm servo, then click Select servo.' : servo.serial_port ? `Arm serial port: ${servo.serial_port}` : 'Start hardware launch with use_arm_servo:=true and arm_serial_port:=<adapter path>.';
  const number = (value, digits = 1, unit = '') => Number.isFinite(value) ? `${value.toFixed(digits)}${unit}` : '—';
  const metric = (id, value) => { $(id).textContent = servo.connected ? value : '—'; };
  metric('arm-goal', armAngle(servo.goal_position));
  metric('arm-torque', typeof servo.torque === 'boolean' ? (servo.torque ? 'On' : 'Off') : '—');
  metric('arm-load', number(servo.load_percent, 1, ' %'));
  metric('arm-torque-limit', number(servo.torque_limit_percent, 1, ' %'));
  metric('arm-max-torque', number(servo.max_torque_percent, 1, ' %'));
  metric('arm-current', number(servo.current_a, 3, ' A'));
  metric('arm-speed', `${number(servo.speed_deg_s, 1, '°/s')} / ${servo.speed_limit_deg_s === 0 ? 'Full speed' : number(servo.speed_limit_deg_s, 1, '°/s')}`);
  metric('arm-moving', typeof servo.moving === 'boolean' ? (servo.moving ? 'Yes' : 'No') : '—');
  metric('arm-power', `${number(servo.voltage, 1, ' V')} / ${number(servo.temperature_c, 0, ' °C')}`);
  metric('arm-model', `${servo.model ?? '—'} / ${servo.firmware ?? '—'}`);
  $('arm-px4').textContent = state.safety?.servo_allowed ? 'Movement permitted (status 4)' : 'Movement blocked';
  $('arm-calibration-enable').disabled = !owner || !state.safety?.servo_allowed || state.drive.calibrating || !servo.connected;
  $('arm-calibration-stop').disabled = !owner;
  $('arm-calibration-enable').textContent = state.drive.calibrating ? 'Jogging enabled' : 'Enable jogging';
  const selectionBlocked = !state.connected ? 'Connect to the dashboard first.'
    : !owner ? 'Take control to select a servo or record calibration. You do not need to arm.'
    : servo.torque ? 'Stop the arm before changing the servo ID.'
    : !servo.serial_connected ? 'Waiting for the arm serial adapter.' : '';
  $('arm-calibration-access').textContent = owner && state.drive.calibrating ? 'Hold a jog button to move. Release it, then record the position.' : selectionBlocked || 'Ready to select a servo and record positions.';
  $('arm-take-control').hidden = owner;
  $('arm-take-control').disabled = !state.connected;
  $('arm-select-id').disabled = Boolean(selectionBlocked) || servoSelect.value === '';
  $('arm-select-id').title = selectionBlocked || 'Select the servo ID; position updates automatically.';
  const canRecord = owner && servo.connected && !servo.torque && armHoldTimer === null;
  $('arm-apply-motion').disabled = !canRecord;
  const motionSignature = JSON.stringify([servo.motion_speed_limit, servo.sweep_acceleration_deg_s2]);
  if ($('arm-apply-motion').dataset.profile !== motionSignature) {
    $('arm-max-speed').value = ((servo.motion_speed_limit || 80) * .684).toFixed(3);
    $('arm-acceleration').value = (servo.sweep_acceleration_deg_s2 || 40).toFixed(3);
    $('arm-apply-motion').dataset.profile = motionSignature;
  }
  $('arm-apply-limits').disabled = !canRecord || !['minimum', 'center', 'maximum'].every(point => Number.isFinite(servo.captured?.[point] ?? servo.calibration?.[point]));
  $('arm-jog-left').disabled = $('arm-jog-right').disabled = !ready || !state.drive.calibrating;
  const jogReason = !state.connected ? 'Dashboard disconnected'
    : !owner ? 'Take control first'
    : !servo.connected ? 'Servo not connected'
    : !state.safety?.servo_allowed ? 'Blocked: PX4 must report status 4'
    : !state.drive.calibrating ? 'Click Enable jogging once, then hold an arrow'
    : armHoldTimer !== null ? 'Jog command held · release to stop'
    : 'Ready · hold left or right to move';
  $('arm-jog-status').textContent = jogReason;
  $('arm-jog-status').dataset.ready = String(Boolean(ready && state.drive.calibrating));
  $('arm-jog-left').title = $('arm-jog-right').title = jogReason;
  for (const point of ['minimum', 'center', 'maximum']) {
    $(`arm-capture-${point}`).disabled = !canRecord;
    $(`arm-move-${point}`).disabled = !ready || !servo.calibration;
    $(`arm-recorded-${point}`).textContent = armAngle(servo.captured?.[point] ?? servo.calibration?.[point]);
  }
  if (!ready) stopArmMove();
  if (servo.calibration) {
    const escape = value => String(value).replaceAll('&', '&amp;').replaceAll('"', '&quot;').replaceAll('<', '&lt;');
    $('arm-launch-xml').value = [
      '<arg name="use_arm_servo" default="true"/>',
      `<arg name="arm_serial_port" default="${escape(servo.serial_port || '')}"/>`,
      `<arg name="${role}_servo_id" default="${servo.servo_id}"/>`,
      ...['minimum', 'center', 'maximum'].map(point => `<arg name="${role}_${point}" default="${servo.calibration[point]}"/>`),
      `<arg name="arm_speed" default="${servo.motion_speed_limit || 20}"/>`,
      `<arg name="arm_sweep_endpoint_tolerance_deg" default="${servo.sweep_endpoint_tolerance_deg ?? 2.0}"/>`,
      `<arg name="arm_sweep_acceleration_deg_s2" default="${servo.sweep_acceleration_deg_s2 || 40}"/>`,
    ].join('\n');
  } else $('arm-launch-xml').value = 'Record and apply all three positions to generate launch XML.';
}
function initializeArmSettings() {
  $('arm-apply-motion').onclick = () => {
    if (!$('arm-max-speed').reportValidity() || !$('arm-acceleration').reportValidity()) return;
    armCommand('configure_motion', {max_speed_deg_s: Number($('arm-max-speed').value),
      acceleration_deg_s2: Number($('arm-acceleration').value)});
  };
  $('arm-reconnect').onclick = () => armCommand('reconnect');
  $('arm-servo-role').onchange = () => {
    stopArmMove();
    armCommand('select_role', {role: $('arm-servo-role').value});
  };
  $('arm-take-control').onclick = () => armControlCommand({type: 'take_control'}, 'Control acquired. Select a servo or enable slow jogging.', () => state.drive.you_control_owner);
  $('arm-servo-id').onchange = updateArmSettings;
  $('arm-select-id').onclick = () => {
    if ($('arm-servo-id').value !== '') armCommand('select', {servo_id: Number($('arm-servo-id').value)});
  };
  $('arm-calibration-enable').onclick = () => armControlCommand({type: 'arm', calibration: true}, 'Slow jogging enabled. Hold left or right; release to stop.', () => state.drive.calibrating);
  $('arm-calibration-stop').onclick = () => { stopArmMove(); armCommand('stop'); send({type: 'estop'}); };
  function bindHold(button, action, values) {
    const start = () => {
      if (button.disabled) return;
      stopArmMove();
      const selected = typeof values === 'function' ? values() : values;
      if (action === 'jog' && (['arm-jog-speed'].some(id => !$(id).checkValidity() || !Number.isFinite(Number($(id).value)) || Number($(id).value) <= 0))) {
        setArmFeedback('Enter a positive jog speed.', 'error'); return;
      }
      const held = {...selected, held: true};
      armCommand(action, held);
      armHoldTimer = setInterval(() => armCommand(action, held, false), 80);
      updateArmSettings();
    };
    button.onpointerdown = event => { event.preventDefault(); button.setPointerCapture(event.pointerId); start(); };
    button.onpointerup = button.onpointercancel = button.onlostpointercapture = stopArmMove;
    button.onkeydown = event => { if ([' ', 'Enter'].includes(event.key) && !event.repeat) { event.preventDefault(); start(); } };
    button.onkeyup = event => { if ([' ', 'Enter'].includes(event.key)) stopArmMove(); };
    button.onblur = stopArmMove;
  }
  const jogOptions = () => ({speed_deg_s: Number($('arm-jog-speed').value)});
  bindHold($('arm-jog-left'), 'jog', () => ({direction: -1, ...jogOptions()}));
  bindHold($('arm-jog-right'), 'jog', () => ({direction: 1, ...jogOptions()}));
  const positionSlider = $('arm-position-slider');
  positionSlider.onpointerdown = () => { armSliderDragging = true; };
  positionSlider.onpointerup = positionSlider.onpointercancel = positionSlider.onlostpointercapture = () => {
    armSliderDragging = false; updateArmSettings();
  };
  positionSlider.onkeydown = event => {
    if (['ArrowLeft', 'ArrowRight', 'ArrowUp', 'ArrowDown', 'Home', 'End', 'PageUp', 'PageDown'].includes(event.key)) armSliderDragging = true;
  };
  positionSlider.onkeyup = positionSlider.onblur = () => { armSliderDragging = false; updateArmSettings(); };
  positionSlider.oninput = () => {
    if (positionSlider.disabled) return;
    armSliderTarget = Number(positionSlider.value);
    $('arm-position-label').textContent = armAngle(armSliderTarget);
    const target = () => ({position: armSliderTarget, held: true});
    if (armSliderMoving) {
      armCommand('position', target(), false);
      return;
    }
    stopArmMove();
    armSliderMoving = true;
    armCommand('position', target());
    armHoldTimer = setInterval(() => armCommand('position', target(), false), 80);
    updateArmSettings();
  };
  for (const point of ['minimum', 'center', 'maximum']) {
    $(`arm-capture-${point}`).onclick = () => armCommand('capture', {point});
    bindHold($(`arm-move-${point}`), 'move', {point});
  }
  $('arm-apply-limits').onclick = () => {
    const servo = state.arm_servo || {};
    const calibration = Object.fromEntries(['minimum', 'center', 'maximum'].map(point => [point, servo.captured?.[point] ?? servo.calibration?.[point]]));
    armCommand('configure', calibration);
  };
  const releaseArmSettings = () => {
    stopArmMove();
    if (state.drive.you_control_owner) {
      armCommand('stop');
      send({type: 'estop'});
    }
  };
  $('settings-dialog').querySelector('form').addEventListener('submit', releaseArmSettings);
  $('settings-dialog').addEventListener('close', releaseArmSettings);
  window.addEventListener('blur', stopArmMove);
  document.addEventListener('visibilitychange', () => { if (document.hidden) stopArmMove(); });
}

let metalPending = null;
let metalFeedback = '';
let metalCalibrationKey = '';
let metalPacketKey = '';
const sensorReferenceVoltage = 5.0;
const metalPackets = [];

function updateMetalSettings() {
  const sensor = state.detector.sensor || {};
  const available = state.connected && sensor.available;
  const fresh = available && sensor.fresh;
  const owner = state.connected && state.drive.you_control_owner;
  $('metal-status').textContent = !state.connected ? 'DISCONNECTED' : !available
    ? (state.detector.fresh ? 'SIMULATED / NO RAW DATA' : 'NO DRIVER') : fresh ? 'LIVE' : 'STALE';
  $('metal-status').className = `badge ${fresh ? 'safe' : 'neutral'}`;
  const amplitudeAdc = fresh ? sensor.packet?.amplitude_adc : null;
  $('metal-raw').textContent = amplitudeAdc == null ? '-- ADC' : `${amplitudeAdc} ADC`;
  $('metal-voltage').textContent = amplitudeAdc == null
    ? '-- V peak' : `${(amplitudeAdc * sensorReferenceVoltage / 255).toFixed(2)} V peak`;
  $('metal-response').textContent = fresh ? `${(state.detector.signal_ratio * 100).toFixed(1)} %` : '-- %';
  $('metal-rate').textContent = available ? `${sensor.rate_hz ?? 0} Hz` : '-- Hz';
  $('metal-age').textContent = sensor.age_ms == null ? '-- ms' : `${sensor.age_ms} ms`;
  $('metal-port').textContent = sensor.port || '--';
  $('metal-counts').textContent = `${sensor.received ?? 0} / ${sensor.invalid_lines ?? 0}`;
  $('metal-sequence').textContent = sensor.packet ? `${sensor.packet.seq} / ${sensor.packet.uptime_ms} ms` : '--';
  $('metal-error').textContent = !available ? 'Arduino driver unavailable' : sensor.error || (fresh ? 'Receiving valid packets' : 'Waiting for valid readings');
  const reference = sensor.reference_voltage ?? 5;
  const voltage = sensor.packet?.amplitude_adc * reference / 255;
  $('metal-voltage').textContent = fresh ? `${voltage.toFixed(3)} V` : '-- V';
  $('metal-voltage-note').textContent = fresh && voltage > reference / 2
    ? 'Above the centered sine range: check clipping or waveform shape. Voltage is an amplitude estimate.'
    : `Sine-equivalent AC peak, excluding DC bias. Reference ${reference.toFixed(2)} V.`;
  const threshold = state.detector.threshold_ratio;
  const key = `${sensor.baseline_adc}:${sensor.full_response_adc}:${reference}:${threshold}`;
  if (key !== metalCalibrationKey && Number.isFinite(sensor.baseline_adc)) {
    metalCalibrationKey = key;
    $('metal-baseline').value = sensor.baseline_adc.toFixed(2);
    $('metal-full').value = (sensor.baseline_adc + (sensor.full_response_adc - sensor.baseline_adc) * threshold).toFixed(2);
    $('metal-launch').value = `<arg name="sensor_baseline_adc" default="${sensor.baseline_adc}"/>\n<arg name="sensor_full_response_adc" default="${sensor.full_response_adc}"/>\n<arg name="sensor_reference_voltage" default="${reference}"/>\n<arg name="detector_threshold_ratio" default="${threshold}"/>`;
  }
  if (metalPending && sensor.command_result?.request_id === metalPending.id) {
    metalFeedback = sensor.command_result.message;
    metalPending = null;
  } else if (metalPending && performance.now() - metalPending.at > 3000) {
    metalFeedback = 'No acknowledgement from detector driver.';
    metalPending = null;
  }
  const moving = Math.abs(state.robot.speed_mps) > 0.03;
  const allowed = fresh && owner && !moving && !metalPending;
  for (const id of ['metal-zero', 'metal-apply', 'metal-reset']) $(id).disabled = !allowed;
  $('metal-control').hidden = owner;
  $('metal-control').disabled = !state.connected;
  $('metal-feedback').textContent = !fresh ? 'Fresh Arduino readings required for calibration.'
    : !owner ? 'Take control to calibrate.' : moving ? 'Stop the vehicle to calibrate.'
    : metalPending ? 'Applying calibration...' : metalFeedback || 'Ready';
  const packetKey = JSON.stringify(sensor.packet);
  if (fresh && sensor.packet && packetKey !== metalPacketKey) {
    metalPacketKey = packetKey;
    if ($('metal-debug-live').checked) {
      metalPackets.push(packetKey);
      if (metalPackets.length > 50) metalPackets.shift();
      $('metal-log').value = metalPackets.join('\n');
      $('metal-log').scrollTop = $('metal-log').scrollHeight;
    }
  }
  $('metal-export').disabled = !(sensor.history?.length);
  if ($('settings-dialog').open && !$('settings-detector').hidden) drawMetalHistory();
}

let metalChart = null;
let detectorChart = null;
let detectorHistoryAt = 0;
let detectorChartAt = 0;
const detectorAngles = [];

function recordDetectorAngle(now) {
  detectorHistoryAt = now;
  const angle = state.detector.fixture_angle_deg;
  const available = state.safety?.simulated || (state.arm_servo?.connected && state.arm_servo?.active_role === 'arm');
  detectorAngles.push({at: now, angle: available && Number.isFinite(angle) ? angle : null});
  while (detectorAngles.length && now - detectorAngles[0].at > 30000) detectorAngles.shift();
}

function drawDetectorHistory(now = performance.now()) {
  if (now - detectorChartAt < 200) return;
  detectorChartAt = now;
  const sensor = state.detector.sensor || {};
  const elapsed = Math.max(0, (now - detectorHistoryAt) / 1000);
  const samples = (sensor.history || []).filter(p => p.age_s + elapsed <= 30);
  const zero = sensor.baseline_adc;
  const trigger = zero + (sensor.full_response_adc - zero) * state.detector.threshold_ratio;
  if (!detectorChart) {
    detectorChart = new Chart($('detector-history'), {
      type: 'line',
      data: {datasets: [
        {label: 'ADC', yAxisID: 'adc', borderColor: '#6edcc7', borderWidth: 1.5, data: []},
        {label: 'Angle (deg)', yAxisID: 'angle', borderColor: '#b9a0ef', borderWidth: 1.5, data: []},
        {label: 'Zero', yAxisID: 'adc', borderColor: '#efbb63', borderDash: [4, 4], borderWidth: 1, data: []},
        {label: 'Trigger', yAxisID: 'adc', borderColor: '#ed7770', borderDash: [4, 4], borderWidth: 1, data: []},
      ]},
      options: {responsive: true, maintainAspectRatio: false, animation: false, parsing: false,
        elements: {point: {radius: 0, hitRadius: 6}, line: {spanGaps: .25}},
        plugins: {legend: {labels: {color: '#a4b5bb', boxWidth: 10, font: {size: 9}}}},
        scales: {
          x: {type: 'linear', min: -30, max: 0, ticks: {color: '#a4b5bb', count: 3, maxRotation: 0,
            callback: v => v === 0 ? 'now' : `${v}s`}, grid: {color: '#2b3d46'}},
          adc: {position: 'left', ticks: {color: '#6edcc7', maxTicksLimit: 4}, grid: {color: '#2b3d46'}},
          angle: {position: 'right', suggestedMin: -1, suggestedMax: 1,
            ticks: {color: '#b9a0ef', maxTicksLimit: 4, callback: v => `${v}\u00b0`}, grid: {drawOnChartArea: false}},
        }},
    });
  }
  detectorChart.data.datasets[0].data = samples.map(p => ({x: -p.age_s - elapsed, y: p.adc}));
  while (detectorAngles.length && now - detectorAngles[0].at > 30000) detectorAngles.shift();
  detectorChart.data.datasets[1].data = detectorAngles.map(p => ({x: (p.at - now) / 1000, y: p.angle}));
  for (const [index, value] of [[2, zero], [3, trigger]]) {
    detectorChart.data.datasets[index].data = Number.isFinite(value) ? [{x: -30, y: value}, {x: 0, y: value}] : [];
  }
  detectorChart.update('none');
}

function drawMetalHistory() {
  const canvas = $('metal-history');
  if (!canvas.parentElement.clientWidth || !canvas.parentElement.clientHeight) return;
  const seconds = Number($('metal-time-range').value) || 30;
  const samples = (state.detector.sensor?.history || []).filter(sample => sample.age_s <= seconds);
  const values = samples.map(sample => sample.adc);
  const baseline = state.detector.sensor?.baseline_adc;
  const trigger = baseline + (state.detector.sensor?.full_response_adc - baseline) * state.detector.threshold_ratio;
  const plotValues = [...values, baseline, trigger].filter(Number.isFinite);
  let low = 0, high = 255;
  if ($('metal-autoscale').checked && plotValues.length) {
    low = Math.max(0, Math.floor(Math.min(...plotValues) - 3));
    high = Math.min(255, Math.ceil(Math.max(...plotValues) + 3));
  }
  if (!metalChart) {
    metalChart = new Chart(canvas, {
      type: 'line',
      data: {datasets: [
        {label: 'Amplitude (ADC)', data: [], borderColor: '#6edcc7', borderWidth: 2,
          pointRadius: 0, pointHitRadius: 8, spanGaps: 0.5},
        {label: 'Zero (ADC)', data: [], borderColor: '#efbb63', borderWidth: 1,
          borderDash: [5, 4], pointRadius: 0},
        {label: 'Mine trigger (ADC)', data: [], borderColor: '#ed7770', borderWidth: 1,
          borderDash: [5, 4], pointRadius: 0},
      ]},
      options: {
        responsive: true, maintainAspectRatio: false, animation: false, parsing: false,
        plugins: {legend: {display: true, labels: {color: '#a4b5bb', boxWidth: 16}}, tooltip: {callbacks: {
          title: items => items.length ? `${(-items[0].parsed.x).toFixed(2)} seconds ago` : '',
        }}},
        scales: {
          x: {type: 'linear', min: -30, max: 0, grid: {color: '#2b3d46'},
            ticks: {color: '#a4b5bb', maxRotation: 0, count: 4,
              callback: value => value === 0 ? 'now' : `${value}s`}},
          y: {min: 0, max: 255, grid: {color: '#2b3d46'},
            ticks: {color: '#a4b5bb', precision: 0, maxTicksLimit: 6}},
        },
      },
    });
  }
  metalChart.data.datasets[0].data = samples.map(sample => ({x: -sample.age_s, y: sample.adc}));
  metalChart.data.datasets[1].data = Number.isFinite(baseline) && baseline >= low && baseline <= high
    ? [{x: -seconds, y: baseline}, {x: 0, y: baseline}] : [];
  metalChart.data.datasets[2].data = Number.isFinite(trigger)
    ? [{x: -seconds, y: trigger}, {x: 0, y: trigger}] : [];
  metalChart.options.scales.x.min = -seconds;
  metalChart.options.scales.y.min = low;
  metalChart.options.scales.y.max = high;
  metalChart.resize();
  metalChart.update('none');
  canvas.setAttribute('aria-label', `Raw metal detector amplitude over the last ${seconds} seconds`);
  $('metal-range').textContent = values.length
    ? `Min ${Math.min(...values)} · Max ${Math.max(...values)} · Peak-to-peak ${Math.max(...values) - Math.min(...values)} ADC`
    : `No raw readings in the last ${seconds} seconds`;
}

function detectorColor(signalRatio) {
  const fraction = Math.max(0, Math.min(1, signalRatio / Math.max(state.detector.threshold_ratio, 0.001)));
  return `hsl(${(1 - fraction) * 120} 78% 55%)`;
}

function initializeMetalSettings() {
  const command = action => {
    const id = `metal-${Date.now()}-${Math.random().toString(36).slice(2, 8)}`;
    const payload = {type: 'detector_calibrate', action, request_id: id};
    if (action === 'apply') {
      if (!$('metal-baseline').value || !$('metal-full').value) { metalFeedback = 'Enter a zero level and mine trigger.'; updateMetalSettings(); return; }
      payload.baseline_adc = Number($('metal-baseline').value);
      payload.full_response_adc = Number($('metal-full').value);
      if (![payload.baseline_adc, payload.full_response_adc].every(v => Number.isFinite(v) && v >= 0 && v <= 255)
          || Math.abs(payload.full_response_adc - payload.baseline_adc) < 1) {
        metalFeedback = 'Zero level and mine trigger must be within 0..255 and at least one count apart.'; updateMetalSettings(); return;
      }
    }
    metalPending = {id, at: performance.now()};
    send(payload); updateMetalSettings();
  };
  $('metal-zero').onclick = () => command('zero');
  $('metal-apply').onclick = () => command('apply');
  $('metal-reset').onclick = () => command('reset');
  $('metal-control').onclick = () => send({type: 'take_control'});
  $('metal-autoscale').onchange = drawMetalHistory;
  $('metal-time-range').onchange = drawMetalHistory;
  $('metal-export').onclick = () => {
    const rows = ['age_seconds,amplitude_adc', ...(state.detector.sensor?.history || []).map(s => `${s.age_s},${s.adc}`)];
    const url = URL.createObjectURL(new Blob([rows.join('\n')], {type: 'text/csv'}));
    const link = document.createElement('a'); link.href = url; link.download = 'detector-readings.csv'; link.click();
    setTimeout(() => URL.revokeObjectURL(url), 1000);
  };
}

function initializeInputs() {
  initializeArmSettings();
  initializeMetalSettings();
  document.querySelectorAll('[data-settings-tab]').forEach(tab => {
    tab.onclick = () => {
      stopArmMove();
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
      updateMetalSettings();
    };
  });
  for (const [key, label, checkbox] of [
    ['steering', 'Steering axis'], ['throttle', 'Forward pedal axis'], ['brake', 'Reverse pedal axis'],
    ['deadzone', 'Steering deadzone'],
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
  $('keyboard-test').onblur = stopLocalInput;
  $('settings-dialog').addEventListener('close', stopInput);
  window.addEventListener('blur', stopInput);
  document.addEventListener('visibilitychange', () => { if (document.hidden) stopInput(); });
  initializeCameraSettings();
  $('refresh-cameras').onclick = () => refreshCameraDevices(true);
  $('forward-camera-source').onchange = async event => {
    stopInput();
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
let lastInputHint = 0;
function renderInputs(pads, target) {
  if (performance.now() - lastInputHint > 100) {
    updateInputHint();
    lastInputHint = performance.now();
  }
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
    ? `${gamepad.id} · ${gamepad.mapping || 'non-standard'} · ${gamepad.axes.length} axes · ${gamepad.buttons.length} buttons${preferences.input === 'controller' ? ` · ${mappingLabel(resolveMapping(gamepad))}` : ''}`
    : 'No device selected. Press a device button, then select it above.';
  document.querySelectorAll('[data-code]').forEach(el => el.classList.toggle('pressed', keys.has(el.dataset.code)));
  const preview = target;
  $('steering-preview').setAttribute('transform', `rotate(${-preview.angular / MAX_ANGULAR_RAD_S * 100} 80 80)`);
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
let lastCanvasFrame = 0;
let lastInstrumentFrame = 0;
function drawDashboard(now) {
  if (!document.hidden && now - lastCanvasFrame >= 1000 / 30) {
    lastCanvasFrame = now;
    drawForwardView();
    drawRadar();
    if (now - lastInstrumentFrame >= 100) {
      lastInstrumentFrame = now;
      drawMap();
      drawPressureHistory();
      drawDetectorHistory(now);
    }
  }
  requestAnimationFrame(drawDashboard);
}
requestAnimationFrame(drawDashboard);
