// Independent arm/probe settings views; all commands go through the ROS web bridge.
let servoRequestSequence = 0;
function createServoSettings(role) {
  const $ = id => document.getElementById(role === 'probe' && id.startsWith('arm-') ? id.replace(/^arm-/, 'probe-servo-') : id);
  let loadChart = null;
  let loadHistory = [];
  let loadSample = null;
  let loadServo = null;
  let loadLatest = null, loadActive = false;
  function updateLoadChart(servo, active, animate = false) {
    loadLatest = servo; loadActive = active;
    if (animate && (!$('settings-dialog').open || document.getElementById(`settings-${role}`).hidden)) return;
    if (animate && loadChart && scrollTimeChart(loadChart, performance.now())) return;
    const now = performance.now();
    const key = `${servo.serial_port || ''}:${servo.servo_id}`;
    if (!state.connected || (active && (!servo.connected || loadServo !== key))) {
      loadHistory = []; loadSample = null;
      if (active) loadServo = key;
    }
    if (state.connected && active && servo.connected && Number.isFinite(servo.sample_time)
        && servo.sample_time !== loadSample && Number.isFinite(servo.load_percent)) {
      if (loadHistory.length && now - loadHistory.at(-1).at > 600)
        loadHistory.push({at: now - 1, value: null});
      loadHistory.push({at: now, value: servo.load_percent});
      loadSample = servo.sample_time;
    }
    loadHistory = loadHistory.filter(sample => now - sample.at <= 60000).slice(-600);
    const panel = document.getElementById(`settings-${role}`);
    if (!$('settings-dialog').open || panel.hidden) return;
    if (!loadChart) loadChart = new Chart($('arm-load-history'), {
      type: 'line', data: {datasets: [{label: 'Estimated load (%)', data: [],
        borderColor: '#76b9ff', borderWidth: 2, pointRadius: 0, spanGaps: false},
        ...(role === 'probe' ? [1, -1].map(sign => ({label: `${sign > 0 ? '+' : '−'}Contact threshold`,
          data: [], borderColor: '#ffb86b', borderDash: [6, 4], borderWidth: 1, pointRadius: 0})) : [])]},
      options: {responsive: true, maintainAspectRatio: false, animation: false,
        parsing: false, plugins: {legend: {display: false}},
        scales: {
          x: {type: 'linear', min: -60, max: 0, title: {display: true, text: 'Seconds ago', color: '#b7c4d8'},
            ticks: {color: '#b7c4d8', callback: value => Math.abs(value)}},
          y: {min: -100, max: 100, title: {display: true, text: 'Signed load (%)', color: '#b7c4d8'},
            ticks: {color: '#b7c4d8'}},
        }},
    });
    loadChart.data.datasets[0].data = loadHistory.map(sample => ({x: (sample.at - now) / 1000, y: sample.value}));
    if (role === 'probe') {
      const threshold = Number($('arm-home-load').value);
      [1, -1].forEach((sign, index) => {
        loadChart.data.datasets[index + 1].data = Number.isFinite(threshold) && threshold >= 10 && threshold <= 60
          ? [{x: -60, y: sign * threshold}, {x: 0, y: sign * threshold}] : [];
      });
    }
    loadChart.$scrollPixels = 0;
    loadChart.$renderedAt = now;
    loadChart.update('none');
  }
  let armHoldTimer = null;
  let armSliderMoving = false;
  let armSliderDragging = false;
  let armSliderTarget = null;
  let armPending = null;
  function setArmFeedback(text, kind = 'pending') {
    $('arm-feedback').textContent = text;
    $('arm-feedback').dataset.kind = kind;
  }
  function armCommand(action, values = {}, feedback = true) {
    const payload = {type: 'arm_servo', action, role, ...values};
    if (feedback) {
      payload.request_id = `${Date.now()}-${++servoRequestSequence}`;
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
    const live = state.actuators?.[role] || state.arm_servo || {};
    const active = state.actuators?.[role] != null || (live.active_role || 'arm') === role;
    const servo = active ? live : {serial_connected: live.serial_connected, scanning: live.scanning, discovered_servos: live.discovered_servos, servo_id: live.role_ids?.[role] ?? (role === 'probe' ? 2 : live.servo_id), reason: `Select the ${role} servo to load its calibration and telemetry.`};
    updateLoadChart(servo, active);
    const servoSelect = $('arm-servo-id');
    const ids = servo.discovered_servos || [];
    const signature = JSON.stringify(ids);
    if (servoSelect.dataset.discovery !== signature) {
      const previous = servoSelect.value;
      servoSelect.replaceChildren(new Option(servo.scanning ? 'Scanning for servos…' : 'Select a detected servo', ''));
      ids.forEach(id => servoSelect.add(new Option(`Servo ${id}`, String(id))));
      const assignedId = live.role_ids?.[role] ?? servo.servo_id;
      servoSelect.value = ids.includes(Number(previous)) && previous !== '' ? previous
        : ids.includes(assignedId) ? String(assignedId) : '';
      servoSelect.dataset.discovery = signature;
    }
    const owner = state.connected && state.drive.you_control_owner;
    $('arm-reconnect').disabled = !owner || Boolean(live.scanning || live.torque || live.sweeping) || armHoldTimer !== null;
    if (armPending) {
      const acknowledgement = live.last_command;
      if (armPending.id && acknowledgement?.request_id === armPending.id) {
        setArmFeedback(acknowledgement.message, acknowledgement.success ? 'success' : 'error');
        armPending = null;
      } else if (armPending.predicate?.()) {
        setArmFeedback(armPending.confirmation, 'success'); armPending = null;
      } else if (performance.now() - armPending.at > 4000) {
        setArmFeedback('No confirmation received. Check the driver connection and retry.', 'error'); armPending = null;
      }
    }
    const ready = owner && state.drive.armed && state.safety?.servo_allowed && servo.connected;
    const slider = $('arm-position-slider');
    if (servo.connected) {
      slider.min = servo.position_minimum ?? servo.eeprom_minimum ?? 0;
      slider.max = servo.position_maximum ?? servo.eeprom_maximum ?? 4095;
      if (!armSliderDragging) slider.value = servo.position;
    }
    slider.disabled = !ready || (armHoldTimer !== null && !armSliderMoving);
    if (armSliderMoving && !armSliderDragging && servo.torque && Math.abs(servo.position - armSliderTarget) <= 3
        && servo.goal_position === armSliderTarget) stopArmMove();
    $('arm-position-label').textContent = servo.connected ? armAngle(Number(slider.value)) : '—';
    $('arm-jog-speed').disabled = armHoldTimer !== null && !armSliderMoving;

    $('arm-servo-status').textContent = servo.reason || 'Servo driver not running';
    $('arm-servo-position').textContent = servo.connected ? `${armAngle(servo.position)} (${servo.position ?? '—'} ticks)` : servo.serial_connected ? 'Adapter connected · no servo selected' : 'Disconnected';
    $('arm-servo-details').textContent = servo.connected ? `Servo ${servo.servo_id} · ${servo.serial_port || 'serial adapter'}` : servo.serial_connected ? `Select the ID of the ${role} servo, then click Select servo.` : servo.serial_port ? `Servo bus: ${servo.serial_port}` : 'Start hardware launch with use_arm_servo:=true and arm_serial_port:=<adapter path>.';
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
    $('arm-calibration-stop').disabled = !owner;
    const selectionBlocked = !state.connected ? 'Connect to the dashboard first.'
      : !owner ? 'Take control to select a servo or record calibration. Motion permission comes from the RC.'
      : live.torque ? 'Stop the active servo before changing the servo ID.'
      : !servo.serial_connected ? 'Waiting for the servo serial adapter.' : '';
    $('arm-calibration-access').textContent = owner && active && state.drive.calibrating ? 'Hold a jog button to move. Release it, then record the position.' : selectionBlocked || 'Ready to select a servo and record positions.';
    $('arm-take-control').hidden = owner;
    $('arm-take-control').disabled = !state.connected;
    $('arm-select-id').disabled = Boolean(selectionBlocked) || servoSelect.value === '';
    $('arm-select-id').title = selectionBlocked || 'Select the servo ID; position updates automatically.';
    const canRecord = owner && servo.connected && !servo.torque && armHoldTimer === null;
    if (role === 'probe') {
      const probe = live.probe || {};
      $('arm-save-extension').disabled = !canRecord || !Number.isFinite(servo.home_position) || servo.home_position - servo.position <= 3;
      $('arm-extension-status').textContent = probe.calibrated
        ? `Saved maximum: ${probe.max_depth_mm} mm · retained after restart` : 'Maximum extension not calibrated';
      if ($('arm-max-extension').dataset.saved !== String(probe.max_depth_mm)) {
        $('arm-max-extension').value = probe.max_depth_mm || '';
        $('arm-max-extension').dataset.saved = String(probe.max_depth_mm);
      }
      $('arm-enable-multiturn').disabled = !canRecord || servo.position_mode === 'multi-turn';
      $('arm-position-mode').textContent = servo.connected ? servo.position_mode === 'multi-turn'
        ? 'Multi-turn · −7 to +7 shaft revolutions' : 'Joint mode · enable multi-turn for travel beyond one revolution' : 'Select probe to read position mode';
      $('arm-home').disabled = !ready;
      $('arm-home-load').disabled = armHoldTimer !== null;
      $('arm-home-status').textContent = servo.connected ? servo.home_status || 'Not homed' : 'Not homed · connect probe';
      $('arm-home-travel').textContent = servo.connected && Number.isFinite(servo.home_position)
        ? `${((servo.home_position - servo.position) * 360 / 4096).toFixed(1)}° from top (shaft)` : 'Home required';
    }
    {
      $('arm-apply-motion').disabled = !canRecord;
      const motionSignature = JSON.stringify([servo.motion_speed_limit, servo.sweep_acceleration_deg_s2]);
      if ($('arm-apply-motion').dataset.profile !== motionSignature) {
        $('arm-max-speed').value = ((servo.motion_speed_limit || 80) * .684).toFixed(3);
        $('arm-acceleration').value = (servo.sweep_acceleration_deg_s2 || 40).toFixed(3);
        $('arm-apply-motion').dataset.profile = motionSignature;
      }
    }
    if (role === 'arm') $('arm-apply-limits').disabled = !canRecord || !['minimum', 'center', 'maximum'].every(point => Number.isFinite(servo.captured?.[point] ?? servo.calibration?.[point]));
    $('arm-jog-left').disabled = $('arm-jog-right').disabled = !ready;
    const jogReason = !state.connected ? 'Dashboard disconnected'
      : !owner ? 'Take control first'
      : !servo.connected ? 'Servo not connected'
      : !state.safety?.servo_allowed ? 'Blocked: PX4 must report status 4'
      : armHoldTimer !== null ? 'Jog command held · release to stop'
      : role === 'probe' ? 'Ready · hold up to retract or down to extend' : 'Ready · hold left or right to move';
    $('arm-jog-status').textContent = jogReason;
    $('arm-jog-status').dataset.ready = String(Boolean(ready));
    $('arm-jog-left').title = $('arm-jog-right').title = jogReason;
    for (const point of role === 'arm' ? ['minimum', 'center', 'maximum'] : []) {
      $(`arm-capture-${point}`).disabled = !canRecord;
      $(`arm-move-${point}`).disabled = !ready || !servo.calibration;
      $(`arm-recorded-${point}`).textContent = armAngle(servo.captured?.[point] ?? servo.calibration?.[point]);
    }
    if (!ready) stopArmMove();
    if (role === 'probe') return;
    if (servo.calibration) {
      const escape = value => String(value).replaceAll('&', '&amp;').replaceAll('"', '&quot;').replaceAll('<', '&lt;');
      $('arm-launch-xml').value = [
        '<arg name="use_arm_servo" default="true"/>',
        `<arg name="arm_serial_port" default="${escape(servo.serial_port || '')}"/>`,
        `<arg name="${role}_servo_id" default="${servo.servo_id}"/>`,
        ...['minimum', 'center', 'maximum'].map(point => `<arg name="${role}_${point}" default="${servo.calibration[point]}"/>`),
        ...(role === 'arm' ? [`<arg name="arm_speed" default="${servo.motion_speed_limit || 20}"/>`] : []),
        ...(role === 'arm' ? [`<arg name="arm_sweep_endpoint_tolerance_deg" default="${servo.sweep_endpoint_tolerance_deg ?? 2.0}"/>`] : []),
        ...(role === 'arm' ? [`<arg name="arm_sweep_acceleration_deg_s2" default="${servo.sweep_acceleration_deg_s2 || 40}"/>`] : []),
      ].join('\n');
    } else $('arm-launch-xml').value = 'Record and apply all three positions to generate launch XML.';
  }
  function initializeArmSettings() {
    if (role === 'probe') $('arm-save-extension').onclick = () => {
      if (!$('arm-max-extension').reportValidity()) return;
      armCommand('save_probe_extension', {max_extension_mm: Number($('arm-max-extension').value)});
    };
    if (role === 'probe') $('arm-home-load').oninput = updateArmSettings;
    if (role === 'probe') $('arm-enable-multiturn').onclick = () => armCommand('enable_multiturn');
    {
      $('arm-apply-motion').onclick = () => {
        if (!$('arm-max-speed').reportValidity() || !$('arm-acceleration').reportValidity()) return;
        armCommand('configure_motion', {max_speed_deg_s: Number($('arm-max-speed').value),
          acceleration_deg_s2: Number($('arm-acceleration').value)});
      };
    }
    $('arm-reconnect').onclick = () => armCommand('reconnect');
    $('arm-take-control').onclick = () => armControlCommand({type: 'take_control'}, 'Control acquired. Select a servo; hold a movement control when the RC permits.', () => state.drive.you_control_owner);
    $('arm-servo-id').onchange = updateArmSettings;
    $('arm-select-id').onclick = () => {
      if ($('arm-servo-id').value !== '') armCommand('select', {servo_id: Number($('arm-servo-id').value)});
    };
    $('arm-calibration-stop').onclick = () => { stopArmMove(); armCommand('stop'); send({type: 'estop'}); };
    function bindHold(button, action, values) {
      const start = () => {
        if (button.disabled) return;
        stopArmMove();
        const selected = typeof values === 'function' ? values() : values;
        if (action === 'home' && !$('arm-home-load').reportValidity()) return;
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
    if (role === 'probe') bindHold($('arm-home'), 'home', () => ({
      hold_id: `${Date.now()}-${++servoRequestSequence}`,
      load_percent: Number($('arm-home-load').value),
    }));
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
    for (const point of role === 'arm' ? ['minimum', 'center', 'maximum'] : []) {
      $(`arm-capture-${point}`).onclick = () => armCommand('capture', {point});
      bindHold($(`arm-move-${point}`), 'move', {point});
    }
    if (role === 'arm') $('arm-apply-limits').onclick = () => {
      const servo = state.actuators?.[role] || state.arm_servo || {};
      const calibration = Object.fromEntries(['minimum', 'center', 'maximum'].map(point => [point, servo.captured?.[point] ?? servo.calibration?.[point]]));
      armCommand('configure', calibration);
    };
    const releaseArmSettings = () => {
      stopArmMove();
      if (state.drive.you_control_owner && (state.arm_servo?.active_role || 'arm') === role) {
        armCommand('stop');
        send({type: 'calibration_end'});
      }
    };
    $('settings-dialog').querySelector('form').addEventListener('submit', releaseArmSettings);
    $('settings-dialog').addEventListener('close', releaseArmSettings);
    window.addEventListener('blur', stopArmMove);
    document.addEventListener('visibilitychange', () => { if (document.hidden) stopArmMove(); });
  }

  return {draw() { if (loadLatest) updateLoadChart(loadLatest, loadActive, true); }, update: updateArmSettings, initialize: initializeArmSettings, stop: stopArmMove, error(message) { if (armPending) { armPending = null; setArmFeedback(message, 'error'); } }};
}
const servoSettings = [createServoSettings('arm'), createServoSettings('probe')];
function updateArmSettings() { servoSettings.forEach(panel => panel.update()); }
function initializeArmSettings() { servoSettings.forEach(panel => panel.initialize()); }
function stopArmMove() { servoSettings.forEach(panel => panel.stop()); }

