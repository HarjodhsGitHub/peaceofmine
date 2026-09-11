const fields = {
  latitude: document.querySelector('#latitude'),
  longitude: document.querySelector('#longitude'),
  altitude: document.querySelector('#altitude'),
  satellites: document.querySelector('#satellites'),
  hdop: document.querySelector('#hdop'),
  dop: document.querySelector('#dop'),
  speed: document.querySelector('#speed'),
  course: document.querySelector('#course'),
  utc: document.querySelector('#utc'),
  device: document.querySelector('#device'),
  baud: document.querySelector('#baud'),
  lastMessage: document.querySelector('#last-message'),
  satelliteView: document.querySelector('#satellite-view'),
  fixMode: document.querySelector('#fix-mode'),
  ntrip: document.querySelector('#ntrip'),
  receiverStatus: document.querySelector('#receiver-status'),
  positionStatus: document.querySelector('#position-status'),
  correctionsStatus: document.querySelector('#corrections-status'),
  rawLog: document.querySelector('#raw-log-content'),
  eventLog: document.querySelector('#event-log-content'),
};

function value(value, suffix = '') {
  return value === null || value === undefined || value === '' ? '--' : `${value}${suffix}`;
}

function update(state) {

  fields.latitude.textContent = positionValid(state) ? state.latitude.toFixed(7) : '--';
  fields.longitude.textContent = positionValid(state) ? state.longitude.toFixed(7) : '--';
  fields.altitude.textContent = value(reading(state, 'altitude_m', true), ' m');
  fields.satellites.textContent = value(reading(state, 'satellites'));
  fields.hdop.textContent = value(reading(state, 'hdop'));
  fields.dop.textContent = `${value(reading(state, 'pdop'))} / ${value(reading(state, 'vdop'))}`;
  fields.speed.textContent = value(reading(state, 'speed_mps', true));
  fields.course.textContent = value(reading(state, 'course_deg', true), '°');
  fields.utc.textContent = value(state.utc);
  fields.device.textContent = state.device;
  fields.baud.textContent = state.baud;
  fields.lastMessage.textContent = state.last_message;
  fields.satelliteView.textContent = value(reading(state, 'satellites_in_view'));
  fields.fixMode.textContent = fresh(state) && (!state.field_age_s || state.field_age_s.fix_mode < 5) ? state.fix_mode : '--';
  fields.rawLog.textContent = (state.raw_log || []).length
    ? state.raw_log.join('\n')
    : 'Waiting for serial data...';
  fields.eventLog.replaceChildren(...((state.event_log || []).length ? state.event_log : [{
    time: '--:--:--', message: 'Waiting for events...', kind: 'info',
  }]).map(event => {
    const row = document.createElement('div');
    row.className = `event-row ${event.kind}`;
    const time = document.createElement('time');
    time.textContent = event.time;
    const message = document.createElement('span');
    message.textContent = event.message;
    row.append(time, message);
    return row;
  }));
  fields.ntrip.textContent = state.ntrip_enabled
    ? `${state.ntrip_status} · ${state.corrections_bytes} B`
    : 'Not configured';
  updateStatus(state);

  const fix = document.querySelector('#fix-label');
  fix.textContent = fresh(state) ? state.fix_label : 'STALE';
  fix.className = `fix ${fresh(state) ? state.fix_label.toLowerCase().replaceAll(' ', '-') : 'stale'}`;
  document.querySelector('#age').textContent = state.age_s === null ? 'No data' : `${state.age_s}s ago`;
  document.querySelector('#message').textContent = state.connected
    ? `${state.last_message} · updates arrive from the receiver over UART`
    : state.last_message;
  const connection = document.querySelector('#connection');
  connection.className = `connection ${state.connected ? 'online' : 'offline'}`;
  connection.innerHTML = `<span class="dot"></span>${state.connected ? 'UART connected' : 'UART disconnected'}`;
}

function updateStatus(state) {
  function card(id, title, detail, tone = 'waiting') {
    const label = document.getElementById(`${id}-status`);
    label.textContent = title;
    label.className = tone;
    document.getElementById(`${id}-detail`).textContent = detail;
  }
  const valid = positionValid(state);
  const quality = valid ? state.fix_quality : 0;
  const modes = {
    1: ['Regular GPS', 'Standalone satellite position. The receiver does not report a corrected solution.', 'waiting'],
    2: ['Differential GPS', 'The receiver reports differential corrections in use. This is not an RTK fixed solution.', 'waiting'],
    4: ['RTK fixed', 'The receiver has resolved carrier ambiguities. This is the highest RTK solution state, not a guarantee of true accuracy.', 'ok'],
    5: ['RTK float', 'The receiver is using RTK corrections, but carrier ambiguities are not fixed. Position can still drift significantly.', 'waiting'],
  };
  const mode = !state ? ['Viewer offline', 'Cannot reach the viewer. Connection and fix states are unknown.', 'error']
    : !state.connected ? ['Receiver disconnected', 'Check the receiver power and serial connection.', 'error']
    : !fresh(state) ? ['Receiver data stale', 'Serial connection is open, but fresh receiver readings are missing.', 'error']
    : !valid ? ['No position fix', 'Receiver is connected but has no fresh valid position. Check antenna sky view.', 'waiting']
    : modes[quality] || [state.fix_label || 'Other fix', 'Receiver reports another position mode; RTK fixed is not confirmed.', 'waiting'];
  document.getElementById('solution-title').textContent = mode[0];
  document.getElementById('solution-detail').textContent = mode[1];
  document.getElementById('solution-summary').className = `solution-summary ${mode[2]}`;
  card('receiver', !state ? 'Unknown' : !state.connected ? 'Disconnected' : fresh(state) ? 'Live data' : 'Data stale',
    !state ? 'Viewer unreachable' : state.connected ? `${state.device} · ${value(state.age_s, ' s since update')}` : 'Check UART and power',
    fresh(state) ? 'ok' : 'error');
  card('position', mode[0], quality === 4 ? 'Carrier ambiguities fixed' : quality === 5 ? 'Corrections in use · not fixed' : quality === 2 ? 'Differential corrections in use' : quality === 1 ? 'No corrected fix reported' : 'No confirmed live solution', mode[2]);
  const ntrip = state?.ntrip_status || '';
  const serviceError = /error/i.test(ntrip);
  const serviceConnected = /^(Receiving RTCM|Connected)/.test(ntrip);
  const receiving = Boolean(state?.ntrip_enabled && numeric(state.correction_age_s) && state.correction_age_s < 15);
  card('service', !state ? 'Unknown' : !state.ntrip_enabled ? 'Not configured' : serviceError ? 'Connection error' : serviceConnected ? 'Connected' : 'Connecting / waiting',
    !state ? 'Viewer unreachable' : !state.ntrip_enabled ? 'No NTRIP account configured' : ntrip,
    serviceError ? 'error' : serviceConnected ? 'ok' : 'waiting');
  card('corrections', !state ? 'Unknown' : !state.ntrip_enabled ? 'Not configured' : receiving ? 'Arriving at receiver' : 'No recent data',
    !state ? 'Viewer unreachable' : receiving ? `${state.correction_age_s.toFixed(1)} s ago · ${(state.corrections_bytes / 1024).toFixed(1)} KiB forwarded` : 'Data delivery does not by itself confirm an RTK fix',
    receiving ? 'ok' : 'waiting');
}

async function poll() {
  let state = null;
  const controller = new AbortController();
  const timeout = setTimeout(() => controller.abort(), 4000);
  try {
    const response = await fetch('/api/state', {cache: 'no-store', signal: controller.signal});
    if (!response.ok) throw new Error(`HTTP ${response.status}`);
    state = await response.json();
  } catch (error) {
    const connection = document.querySelector('#connection');
    connection.className = 'connection offline';
    connection.innerHTML = '<span class="dot"></span>Viewer offline';
    document.querySelector('#message').textContent = `Cannot reach telemetry: ${error.message}. Retrying…`;
    for (const key of ['latitude', 'longitude', 'altitude', 'satellites', 'hdop', 'dop', 'speed', 'course']) fields[key].textContent = '--';
    fields.positionStatus.textContent = 'Position unavailable';
    fields.positionStatus.className = 'waiting';
    document.querySelector('#fix-label').textContent = 'OFFLINE';
    document.querySelector('#fix-label').className = 'fix offline';
    document.querySelector('#age').textContent = 'No live data';
    fields.receiverStatus.textContent = 'Unknown';
    fields.receiverStatus.className = 'waiting';
    fields.correctionsStatus.textContent = 'Unknown';
    fields.correctionsStatus.className = 'waiting';
  } finally {
    clearTimeout(timeout);
  }
  try {
    if (state) update(state);
    else updateStatus(null);
    updateTrends(state);
  } catch (error) {
    document.querySelector('#message').textContent = `Display error: ${error.message}. Try reloading the page.`;
    console.error(error);
  } finally {
    setTimeout(poll, 1000);
  }
}

poll();
