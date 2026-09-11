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
  fields.receiverStatus.textContent = state.connected ? 'UART connected' : 'UART offline';
  fields.receiverStatus.className = state.connected ? 'ok' : 'error';
  fields.positionStatus.textContent = positionValid(state) ? state.fix_label : fresh(state) ? 'No fix yet' : 'Position stale';
  fields.positionStatus.className = positionValid(state) ? 'ok' : 'waiting';
  const correctionsWorking = state.ntrip_enabled && state.correction_age_s !== null && state.correction_age_s < 15;
  const correctionsError = state.ntrip_enabled && state.ntrip_status.startsWith('NTRIP error');
  fields.correctionsStatus.textContent = !state.ntrip_enabled
    ? 'Not configured'
    : correctionsWorking ? 'Receiving RTCM' : correctionsError ? 'Connection error' : 'Waiting';
  fields.correctionsStatus.className = correctionsWorking ? 'ok' : correctionsError ? 'error' : 'waiting';

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
    updateTrends(state);
  } catch (error) {
    document.querySelector('#message').textContent = `Display error: ${error.message}. Try reloading the page.`;
    console.error(error);
  } finally {
    setTimeout(poll, 1000);
  }
}

poll();
