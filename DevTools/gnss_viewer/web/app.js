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
  fields.latitude.textContent = state.latitude === null ? '--' : state.latitude.toFixed(7);
  fields.longitude.textContent = state.longitude === null ? '--' : state.longitude.toFixed(7);
  fields.altitude.textContent = value(state.altitude_m, ' m');
  fields.satellites.textContent = value(state.satellites);
  fields.hdop.textContent = value(state.hdop);
  fields.dop.textContent = `${value(state.pdop)} / ${value(state.vdop)}`;
  fields.speed.textContent = value(state.speed_mps);
  fields.course.textContent = value(state.course_deg, '°');
  fields.utc.textContent = value(state.utc);
  fields.device.textContent = state.device;
  fields.baud.textContent = state.baud;
  fields.lastMessage.textContent = state.last_message;
  fields.satelliteView.textContent = value(state.satellites_in_view);
  fields.fixMode.textContent = state.fix_mode;
  fields.rawLog.textContent = state.raw_log.length
    ? state.raw_log.join('\n')
    : 'Waiting for serial data...';
  fields.eventLog.replaceChildren(...(state.event_log.length ? state.event_log : [{
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
  fields.positionStatus.textContent = state.fix_quality > 0 ? state.fix_label : 'No fix yet';
  fields.positionStatus.className = state.fix_quality > 0 ? 'ok' : 'waiting';
  const correctionsWorking = state.ntrip_enabled && state.correction_age_s !== null && state.correction_age_s < 15;
  const correctionsError = state.ntrip_enabled && state.ntrip_status.startsWith('NTRIP error');
  fields.correctionsStatus.textContent = !state.ntrip_enabled
    ? 'Not configured'
    : correctionsWorking ? 'Receiving RTCM' : correctionsError ? 'Connection error' : 'Waiting';
  fields.correctionsStatus.className = correctionsWorking ? 'ok' : correctionsError ? 'error' : 'waiting';

  const fix = document.querySelector('#fix-label');
  fix.textContent = state.fix_label;
  fix.className = `fix ${state.fix_label.toLowerCase().replaceAll(' ', '-')}`;
  document.querySelector('#age').textContent = state.age_s === null ? 'No data' : `${state.age_s}s ago`;
  document.querySelector('#message').textContent = state.connected
    ? `${state.last_message} · updates arrive from the receiver over UART`
    : state.last_message;
  const connection = document.querySelector('#connection');
  connection.className = `connection ${state.connected ? 'online' : 'offline'}`;
  connection.innerHTML = `<span class="dot"></span>${state.connected ? 'UART connected' : 'UART disconnected'}`;
}

async function poll() {
  try {
    const response = await fetch('/api/state', {cache: 'no-store'});
    if (!response.ok) throw new Error(`HTTP ${response.status}`);
    update(await response.json());
  } catch (error) {
    const connection = document.querySelector('#connection');
    connection.className = 'connection offline';
    connection.innerHTML = '<span class="dot"></span>Viewer offline';
    document.querySelector('#message').textContent = error.message;
  }
}

poll();
setInterval(poll, 1000);
