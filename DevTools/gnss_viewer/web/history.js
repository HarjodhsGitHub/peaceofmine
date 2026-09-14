/* Rolling browser-session telemetry; no history leaves the browser. */
const WINDOW_MS = 30000;
const samples = [];
const numeric = n => typeof n === 'number' && Number.isFinite(n);
const fresh = s => Boolean(s && s.connected && numeric(s.age_s) && s.age_s < 5);
const positionValid = s => fresh(s) && s.fix_quality > 0 &&
  numeric(s.latitude) && numeric(s.longitude) && Math.abs(s.latitude) <= 90 && Math.abs(s.longitude) <= 180 &&
  (!s.field_age_s || (s.field_age_s.latitude < 5 && s.field_age_s.longitude < 5 && s.field_age_s.fix_quality < 5));
function reading(s, key, position = false) {
  if (!fresh(s) || (position && !positionValid(s))) return null;
  if (s.field_age_s && (!numeric(s.field_age_s[key]) || s.field_age_s[key] >= 5)) return null;
  return numeric(s[key]) ? s[key] : null;
}
const definitions = [
  ['latitude', 'Latitude (degrees)', s => reading(s, 'latitude', true)],
  ['longitude', 'Longitude (degrees)', s => reading(s, 'longitude', true)],
  ['accuracy', 'Receiver-reported uncertainty (m)', s => reading(s, 'horizontal_accuracy_m', true)],
  ['altitude', 'Altitude (m)', s => reading(s, 'altitude_m', true)],
  ['satellites', 'Satellites in solution', s => reading(s, 'satellites')],
  ['hdop', 'HDOP', s => reading(s, 'hdop')],
  ['dop', 'PDOP', s => reading(s, 'pdop'), s => reading(s, 'vdop'), 'VDOP'],
  ['speed', 'Speed (m/s)', s => reading(s, 'speed_mps', true)],
  ['course', 'Course (degrees)', s => reading(s, 'course_deg', true)],
  ['correction-rate', 'RTCM bytes/s', s => s?._rate ?? null],
  ['correction-age', 'Correction age (s)', s => s?.correction_age_s ?? null],
  ['data-age', 'Data age (s)', s => s?.age_s ?? null],
];
const sourceMessages = {
  accuracy: 'UBX NAV-PVT / GST', latitude: 'GGA / RMC', longitude: 'GGA / RMC', altitude: 'GGA', satellites: 'GGA',
  hdop: 'GGA / GSA', dop: 'GSA', speed: 'RMC / VTG', course: 'RMC / VTG',
  'satellite-view': 'GSV', 'fix-mode': 'GSA',
};
function unavailableReason(def, state) {
  if (!state) return 'Viewer disconnected · retrying every second';
  if (!state.connected) return 'Receiver disconnected · check UART';
  if (def[0].startsWith('correction')) return state.ntrip_enabled ? 'No RTCM received yet' : 'NTRIP not configured';
  if (!fresh(state)) return 'Receiver data is stale · check serial output';
  if (['latitude', 'longitude', 'altitude', 'speed', 'course'].includes(def[0]) && !positionValid(state)) return 'No fresh position fix';
  return sourceMessages[def[0]] ? `No fresh ${sourceMessages[def[0]]} reading · check receiver message output` : 'No recent reading';
}
const charts = definitions.map(def => {
  const container = document.createElement('figure');
  container.className = 'trend';
  const frame = document.createElement('div');
  frame.className = 'chart-frame';
  const canvas = document.createElement('canvas');
  canvas.setAttribute('role', 'img');
  canvas.setAttribute('aria-label', `${def[1]}, rolling 30 second history`);
  frame.append(canvas);
  const caption = document.createElement('figcaption');
  caption.textContent = 'Connecting to receiver…';
  container.append(frame, caption);
  document.getElementById(def[0]).parentElement.append(container);
  let instance = null;
  if (window.Chart) {
    instance = new Chart(canvas, {
      type: 'line',
      data: {datasets: [2, 3].filter(i => def[i]).map((i, index) => ({
        label: index ? def[4] : def[1], data: [],
        borderColor: def[0] === 'accuracy' ? '#69b9ff' : index ? '#e4bd64' : '#6bd0a2',
        backgroundColor: def[0] === 'accuracy' ? '#69b9ff' : index ? '#e4bd64' : '#6bd0a2',
        borderWidth: 2, pointRadius: 2, pointHoverRadius: 5,
        stepped: Boolean(def[5]), spanGaps: false,
      }))},
      options: {
        responsive: true, maintainAspectRatio: false, animation: false,
        parsing: false,
        interaction: {intersect: false, mode: 'nearest'},
        plugins: {legend: {display: Boolean(def[4]), labels: {color: '#9ba9a0', boxWidth: 10}},
          tooltip: {callbacks: {title: items => `${Math.abs(items[0].parsed.x).toFixed(1)} seconds ago`}}},
        scales: {
          x: {type: 'linear', min: -30, max: 0, grid: {color: '#33433f66'},
            ticks: {stepSize: 15, color: '#9ba9a0', callback: n => n === 0 ? 'now' : `${n}s`}},
          y: {title: {display: def[0] === 'accuracy', text: 'Uncertainty (m)', color: '#8fcaff'}, min: def[0] === 'accuracy' ? 0 : def[5]?.[0], max: def[5]?.[1], grid: {color: '#33433f66'},
            ticks: {maxTicksLimit: 3, color: '#9ba9a0'}},
        },
      },
    });
  }
  return {def, instance, caption, canvas};
});
function formatNumber(n) {
  return Math.abs(n) < 0.001 && n !== 0 ? n.toExponential(2) : Number(n.toFixed(7)).toString();
}
function drawCharts(now) {
  for (const chart of charts) {
    if (!chart.instance) {
      chart.caption.textContent = 'Chart library unavailable · reload the page';
      continue;
    }
    const series = [2, 3].filter(i => chart.def[i]).map(i => {
      const points = [];
      let previous = null;
      for (const p of samples) {
        const v = chart.def[i](p.s);
        if (previous && (p.t - previous.t > 2500 || (chart.def[0] === 'course' && numeric(v) && Math.abs(v - previous.v) > 180))) {
          points.push({x: (p.t - now - 1) / 1000, y: null});
        }
        points.push({x: (p.t - now) / 1000, y: numeric(v) ? v : null});
        previous = {t: p.t, v};
      }
      return points;
    });
    const values = series.flat().map(p => p.y).filter(numeric);
    const latest = samples.at(-1)?.s;
    const current = [2, 3].some(i => chart.def[i] && numeric(chart.def[i](latest)));
    chart.caption.textContent = current
      ? `30s range ${formatNumber(Math.min(...values))} – ${formatNumber(Math.max(...values))}`
      : unavailableReason(chart.def, latest);
    chart.canvas.setAttribute('aria-label', `${chart.def[1]}. ${chart.caption.textContent}`);
    series.forEach((data, i) => { chart.instance.data.datasets[i].data = data; });
    chart.instance.update('none');
  }
}

let accuracyCircle;
let map, marker, trail, lastPosition, following = true;
const followButton = document.getElementById('follow-position');
function setFollowing(enabled) {
  following = enabled;
  followButton.setAttribute('aria-pressed', String(enabled));
  followButton.textContent = `Follow position: ${enabled ? 'on' : 'off'}`;
  if (enabled && lastPosition && map) map.panTo(lastPosition, {animate: false});
}
followButton.addEventListener('click', () => setFollowing(!following));
if (window.L) {
  map = L.map('map', {scrollWheelZoom: false}).setView([59.3, 18.0], 7);
  // Tiles start only once a valid receiver position is available.
  trail = L.polyline([], {color: '#147a62', weight: 3}).addTo(map);
  L.control.scale({imperial: false}).addTo(map);
  map.on('dragstart', () => setFollowing(false));
} else {
  document.getElementById('tile-status').textContent = 'Map library unavailable; telemetry continues.';
  followButton.disabled = true;
}
let referencePoint = null;
let referenceMarker, referenceLine;
function updateReference(s) {
  const status = document.getElementById('reference-status');
  if (!map) return;
  if (!referencePoint) {
    if (referenceMarker) referenceMarker.remove();
    if (referenceLine) referenceLine.remove();
    referenceMarker = referenceLine = null;
    status.textContent = 'No reference selected. Actual position error is unknown.';
    return;
  }
  if (!referenceMarker) referenceMarker = L.circleMarker(referencePoint, {
    radius: 7, color: '#ffb76b', fillColor: '#ffb76b', fillOpacity: 0.8,
  }).bindTooltip('Approximate reference').addTo(map);
  referenceMarker.setLatLng(referencePoint);
  if (!positionValid(s)) {
    if (referenceLine) referenceLine.remove();
    referenceLine = null;
    status.textContent = 'Approximate reference set · waiting for a fresh receiver position.';
    return;
  }
  const point = [s.latitude, s.longitude];
  const distance = L.latLng(point).distanceTo(referencePoint);
  const radius = reading(s, 'horizontal_accuracy_m', true);
  if (!referenceLine) referenceLine = L.polyline([], {color: '#ffb76b', weight: 2, dashArray: '5 6'}).addTo(map);
  referenceLine.setLatLngs([point, referencePoint]);
  status.textContent = `Offset to approximate reference: ${distance.toFixed(1)} m. ` +
    (numeric(radius) && distance > radius ? 'Reference is OUTSIDE receiver uncertainty circle. ' : '') +
    'Reference accuracy is unverified; offset includes any reference error.';
}
document.getElementById('reference-form').addEventListener('submit', event => {
  event.preventDefault();
  const lat = document.getElementById('reference-lat').valueAsNumber;
  const lon = document.getElementById('reference-lon').valueAsNumber;
  if (!numeric(lat) || !numeric(lon) || Math.abs(lat) > 90 || Math.abs(lon) > 180) return;
  referencePoint = [lat, lon];
  updateReference(samples.at(-1)?.s);
  if (map && lastPosition) { setFollowing(false); map.fitBounds([referencePoint, lastPosition], {padding: [35, 35], maxZoom: 19}); }
});
document.getElementById('clear-reference').addEventListener('click', () => {
  referencePoint = null;
  updateReference(samples.at(-1)?.s);
});
let tiles;
function updateMap(s, now) {
  const valid = positionValid(s);
  const status = document.getElementById('map-status');
  if (valid && map) {
    const point = [s.latitude, s.longitude];
    if (!lastPosition) {
      map.setView(point, 18);
      tiles = L.tileLayer('https://tile.openstreetmap.org/{z}/{x}/{y}.png', {
        maxZoom: 19, attribution: '&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors',
      }).addTo(map);
      tiles.on('tileerror', () => { document.getElementById('tile-status').textContent = 'Map tiles unavailable. Position and trail still update.'; });
      tiles.on('tileload', () => { document.getElementById('tile-status').textContent = 'OpenStreetMap · scale shown below'; });
    }
    lastPosition = point;
    if (!marker) marker = L.circleMarker(point, {radius: 8, weight: 3, fillOpacity: 1}).addTo(map);
    marker.setLatLng(point).setStyle({color: '#fff', fillColor: s.fix_quality === 4 ? '#147a62' : '#bc851f'});
    if (following) map.panTo(point, {animate: false});
    status.textContent = `${s.fix_label} · ${point[0].toFixed(7)}, ${point[1].toFixed(7)}`;
  } else {
    status.textContent = lastPosition ? 'Position unavailable · marker shows last valid fix' : 'Waiting for a valid position. No marker placed yet.';
    if (marker) marker.setStyle({color: '#9ba9a0', fillColor: '#777'});
  }
  const radius = reading(s, 'horizontal_accuracy_m', true);
  const accuracyStatus = document.getElementById('accuracy-status');
  if (valid && numeric(radius) && radius >= 0 && map) {
    if (!accuracyCircle) accuracyCircle = L.circle([s.latitude, s.longitude], {
      radius, color: '#69b9ff', fillColor: '#69b9ff', fillOpacity: 0.18,
      weight: 2, interactive: false,
    }).addTo(map);
    accuracyCircle.setLatLng([s.latitude, s.longitude]).setRadius(radius);
    accuracyCircle.bringToBack();
    accuracyStatus.textContent = `Blue circle: ${formatNumber(radius)} m receiver-reported uncertainty · not measured position error · ${s.accuracy_source || 'receiver estimate'}`;
  } else {
    if (accuracyCircle) { accuracyCircle.remove(); accuracyCircle = null; }
    accuracyStatus.textContent = !valid ? 'Accuracy unavailable without a fresh position fix.'
      : !Object.hasOwn(s, 'horizontal_accuracy_m') ? 'Accuracy unavailable · restart the viewer service to request receiver accuracy.'
      : 'Accuracy unavailable · waiting for receiver NAV-PVT or GST output.';
  }
  updateReference(s);
  if (trail) {
    const segments = []; let segment = []; let previousTime = null;
    for (const p of samples) {
      if (!positionValid(p.s) || (previousTime !== null && p.t - previousTime > 2500)) {
        if (segment.length) segments.push(segment);
        segment = [];
      }
      if (positionValid(p.s)) segment.push([p.s.latitude, p.s.longitude]);
      previousTime = p.t;
    }
    if (segment.length) segments.push(segment);
    trail.setLatLngs(segments);
  }
}
let previousCounter = null;
function updateTrends(state) {
  const now = performance.now();
  let rate = null;
  if (state && previousCounter && now - previousCounter.t < 5000 && state.corrections_bytes >= previousCounter.bytes) {
    rate = (state.corrections_bytes - previousCounter.bytes) * 1000 / (now - previousCounter.t);
  }
  previousCounter = state ? {t: now, bytes: state.corrections_bytes} : null;
  const s = state ? {...state, _rate: rate} : null;
  samples.push({t: now, s});
  while (samples.length && samples[0].t < now - WINDOW_MS) samples.shift();
  document.getElementById('correction-rate').textContent = numeric(rate) ? rate.toFixed(0) : '--';
  document.getElementById('correction-age').textContent = numeric(s?.correction_age_s) ? s.correction_age_s.toFixed(1) : '--';
  document.getElementById('data-age').textContent = numeric(s?.age_s) ? s.age_s.toFixed(1) : '--';
  const accuracy = reading(s, 'horizontal_accuracy_m', true);
  document.getElementById('accuracy').textContent = numeric(accuracy) && accuracy >= 0 ? `${formatNumber(accuracy)} m` : '--';
  drawCharts(now);
  try { updateMap(s, now); } catch (error) {
    document.getElementById('map-status').textContent = `Map unavailable: ${error.message}`;
  }
}
