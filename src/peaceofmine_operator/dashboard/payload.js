// Receiver fields expire independently: satellite traffic cannot refresh position.
window.operatorPayloadState = state => {
  const el = id => document.getElementById(`gnss-${id}`);
  const g = state.gnss || {};
  const live = state.connected && g.connected && Number.isFinite(g.age_s) && g.age_s < 5;
  const fresh = key => live && Number.isFinite(g.field_age_s?.[key]) && g.field_age_s[key] < 5;
  const position = fresh('latitude') && fresh('longitude') && fresh('fix_quality') && g.fix_quality > 0 &&
    Number.isFinite(g.latitude) && Number.isFinite(g.longitude);
  el('status').textContent = !live ? 'UNAVAILABLE' : `${g.simulation ? 'SIM · ' : ''}${position ? g.fix_label : 'NO FRESH FIX'}`;
  el('status').className = `badge ${position && g.fix_quality === 4 ? 'safe' : 'neutral'}`;
  el('position').textContent = position ? `${g.latitude.toFixed(7)}, ${g.longitude.toFixed(7)}` : 'Waiting for a fresh position fix';
  for (const [id, key, digits, unit] of [
    ['satellites', 'satellites', 0, ''], ['altitude', 'altitude_m', 2, ' m'],
    ['accuracy', 'horizontal_accuracy_m', 3, ' m'], ['hdop', 'hdop', 2, '']]) {
    const valid = fresh(key) && Number.isFinite(g[key]) && (id !== 'accuracy' || position);
    el(id).textContent = valid ? g[key].toFixed(digits) + unit : '—';
  }
  const recent = live && Number.isFinite(g.correction_age_s) && g.correction_age_s < 10;
  el('corrections').textContent = !live ? 'No correction telemetry' :
    `${g.ntrip_status || 'Not configured'} · ${recent ? 'RTCM recent' : 'No recent RTCM'} · ${g.corrections_bytes || 0} bytes`;
  el('detail').textContent = g.simulation ? 'Simulated receiver position; no hardware connection.' :
    `${g.device || 'GNSS disabled or unavailable'} · ${g.accuracy_source || 'No accuracy estimate'}. RTCM delivery does not imply RTK fixed.`;
};
