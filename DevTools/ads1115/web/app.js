'use strict';
const $ = s => document.querySelector(s);
const ranges = [6.144,4.096,2.048,1.024,.512,.256];
const rates = [8,16,32,64,128,250,475,860];
const names = ['AIN0 - AIN1','AIN0 - AIN3','AIN1 - AIN3','AIN2 - AIN3','AIN0','AIN1','AIN2','AIN3'];
const colors = ['#087e8b','#b05b24','#8647a4','#5a7b20','#008579','#346ab7','#b97916','#ad4e71'];
const form = $('#config-form');
let state, revision = 0, cursor = 0, history = [], charts = new Map(), dirty = false, busy = false;
let frameCount = 0, frameStart = performance.now(), lastRunState;
let pendingFrame = false, lastRender = 0, serverTime = 0, receivedAt = 0;
let lastStats = -Infinity;
const icons = () => lucide.createIcons();
const field = name => form.elements.namedItem(name);
const fmt = x => Number.isFinite(x) ? (Math.abs(x)>=1e5 ? x.toExponential(4) : x.toFixed(5)) : '--';
const error = text => { $('#error').hidden = !text; $('#error').textContent = text || ''; };
rates.forEach(n => field('rate').add(new Option(`${n} SPS`, n)));
ranges.forEach((n,i) => field('pga').add(new Option(`${['2/3','1','2','4','8','16'][i]}x / +/-${n} V`, i)));
$('#input-settings').innerHTML = names.map((name,i) => `<tr><td><input type="checkbox" name="enabled${i}" aria-label="Enable ${name}"></td><td>${name}</td><td><input type="number" step="any" min="-1000000000" max="1000000000" name="multiplier${i}" aria-label="${name} multiplier" required></td><td><input type="number" step="any" min="-1000000000" max="1000000000" name="offset${i}" aria-label="${name} offset" required></td></tr>`).join('');

function fill(c) {
  for (const [key,value] of Object.entries(c)) {
    if (key === 'channels') continue;
    if (field(key).type === 'checkbox') field(key).checked = value;
    else field(key).value = value;
  }
  c.channels.forEach((ch,i) => {
    field(`enabled${i}`).checked = ch.enabled;
    field(`multiplier${i}`).value = ch.multiplier;
    field(`offset${i}`).value = ch.offset;
  });
  dirty = false;
  hints();
}
function readForm() {
  const c = {};
  for (const key of ['bus','address','rate','pga','polarity','queue','low','high','interval_ms']) c[key] = Number(field(key).value);
  for (const key of ['mode','alert']) c[key] = field(key).value;
  c.latch = field('latch').checked;
  c.channels = names.map((_,i) => ({mux:i,enabled:field(`enabled${i}`).checked,multiplier:Number(field(`multiplier${i}`).value),offset:Number(field(`offset${i}`).value)}));
  return c;
}
function hints() {
  const lsb = ranges[Number(field('pga').value)] / 32768;
  $('#resolution').textContent = `${(lsb*1e6).toFixed(4)} uV / count`;
  ['low','high'].forEach(k => $(`#${k}-volts`).textContent = `${fmt(Number(field(k).value)*lsb)} V`);
  const comparator = ['traditional','window'].includes(field('alert').value);
  ['low','high','latch','queue'].forEach(k => field(k).disabled = !comparator);
  field('polarity').disabled = field('alert').value === 'disabled';
}
async function api(path, value) {
  const r = await fetch(path, {method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(value),signal:AbortSignal.timeout(5000)});
  const data = await r.json();
  if (!r.ok) throw new Error(data.error || r.statusText);
  return data;
}
async function action(fn) {
  if (busy) return;
  busy = true;
  try { error(''); await fn(); } catch(e) { error(e.message); }
  finally { busy = false; }
}
function rebuild(c) {
  charts.forEach(x => x.destroy()); charts.clear();
  const enabled = c.channels.filter(ch => ch.enabled);
  $('#empty').hidden = enabled.length > 0;
  $('#channels').innerHTML = enabled.map(ch => `<article class="channel" style="--channel:${colors[ch.mux]}" id="channel${ch.mux}"><div class="channel-head"><h3>${names[ch.mux]}</h3><span class="flag">Waiting</span></div><div class="value">--</div><div class="subvalue">--</div><div class="plot"><canvas aria-label="${names[ch.mux]} readings" role="img"></canvas></div><div class="stats"><div>MIN<strong data-stat="min">--</strong></div><div>MAX<strong data-stat="max">--</strong></div><div>STD DEV<strong data-stat="std">--</strong></div><div>READINGS / S<strong data-stat="rate">--</strong></div></div></article>`).join('');
  enabled.forEach(ch => {
    charts.set(ch.mux, new Chart($(`#channel${ch.mux} canvas`), {type:'line',data:{datasets:[{data:[],borderColor:colors[ch.mux],borderWidth:1.5,pointRadius:0,pointHitRadius:8}]},options:{animation:false,responsive:true,maintainAspectRatio:false,parsing:false,normalized:true,plugins:{legend:{display:false},decimation:{enabled:true,algorithm:'min-max'},tooltip:{callbacks:{label:c=>`${c.parsed.y.toFixed(5)}`}}},scales:{x:{type:'linear',ticks:{maxTicksLimit:6,callback:v=>`${v}s`},grid:{display:false}},y:{ticks:{maxTicksLimit:5},grid:{color:'#edf1f1'}}}}}));
  });
}
// Keep extrema in time order while bounding chart allocations by screen width.
// Full-resolution readings remain in history for statistics and CSV export.
function plotPoints(samples, now, seconds, quantity, width) {
  const points = [], buckets = Math.max(1, Math.floor(width / 2));
  let bucket = -1, first, last, low, high;
  const flush = () => {
    if (!first) return;
    for (const sample of [...new Set([first, low, high, last])].sort((a,b)=>a.time-b.time)) {
      points.push({x:sample.time-now, y:sample[quantity]});
    }
  };
  for (const sample of samples) {
    const next = Math.floor((sample.time - now + seconds) * buckets / seconds);
    if (next !== bucket) {
      flush(); bucket = next; first = low = high = sample;
    }
    if (sample[quantity] < low[quantity]) low = sample;
    if (sample[quantity] > high[quantity]) high = sample;
    last = sample;
  }
  flush();
  return points;
}
function render(now) {
  if (!state || $('#monitor').hidden || document.hidden) return;
  const seconds = Number($('#window').value), quantity = $('#quantity').value;
  const updateStats = performance.now()-lastStats >= 100;
  if (updateStats) lastStats = performance.now();
  for (const [mux,chart] of charts) {
    const samples = history.filter(s => s.mux === mux && s.time >= now-seconds);
    const last = samples.at(-1), root = $(`#channel${mux}`);
    const fresh = last && state.connected && now-last.time < Math.max(2, state.config.interval_ms/1000 + 3);
    root.querySelector('.value').textContent = fresh ? `${quantity === 'raw' ? last.raw : fmt(last[quantity])} ${quantity === 'volts' ? 'V' : quantity === 'raw' ? 'counts' : 'scaled'}` : '--';
    root.querySelector('.subvalue').textContent = fresh ? `${last.raw} counts | ${fmt(last.volts)} V | ${fmt(last.scaled)} scaled` : last ? 'Last sample is stale' : 'No samples';
    const flag = root.querySelector('.flag');
    flag.textContent = !state.connected ? 'Disconnected' : !state.running ? 'Stopped' : !fresh ? 'Waiting' : last.clipped ? 'CLIPPED' : 'Live';
    flag.classList.toggle('clip', Boolean(fresh && last.clipped));
    chart.data.datasets[0].data = plotPoints(samples, now, seconds, quantity, chart.width);
    chart.options.scales.x.min = -seconds; chart.options.scales.x.max = 0;
    chart.update('none');
    if (!updateStats) continue;
    const ys = samples.map(s=>s[quantity]);
    const mean = ys.reduce((a,b)=>a+b,0)/(ys.length||1);
    const recent = samples.filter(s=>s.time>=now-2);
    const stats = {min:ys.length?Math.min(...ys):NaN,max:ys.length?Math.max(...ys):NaN,std:ys.length?Math.sqrt(ys.reduce((a,b)=>a+(b-mean)**2,0)/ys.length):NaN,rate:recent.length>1?(recent.length-1)/(recent.at(-1).time-recent[0].time):0};
    for(const [k,v] of Object.entries(stats)) root.querySelector(`[data-stat=${k}]`).textContent = k==='rate' && Number.isFinite(v) ? v.toFixed(1) : fmt(v);
  }
  frameCount++;
  const elapsed = performance.now() - frameStart;
  if (elapsed >= 1000) {
    $('#refresh').textContent = `Display: ${(frameCount*1000/elapsed).toFixed(0)} fps`;
    frameCount = 0; frameStart = performance.now();
  }
}
function animate(now) {
  if (state && (pendingFrame || (state.running && state.connected) || now-lastRender>=250)) {
    render(serverTime+(now-receivedAt)/1000);
    pendingFrame = false; lastRender = now;
  }
  requestAnimationFrame(animate);
}
function registers(s) {
  const hex = n => '0x'+n.toString(16).toUpperCase().padStart(4,'0');
  $('#register-values').innerHTML = ['conversion','config','low','high'].map((k,i) => `<tr><td>${k === 'conversion' ? 'Last conversion' : k}</td><td>0x0${i}</td><td>${s.registers[k] === undefined ? '--' : hex(s.registers[k])}</td><td>${s.registers[k] === undefined ? '--' : s.registers[k].toString(2).padStart(16,'0')}</td></tr>`).join('');
  $('#applied').textContent = s.connected && s.applied===s.revision ? `Applied / revision ${s.applied}` : 'Pending device configuration';
  const r = s.registers.config;
  $('#decoded').innerHTML = r === undefined ? '' : `<dt>MUX</dt><dd>${names[(r>>12)&7]}</dd><dt>PGA</dt><dd>+/-${ranges[Math.min((r>>9)&7,5)]} V</dd><dt>Data rate</dt><dd>${rates[(r>>5)&7]} SPS</dd><dt>MODE</dt><dd>${r&256?'Single-shot / power-down':'Continuous'}</dd><dt>OS</dt><dd>${r&32768?'Idle / conversion complete':'Converting'}</dd><dt>COMP_QUE</dt><dd>${['1 conversion','2 conversions','4 conversions','Disabled'][r&3]}</dd>`;
  if(r !== undefined) $('#decoded').innerHTML += `<dt>COMP_MODE</dt><dd>${r&16?'Window':'Traditional'}</dd><dt>COMP_POL</dt><dd>${r&8?'Active high':'Active low'}</dd><dt>COMP_LAT</dt><dd>${r&4?'Latching':'Nonlatching'}</dd>`;
}
async function poll() {
  const started = performance.now();
  let failed = false;
  try {
    const response = await fetch(`/api/state?after=${cursor}`, {signal:AbortSignal.timeout(5000)});
    if (!response.ok) throw new Error(`HTTP ${response.status}`);
    const s = await response.json();
    if (s.sequence < cursor) { cursor = 0; history = []; revision = 0; }
    const changedState = !state || state.running !== s.running || state.connected !== s.connected || state.revision !== s.revision;
    state = s;
    serverTime = s.now; receivedAt = performance.now();
    if (s.revision !== revision) {
      history = []; revision = s.revision; rebuild(s.config);
      if (!dirty) fill(s.config);
    }
    history.push(...s.samples);
    if(s.samples.length) cursor = s.samples.at(-1).seq;
    let expired = Math.max(0, history.length-30000);
    while (expired < history.length && history[expired].time < s.now-60) expired++;
    if (expired) history.splice(0, expired);
    let recentStart = history.length;
    while (recentStart > 0 && history[recentStart-1].time >= s.now-2) recentStart--;
    const recentCount = history.length-recentStart;
    const measuredRate = recentCount>1 ? (recentCount-1)/(history.at(-1).time-history[recentStart].time) : 0;
    $('#sampling').textContent = `ADC: ${s.running && s.connected ? measuredRate.toFixed(1) : '0'} samples/s total`;
    $('#source').textContent = s.demo ? 'DEMO / synthetic' : `I2C ${s.config.bus} / 0x${s.config.address.toString(16)}`;
    $('#status').textContent = s.error ? 'Device offline' : s.connected ? (s.running ? 'Acquiring' : 'Ready') : 'Connecting';
    $('#status').classList.toggle('bad', !!s.error);
    $('#summary').textContent = `${s.config.rate} SPS configured | ${s.config.channels.filter(ch=>ch.enabled).length} enabled | +/-${ranges[s.config.pga]} V | ${s.config.mode === 'single' ? 'Single-shot scan' : 'Continuous'}`;
    if (lastRunState !== s.running) {
      $('#run span').textContent = s.running ? 'Stop' : 'Start';
      $('#run svg').setAttribute('data-lucide', s.running ? 'square' : 'play'); icons();
      lastRunState = s.running;
    }
    $('#notice').textContent = s.error || (s.dropped ? `${s.dropped} readings missed while browser was disconnected` : dirty ? 'Unapplied settings' : s.applied!==s.revision ? 'Applying configuration...' : '');
    if (!$('#registers').hidden) registers(s);
    pendingFrame ||= changedState || s.samples.length > 0;
  } catch(e) {
    failed = true;
    $('#status').textContent = 'Server offline'; $('#status').classList.add('bad');
    $('#notice').textContent = e.message;
    if (state) { state.connected = false; pendingFrame = true; }
    $('#sampling').textContent = 'ADC: offline';
  } finally {
    const interval = failed ? 1000 : document.hidden ? 500 : 1000/60;
    setTimeout(poll, Math.max(0, interval-(performance.now()-started)));
  }
}
document.querySelectorAll('[data-view]').forEach(b => b.onclick = () => {
  for (const id of ['monitor','settings','registers']) $(`#${id}`).hidden = id !== b.dataset.view;
  document.querySelectorAll('[data-view]').forEach(x=>x.classList.toggle('active',x===b));
  frameCount = 0; frameStart = performance.now();
  pendingFrame = true;
  if (state && b.dataset.view === 'registers') registers(state);
});
form.oninput = () => { dirty = true; hints(); };
form.onsubmit = e => {e.preventDefault(); action(async()=> {await api('/api/settings',readForm());dirty=false;});};
$('#revert').onclick = () => {if(state) fill(state.config); error('');};
$('#run').onclick = () => action(async()=> {if(state) await api('/api/run',{running:!state.running});});
$('#clear').onclick = () => {history=[]; if(state) render(state.now);};
function download(name, contents, type) {
  const url = URL.createObjectURL(new Blob([contents], {type}));
  const a = document.createElement('a'); a.href=url; a.download=name; a.click();
  setTimeout(()=>URL.revokeObjectURL(url),1000);
}
$('#export').onclick = () => download('ads1115.csv','timestamp,input,raw,volts,scaled,clipped\n'+history.map(s=>[new Date(s.time*1000).toISOString(),names[s.mux],s.raw,s.volts,s.scaled,s.clipped].join(',')).join('\n'),'text/csv');
$('#save').onclick = () => download('ads1115-settings.json',JSON.stringify(readForm(),null,2),'application/json');
$('#import').onclick = () => $('#config-file').click();
$('#config-file').onchange = () => action(async()=> {
  const file = $('#config-file').files[0]; if(!file) return;
  const c = JSON.parse(await file.text());
  if(!state || Object.keys(state.config).some(k=>!(k in c)) || !Array.isArray(c.channels) || c.channels.length!==8) throw new Error('Invalid configuration file');
  fill(c); dirty = true; $('#config-file').value='';
});
icons(); requestAnimationFrame(animate); poll();
