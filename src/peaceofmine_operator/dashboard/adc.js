/* ADS1115 settings and batched telemetry. Acquisition belongs to the gateway. */
(() => {
  const el = id => document.getElementById(`adc-${id}`);
  const names = ['A0 − A1', 'A0 − A3', 'A1 − A3', 'A2 − A3', 'A0', 'A1', 'A2', 'A3'];
  const colors = ['#d896ef', '#f5ac72', '#8ca8ff', '#ec8fac', '#66dbbd', '#efcf70', '#73c9ff', '#bac888'];
  const ranges = [6.144, 4.096, 2.048, 1.024, .512, .256];
  const fields = [...document.querySelectorAll('[data-adc-field]')];
  const field = key => fields.find(input => input.dataset.adcField === key);
  field('rate').innerHTML = [8,16,32,64,128,250,475,860].map(n => `<option value="${n}">${n} SPS</option>`).join('');
  field('pga').innerHTML = ranges.map((n,i) => `<option value="${i}">±${n} V</option>`).join('');
  el('input-settings').innerHTML = names.map((name,i) => `<tr data-adc-channel="${i}"><td><input type="checkbox" aria-label="Enable ${name}"></td><td>${name}</td><td><input type="number" step="any" aria-label="${name} multiplier"></td><td><input type="number" step="any" aria-label="${name} offset"></td><td><select aria-label="${name} signal use"><option value="none">None</option><option value="detector">Metal detector</option><option value="probe">Probe</option><option value="both">Both</option></select></td></tr>`).join('');
  const rows = [...document.querySelectorAll('[data-adc-channel]')];
  el('trigger-mux').innerHTML = names.map((name,i) => `<option value="${i}">${name}</option>`).join('');
  let latest = null, revision = null, session = null, cursor = 0, history = [], chart = null, clockOffset = 0;
  let pending = false, importedTrigger = null;
  const detectorFilters = new Map();
  const contactDirection = document.getElementById('probe-contact-direction');
  const contactVolts = document.getElementById('probe-contact-volts');
  const contactSave = document.getElementById('probe-contact-save');
  const contactFeedback = document.getElementById('probe-contact-feedback');
  let contactDraftKey = '', contactDirty = false, contactPending = false, contactMessage = '';
  const opposite = direction => direction === 'above' ? 'below' : 'above';
  function contactConfigError() {
    if (!latest?.probe_trigger) return 'Waiting for contact settings.';
    const mux = latest.probe_trigger.mux;
    if (!Number.isInteger(mux) || mux < 0 || mux >= names.length) return 'Contact input is invalid. Check Settings → ADS1115.';
    if (!Number.isFinite(latest.probe_trigger.level) || !['above', 'below'].includes(latest.probe_trigger.direction)) return 'Contact trigger is invalid. Check Settings → ADS1115.';
    const channel = latest.config.channels.find(channel => channel.mux === mux);
    if (!channel?.enabled) return `Enable ${names[mux]} in Settings → ADS1115.`;
    if (!['probe', 'both'].includes(latest.routes[mux])) return `Assign ${names[mux]} to Probe in Settings → ADS1115.`;
    if (!Number.isFinite(channel.multiplier) || channel.multiplier === 0) return 'Contact input multiplier must be nonzero. Check Settings → ADS1115.';
    if (!Number.isFinite(channel.offset)) return 'Contact input offset is invalid. Check Settings → ADS1115.';
    return '';
  }
  window.operatorContactBlock = () => contactConfigError() || (pending ? 'Wait for ADC settings to finish saving.' : contactDirty ? 'Save the contact limit first.' : '');
  function renderMainContact() {
    const error = contactConfigError();
    contactSave.disabled = !state.connected || !state.drive.you_control_owner || pending || Boolean(error);
    contactDirection.disabled = contactVolts.disabled = pending;
    if (!error) {
      const trigger = latest.probe_trigger;
      const channel = latest.config.channels.find(channel => channel.mux === trigger.mux);
      const key = JSON.stringify([trigger, channel.multiplier, channel.offset]);
      if (!contactDirty && key !== contactDraftKey) {
        contactVolts.value = ((trigger.level - channel.offset) / channel.multiplier).toFixed(6);
        contactDirection.value = channel.multiplier < 0 ? opposite(trigger.direction) : trigger.direction;
        contactDraftKey = key;
      }
      const contact = latest.probe_contact;
      const volts = contact?.fresh && Number.isFinite(contact.value) ? (contact.value - channel.offset) / channel.multiplier : null;
      document.getElementById('probe-adc-contact').textContent = !trigger.enabled ? 'Contact limit disabled'
        : volts === null ? 'Waiting for contact reading.' : `${contact.detected ? 'CONTACT' : 'Clear'} · ${volts.toFixed(6)} V`;
    }
    contactFeedback.textContent = error || (contactPending ? 'Saving contact limit…' : contactMessage)
      || (!state.drive.you_control_owner ? 'Take control to save the contact limit.'
        : contactDirty ? 'Contact limit changed. Save before scanning.'
        : latest?.probe_trigger.enabled ? 'Saved contact limit' : 'Save a limit to enable contact detection.');
  }
  const editContact = () => { contactDirty = true; contactMessage = ''; renderMainContact(); };
  contactVolts.oninput = editContact;
  contactDirection.onchange = editContact;
  contactSave.onclick = () => {
    const error = contactConfigError();
    if (error) { contactMessage = error; renderMainContact(); return; }
    if (!state.connected || !state.drive.you_control_owner || pending) return;
    if (!contactVolts.reportValidity()) return;
    const volts = contactVolts.valueAsNumber;
    const trigger = latest.probe_trigger;
    const channel = latest.config.channels.find(channel => channel.mux === trigger.mux);
    const level = volts * channel.multiplier + channel.offset;
    if (!Number.isFinite(volts) || !Number.isFinite(level)) { contactMessage = 'Enter a finite contact limit in volts.'; renderMainContact(); return; }
    const direction = channel.multiplier < 0 ? opposite(contactDirection.value) : contactDirection.value;
    const settings = {config: structuredClone(latest.config), routes: [...latest.routes],
      probe_trigger: {...trigger, enabled: true, level, direction}};
    contactPending = true;
    contactMessage = '';
    command({action: 'settings', settings});
    renderMainContact();
  };

  function fill(value) {
    fields.forEach(input => {
      const v = value.config[input.dataset.adcField];
      if (input.type === 'checkbox') input.checked = v; else input.value = v;
    });
    rows.forEach((row,i) => {
      const inputs = row.querySelectorAll('input');
      inputs[0].checked = value.config.channels[i].enabled;
      inputs[1].value = value.config.channels[i].multiplier;
      inputs[2].value = value.config.channels[i].offset;
      row.querySelector('select').value = value.routes[i];
    });
    const trigger = value.probe_trigger || latest?.probe_trigger || {enabled:false,mux:4,level:1,direction:'above'};
    el('trigger-enabled').checked = trigger.enabled;
    el('trigger-mux').value = trigger.mux;
    el('trigger-hysteresis').value = trigger.hysteresis || 0;
    el('trigger-debounce').value = trigger.debounce_ms || 0;
    equivalents();
  }
  function draft() {
    const config = {};
    fields.forEach(input => {
      if (input.type === 'number' && (!input.value || !input.checkValidity())) throw Error(`Invalid ${input.dataset.adcField}`);
      config[input.dataset.adcField] = input.type === 'checkbox' ? input.checked :
        ['mode','alert'].includes(input.dataset.adcField) ? input.value : Number(input.value);
    });
    config.channels = rows.map((row,mux) => {
      const inputs = row.querySelectorAll('input');
      if ([...inputs].slice(1).some(input => !input.value || !input.checkValidity())) throw Error('Enter finite scaling values');
      return {mux, enabled: inputs[0].checked, multiplier: Number(inputs[1].value), offset: Number(inputs[2].value)};
    });
    const trigger = importedTrigger || latest?.probe_trigger;
    if (!Number.isFinite(trigger?.level) || Math.abs(trigger.level) > 1e12 || !['above','below'].includes(trigger.direction)) throw Error('Invalid saved contact limit. Set the voltage and direction in Probe.');
    for (const key of ['hysteresis', 'debounce']) {
      if (!el(`trigger-${key}`).value || !el(`trigger-${key}`).checkValidity()) throw Error(`Invalid contact ${key}`);
    }
    return {config, routes: rows.map(row => row.querySelector('select').value),
      probe_trigger:{enabled:el('trigger-enabled').checked,mux:Number(el('trigger-mux').value),
        level:trigger.level,direction:trigger.direction,
        hysteresis:Number(el('trigger-hysteresis').value),debounce_ms:Number(el('trigger-debounce').value)}};
  }
  function equivalents() {
    const lsb = ranges[Number(field('pga').value)] / 32768;
    el('resolution').textContent = `${(lsb * 1e6).toFixed(3)} µV / count`;
    ['low','high'].forEach(key => el(`${key}-volts`).textContent = `${(Number(field(key).value) * lsb).toFixed(6)} V`);
  }
  fields.forEach(input => input.addEventListener('input', equivalents));
  function command(value) {
    if (!state.connected || !state.drive.you_control_owner) { el('feedback').textContent = 'Take control before changing ADC settings.'; return; }
    if (pending) return;
    pending = true;
    el('apply').disabled = true;
    el('feedback').textContent = 'Saving…';
    send({type:'adc', ...value});
  }
  window.operatorADCResult = message => {
    pending = false;
    el('feedback').textContent = message.ok ? message.message || 'Saved' : message.message;
    if (contactPending) {
      contactPending = false;
      if (message.ok) { contactDirty = false; importedTrigger = null; }
      contactMessage = message.ok ? 'Contact limit saved' : message.message;
    }
    render();
  };
  el('apply').onclick = () => { try { command({action:'settings', settings:draft()}); } catch (error) { el('feedback').textContent = error.message; } };
  el('revert').onclick = () => { if (latest) { importedTrigger = null; fill(latest); } };
  function download(text, type, name) {
    const url = URL.createObjectURL(new Blob([text], {type}));
    const a = document.createElement('a'); a.href = url; a.download = name; a.click();
    setTimeout(() => URL.revokeObjectURL(url), 1000);
  }
  el('save').onclick = () => { try { download(JSON.stringify(draft(),null,2), 'application/json', 'ads1115.json'); } catch (error) { el('feedback').textContent = error.message; } };
  el('import').onclick = () => el('file').click();
  el('file').onchange = async () => {
    try {
      const value = JSON.parse(await el('file').files[0].text());
      const config = value.config || value; // Also accept the standalone bench export.
      const routes = value.routes || latest?.routes || Array(8).fill('none');
      if (!config.channels || config.channels.length !== 8 || routes.length !== 8 || fields.some(f => !(f.dataset.adcField in config))) throw Error('Invalid ADC configuration');
      if (value.probe_trigger && (!Number.isFinite(value.probe_trigger.level) || Math.abs(value.probe_trigger.level) > 1e12 || !['above','below'].includes(value.probe_trigger.direction))) throw Error('Invalid imported contact limit');
      fill({config,routes,probe_trigger:value.probe_trigger});
      importedTrigger = value.probe_trigger || null;
      el('feedback').textContent = 'Imported draft. Apply & save to use it.';
    } catch (error) { el('feedback').textContent = error.message; }
    el('file').value = '';
  };
  el('csv').onclick = () => download('time,input,raw,volts,scaled,clipped\n' + history.map(s => [s.time,names[s.mux],s.raw,s.volts,s.scaled,s.clipped].join(',')).join('\n'), 'text/csv', 'ads1115.csv');
  const now = () => Date.now()/1000 + clockOffset;
  function datasets(target, quantity, seconds) {
    if (!latest) return [];
    const time = now();
    return latest.config.channels.filter(ch => ch.enabled && (!target || [target,'both'].includes(latest.routes[ch.mux]))).map(ch => {
      const samples = history.filter(s => s.mux === ch.mux && time-s.time <= seconds);
      const data = [];
      const gap = Math.max(.5, latest.config.interval_ms/1000 + 3 * latest.config.channels.filter(c => c.enabled).length/latest.config.rate);
      samples.forEach((s,i) => {
        if (i && s.time-samples[i-1].time > gap) data.push({x:s.time-time-.001, y:null});
        data.push({x:s.time-time,y:s[quantity]});
      });
      return {label:`ADS ${names[ch.mux]} (${quantity === 'detector_filtered' ? 'filtered scaled' : quantity})`, yAxisID:'ads1115', data, borderColor:colors[ch.mux], borderWidth:1.5, pointRadius:0, spanGaps:false};
    });
  }
  window.operatorADCOverlay = (target, route, baseCount) => {
    const extra = datasets(route, route === 'detector' ? 'detector_filtered' : 'scaled', 30);
    const trigger = latest?.probe_trigger;
    if (route === 'probe' && trigger?.enabled) extra.push({label:`Contact trigger (${names[trigger.mux]})`,
      yAxisID:'ads1115', data:[{x:-30,y:trigger.level},{x:0,y:trigger.level}],
      borderColor:'#ff9b89',borderDash:[5,4],borderWidth:1,pointRadius:0});
    target.data.datasets = [...target.data.datasets.slice(0,baseCount), ...extra];
    target.options.scales.ads1115 = {type:'linear', position:'right', display:extra.length > 0,
      title:{display:true,text:'ADS1115 scaled'}, grid:{drawOnChartArea:false}, ticks:{color:'#efcf70'}};
  };
  function render() {
    if (!latest) return;
    el('apply').disabled = pending || !state.connected || !state.drive.you_control_owner;
    el('status').textContent = `${latest.demo ? 'DEMO · ' : ''}${latest.error || latest.load_error ? 'ERROR' : !latest.running ? 'UNAVAILABLE' : latest.applied !== latest.revision ? 'PENDING' : 'SAMPLING'}`;
    el('error').textContent = latest.error || latest.load_error || (!latest.running ? 'Acquisition is unavailable. Check the ADC connection.' : '');
    const contact = latest.probe_contact;
    const contactText = !contact?.enabled ? 'Trigger disabled' : !contact.fresh ? 'No fresh trigger reading' :
      `${contact.detected ? 'CONTACT' : 'Clear'} · ${names[contact.mux]} ${contact.value.toFixed(4)} (trigger ${contact.direction === 'above' ? '≥' : '≤'} ${contact.level})`;
    el('trigger-state').textContent = contactText;
    renderMainContact();
    if (!document.getElementById('settings-dialog').open || document.getElementById('settings-adc').hidden) return;
    const seconds = Number(el('window').value), quantity = el('quantity').value;
    window.operatorADCFrame?.();
    const recent = history.filter(s => now()-s.time <= 2);
    el('rate').textContent = `${(recent.length/2).toFixed(1)} samples/s total · ${latest.config.rate} SPS chip rate shared across enabled inputs`;
    el('readings').innerHTML = latest.config.channels.filter(ch => ch.enabled).map(ch => {
      const samples = history.filter(s => s.mux===ch.mux && now()-s.time<=seconds);
      const last = samples.at(-1), values = samples.map(s=>s[quantity]);
      const fresh = last && latest.running && latest.connected && now()-last.time < Math.max(1, latest.config.interval_ms/1000 + 2);
      const fmt = n => Number(n).toFixed(4);
      return `<tr><td>${names[ch.mux]}</td><td>${fresh ? last.raw : '—'}</td><td>${fresh ? fmt(last.volts) : '—'}</td><td>${fresh ? fmt(last.scaled) : '—'}</td><td>${values.length ? [Math.min(...values),Math.max(...values),values.reduce((a,b)=>a+b,0)/values.length].map(fmt).join(' / ') : '—'}</td><td>${!fresh ? 'STALE' : last.clipped ? 'CLIPPED' : 'Live'}</td></tr>`;
    }).join('');
    el('registers').textContent = `${latest.applied === latest.revision ? 'Applied' : 'Applying settings'} · revision ${latest.revision}\n` + Object.entries(latest.registers).map(([k,v])=>`${k}: 0x${v.toString(16).padStart(4,'0')}  ${v.toString(2).padStart(16,'0')}`).join('\n');
  }
  // Plot refresh follows the display; acquisition and table updates do not.
  window.operatorADCFrame = () => {
    if (!latest || !document.getElementById('settings-dialog').open || document.getElementById('settings-adc').hidden) return;
    if (chart && scrollTimeChart(chart, performance.now())) return;
    const seconds = Number(el('window').value), quantity = el('quantity').value;
    if (!chart) chart = new Chart(el('history'), {type:'line', data:{datasets:[]}, options:{animation:false, responsive:true, maintainAspectRatio:false, parsing:false,
      plugins:{decimation:{enabled:true,algorithm:'min-max'},legend:{labels:{color:'#b7c4d8'}}}, scales:{x:{type:'linear',min:-seconds,max:0},ads1115:{type:'linear',position:'left'}}}});
    chart.data.datasets = datasets(null,quantity,seconds); chart.options.scales.x.min = -seconds; chart.$scrollPixels = 0; chart.$renderedAt = performance.now(); chart.update('none');
  };
  async function poll() {
    try {
      const response = await fetch(`/api/adc?after=${cursor}&session=${encodeURIComponent(session || "")}`, {cache:'no-store', signal:AbortSignal.timeout(3000)});
      if (!response.ok) throw Error(`ADC telemetry: HTTP ${response.status}`);
      const value = await response.json();
      if (session !== value.session) { cursor=0; history=[]; detectorFilters.clear(); revision=null; session=value.session; }
      if (value.sequence < cursor) { cursor=0; history=[]; detectorFilters.clear(); revision=null; }
      if (revision !== value.revision) { history=[]; detectorFilters.clear(); if (revision === null) fill(value); revision=value.revision; }
      latest=value; clockOffset=(value.now-Date.now()/1000);
      const tau = state.detector.sensor?.filter_tau_s ?? 0.1;
      if (!Number.isFinite(tau) || tau <= 0) throw Error('Invalid detector filter time constant');
      for (const sample of value.samples) {
        if (!['detector','both'].includes(value.routes[sample.mux])) continue;
        if (!Number.isFinite(sample.time) || !Number.isFinite(sample.scaled)) throw Error('Invalid detector ADC sample');
        const previous = detectorFilters.get(sample.mux);
        const dt = previous ? sample.time - previous.time : 0;
        sample.detector_filtered = !previous || dt < 0 || dt > 0.5 ? sample.scaled
          : previous.value + (1 - Math.exp(-dt / tau)) * (sample.scaled - previous.value);
        detectorFilters.set(sample.mux, {time: sample.time, value: sample.detector_filtered});
      }
      history.push(...value.samples); cursor=value.samples.at(-1)?.seq ?? cursor;
      history=history.filter(s=>value.now-s.time<=60).slice(-30000);
      if (value.dropped) el('feedback').textContent = `${value.dropped} readings missed during disconnection.`;
      render();
    } catch (error) { el('status').textContent='DISCONNECTED'; el('error').textContent=error.message; pending=false; if (contactPending) { contactPending=false; contactMessage='Contact limit save was not confirmed. Check the connection.'; } renderMainContact(); document.getElementById('probe-adc-contact').textContent='ADC disconnected'; }
    setTimeout(poll, document.hidden ? 1000 : 100);
  }
  poll();
})();
