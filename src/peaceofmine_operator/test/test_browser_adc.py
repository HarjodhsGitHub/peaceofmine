"""Exercise real ADS1115 settings and Chart.js overlays in Chromium."""
import copy
import json
from pathlib import Path
import re
import shutil
import subprocess
import tempfile
import unittest
from peaceofmine_operator.ads1115 import DEFAULTS
from peaceofmine_operator.camera_settings import DEFAULTS as CAMERA_DEFAULTS

DASHBOARD = Path(__file__).resolve().parents[1] / 'dashboard'


@unittest.skipUnless(shutil.which('chromium'), 'Chromium required')
class BrowserADCTest(unittest.TestCase):
    def test_settings_routing_and_sample_batches(self):
        config = copy.deepcopy(DEFAULTS)
        for channel in config['channels']:
            channel['enabled'] = channel['mux'] in (4, 5)
        value = dict(config=config, routes=['none']*4+['detector','probe','none','none'],
                     revision=1, applied=1, running=True, connected=True, demo=True,
                     error=None, registers={'config': 0x8583}, now=100, sequence=2,
                     samples=[dict(seq=1, time=99.9, mux=4, raw=100, volts=.1, scaled=2, clipped=False),
                              dict(seq=2, time=99.9, mux=5, raw=200, volts=.2, scaled=3, clipped=False)])
        setup = '''
window.requestAnimationFrame = () => 0;
window.setInterval = () => 0;
window.setTimeout = (fn) => { if(fn.name === 'poll') window.nextPoll = fn; return 0; };
window.WebSocket = class {static OPEN=1; readyState=1; send(data) {window.sent.push(JSON.parse(data));}};
window.sent=[];
window.fetch = async () => ({ok:true, json:async () => window.adcResponse});
'''
        checks = '''
const assert = (v,label) => {if(!v) throw Error(label);};
for(let i=0;i<10;i++) await Promise.resolve();
state.connected=true; state.drive.you_control_owner=true;
$('settings').click(); document.querySelector('[data-settings-tab="adc"]').click();
await window.nextPoll();
assert($('adc-status').textContent.includes('DEMO'), 'demo is explicit: '+$('adc-status').textContent+' '+$('adc-error').textContent);
assert(document.querySelectorAll('[data-adc-channel]').length === 8, 'all mux inputs');
assert($('adc-readings').textContent.includes('Live'), 'live readings');
const draftField=document.querySelector('[data-adc-field="rate"]');
draftField.value='64'; await window.nextPoll();
assert(draftField.value==='64', 'poll preserves edits');
$('adc-apply').click();
assert(sent.at(-1).settings.config.rate===64, 'settings sent');
assert(sent.at(-1).settings.routes[4]==='detector', 'default A0 routing');
operatorADCResult({ok:true});
drawPressureHistory(); drawDetectorHistory(1000);
assert(probePressureChart.data.datasets.at(-1).label.includes('A1'), 'A1 probe trace');
assert(probePressureChart.data.datasets.at(-1).data[0].y===3, 'scaled probe value');
assert(detectorChart.data.datasets.at(-1).label.includes('A0'), 'A0 detector trace');
assert(detectorChart.data.datasets.at(-1).data[0].y===2, 'scaled detector value');
adcResponse={...adcResponse,revision:2,routes:['none','none','none','none','both','none','none','none'],sequence:3,
 samples:[{seq:3,time:100,mux:4,raw:300,volts:.3,scaled:7,clipped:true}]};
await window.nextPoll(); drawPressureHistory(); drawDetectorHistory(2000);
assert(probePressureChart.data.datasets.length===3, 'old routing removed');
assert(probePressureChart.data.datasets.at(-1).data.length===1, 'revision clears old samples');
assert(probePressureChart.data.datasets.at(-1).data[0].y===7, 'new samples arrive');
assert($('adc-readings').textContent.includes('CLIPPED'), 'clipping shown');
adcResponse={...adcResponse,probe_trigger:{enabled:true,mux:4,level:6,direction:'above'},
 probe_contact:{enabled:true,fresh:true,detected:true,mux:4,value:7,level:6,direction:'above'},samples:[]};
await window.nextPoll(); drawPressureHistory();
assert(probePressureChart.data.datasets.at(-1).label.includes('Contact trigger'), 'probe threshold plotted');
assert(probePressureChart.data.datasets.at(-1).data[0].y===6, 'threshold uses scaled axis');
assert($('probe-adc-contact').textContent.includes('CONTACT'), 'contact indication');
adcResponse={...adcResponse,probe_contact:{...adcResponse.probe_contact,fresh:false,detected:null}};
await window.nextPoll();
assert($('probe-adc-contact').textContent.includes('No fresh'), 'stale sensor never reports contact');
assert(detectorChart.data.datasets.at(-1).data[0].y===7, 'both destination');
$('adc-revert').click(); assert(document.querySelector('[data-adc-channel="4"] select').value==='both', 'revert loads shared settings');
state.settings={cameras:structuredClone(cameraDefaults),saved:true};
state.settings.cameras.forward.rotation=90;
operatorSettingsState(state.settings);
assert(preferences.cameraRotation.forward===90, 'robot camera rotation restored');
$('forward-capture-width').value='800'; $('camera-save-shared').click();
assert(sent.at(-1).type==='settings' && sent.at(-1).value.capture.forward.width===800, 'camera settings go through ROS gateway');
preferences.cameras.forward='laptop-private-id';
state.settings.cameras.forward.rotation=180; operatorSettingsState(state.settings);
assert(preferences.cameras.forward==='laptop-private-id', 'laptop device selection remains local');
$('camera-save-shared').click();
assert(sent.at(-1).value.forward.source==='virtual', 'laptop ID never stored on robot');
state.drive.you_control_owner=false; operatorSettingsState(state.settings);
assert($('camera-save-shared').disabled, 'spectator cannot save robot defaults');
await window.nextPoll();
assert($('adc-apply').disabled && $('adc-run').disabled, 'spectators cannot configure');
window.fetch=async()=>{throw Error('offline');}; await window.nextPoll();
assert($('adc-status').textContent==='DISCONNECTED', 'network failure surfaced');
document.body.textContent='ADC BROWSER TESTS PASSED';
'''
        html = re.sub(r'<script src="/assets/[^"]+"></script>', '', (DASHBOARD/'index.html').read_text())
        html = html.replace('<link rel="stylesheet" href="/assets/style.css">', '<style>'+(DASHBOARD/'style.css').read_text()+'</style>')
        scripts = setup + 'window.adcResponse='+json.dumps(value)+';\nwindow.cameraDefaults='+json.dumps(CAMERA_DEFAULTS)+';\n'
        scripts += '\n'.join((DASHBOARD/name).read_text() for name in ('chart-4.4.8.umd.js','keydrown-1.3.0.js','actuator-settings.js','app.js','adc.js','camera-config.js'))
        html += '<script>(async()=>{try{'+scripts+checks+"}catch(e){document.body.textContent='FAILED: '+e.stack;}})();</script>"
        with tempfile.TemporaryDirectory() as tmp:
            page=Path(tmp)/'test.html'; page.write_text(html)
            result=subprocess.run(['chromium','--headless','--no-sandbox','--disable-gpu',
                                   '--user-data-dir='+tmp+'/profile','--dump-dom',page.as_uri()],
                                  capture_output=True,text=True,timeout=60)
        self.assertIn('ADC BROWSER TESTS PASSED</body>',re.sub(r'<head>.*?</head>', '', result.stdout, flags=re.S),re.sub(r'<head>.*?</head>', '', result.stdout, flags=re.S)+result.stderr[-2000:])
