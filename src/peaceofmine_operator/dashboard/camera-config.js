/* Shared robot camera defaults travel through operator/settings ROS topics. */
(() => {
  const slots = ['forward','auxiliary'];
  let latest = null, signature = null;
  const capture = document.getElementById('camera-capture-settings');
  capture.innerHTML = slots.map(slot => `<fieldset><legend>${slot === 'forward' ? 'Main' : 'Inset'} robot camera capture</legend>
    <label>Device<input id="${slot}-capture-device" type="text" placeholder="auto or /dev/v4l/by-id/…"></label>
    <label>Width<input id="${slot}-capture-width" type="number" min="16" max="4096"></label>
    <label>Height<input id="${slot}-capture-height" type="number" min="16" max="4096"></label>
    <label>Frames/s<input id="${slot}-capture-framerate" type="number" min="1" max="120" step="any"></label>
    <label>Pixel format<select id="${slot}-capture-pixel_format">${['mjpeg2rgb','yuyv2rgb','uyvy2rgb','rgb8','mono8'].map(v=>`<option>${v}</option>`).join('')}</select></label>
    </fieldset>`).join('');
  const portable = source => ['off','virtual'].includes(source) || source.startsWith('raspberry:');
  function applyViews(cameras) {
    for (const slot of slots) {
      if (!portable(preferences.cameras[slot])) continue;
      preferences.cameras[slot] = cameras[slot].source;
      preferences.cameraRotation[slot] = cameras[slot].rotation;
      startCamera(slot, cameras[slot].source);
    }
    savePreferences(); rebuildCameraOptions(); applyCameraRotations();
  }
  window.operatorSettingsState = value => {
    $('camera-save-shared').disabled = !value || !state.connected || !state.drive.you_control_owner;
    if (!value?.cameras) return;
    latest = value.cameras;
    const key = JSON.stringify([latest, value.saved]);
    if (key !== signature) {
      signature = key;
      for (const slot of slots) for (const [name,v] of Object.entries(latest.capture[slot])) $(slot+'-capture-'+name).value=v;
      if (value.saved) applyViews(latest);
    }
    if (value.error) $('camera-config-feedback').textContent = value.error;
  };
  $('camera-save-shared').onclick = () => {
    if (!latest || !state.connected || !state.drive.you_control_owner) return;
    try {
      const value = structuredClone(latest);
      for (const slot of slots) {
        if (portable(preferences.cameras[slot])) value[slot] = {source:preferences.cameras[slot],rotation:preferences.cameraRotation[slot]};
        for (const key of Object.keys(value.capture[slot])) {
          const input = $(slot+'-capture-'+key);
          if (!input.value || !input.checkValidity()) throw Error('Enter valid camera capture settings');
          value.capture[slot][key] = ['width','height','framerate'].includes(key) ? Number(input.value) : input.value;
        }
      }
      send({type:'settings', section:'cameras', value});
      $('camera-config-feedback').textContent='Saving shared camera defaults…';
    } catch (error) { $('camera-config-feedback').textContent=error.message; }
  };
  $('camera-revert-shared').onclick = () => { if (latest) { signature=null; operatorSettingsState(state.settings); } };
  window.operatorSettingsResult = message => $('camera-config-feedback').textContent = message.message || (message.ok ? 'Saved' : 'Settings rejected');
})();
