# Operator UI audit — 2026-09-14

Workspace: `/home/mine/Documents/nils/PeaceOfMine`.

The dashboard at `http://svea-mine.tailead4c5.ts.net:8080/` now runs this
workspace in simulation mode, in container `peaceofmine-nils`. The other
container used `/home/mine/Documents/peaceofmine/src`; it was stopped with
user approval. The new container uses `ROS_DOMAIN_ID=81` to isolate simulation
traffic. Vehicle and payload are simulated; the front USB camera is real.
No hardware driving was performed during the audit.

## Fixed issues

- Global `.active` styling colored entire Settings panels red. Alert colors
  now apply only to badges.
- Short windows clipped controls; headers and device names could overflow.
  Panels grow with content, and narrow screens stack the controls.
- Camera images were cropped by `object-fit: cover`. Both previews now retain
  the full image. Inset labels follow the actual inset size.
- Camera labels intercepted clicks intended to focus keyboard driving.
- The field map was aligned to one edge, leaving an unexplained blank area.
  The lane is now centered and the grid fills the viewport.
- Discovery discarded laptop camera options, duplicated them on refresh,
  reset missing saved cameras, and overrode explicit virtual-camera choices.
  Options are rebuilt from both device lists while preserving selections.
- Camera permission results could replace a newer selection and leak tracks.
  Per-slot generations reject late results and stop obsolete captures.
- Camera errors had no automatic retry. Network previews now retry; local
  camera failures are labeled rather than silently showing a simulation.
- Slow camera viewers could accumulate buffered video and each viewer opened
  another upstream encoder. The gateway now shares one upstream stream per
  topic, keeps one latest frame per viewer, and disconnects stalled writes.
  The ROS stream uses the `sensor_data` QoS profile supported by
  [web_video_server](https://github.com/RobotWebTools/web_video_server).
- Sliders could discard their final value during throttling; release always
  sends the final value. Focus alone no longer suppresses telemetry updates.
- Keyboard, controller and wheel steering signs were opposite ROS yaw signs.
  Left now produces positive yaw; right produces negative yaw.
- Retrying a connection retained stale ownership until the old socket closed.
  Retry now stops input and clears the local lease immediately; superseded
  socket messages are ignored. Missing initial telemetry also times out.
- Spectators sent drive-stop commands when opening Settings or losing focus,
  producing avoidable server errors. Stop commands now require ownership.
- Instrument canvases redrew at display refresh rate despite 10 Hz telemetry.
  They now draw at 10 Hz, with a separate 30 Hz virtual-camera limit; hidden
  virtual cameras and hidden pages do not waste drawing work.
- An inherited `DEBUG=release` caused shell numeric-comparison errors in
  `util/build` and `util/run`. Boolean flags now use string comparison.

## Validation

- `util/build` and a container `colcon build --symlink-install
  --packages-select peaceofmine_operator` succeeded.
- Four unittest methods pass: browser controls/settings/camera races, stream
  fanout, latest-frame buffering, and unavailable-upstream recovery.
- Chromium inspected the live URL at 1440×900, 1024×768, 800×600 and 390×844.
  All Settings tabs opened; no JavaScript exceptions, horizontal page overflow,
  or clipped panel contents were detected.
- The live front preview decoded 640×480 MJPEG. CameraInfo reported roughly
  30 source FPS. A 61-frame proxy sample measured 26.2 received FPS and 55.4 ms
  to first frame. Another sample during the image rebuild measured 23.6 FPS
  and 19.1 ms to first frame. These are local-host measurements via the
  Tailscale hostname, not measurements across a remote operator's network.
- Source frame age, source FPS and WebSocket RTT are explicitly labeled.
  They do not measure camera-to-display latency. A synchronized visual target
  is needed for that measurement.

Run regressions with Chromium and aiohttp installed:

```bash
python3 -m unittest discover -s src/peaceofmine_operator/test -v
```

The audit used a temporary virtual environment at `/tmp/pom-audit-venv` for
browser automation and aiohttp, leaving system Python unchanged.

## Running instance

Inspect the launch log:

```bash
docker exec peaceofmine-nils tail -60 /tmp/pom-operator.log
```

The active launch is equivalent to:

```bash
source /opt/svea/jazzy/setup.bash
source /svea_ws/install/setup.bash
ros2 launch peaceofmine_operator operator.launch.xml is_sim:=true port:=8080
```

Use Settings → Cameras to select the discovered front camera. Saved camera
choices remain selected across reloads and discovery changes.

## Remaining limits

- Remote gamepads and laptop-camera permission require HTTPS; this requested
  HTTP URL supports network cameras and keyboard input. The UI now explains
  the gamepad restriction directly in the Drive panel.
- The auxiliary camera remains disabled by the existing launch configuration;
  only the front physical camera was validated. Fanout was also tested with
  two synthetic viewers sharing the same source.
- Real wheel enumeration, hardware driving, and remote-network video latency
  were not tested.
- Git HEAD is corrupt: object `665178ea887d023a9cd7470dc979746dadc69e2b`
  is empty. Changes remain in the working files; Git history was not modified.
- Docker build output reports a missing Zenoh repository signing key
  (`829768EDD9BD8B8F`) inherited from the base image. It did not prevent the
  build; repository trust configuration was not changed by this UI audit.
