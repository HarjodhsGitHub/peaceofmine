# PeaceOfMine operator simulator

A browser operator surface for the SVEA stack, split into two nodes:

- `operator_gateway.py` serves the dashboard and translates a small WebSocket
  protocol to and from ROS. It publishes `cmd_vel` and nothing else.
- `simulated_payload.py` stands in for the detector and probe hardware. It is
  the only source of simulated payload data.

The gateway and the dashboard never compute detector, pressure, depth, or
fixture values; they only display what arrives on ROS topics. Swapping in real
drivers means replacing `simulated_payload.py` alone.

## Run the simulator

Build the workspace in the SVEA container, source the overlay, then run:

```bash
ros2 launch peaceofmine_operator operator.launch.xml is_sim:=true
```

Open the dashboard on port `8080`. The XML file is the editable development
configuration: its arguments and defaults select simulation or hardware
without requiring a long command. Set `is_sim:=true` for simulation; the default
profile is hardware.
The launch starts without TLS for local
debugging; remote operation must supply both `tls_cert` and `tls_key`, because
the browser Gamepad API requires a secure context on non-localhost origins.

For a persistent local setup, edit the defaults near the top of
`launch/operator.launch.xml`. The main switches are `is_sim`,
`simulate_payload`, and `use_camera`. A real vehicle should use
`is_sim="false"`; this starts the SVEA low-level interface, so verify the serial
device and keep the vehicle safe before launching it. A USB camera can be used
with either vehicle mode by setting `use_camera="true"`; it requires the
`usb_cam` ROS package and container access to `camera_device`. It publishes ROS
images for onboard integration work.

The Settings dialog is split into **Controls**, **Cameras**, and **Connection**
tabs. The Cameras tab independently assigns either Raspberry Pi ROS camera to
the main preview or optional inset preview. It can also use cameras connected
to the computer running the browser; click **Allow laptop cameras** to grant
access and populate those device names. Browser camera access and Gamepad input
require HTTPS on a remote address; localhost is treated as secure by browsers.

The camera resolver discovers available V4L2 capture devices through
`/dev/v4l/by-id` and does not require a vendor or model configured in the
launch file. The first discovered camera is used for the front stream. The
auxiliary stream is optional and disabled by default; enable it when a second
capture device is connected with `use_auxiliary_camera:=true`.

For a vehicle with no front camera, disable it with
`use_front_camera:=false`. To override discovery for a specific device, set
`front_camera_device` or `auxiliary_camera_device` to an explicit capture node.
For example:

```bash
ros2 launch peaceofmine_operator operator.launch.xml is_sim:=false \
  front_camera_device:=/dev/v4l/by-id/usb-Sonix_Technology_Co.__Ltd._H264_USB_Camera_SN0001-video-index0
```

The resolver converts that stable link to its capture node before starting
`usb_cam`. Live ROS camera topics are discovered by the gateway and are the
only Raspberry Pi camera sources shown in the dashboard settings.
Their topics are `/self/camera_front/image_raw` and
`/self/camera_auxiliary/image_raw`. `web_video_server` converts those ROS image
topics into browser streams, and the operator gateway proxies them on the same
origin as the dashboard. If the desktop PipeWire service owns a video node,
`usb_cam` cannot open it; close applications using that camera or stop its
camera session before starting the ROS stack.

`fixture_angle_offset_deg` sets the centre angle of the forward fixture, for
example `fixture_angle_offset_deg:=10.0`.

## Arduino detector launch

The operator XML includes the standalone `sensor_serial.launch.xml` by default
in hardware mode. Set `use_sensor_serial:=false` to disable it.
`sensor_serial_port` defaults to the configured Uno's
full `/dev/serial/by-id/...` path; override it for another board. Optional settings are
`sensor_baud_rate` (115200), `sensor_baseline_adc` (0.0), and
`sensor_full_response_adc` (255.0). The endpoints require detector calibration.
The real sensor is excluded in vehicle or payload simulation. Its topics share
the operator namespace, for example `/self/detector/amplitude_adc`.
See [sensor setup](../../DevTools/sensorArduinoFast/README.md) for bench testing.

## Coordinate convention

The search lane runs along world **+X**, matching the ROS convention that a
vehicle at yaw 0 faces +X. Cross-lane offsets are world Y, positive to the
rover's left. The forward camera, the field map, and the two buried targets at
(4.8, 0.8) and (9.2, -1.3) all use this frame.

## Control

Every connected browser receives the same telemetry. The header shows the
viewer count and whether this browser holds the control lease. Any viewer can
take or steal control at any time; ownership transfers immediately, the
previous owner becomes a spectator, and the gateway stops active motion.
Only the owner can drive, sweep, move the probe, or change calibration.
The physical RC determines drive authority; there is no extra browser arm step.

Plug the Xbox 360 (or another gamepad) into the computer that is showing this
dashboard, then press any button if the Drive panel still says no controller
is connected. Chrome often ignores a pad until that first press.

The default **Auto** mapping uses the browser `standard` layout when the pad
reports it: left stick steers, right trigger drives forward, left trigger
reverses. An Xbox 360 on Linux often
reports an empty mapping and puts the triggers on axes; Auto then uses that
Xbox 360 layout. Settings → **Controller mapping** can force Standard or
Xbox 360 (Linux) if the steer/throttle meters do not follow the pad. For
keyboard driving, choose **WASD keyboard** in Settings, click the forward
camera to focus it, then use WASD without Shift. Gamepad sticks map linearly (0.12 deadzone, no extra smoothing). WASD
throttle and steering ramp progressively. Both send changed commands promptly,
plus a 20 Hz keepalive while moving.

Browser gamepad input on a non-localhost URL requires HTTPS (`tls_cert` and
`tls_key`). Alternatively, connect the Xbox to the SVEA USB and launch with
`use_joy:=true`: ROS joystick input works over plain HTTP and takes priority
over browser drive commands while the joystick is live.

The gateway publishes `cmd_vel` only while a lease is held, RC permits ROS
driving, and input is newer than 0.25 s. It publishes zeros for 0.5 s after input stops and then goes
silent, so `twist_consumer`'s own command timeout stays an independent safety
layer and an idle dashboard does not compete with other `cmd_vel` publishers.

## Hardware replacement points

Replace `simulated_payload.py` with hardware drivers, keeping this interface:

| Topic | Type | Direction | Meaning |
| --- | --- | --- | --- |
| `detector/signal_ratio` | `Float32` | published | Calibrated detector strength, 0.0-1.0 |
| `fixture/angle_deg` | `Float32` | published | Measured fixture angle from the vehicle forward axis |
| `fixture/sweep_enabled_state` | `Bool` | published | Measured scan state |
| `fixture/sweep_speed_deg_s_state` | `Float32` | published | Measured scan speed |
| `fixture/sweep_enabled` | `Bool` | subscribed | Operator scan request |
| `fixture/sweep_speed_deg_s` | `Float32` | subscribed | Operator scan speed request |
| `probe/depth_mm` | `Float32` | published | Measured depth, never a requested depth |
| `probe/pressure_ratio` | `Float32` | published | Measured contact pressure, 0.0-1.0 |
| `probe/fault` | `Bool` | published | Probe fault state |
| `probe/target_depth_mm` | `Float32` | subscribed | Operator depth request, to be applied only after the driver's own limits and interlocks |

Actuator limits live in `operator_gateway.py` (`SWEEP_SPEED_MIN_DEG_S`,
`SWEEP_SPEED_MAX_DEG_S`, and the `max_probe_depth_mm` parameter). The dashboard
reads them from telemetry, so the browser holds no copy of its own.

The gateway commands `cmd_vel`, which `svea_examples/twist_consumer.py`
consumes, so the simulator's `sim_svea.py` and the real PX4 path share one
drive interface. Network cameras replace the virtual canvas with a same-origin MJPEG preview;
laptop cameras use a local `MediaStream`. Neither changes telemetry or controls.
The gateway shares one upstream encoder per topic and retains only the newest
queued frame per viewer. Camera badges show source FPS, source frame age and
WebSocket RTT; these are not camera-to-display latency measurements.

## Known simulator artifacts

`sim_svea.py` divides the incoming actuation percentage by
`PERC_TO_LLI_COEFF = 1.27`, so a commanded 0.8 m/s settles near 0.63 m/s in
simulation. The real vehicle does not have this scaling.

## Client input settings and wheel setup

Open **Settings** for the input lab. Opening it sends a stop and disarms this
client's control lease; previews never drive the rover. Closing it requires
arming again. Losing browser focus or hiding the page also stops and disarms.

Choose one of three schemes:

- **Xbox / standard gamepad**: the dashboard selects an active browser pad
  automatically; Settings can select a specific device and override the
  automatically detected `standard` or Xbox 360 mapping.
  Left stick steers, RT drives forward, LT reverses. RC controls drive authority.
- **WASD keyboard**: focus the test button to preview keys, or close Settings
  and focus the forward camera to drive with WASD.
- **Steering wheel**: select the wheel, inspect raw axes and buttons, then set
  steering, forward pedal, and reverse pedal indices. Invert axes as
  needed. Defaults are starting points, not a verified G27 mapping. Pedals
  must use distinct axes spanning -1 to +1; combined pedal axes are rejected.
  The reverse pedal commands reverse motion, not a separate vehicle brake.

Before driving, verify released pedals produce zero speed, steering is centered,
and full pedal travel reaches the expected signed speed. Preferences are stored locally in this browser;
device selection is deliberately required again after a page reload.

Connect the wheel to the Chrome client computer. Use HTTPS when accessing a
remote gateway (localhost is also supported), focus the page, and press a
controller button to expose the device through the Gamepad API. Seeing the
G27 in USB tools as `046d:c294` does not establish browser compatibility:
OS drivers and VM USB passthrough can affect what Chrome sees. There is no
Gamepad API device permission picker. Raw WebHID driving and force feedback
are not implemented.

Useful upstream diagnostics and examples:

- [Chrome Gamepad tester](https://googlechrome.github.io/samples/gamepad-demo/)
- [Tester source on GitHub](https://github.com/GoogleChrome/samples/tree/gh-pages/gamepad-demo)
- [Chrome WebHID documentation](https://developer.chrome.com/docs/capabilities/hid)

Run browser regression checks with Chromium installed:

```bash
python3 -m unittest discover -s src/peaceofmine_operator/test -v
```

The test uses synthetic devices in headless Chromium; actual G27 enumeration,
axis mapping, and ROS simulation still need verification in the deployment
environment before hardware driving.

See [the UI audit](../../docs/development/operator-ui-audit.md) for fixes,
validation results, and the running nils simulation configuration.

### Camera settings and keyboard response

Settings → Cameras shows the current main and inset selections using the
existing capture streams. Each card includes a live preview, source details,
resolution, and source frame age for ROS cameras. FPS is labeled as measured
source FPS for ROS, configured capture FPS for browser cameras, or the rendering
target for the virtual camera. Unavailable metadata is shown as a dash.
Rotate changes the local view by 90° and saves the orientation independently for
each slot; it does not modify the camera sensor or steering directions.

WASD uses bundled Keydrown 1.3.0 for held-key input. While drive keys are held,
throttle reaches full scale in about 0.63 seconds and steering in about 0.31
seconds. Releasing WASD ramps throttle down and recenters steering. Releasing
the drive keys, losing focus, disconnecting, or opening Settings bypasses the ramp and
stops immediately. The Settings keyboard test previews the same ramp without
sending motion. Xbox and wheel mappings are unchanged. Connection status and
retry remain in the main dashboard; the redundant Connection settings tab is removed.

### Arm servo and RC authority

Servo motion requires fresh connected PX4 `system_status == 4` and fresh RC input
with kill clear; RC override does not block servos. Vehicle drive additionally
requires RC ROS authority and PX4 arming, with no separate browser arming.
The header distinguishes ROS mode, RC override, kill, disarmed, and unknown/lost
status. Settings → Arm calibration records safe limits and exports launch XML.
See [the safety audit and setup](../../docs/development/arm-servo-safety.md) before
enabling the hardware driver. Safety uses existing `mavros/state` and `mavros/rc/in`;
no PX4 firmware changes are required. The ArbotiX requires
[safe_arm firmware v1](../../DevTools/servodemo/firmware/safe_arm/README.md) for
the permission watchdog and locally eased sweep. Legacy ArbotiX firmware is
rejected before motion. Missing or stale MAVROS state leaves actuation locked.
The XML defaults to hardware mode with the verified FTDI adapter and servo ID 1 enabled.
Use `is_sim:=true` for simulation; this excludes the hardware arm driver.
Servo limits remain unset until calibration.
