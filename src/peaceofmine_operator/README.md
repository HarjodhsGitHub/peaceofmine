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
ros2 launch peaceofmine_operator operator.launch.xml
```

Open the dashboard on port `8080`. The XML file is the editable development
configuration: its arguments and defaults select simulation or hardware
without requiring a long command. The default profile is fully simulated.
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

The default camera mapping discovers the Logitech C922 (`046d:085c`) and H264
USB Camera (`05a3:9422`) through libudev, so Linux may renumber their
`/dev/video*` nodes without breaking the launch. Set `front_camera_device` or
`auxiliary_camera_device` to an explicit capture node to override discovery.
For a vehicle with only one camera, disable the missing camera with
`use_front_camera:=false` or `use_auxiliary_camera:=false`. For example, the
H264 camera visible on the PeaceOfMine vehicle can be launched as:

```bash
ros2 launch peaceofmine_operator operator.launch.xml is_sim:=false \
  use_front_camera:=false \
  auxiliary_camera_device:=/dev/v4l/by-id/usb-Sonix_Technology_Co.__Ltd._H264_USB_Camera_SN0001-video-index0
```

The resolver converts that stable link to its capture node before starting
`usb_cam`.
Their topics are `/self/camera_front/image_raw` and
`/self/camera_auxiliary/image_raw`. `web_video_server` converts those ROS image
topics into browser streams, and the operator gateway proxies them on the same
origin as the dashboard. If the desktop PipeWire service owns a video node,
`usb_cam` cannot open it; close applications using that camera or stop its
camera session before starting the ROS stack.

`fixture_angle_offset_deg` sets the centre angle of the forward fixture, for
example `fixture_angle_offset_deg:=10.0`.

## Coordinate convention

The search lane runs along world **+X**, matching the ROS convention that a
vehicle at yaw 0 faces +X. Cross-lane offsets are world Y, positive to the
rover's left. The forward camera, the field map, and the two buried targets at
(4.8, 0.8) and (9.2, -1.3) all use this frame.

## Control

Every connected browser receives the same telemetry. The header shows the
viewer count and whether this browser holds the control lease. Any viewer can
take or steal control at any time; ownership transfers immediately, the
previous owner becomes a spectator, and the gateway disarms so the new owner
must arm before sending motion.

The default controller mapping is the browser `standard` mapping: left stick
steers, right trigger drives forward, left trigger reverses, and the right
bumper is the deadman. For keyboard driving, choose **WASD keyboard** in
Settings, click the forward camera to focus it, then hold Shift while using
WASD. Both input paths share the same response smoothing and publish drive
commands at 20 Hz.

The gateway publishes `cmd_vel` only while a lease is held, armed, and the
deadman is down. It publishes zeros for 0.5 s after that stops and then goes
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
drive interface. A real camera replaces the virtual canvas with a WebRTC
`MediaStream` without changing telemetry or controls.

## Known simulator artifacts

`sim_svea.py` divides the incoming actuation percentage by
`PERC_TO_LLI_COEFF = 1.27`, so a commanded 0.8 m/s settles near 0.63 m/s in
simulation. The real vehicle does not have this scaling.

## Client input settings and wheel setup

Open **Settings** for the input lab. Opening it sends a stop and disarms this
client's control lease; previews never drive the rover. Closing it requires
arming again. Losing browser focus or hiding the page also stops and disarms.

Choose one of three schemes:

- **Xbox / standard gamepad**: explicitly select a browser device with the
  `standard` mapping. Left stick steers, RT drives forward, LT reverses, RB is
  the deadman. Non-standard mappings are blocked in this scheme.
- **WASD keyboard**: focus the test button to preview keys, or close Settings
  and focus the forward camera to drive while holding Shift.
- **Steering wheel**: select the wheel, inspect raw axes and buttons, then set
  steering, forward pedal, reverse pedal, and deadman indices. Invert axes as
  needed. Defaults are starting points, not a verified G27 mapping. Pedals
  must use distinct axes spanning -1 to +1; combined pedal axes are rejected.
  The reverse pedal commands reverse motion, not a separate vehicle brake.

Before arming, verify released pedals produce zero speed, steering is centered,
full pedal travel reaches the expected signed speed, and only the chosen
button activates the deadman. Preferences are stored locally in this browser;
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
