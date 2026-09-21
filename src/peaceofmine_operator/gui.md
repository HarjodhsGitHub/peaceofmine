# Operator GUI

Browser dashboard for driving a PeaceOfMine SVEA. The Xbox on your laptop
does **not** publish a ROS topic. Chrome reads the pad and sends
`{linear_x, angular_z, deadman}` over a WebSocket. ROS starts on the car
in `operator_gateway`, which publishes `/<ns>/cmd_vel`.

Yes, this is meant to run on the **real car**. Simulation and hardware share
that same `cmd_vel` path. Hardware replaces `sim_svea` with MAVROS → PX4.
The physical RC transmitter remains the override.

## Architecture

```
Xbox on the laptop
  → Chrome Gamepad API          (no ROS on the laptop)
  → WebSocket drive messages
  → operator_gateway            (on the SVEA)
  → /self/cmd_vel               geometry_msgs/Twist
  → twist_consumer
  → /self/mavros/manual_control/send
  → PX4
```

`cmd_vel` is published as soon as a valid drive command arrives (browser
WebSocket or `/joy`), not on the next watchdog tick. Publishing continues
only while you hold the control lease, the physical RC permits ROS driving,
and commands are newer than 0.25 s. No additional GUI arming or Shift key is
required. After that the
gateway sends zeros for 0.5 s and goes silent. Stick mapping is linear
with a 0.12 deadzone; there is no extra command smoothing.

Optional second input: plug the Xbox into the **SVEA USB** and pass
`use_joy:=true`. Then `joy_node` publishes `/self/joy` and the gateway
drives from that. A live `/joy` stream wins over browser sticks. You do
not need this if the pad stays on the laptop.

## Will the laptop pad work?

Yes, if the dashboard is opened as a **secure context**:

| How you open the GUI | Laptop Xbox |
| --- | --- |
| `https://<svea-ip>:<port>` | Works (accept the self-signed cert) |
| `http://localhost:8080` on the SVEA itself | Works only if the pad is on the SVEA |
| `http://<svea-ip>:8080` from the laptop | **No** — Chrome hides the Gamepad API |

## Safety before hardware

- Stop any other operator / sim launch first (two stacks must not share
  `cmd_vel` or port 8080).
- Keep the vehicle lifted or in a clear area for the first test.
- Have the physical RC ready. Its kill switch stops actuation. Releasing the
  drive input, losing browser focus, or losing the control connection stops
  software driving.
- Do not enable localization, lidar, or RTK until those sensors are
  brought up separately.

## Exact steps (real car, Xbox on the laptop)

Run these on the SVEA. Privileged Docker is required so `/dev` and the
PX4 serial device are visible (`util/run`, not `DEV=1`).

### 1. One container, this workspace

```bash
util/run
```

Inside the container, from `/svea_ws`:

```bash
colcon build --symlink-install --packages-up-to peaceofmine_operator
source /opt/ros/jazzy/setup.bash
source /opt/svea/jazzy/setup.bash
source /svea_ws/install/setup.bash
```

If that fails with `can't copy '.../usb_camera_auto.py': doesn't exist or not a regular file`, the build tree still lists a script that is no longer in the package. Wipe that package and rebuild, then source again in the **same** shell (do not launch on a failed overlay):

```bash
rm -rf build/peaceofmine_operator install/peaceofmine_operator
colcon build --symlink-install --packages-select peaceofmine_operator
source /opt/ros/jazzy/setup.bash
source /opt/svea/jazzy/setup.bash
source /svea_ws/install/setup.bash
```

Confirm the PX4 is present:

```bash
ls /dev/serial/by-id/usb-SVEA_PX4_AUTOPILOT_0-if00
```

### 2. Launch hardware (not sim)

Still inside the container, after sourcing:

```bash
ros2 launch peaceofmine_operator operator.launch.xml is_sim:=false
```

This starts MAVROS/LLI, `twist_consumer`, and the gateway. XML is the main
operator configuration. Without certificates the dashboard uses HTTP on port
8080. Choose a free port explicitly, for example `port:=8082`, if it is busy.

It does **not** start `sim_svea`. Payload simulation and `joy_node` stay
off unless you pass `simulate_payload:=true` or `use_joy:=true`.

For a remote browser controller, provide HTTPS certificates. To generate a
development certificate, replace the example address with the SVEA's actual IP:

```bash
mkdir -p /tmp/operator-tls
openssl req -x509 -newkey rsa:2048 -nodes \
  -keyout /tmp/operator-tls/key.pem -out /tmp/operator-tls/cert.pem \
  -days 30 -subj '/CN=svea-mine' \
  -addext 'subjectAltName=DNS:localhost,DNS:svea-mine,IP:127.0.0.1,IP:192.168.1.100'
```

Then launch with both certificate paths:

```bash
ros2 launch peaceofmine_operator operator.launch.xml is_sim:=false \
  tls_cert:=/tmp/operator-tls/cert.pem \
  tls_key:=/tmp/operator-tls/key.pem
```

If the Wi-Fi address changes, regenerate the certificate with the new address.

### 3. On the laptop

1. Plug in the Xbox 360 and press a button.
2. Open `https://<svea-ip>:8080` (same IP as in the certificate; use your chosen port).
3. Accept the certificate warning.
4. Confirm the Drive panel shows the pad and the graphic moves.
5. **Take control** and select ROS authority on the physical RC.
6. RT forward, LT reverse, left stick steer. No separate browser arm step.

The status line must read `ROS cmd_vel …`. Stick lights alone are not
enough. The virtual forward camera is first-person, so the car will not
slide across that view; watch the vehicle and the speed readout.

### 4. Stop

Stop with the physical RC, then `Ctrl+C` the launch.

## Simulation only

Hardware-free check on the same `cmd_vel` path:

```bash
ros2 launch peaceofmine_operator operator.launch.xml is_sim:=true
```

Open `http://localhost:8080` on the machine that has the pad, or use
HTTPS for a remote laptop as above.

## Settings

Settings -> Servos selects Arm (default ID 1) or Probe (default ID 2) on the
shared ArbotiX. Each has independent captured positions and calibration;
switching requires stopped motion. An unplugged probe is labelled not connected
without removing the detected arm. Connect it and restart the arm driver to
rescan, or use Reconnect while stopped. Saved launch calibration uses `arm_*` or `probe_*` arguments respectively.
This selection controls servo calibration/jogging; it does not establish the
probe linkage's conversion from shaft angle to millimetres of depth.

The launch `arm_speed` is in servo register units, not degrees/s. Values above
the firmware limit of 80 are capped with a warning rather than disconnecting
the adapter. Calibration setup errors retain the detected ID list.

One failed servo-bus read reported as gateway status 32 is retried once, without
retrying motion writes or renewing permission. A repeated failure still stops
and disconnects the driver. Settings -> Servos -> Reconnect explicitly rescans
with targets cleared; it does not resume a sweep or jogging. Session calibration
is retained for the same role/ID. Fresh RC/owner permission and a new movement
request are required after reconnection.

| Controller mapping | When to use |
| --- | --- |
| Auto | Default. Browser `standard` layout if the pad reports it, otherwise Linux Xbox 360 (triggers on axes 2 and 5). |
| Standard | Force LT/RT on buttons 6/7. |
| Xbox 360 (Linux) | Force bipolar trigger axes. |

WASD: Settings → keyboard, then click the forward camera. Shift is not required.

The physical RC is authoritative. RC override blocks browser driving but permits
the control owner to sweep the servo while PX4 is active and RC kill is clear.
RC loss, kill, stale heartbeat, gateway permission timeout, or control-owner
disconnect stops the arm. Settings jogging does not require saved limits;
sweeping requires minimum, center and maximum calibration. The Drive panel shows
RC channel input in override mode. Visualization defaults to steering CH1 and
throttle CH2; adjust `rc_steering_channel` and `rc_throttle_channel` in launch XML
to match your transmitter. PWM visualization assumes 1000/1500/2000 endpoints.

Settings calibration jog/position controls use the full EEPROM joint range,
not the saved sweep endpoints. Check mechanical clearance before extending the
calibration range. Normal sweeps still enforce saved minimum/maximum bounds.
The Servos tab also exposes maximum sweep speed and acceleration while stopped.
The main sweep slider spans that configured speed range with no separate 28
degrees/s clamp. Speed register values 1..1023 are supported; actual achievable
speed depends on load, acceleration and travel distance. Motion settings apply
for this driver session; the launch XML export includes speed and acceleration
for persistence across restarts.

The arm driver requires [safe_arm firmware v1](../../DevTools/servodemo/firmware/safe_arm/README.md).
It sends a sweep profile once and refreshes permission every 50 ms while the
ROS safety gate allows motion. Firmware slows near each endpoint and waits for
low measured speed before reversing. If permission heartbeats stop for 350 ms,
it cancels motion and releases torque. RC kill still uses ROS/USB; no extra
wiring is required. The firmware must be flashed separately before this driver
can move the arm. See the linked build and commissioning instructions.

Arm position is published on each 50 ms control tick, independent of the slower
electrical diagnostics. Dashboard telemetry targets 20 Hz; the radar redraws
at 30 Hz. These are scheduling targets, not guaranteed measured rates.
Confirmed firmware motion stops retain the connection after torque-off is
verified and require a new user motion request, not adapter rediscovery.
Transport disconnects retry discovery in the background after a two-second
backoff. Reconnection restores the selected role and session calibration but
never resumes an old sweep or jog. RC kill keeps a healthy connection open.

Metal detector calibration uses two ADC values: Zero and Mine trigger. Apply
sets the trigger to the calibrated full response; detection starts at that
value. Both levels are plotted and included in chart autoscaling. Zero now
updates the baseline without changing an applied mine-trigger endpoint.

Settings → Metal detector shows a 30-second raw ADC graph, receive rate, packet
log, rejected-line count, freshness and estimated peak voltage. Only the control
owner can zero or apply calibration. Zero averages the last second of fresh
samples; the full-response endpoint maps to 100%. Changes affect the ROS
signal-ratio topic for all clients. They last for the node session; the tab
exports launch XML for persistence. With a 5 V reference, 20 ADC is about
0.392 V peak and 150 ADC about 2.941 V peak. This is the firmware's sine-equivalent
AC amplitude, not DC pin voltage; values above half the reference warrant
checking waveform shape or clipping. Set the reference to measured AVcc for
better conversion accuracy. Simulated payloads do not provide raw ADC readings.

## Files

- `dashboard/index.html`, `app.js`, `style.css` — pad graphic, mapping, HUD
- `scripts/operator_gateway.py` — control lease, RC authority, command timeout, `cmd_vel`
- `launch/operator.launch.xml` — hardware or simulation, selected by `is_sim`
- `launch/operator_sim.launch.py` — `sim_svea` + simulated payload
- `gui_plan.md` — longer roadmap (cameras, map, payload, mission record)
