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
only while you hold the control lease, the GUI is armed, a deadman is true
(RT, LT, LB, or RB), and commands are newer than 0.25 s. After that the
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
- Have the physical RC ready. Disarm in the GUI or release the deadman
  to stop software commands.
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
5. **Take control**, then **Arm** (or press **A**).
6. RT forward, LT reverse, left stick steer. Hold a trigger or bumper.

The status line must read `ROS cmd_vel …`. Stick lights alone are not
enough. The virtual forward camera is first-person, so the car will not
slide across that view; watch the vehicle and the speed readout.

### 4. Stop

Disarm / stop in the GUI, then `Ctrl+C` the launch. Physical RC still
overrides.

## Simulation only

Hardware-free check on the same `cmd_vel` path:

```bash
ros2 launch peaceofmine_operator operator.launch.xml is_sim:=true
```

Open `http://localhost:8080` on the machine that has the pad, or use
HTTPS for a remote laptop as above.

## Settings

| Controller mapping | When to use |
| --- | --- |
| Auto | Default. Browser `standard` layout if the pad reports it, otherwise Linux Xbox 360 (triggers on axes 2 and 5). |
| Standard | Force LT/RT on buttons 6/7. |
| Xbox 360 (Linux) | Force bipolar trigger axes. |

WASD: Settings → keyboard, click the forward camera, hold Shift as deadman.

## Files

- `dashboard/index.html`, `app.js`, `style.css` — pad graphic, mapping, HUD
- `scripts/operator_gateway.py` — lease, arm, deadman, `cmd_vel`
- `launch/operator.launch.xml` — hardware or simulation, selected by `is_sim`
- `launch/operator_sim.launch.py` — `sim_svea` + simulated payload
- `gui_plan.md` — longer roadmap (cameras, map, payload, mission record)
