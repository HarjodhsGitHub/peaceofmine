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

`cmd_vel` is published only while you hold the control lease, the GUI is
armed, a deadman is true (RT, LT, LB, or RB), and commands are newer than
0.25 s. After that the gateway sends zeros for 0.5 s and goes silent.

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

Confirm the PX4 is present:

```bash
ls /dev/serial/by-id/usb-SVEA_PX4_AUTOPILOT_0-if00
```

### 2. HTTPS cert (required for the laptop Gamepad API)

Put the SVEA Wi‑Fi address you will type in the browser into the
certificate. Example for `10.0.8.246`:

```bash
mkdir -p /tmp/operator-tls
openssl req -x509 -newkey rsa:2048 -nodes \
  -keyout /tmp/operator-tls/key.pem \
  -out /tmp/operator-tls/cert.pem \
  -days 30 \
  -subj "/CN=svea-mine" \
  -addext "subjectAltName=DNS:localhost,DNS:svea-mine,IP:127.0.0.1,IP:10.0.8.246"
```

Add a Tailscale IP to `subjectAltName` if you will use that instead.

### 3. Launch hardware (not sim)

```bash
ros2 launch peaceofmine_operator operator.launch.py \
  port:=8080 \
  tls_cert:=/tmp/operator-tls/cert.pem \
  tls_key:=/tmp/operator-tls/key.pem
```

This starts MAVROS/LLI (`is_sim:=false`), `twist_consumer`, and the
gateway. It does **not** start `sim_svea`. Payload simulation and
`joy_node` stay off unless you pass `simulate_payload:=true` or
`use_joy:=true`.

If 8080 is already taken, use another port (`port:=8082`) and open that
port in the browser.

### 4. On the laptop

1. Plug in the Xbox 360 and press a button.
2. Open `https://<svea-ip>:8080` (same IP as in the certificate).
3. Accept the certificate warning.
4. Confirm the Drive panel shows the pad and the graphic moves.
5. **Take control**, then **Arm** (or press **A**).
6. RT forward, LT reverse, left stick steer. Hold a trigger or bumper.

The status line must read `ROS cmd_vel …`. Stick lights alone are not
enough. The virtual forward camera is first-person, so the car will not
slide across that view; watch the vehicle and the speed readout.

### 5. Stop

Disarm / stop in the GUI, then `Ctrl+C` the launch. Physical RC still
overrides.

## Simulation only

Hardware-free check on the same `cmd_vel` path:

```bash
ros2 launch peaceofmine_operator operator_sim.launch.py
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
- `launch/operator.launch.py` — real car (MAVROS / PX4)
- `launch/operator_sim.launch.py` — `sim_svea` + simulated payload
- `gui_plan.md` — longer roadmap (cameras, map, payload, mission record)
