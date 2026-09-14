# Operator GUI

Browser dashboard for the PeaceOfMine SVEA stack. The pad must be plugged into
the **computer running the browser**, not the SVEA USB ports.

## Run

Inside the SVEA Docker container, from the workspace root:

```bash
colcon build --symlink-install --packages-up-to peaceofmine_operator
source install/setup.bash
ros2 launch peaceofmine_operator operator_sim.launch.py
```

Open **http://localhost:8080** on that same computer. Chrome will not expose
the Gamepad API on `http://<lan-ip>:8080`. Remote pads need HTTPS
(`tls_cert` and `tls_key`).

The image must include `aiohttp` (`requirements.txt`, or
`apt install python3-aiohttp` in an already-built container). Rebuild with
`util/build` to keep it after the container exits.

## Xbox 360 driving

1. Plug the Xbox 360 into the GUI computer and press any button.
2. **Take control**, then **Arm** (or press **A**).
3. Pull **RT** to go, **LT** to reverse, left stick to steer.
4. **LB** or **RB** is also a deadman. Releasing throttle and bumpers stops
   the rover.

The Drive panel shows a live pad graphic, raw axes/buttons, and whether ROS
is actually publishing. Local lights are not enough: the status line must
read `ROS cmd_vel …` and the **map** / speed readout must change. The
forward camera is first-person, so the car does not slide across that view.

Settings → **Controller mapping**:

| Mode | When to use |
| --- | --- |
| Auto | Default. Uses the browser `standard` layout when the pad reports it, otherwise Linux Xbox 360 (triggers on axes 2 and 5). |
| Standard | Force LT/RT on buttons 6/7, RB on button 5. |
| Xbox 360 (Linux) | Force bipolar trigger axes. |

WASD: Settings → keyboard, click the forward camera, hold Shift as deadman.

## ROS path

```
browser Gamepad API
  → WebSocket drive {linear_x, angular_z, deadman}
  → operator_gateway
  → /<ns>/cmd_vel          geometry_msgs/Twist
  → twist_consumer
  → /<ns>/mavros/manual_control/send
  → sim_svea or PX4
```

The gateway publishes `cmd_vel` only while a lease is held, the operator is
armed, a deadman is down, and drive messages are fresher than 0.25 s. After
that it sends zeros for 0.5 s and goes silent so it does not hold
`twist_consumer`'s watchdog open.

Telemetry `drive.publishing`, `drive.cmd_linear_x`, and
`drive.cmd_angular_z` are the authoritative ROS output, not the browser
stick graphic.

## Files

- `dashboard/index.html`, `app.js`, `style.css` — pad graphic, mapping, HUD
- `scripts/operator_gateway.py` — lease, arm, deadman, `cmd_vel`
- `launch/operator_sim.launch.py` — sim + twist consumer + gateway
- `gui_plan.md` — longer roadmap (cameras, map, payload, mission record)
