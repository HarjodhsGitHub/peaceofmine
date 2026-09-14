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
ros2 launch peaceofmine_operator operator_sim.launch.py
```

Open the dashboard on port `8080`. The launch starts without TLS for local
debugging; remote operation must supply both `tls_cert` and `tls_key`, because
the browser Gamepad API requires a secure context on non-localhost origins.

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

Plug the Xbox 360 (or another gamepad) into the computer that is showing this
dashboard, then press any button if the Drive panel still says no controller
is connected. Chrome often ignores a pad until that first press.

The default **Auto** mapping uses the browser `standard` layout when the pad
reports it: left stick steers, right trigger drives forward, left trigger
reverses, and the right bumper is the deadman. An Xbox 360 on Linux often
reports an empty mapping and puts the triggers on axes; Auto then uses that
Xbox 360 layout. Settings → **Controller mapping** can force Standard or
Xbox 360 (Linux) if the steer/throttle meters do not follow the pad. For
keyboard driving, choose **WASD keyboard** in Settings, click the forward
camera to focus it, then hold Shift while using WASD. Both input paths share
the same response smoothing and publish drive commands at 20 Hz.

Gamepad input on a non-localhost URL requires HTTPS (`tls_cert` and
`tls_key`). Opening `http://<host-ip>:8080` from another computer will not
see the controller.

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
