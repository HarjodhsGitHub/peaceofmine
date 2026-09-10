# PeaceOfMine operator simulator

This package provides the browser operator surface without bypassing SVEA's
existing command stack. It is deliberately split into a narrow dashboard
gateway and simulation-only sensor/payload nodes, so real hardware swaps in
drivers while the browser protocol, operator safety boundary, and `cmd_vel`
integration stay unchanged.

## Run the simulator

Build the workspace in the normal SVEA container, source the overlay, then run:

```bash
ros2 launch peaceofmine_operator operator_sim.launch.py
```

To set the centre angle of the shared forward fixture in simulation, pass (for
example) `fixture_angle_offset_deg:=10.0`. The simulator sweeps its measured
fixture angle through +/-45° around that value whenever the rover moves.

Open the robot computer's dashboard address from the remote operator computer.
The default is port `8080`. The launch starts without TLS for local debugging;
remote controller operation must provide both `tls_cert` and `tls_key` launch
arguments. This is required for a dependable browser Gamepad API deployment.

The default controller mapping is the browser's `standard` mapping: left stick
for steering, right trigger forward, left trigger reverse, and right bumper as
the required deadman.

Every connected browser receives the same telemetry. The header shows the
viewer count and whether this browser owns the control lease. A viewer can use
**Steal control** at any time: ownership transfers immediately, the previous
operator becomes a spectator automatically, and the gateway disarms before
the new owner can send motion. No previous-owner acknowledgement is required.
For local keyboard driving, choose **WASD keyboard** in Settings, click the
forward camera, then hold Shift while using WASD. Browser Gamepad input and
keyboard input use the same short response smoothing before publishing drive
commands.

## Hardware replacement points

`simulated_payload.py` is the only source of simulated payload data. The
gateway and website consume these ROS topics only; neither computes detector,
pressure, depth, or fixture state. Replace that node with the hardware drivers
when the vehicle is ready, keeping this interface:

- `detector/signal_ratio` (`std_msgs/Float32`): measured calibrated detector
  strength, normalized to `0.0`–`1.0`;
- `fixture/angle_deg` (`std_msgs/Float32`): measured detector/probe fixture
  angle relative to the vehicle forward axis. The simulator performs a +/-45°
  scan when enabled; its centre offset is the
  `fixture_angle_offset_deg` launch/node parameter;
- `fixture/sweep_enabled_state` (`std_msgs/Bool`) and
  `fixture/sweep_speed_deg_s_state` (`std_msgs/Float32`): measured scan
  configuration; and subscribe to `fixture/sweep_enabled` and
  `fixture/sweep_speed_deg_s` for the corresponding actuator requests;
- `probe/depth_mm` (`std_msgs/Float32`): measured depth, never a requested
  depth;
- `probe/pressure_ratio` (`std_msgs/Float32`): measured contact pressure,
  normalized to `0.0`–`1.0`;
- `probe/fault` (`std_msgs/Bool`); and
- subscribe to `probe/target_depth_mm` (`std_msgs/Float32`) only after
  applying the hardware driver's own limits and interlocks.

The simulated probe increases pressure gradually with depth in soil and makes
a sharp rise after contact with a buried target. This makes the 30-second
pressure graph an actual view of the ROS measurement path rather than a UI
animation.

The gateway always commands `cmd_vel`, which is consumed by the existing
`svea_examples/twist_consumer.py`. The simulator's `sim_svea.py` and real PX4
path therefore retain the same drive interface. A real camera is a presentation
source replacement: the virtual canvas can be replaced with a WebRTC
`MediaStream` without changing the dashboard telemetry or controls.
