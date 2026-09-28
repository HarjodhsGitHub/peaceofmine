# PeaceOfMine operator

Browser dashboard, ROS gateway, and simulated payload for the SVEA. This
file covers work inside `src/peaceofmine_operator`. Workspace setup, Docker,
and the rest of the stack are in the repository root `AGENTS.md`.

The package is in hardware testing on the real SVEA. Changes that affect
motion, servos, or safety still go through simulation first; see
[Hardware testing](#hardware-testing). Run commands from the repository root
unless a section says otherwise.

## Package layout

- `dashboard/`: browser UI (`index.html`, `app.js`, `style.css`) plus vendored
  chart and keyboard libraries. The gateway serves this directory; it does not
  compute detector, pressure, depth, or fixture values.
- `scripts/operator_gateway.py`: WebSocket server and the only node in this
  package that publishes `cmd_vel`.
- `scripts/simulated_payload.py`: the only source of simulated detector,
  fixture, and probe data. Hardware replaces this node; it does not change
  the gateway or the dashboard.
- `scripts/arm_servo_node.py`, `scripts/sensor_serial_node.py`,
  `scripts/usb_camera_auto.py`: hardware arm, Arduino detector, and V4L2
  camera discovery.
- `peaceofmine_operator/`: reusable Python used by those nodes (`rc_safety`,
  `arm_servo`, `camera_stream`, `calibration`, `detector`, `power`,
  `probe_contact`, `sensor_serial`). `bringup.py` is the Setup-panel
  supervisor: a fixed table of subsystems, each run as a child
  `ros2 launch` in its own process group.
- `launch/operator.launch.xml`: editable hardware and simulation profile that
  starts everything at once. Default `is_sim` is `false`.
- `launch/operator_setup.launch.xml`: real car, GUI-driven. Starts only the
  gateway with `bringup_enabled`; its `bringup.*` parameters are forwarded to
  the subsystem launches below. Every key in `bringup.DEFAULTS` must appear
  here (`test_launch_xml.py` checks this).
- Subsystem launches, shared by both profiles: `operator_drive.launch.xml`
  (car link: `operator_vehicle.launch.py`, `twist_consumer`, optional
  `joy_node`), `operator_cameras.launch.xml`, `operator_detector.launch.xml`
  and `operator_arm.launch.xml` (namespaced wrappers around
  `sensor_serial.launch.xml` and `arm_servo.launch.xml`).
- `launch/operator_sim.launch.py`: hardware-free path. It includes
  `svea_core` `svea.launch.py` with `is_sim:=true` and a fake payload.
- `test/`: unittest modules. `simulation_check.py` and
  `mavros_bringup_check.py` are manual checks, not part of the default suite.
- `calibration.json`: servo calibration installed with the package.
- `README.md` and `gui.md`: operator behavior and hardware steps.

Do not edit generated `build/`, `install/`, or `log/` output.

## Run

Build and source from the repository root first. See the root `AGENTS.md`.
While iterating on this package:

```bash
colcon build --symlink-install --packages-up-to peaceofmine_operator
source install/setup.bash
```

Simulation:

```bash
ros2 launch peaceofmine_operator operator_sim.launch.py
```

The XML launch is the shared profile. Simulation from that file:

```bash
ros2 launch peaceofmine_operator operator.launch.xml is_sim:=true
```

Real vehicle (privileged `util/run`, PX4 serial present). Full steps are in
`gui.md`:

```bash
ros2 launch peaceofmine_operator operator.launch.xml is_sim:=false \
  tls_cert:=/tmp/operator-tls/cert.pem \
  tls_key:=/tmp/operator-tls/key.pem
```

Open the dashboard on port `8080`. Simulation starts `sim_svea` and
`simulated_payload`. Hardware starts MAVROS to PX4 and, by default, the arm
servo and Arduino detector. Keep the vehicle lifted or clear for the first
hardware test and keep the physical RC ready.

Real vehicle, GUI-driven (no terminal for the operator). The boot service
runs `operator_setup.launch.xml` with HTTPS in a privileged container;
the dashboard **Setup** panel then starts each subsystem:

```bash
util/build                    # once, and after dependency changes
util/install-operator-service # enable at boot; --remove to uninstall
journalctl -u peaceofmine-operator -f
```

Stop the service (`sudo systemctl stop peaceofmine-operator`) before using
`util/run` on the same car; both need the serial devices and port `8080`.

Inspect `launch/operator.launch.xml` before changing assumptions about
topics, frames, serial devices, cameras, or simulation state. Arguments use
standard ROS syntax, `argument_name:=value`.

## Driving

To drive, take the control lease in the dashboard. The physical RC decides
whether ROS may drive; there is no separate browser arm step. Plug an Xbox
into the computer running the browser, or into the SVEA USB with
`use_joy:=true` (`joy_node` to `joy`). A live joystick stream takes priority
over browser drive commands.

The left stick steers, the right trigger goes forward, and the left trigger
reverses. The gateway reports a `deadman` flag for display, but does not gate
`cmd_vel` on it. Motion stops when the input returns to zero, goes stale, or
loses the lease or RC authority. Browser Gamepad input on a
non-localhost URL needs HTTPS; pass both `tls_cert` and `tls_key`. The SVEA
USB path works over plain HTTP. Settings, **Controller mapping**, applies to
the browser pad only. For keyboard control, select **WASD keyboard** in
Settings, focus the forward camera, and use WASD.

## Safety invariants

Keep these layers. A dashboard or gateway change that drops one of them is
not ready to merge.

- The gateway publishes `cmd_vel` only while a lease is held, RC permits ROS
  driving, and input is newer than 0.25 s. It then publishes zeros for 0.5 s
  and goes silent so `twist_consumer`'s own timeout stays independent.
- Losing the lease, browser focus, or the page stops motion. Opening Settings
  stops and releases the lease.
- Use relative topic names inside nodes so the `self` namespace in both
  launch files keeps working. Keep topic names and message types aligned
  across publishers, subscribers, launch files, `README.md`, and `gui.md`.
- Actuator limits live in the gateway (`SWEEP_SPEED_MIN_DEG_S`,
  `SWEEP_SPEED_MAX_DEG_S`, `max_probe_depth_mm`). The dashboard reads them
  from telemetry.
- Servo motion requires fresh connected PX4 state and fresh RC input with
  the kill clear. `RcSafety` in `peaceofmine_operator/rc_safety.py` is that
  interlock. Simulation may bypass it only through `safety_simulation`.
- Reject legacy ArbotiX firmware before motion. Servo limits stay unset
  until calibration.
- Setup commands require the control lease. Stopping or restarting any
  subsystem first disables actuation and publishes zero drive; a stopped car
  link leaves `RcSafety` stale, which locks drive and servos. Setup commands
  are handled before the RC safety gate so the car can be connected while
  PX4 is offline. Child commands come only from `bringup.SUBSYSTEMS` and
  launch settings, never from browser text.
- The gateway stops every child launch on exit; children also receive
  SIGTERM if the gateway dies (`PR_SET_PDEATHSIG`).
- Never test an unverified motion-control change first on a real vehicle.

## Hardware testing

Run on the SVEA in a privileged container (`util/run`, not `DEV=1`) so the
PX4, ArbotiX, Arduino, and cameras are visible. Full steps are in `gui.md`.

Before launching:

- Stop any other operator or simulation launch. Two stacks must not share
  `cmd_vel` or the dashboard port.
- Keep the vehicle lifted or in a clear area. Have the physical RC in hand;
  its kill switch is the first stop.
- Confirm the PX4 link exists:
  `ls /dev/serial/by-id/usb-SVEA_PX4_AUTOPILOT_0-if00`.
- Leave `use_localization`, `use_lidar`, and `use_rtk` off until those
  sensors are brought up separately.
- For the arm, flash safe_arm firmware v1 first
  (`DevTools/servodemo/firmware/safe_arm/README.md`) and read
  `docs/development/arm-servo-safety.md`. Servo limits stay unset until
  calibrated.

Bring-up order, one subsystem at a time:

1. With the boot service, open `https://<svea-ip>:8080`, take control in
   **Setup**, and start **Car link (PX4)** first. Each row shows whether its
   hardware is plugged in, its state, and a log. Without the service, launch
   `operator.launch.xml` with `is_sim:=false`; for a laptop gamepad, generate
   a certificate for the SVEA's current IP and pass `tls_cert` and
   `tls_key`. Disable a driver that is not connected with
   `use_arm_servo:=false`, `use_sensor_serial:=false`, or `use_cameras:=false`.
2. Open the dashboard. Check that the header shows PX4 and RC status.
   Missing or stale MAVROS state must leave actuation locked.
3. Drive with the wheels off the ground: take control, select ROS authority on
   the RC, and confirm the status reads `ROS cmd_vel`. Release the trigger,
   close the tab, flip RC kill, and stop the car link in Setup; each must
   stop the wheels.
4. Servos: jog in Settings, then calibrate arm minimum, centre, and maximum
   and the probe maximum extension. Confirm RC kill stops the arm.
5. Detector: zero and apply the mine trigger in Settings. Check readings are
   fresh at `detector/amplitude_adc`.

Stop with the physical RC, then `Ctrl+C` the launch.

Settings saves go to `calibration.json`. Commit updated measurements
deliberately; do not overwrite another vehicle's calibration. If a
hardware test behaves differently from simulation, record the difference.
`sim_svea.py` scales speed by 1/1.27; the real vehicle does not.

## Making changes

- Keep ROS dependencies in `package.xml`.
- Register new scripts, launch files, parameters, and dashboard assets in
  `setup.py` `data_files`. A changed launch file or dashboard file is picked
  up only after rebuilding this package and sourcing `install/setup.bash`.
- Shared Python dependencies belong in the workspace `requirements.txt`.
- Put vehicle models and the `cmd_vel` consumer in `svea_core` or
  `svea_examples`. This package owns the operator surface and the payload
  stand-in.
- Payload hardware replaces `simulated_payload.py` and keeps the topic
  interface in `README.md`. The gateway and dashboard stay display and
  command clients.

## Verification

Focused tests:

```bash
colcon test --packages-select peaceofmine_operator --event-handlers console_direct+
colcon test-result --verbose
```

Browser input checks, with Chromium installed, from the repository root:

```bash
python3 -m unittest discover -s src/peaceofmine_operator/test -v
```

After a dashboard change, exercise the affected flow in a browser: lease,
drive input, Settings, and any camera or calibration control that changed.
Launch the simulation briefly and confirm there are no ROS exceptions, missing
packages, or topic mismatches, and that motion stops when the command source
stops.

## Troubleshooting

- `Package 'peaceofmine_operator' not found`: rebuild, then source
  `install/setup.bash` in the current shell.
- Dashboard or launch edit has no effect: rebuild this package and source
  the overlay again. Stop the previous launch before switching `is_sim`.
- Port `8080` is busy: stop the existing gateway or pass a different `port`.
- GUI loads but will not drive: take the control lease and select ROS
  authority on the RC. On a non-localhost URL the browser pad also needs HTTPS.
- Hardware launch fails with `can't copy '.../usb_camera_auto.py'`: remove
  `build/peaceofmine_operator` and `install/peaceofmine_operator`, rebuild,
  and source again in the same shell.
- Servos will not move: check PX4 is connected, RC input is fresh with kill
  clear, the ArbotiX runs safe_arm firmware v1, and limits are calibrated.
- Two stacks must not share `cmd_vel` or port `8080`.
- Setup row says hardware not found: check the by-id path in
  `operator_setup.launch.xml` (for example a different Arduino serial) and
  that the container was started privileged with `/dev` mounted.
- Setup row failed: expand **Log**; the last lines are the child launch
  output. Dashboard never appears after boot: `journalctl -u
  peaceofmine-operator -f` shows the build and launch.
