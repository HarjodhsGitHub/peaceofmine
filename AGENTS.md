# PeaceOfMine development guide

ROS 2 Jazzy workspace. Work from the repository root. Develop and verify in
simulation before any supervised hardware test; keep the car lifted/clear and
physical RC ready. Follow the user's instruction to leave ESP code/configuration
untouched.

## Layout

- `src/peaceofmine_operator`: dashboard, ROS owners, settings, payload simulation.
- `src/peaceofmine_interfaces`: used messages/services.
- `src/svea_core`: basic SVEA interfaces, MAVROS, vehicle simulation and telemetry.
- `src/svea_localization`: optional compatible localization; disabled by default.
- `firmware`: retained ArbotiX firmware/prerequisites and watchdog tests.
- `tools`: bounded calibration and acquisition verification utilities.
- `DevTools`: ESP and hardware paths retained pending verification; do not prune
  uncertain encoder dependencies.
- `docs`: concise Markdown; no documentation publishing workflow.

Never edit generated `build/`, `install/`, `log/` or `site/` output.

## Development

```bash
util/build
DEV=1 util/run
colcon build --symlink-install
source install/setup.bash
ros2 launch peaceofmine_operator operator_sim.launch.py
```

Hardware uses privileged `util/run` and `operator.launch.xml is_sim:=false`;
see `docs/hardware.md`. Browser GUI is on 8080, remote browser controllers need
HTTPS. Preserve both launch commands, namespace behavior, `/ws` and settings.
Rebuild after package metadata, launch or installed asset changes.

## Changes and safety

Keep ROS dependencies in each `package.xml`, shared Python dependencies in
`requirements.txt`, and installed assets/scripts in `setup.py`. Use relative ROS
topics. Preserve RC/manual override, leases, permission/arming gates, deadman,
command timeout, zero-command shutdown, ArbotiX watchdog/homing/limits and probe
fault protection. Probe movement uses RC permission independently of wheel velocity;
missing wheel telemetry must not block probe motion.

One ADS1115 owner acquires probe A0 (MUX 4) and detector A3 (MUX 7). Do not open
a second driver or assume physical sample feasibility. Back up settings before
migration; never transfer old-channel calibration. Simulation calibration must
not overwrite hardware sections. No pump or new automatic stop/retract policy.

The GNSS ROS node exclusively owns the ZED-F9P UART. Keep NTRIP credentials out
of git and telemetry. GNSS loss is degraded status, not a new drive interlock.
Optional localization uses external `gnss/fix` and must not launch its legacy RTK
manager. Missing pose must not invent positioned detections or map markers.

## Verification

```bash
colcon build --symlink-install --packages-up-to peaceofmine_operator
source install/setup.bash
colcon test --event-handlers console_direct+
colcon test-result --verbose
```

For cleanup audits use `/tmp` build/install/log bases. Run both affected
simulation launches and check exceptions, topic contracts and stopped motion on
command-source loss. Native setup and targeted checks are in `docs/setup.md` and
`docs/verification.md`. Hardware sampling/performance acceptance remains explicit.
