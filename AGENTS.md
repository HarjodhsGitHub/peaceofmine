# SVEA development guide

This repository is a ROS 2 Jazzy workspace for the SVEA vehicle. Develop and
test in simulation before using physical hardware. Run commands from the
repository root unless a section says otherwise.

## Repository layout

- `src/svea_core`: vehicle models, controllers, ROS interfaces, simulation,
  visualization, maps, and the base SVEA launch files.
- `src/svea_examples`: runnable control and navigation examples.
- `src/svea_localization`: transforms, SLAM, EKF, lidar, GPS, and RTK support.
- `src/svea_mocap`: motion-capture localization.
- `src/peaceofmine_operator`: browser operator dashboard, ROS gateway, and
  simulated payload.
- `docs`: tutorials, setup instructions, examples, and reference material.
- `foxglove`: importable Foxglove Studio layouts.
- `util`: Docker image and container helper scripts.

Do not edit generated `build/`, `install/`, `log/`, or `site/` output.

## Development environment

Docker is the recommended environment. Build the image once, and rebuild it
after changing `requirements.txt`, package dependencies, or a Dockerfile:

```bash
util/build
```

For simulation and normal development, start an unprivileged container with
the required ports forwarded:

```bash
DEV=1 util/run
```

For a physical SVEA, `util/run` starts a privileged container with host
networking and device access:

```bash
util/run
```

Inside the container, build and source the workspace:

```bash
colcon build --symlink-install
source install/setup.bash
```

The source tree is mounted into the container. Rebuild after adding files,
changing package metadata, or modifying launch/data files. Source
`install/setup.bash` in every new shell.

For native development, use Ubuntu 24.04 and ROS 2 Jazzy, then follow
`docs/development/native_build.md`. The normal native setup is:

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
python3 -m pip install -r requirements.txt
colcon build --symlink-install
source install/setup.bash
```

## Common workflows

Run the main SVEA simulation:

```bash
ros2 launch svea_core svea.launch.py is_sim:=true
```

Run the Floor 2 example:

```bash
ros2 launch svea_examples floor2.launch.py
```

Run localization, SLAM, or motion capture:

```bash
ros2 launch svea_localization localization.launch.py
ros2 launch svea_localization slam.launch.py
ros2 launch svea_mocap mocap.launch.py
```

Launch files expose additional arguments through standard ROS syntax,
`argument_name:=value`. Inspect the relevant file under `src/*/launch` before
changing assumptions about topics, frames, sensors, or simulation state.

Use Foxglove Studio for ROS visualization. Connect to the Foxglove bridge and
import a suitable layout from `foxglove/`.

## Operator GUI

After building and sourcing the workspace, simulation:

```bash
ros2 launch peaceofmine_operator operator_sim.launch.py
```

Real car (privileged `util/run`, PX4 serial present, HTTPS for a laptop
pad). Full steps are in `src/peaceofmine_operator/gui.md`:

```bash
ros2 launch peaceofmine_operator operator.launch.py \
  tls_cert:=/tmp/operator-tls/cert.pem \
  tls_key:=/tmp/operator-tls/key.pem
```

Open the dashboard on port `8080`. Simulation starts `sim_svea` and a
fake payload. Hardware starts MAVROS → PX4 instead. Keep the vehicle
lifted or clear for the first hardware test and keep the physical RC
ready.

To drive, take control and arm in the dashboard. Plug an Xbox 360 into the
computer running the browser, or into the SVEA USB (`joy_node` → `/joy`).
The left stick steers, the right trigger goes forward, the left trigger
reverses, and a trigger or bumper is the deadman. Browser Gamepad input on a
non-localhost URL needs HTTPS; the SVEA USB path works over plain HTTP.
Settings → **Controller mapping** is for the browser pad only. For keyboard
control, select **WASD keyboard** in Settings, focus the forward camera, and
hold `Shift` while using WASD.

Example launch options:

```bash
ros2 launch peaceofmine_operator operator_sim.launch.py \
  port:=8080 \
  fixture_angle_offset_deg:=10.0
```

For another computer, open `http://<host-ip>:8080`. Browser gamepad input on a
non-localhost address requires HTTPS; pass both `tls_cert:=<certificate-path>`
and `tls_key:=<key-path>`.

## Making changes

- Keep ROS package dependencies in each package's `package.xml`.
- Keep Python dependencies shared by the workspace in `requirements.txt`.
- Register new Python packages, scripts, launch files, parameters, maps, and
  dashboard assets in the package's `setup.py` `data_files` when required.
- Prefer relative topic names inside nodes so launch namespaces continue to
  work. Keep topic names and message types consistent across publishers,
  subscribers, launch files, and documentation.
- Put reusable models, controllers, and interfaces in `svea_core`; put runnable
  demonstrations in `svea_examples`.
- Preserve independent safety layers. Motion output must retain command
  timeouts, explicit arming, deadman behavior, and zero-command shutdown where
  applicable.
- Never test an unverified motion-control change first on a real vehicle. Use
  simulation, keep the vehicle lifted or in a clear area for initial hardware
  tests, and be ready to use the physical RC override.

## Verification

Build only an affected package and its dependencies while iterating:

```bash
colcon build --symlink-install --packages-up-to <package_name>
source install/setup.bash
```

Run the ROS test suite and print failures:

```bash
colcon test --event-handlers console_direct+
colcon test-result --verbose
```

For focused core tests:

```bash
colcon test --packages-select svea_core --event-handlers console_direct+
colcon test-result --verbose
```

Also launch the affected simulation briefly and check for ROS exceptions,
missing packages, topic mismatches, and unsafe motion when the command source
stops.

Documentation uses Zensical. Validate documentation changes with:

```bash
python3 -m pip install zensical
zensical build --clean
```

## Troubleshooting

- `Package '<name>' not found`: rebuild, then source `install/setup.bash` in
  the current shell.
- Python dependency import failure: reinstall `requirements.txt`; rebuild the
  Docker image if using Docker.
- A changed launch file or dashboard asset is not reflected: rebuild its ROS
  package and source the overlay again.
- Port `8080` is busy: stop the existing gateway or launch the operator with a
  different `port`.
- GUI loads but will not drive: take the control lease, arm, and hold the
  configured deadman.
