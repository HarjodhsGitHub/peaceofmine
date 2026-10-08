# Setup

Use Docker, or Ubuntu 24.04 with ROS 2 Jazzy. Run commands from the repository
root. Rebuild the image after dependency changes with `util/build`.

```bash
DEV=1 util/run
colcon build --symlink-install
source install/setup.bash
ros2 launch peaceofmine_operator operator_sim.launch.py
```

Development forwards only port 8080. Hardware `util/run` uses privileged device
access and host networking. Both modes mount source, firmware and tools.
Saved settings live in the source-mounted operator configuration and survive
container recreation. Do not run hardware and simulation simultaneously.

For native development:

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
python3 -m pip install -r requirements.txt
colcon build --symlink-install
source install/setup.bash
```

Retained launch files need BetterLaunch v1.6.0 and PX4 messages v1.16.1 in a
sourced dependency workspace; the Dockerfile builds them under `/opt/svea/jazzy`.
Source both dependency and project overlays in every new native shell.

To change the simulation port:

```bash
ros2 launch peaceofmine_operator operator_sim.launch.py port:=8088
```

Browser controller access on remote addresses needs HTTPS. Robot USB controller
input works over HTTP. Foxglove, Zenoh, autonomous examples and mocap are absent
from the default image/runtime.
