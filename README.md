# PeaceOfMine

ROS 2 Jazzy workspace for manual SVEA driving, an ArbotiX arm/probe, shared
ADS1115 telemetry and integrated ZED-F9P GNSS/RTK. The supported GUI is the
operator dashboard on port **8080**. Desktop camera/functions columns use a
responsive **60/40** split; narrow screens stack vertically.

```bash
util/build
DEV=1 util/run
# Inside the development container:
colcon build --symlink-install
source install/setup.bash
ros2 launch peaceofmine_operator operator_sim.launch.py
```

Open `http://localhost:8080`. Take control; driving requires the configured
permission gates and deadman. Simulation exercises the shared A0/A3 ADC and
GNSS without opening hardware devices. Hardware testing follows simulation
and requires a clear/lifted vehicle and the physical RC available.

- [Setup and simulation](docs/setup.md)
- [Hardware and wiring](docs/hardware.md)
- [ROS interfaces](docs/interfaces.md)
- [Calibration and camera settings](docs/calibration.md)
- [Firmware and tools](docs/firmware.md)
- [Verification and troubleshooting](docs/verification.md)

The retained packages are `peaceofmine_operator`, `peaceofmine_interfaces`,
`svea_core` and `svea_localization`. Localization is optional and disabled by
default. Wheel/IMU support remains intact; GNSS is independent of localization.
ESP code and configuration remain untouched. Legacy detector code is retained
until physical ADS1115 sampling and detection acceptance pass.

Original project history and licences remain in place. See
[LEAN_REPO_PLAN.md](LEAN_REPO_PLAN.md) for scope and outstanding hardware gates.
