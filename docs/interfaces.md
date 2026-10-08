# Interfaces

All operator topics are relative to the launch `name` namespace (default `self`).
The dashboard uses the existing `/ws` endpoint on the HTTP/TLS server at 8080.

| Topic | Message | Purpose |
|---|---|---|
| `cmd_vel` | `geometry_msgs/Twist` | Lease/permission/deadman-gated drive commands |
| `mavros/wheel_odometry/velocity` | `geometry_msgs/TwistWithCovarianceStamped` | Wheel velocity telemetry (not a probe interlock) |
| `adc/samples` | `peaceofmine_interfaces/AdcSamples` | Timestamped samples, acquisition validity and session/revision |
| `adc/state` | `std_msgs/String` JSON | Acquisition, routing, configuration and diagnostics |
| `adc/a0/volts`, `adc/a3/volts` | `std_msgs/Float64` | Latest analog voltages |
| `detector/state`, `detector/telemetry` | `std_msgs/String` JSON | Voltage/calibration/history and interpreted sweep/detections |
| `gnss/fix` | `sensor_msgs/NavSatFix` | Fresh fix, receive timestamp and sensor frame |
| `gnss/state` | `std_msgs/String` JSON | Exact RTK status, field freshness, accuracy, corrections and diagnostics |
| `power/state`, `cameras/state` | `std_msgs/String` JSON | Existing presentation telemetry |
| `arm/state`, `probe/state` | `peaceofmine_interfaces/ActuatorStatus` | Independent actuator status |

`gnss/state` enters the normal WebSocket snapshot as `gnss`; unavailable numbers
are null. NavSatFix uses no-fix status for expired/absent solutions and unknown
covariance: a horizontal uncertainty radius does not establish per-axis variance.
The GUI preserves receiver accuracy source, RTK float/fixed status and correction
freshness separately. The integrated receiver view retains map, reference
comparison, charts and diagnostic logs. Map tiles require internet; bundled
Leaflet/Chart.js assets and licences are installed with the package.

Motion permission, command timeout, disconnect shutdown, RC priority and ArbotiX
watchdogs are independent safety layers. The cleanup retains their behavior.
