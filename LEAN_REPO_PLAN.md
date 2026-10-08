# Lean PeaceOfMine repository

Status: software implementation and simulation/browser verification completed on 2026-10-05; the final image rebuild is deferred to the user. Physical ADS1115 bus/address, shared A3 sampling and detector performance remain unverified. ESP code, configuration and acquisition paths are explicitly untouched at the user’s request.

## 1. Goal and retained functionality

Turn the existing repository into a focused PeaceOfMine workspace, preserving:

- Current operator-dashboard controls, cameras, power telemetry, plots, settings and calibration. Keep map/detection views, but display positioned markers only when a valid pose source is available; do not require a localization stack to run the vehicle.
- Desktop dashboard layout uses a **60/40 horizontal split**: 60% of the available workspace for the main camera area and 40% for functions/controls.
- Browser Xbox, Pi-connected Xbox and keyboard/wheel teleoperation.
- Basic SVEA vehicle functionality: PX4/MAVROS steering/throttle, existing gear/differential controls, RC/manual override, arming/permission gates, command timeouts and safe shutdown.
- ESP-based encoder acquisition, its firmware and the existing downstream integration.
- Encoder counts/direction, wheel speed/distance and available wheel odometry; IMU, battery/power and vehicle health telemetry, with their required drivers, configuration and calibration support.
- **ArbotiX arm/probe control**, required firmware, Dynamixel support and independent safety checks.
- One Pi-connected **ADS1115 ADC** shared by both analog sensors: **A0 = probe force/contact; A3 = metal detector**. Keep these roles consistent across acquisition, ROS consumers, settings, plots and tests, and validate that its effective A3 sampling rate is sufficient for the detector output.
- **Mandatory GNSS/RTK adapted from the existing `gnss_viewer` implementation**: retain ZED-F9P UART acquisition, SWEPOS/NTRIP corrections, fix/RTK quality, accuracy, charts and map. Integrate these into the operator GUI through ROS topics and the existing WebSocket JSON state format. GNSS is required retained functionality and runs independently of localization.
- Lightweight simulation and automated tests.

Exclude pump implementation. Localization, sensor fusion and related SVEA support may remain in the repository when retaining them is safer or materially simpler, but they are disabled in the default PeaceOfMine launch and are not required for basic vehicle or payload operation. Autonomous navigation, SLAM, mocap and unrelated demonstrations are candidates for removal only after dependency and regression checks. Make changes in the existing repository; retain history and licences.

Priority: preserve a fully usable basic SVEA before minimizing file count. Audit launch files, imports, firmware, device rules, calibration/configuration and ROS producers/consumers before deleting vehicle support. Prefer disabling optional subsystems in the default launch over invasive extraction when removal creates disproportionate risk or maintenance work. Delete a package only when its retained consumers have been moved, clean builds/tests pass without it and the dependency/image reduction is worthwhile. If a dependency is uncertain, retain it pending verification.

There is no pump implementation in the current repository. Excluding it means not adding pump control during this cleanup, not removing an existing feature.

User-confirmed roles: ArbotiX controls the arm/probe motors; an ESP reads encoders. The ESP's upstream connection (Pi, PX4 or another interface) is not yet confirmed and must not be inferred. Keep this encoder path independently of any detector cleanup.

### Confirmed analog wiring

There is **one shared physical ADS1115**, and the ADS1115 is the ADC:

```text
Probe force sensor ─────→ ADS1115 A0 ┐
                                      ├─ I²C → Raspberry Pi → ROS/dashboard
Metal-detector output ──→ ADS1115 A3 ┘
```

There is no second detector ADC and no ESP in this analog path. Confirm the deployed ADS1115 I²C bus/address and validate the effective A3 detector sample rate before removing the old detector path. In the existing software, single-ended A0 is MUX/input index 4 and A3 is MUX/input index 7, not indices 0 and 3 (which select differential inputs).

## 2. Target structure and removals

Retain four principal ROS packages rather than introducing a new framework:

- **`peaceofmine_operator`** — dashboard, gateway, payload drivers/controllers, settings and payload simulation.
- **`peaceofmine_interfaces`** — actively used messages, services and actions.
- **`svea_core`** — basic vehicle command/control interfaces, MAVROS bringup, encoders/wheel telemetry, IMU/power/health telemetry, required sensor-frame transforms, vehicle model/simulation and their dependencies.
- **`svea_localization`** — retained for compatibility and future use, but disabled by default. Its EKF, AMCL, lidar, SLAM and navsat paths are not prerequisites for teleoperation, payload control or mandatory `gnss_viewer` operation.

Move the required `twist_consumer` out of `svea_examples` into `svea_core` before removing `svea_examples`; if that extraction causes unnecessary churn, retain the package temporarily and mark only the operator-required executable as supported. Promote `DevTools/gnss_viewer` into supported runtime code under `peaceofmine_operator`: reuse its acquisition/parsing/NTRIP backend in a ROS node, migrate its display features and assets into the operator GUI, and adapt its regression tests. The GNSS node is the sole owner of the ZED-F9P serial device. Retire the standalone port-8090 server after integrated feature parity passes. Keep `svea_localization` installed but do not start it from the default operator launch. Its legacy RTK manager must remain disabled whenever the GNSS node is running. Review encoder/IMU utilities and their hardware configurations before removing anything; retain inactive filters or support files when their role is uncertain. Remove mocap, autonomous controllers, examples, maps or meshes only when verified unused. The dashboard’s schematic map does not require SLAM.

Consolidate required ArbotiX and ESP encoder firmware under `firmware/`, with build/flash instructions and firmware tests. Locate the deployed encoder firmware and document its source if it is maintained outside this repository. Keep required Arduino-framework dependencies; remove obsolete detector-Arduino implementations, not everything containing “Arduino” or “ESP”. Remove the ESP32 detector sketches and old detector serial reader only after extracting applicable signal-processing logic and validating the direct-Pi acquisition replacement. Do not remove shared code or dependencies required by encoder acquisition.

Remove duplicate standalone dashboards, abandoned experiments, compatibility wrappers with no remaining callers and generated files tracked in source. Retain firmware prerequisites and essential calibration utilities. Do not alter local generated build/install/log output during cleanup.

### DevTools and Foxglove clarification

- Selectively extract required firmware and relevant firmware/watchdog tests from `DevTools/` into `firmware/`. Promote `gnss_viewer` into supported runtime code and retain its regression tests. Move essential hardware-calibration utilities into a small `tools/` directory.
- Remove standalone demo GUIs, experiments and duplicate tools. Remove the old `DevTools/` directory only after extracting retained functionality and updating references.
- Remove Foxglove infrastructure from the default Docker/runtime setup. The PeaceOfMine dashboard remains the supported GUI and does not require Foxglove.
- Keep relevant automated tests; do not delete tests merely because they currently live under `DevTools/`.

## 3. Integration and interface changes

**Preserve working interfaces.** Keep the two operator launch commands, namespace behavior, WebSocket endpoint, existing actuator/ADC contracts, retained vehicle/sensor topics and saved settings. Keep `use_localization:=false` as the default and preserve an explicit opt-in localization path for compatibility. Package the adapted `gnss_viewer` as a ROS node and expose its display through the existing operator HTTP/TLS server and `/ws` connection on port 8080. Keep BetterLaunch while retained launch files depend on it; remove or convert it only if the remaining launch graph no longer needs it.

**Dashboard layout and camera selection:** set the desktop workspace columns to `minmax(0, 3fr) minmax(0, 2fr)`, so the main camera occupies 60% and the function column occupies 40% of usable width. The 40% column retains driving, detector, arm/probe and status controls without clipping; lower supporting panels remain usable. At narrow/mobile widths, stack camera and functions vertically instead of forcing the ratio.

Make `auto` the default main-camera source. Evaluate automatic selection as soon as the GUI opens and receives settings/camera state, on WebSocket reconnection, and whenever camera setup, hot-plug or stream recovery makes a real camera ready. No page reload or additional source-selection click is required. In auto mode, prefer the configured forward ROS/robot camera, then use a stable sorted camera-ID order as fallback; require a working image stream before displaying it as live. Keep the currently working camera unless the preferred forward camera becomes ready. Mark stale/disconnected streams unavailable and fall back to the clearly labelled virtual camera; automatically return to a real camera on recovery. Preserve teleoperation state and existing input-focus behavior during source changes. Laptop-browser cameras require user permission and must not be selected automatically.

Add `auto` to backend camera-settings validation, the settings UI, shared settings serialization and browser preferences. Store the selection policy as `auto` while keeping the resolved live camera separate, so a refresh or late settings message cannot overwrite it with a transient source. Version and migrate both shared and browser preferences once: old main-camera `virtual`/missing defaults become `auto`; explicit `off`, named real-camera and laptop-camera choices remain unchanged. After migration, deliberate manual selection (including virtual) is a persistent opt-out until the user selects auto again. When auto is active, completing camera setup immediately triggers source resolution once frames arrive.

**Basic vehicle support:** preserve the complete working steering/throttle and feedback path, physical RC override, existing gear/differential commands, limits, permissions, watchdogs and zero-command shutdown. Keep encoder/wheel, IMU, battery/ESC/power and connection/fault telemetry and the device access/configuration they require. Retain physical sensor-frame transforms but do not introduce fictitious map/odom poses. Hardware bringup must launch GNSS, but a temporary receiver, correction-service or internet failure must be reported as degraded GNSS rather than disabling safe manual teleoperation. Localization and map services are not required; preserve all existing interlocks.

**Detector:** consume timestamped **ADS1115 A3** measurements and validity through the shared Pi ADS1115 acquisition node into the existing ROS/dashboard flow, replacing the old detector MCU serial reader once validated. One process owns the ADS1115; do not open a second driver for A3. Determine the detection value from the actual A3 signal and achievable sampling rate; do not assume ESP32 JSON packets, raw waveform capture or `opamp_vpp_v`. Retain applicable threshold/calibration behavior, and update plots, CSV units and simulation for the actual measurement. Require new detector calibration rather than converting old 0–255 calibration values silently. Physical detection performance must be verified separately.

**Arm/probe:** retain the current ArbotiX protocol, homing, sweep, depth control, limits, watchdogs and RC interlocks. Keep motor-load protection and diagnostic displays; distinguish those from external force sensing.

**Encoders:** retain the ESP acquisition path and existing consumers. Verify firmware ownership, transport, count/direction conventions, scaling and freshness handling before pruning related vehicle code. Do not assume the ESP connects directly to PX4 or directly to the Pi. Preserve any encoder data needed by odometry, motion control or probe stationary checks.

**Force sensing:** route **ADS1115 A0** to probe force/contact processing, retaining scaling, threshold, hysteresis, debounce and stale-data handling. Update defaults, saved channel selection, plot routing and simulation to the confirmed A0/A3 roles with a backed-up configuration migration; do not carry an old A1 probe assignment forward. Keep per-channel scaling/calibration separate and do not silently transfer old-channel calibration. Configure the shared ADS1115 for multi-channel acquisition and verify effective A0/A3 sample rates and freshness. Label readings as volts/scaled values unless a sensor calibration establishes physical force units. Do not introduce automatic stop/retract behavior as part of this cleanup.

**GNSS/RTK:** adapt the existing `gnss_viewer` acquisition and parsing code into the mandatory GNSS ROS node. Preserve its MikroE ZED-F9P UART input (`/dev/ttyAMA0`, 115200 baud by default), checked NMEA and UBX NAV-PVT parsing, SWEPOS/NTRIP client and RTCM forwarding, reconnect/staleness behavior and horizontal-accuracy handling. Start it by default in hardware bringup and provide simulated data through the same topics for development. Only this node owns the receiver serial port; disable the legacy RTK manager. Keep NTRIP credentials outside version control and out of GUI telemetry. GNSS loss remains a visible degraded state rather than a new vehicle-motion interlock.

Publish relative ROS topics `gnss/fix` (`sensor_msgs/NavSatFix`, with sensor timestamp/frame, valid fix status and supported covariance) and `gnss/state` (`std_msgs/String` containing JSON, following the existing power/camera presentation-state pattern). Preserve exact RTK float/fixed status, receiver connectivity, NTRIP/correction state, per-field freshness, accuracy source and current viewer metrics in `gnss/state`; do not collapse these into the coarser NavSatFix status. The existing web bridge subscribes to `gnss/state` and adds a `gnss` object to its normal `/ws` state snapshot. Expired or absent GNSS data must produce explicit unavailable/stale state; missing numeric JSON values use null. New browser connections receive the latest snapshot through the existing broadcast flow.

Move the viewer's map, accuracy display, charts/history and diagnostics into the operator GUI using this WebSocket state; retain bundled asset licences and include the assets in package installation/server routing. Remove the separate HTTP polling path and port-8090 requirement after parity verification. The flow is receiver → adapted GNSS ROS node → ROS topics → existing web bridge → WebSocket JSON → operator GUI. For explicitly enabled GNSS-based localization, remap its fix input to `gnss/fix` and keep the old RTK manager disabled; local wheel/IMU-only localization can run independently. A GNSS fix or course over ground must not be represented as a complete fused vehicle pose/heading.

**Default operation without localization:** use fresh wheel-velocity telemetry for the existing probe stationary interlock; missing telemetry must continue to block probe motion. Keep simulation pose and any already available valid odometry for map/detection views. If hardware provides no usable pose, explicitly show position unavailable and suppress new positioned detections rather than fabricating coordinates. When localization is explicitly enabled, it may supply pose without changing payload or drive interfaces.

Preserve existing calibration/settings through file moves. Back up configuration before any schema migration; simulation must not overwrite hardware calibration.

## 4. Docker, documentation and delivery order

Rebuild Docker from ROS 2 Jazzy rather than inheriting the broad legacy SVEA image. Install retained dependencies, including MAVROS, required PX4 messages, basic vehicle/encoder support, cameras, the adapted `gnss_viewer` parsing/NTRIP dependencies, serial/I²C, Dynamixel support, ADS1115 access and the dependencies needed to keep the retained localization package buildable. Expose the integrated GUI on port 8080 in development mode; port 8090 is unnecessary after GNSS migration. Remove clearly unused infrastructure such as mocap, Zenoh and Foxglove from the default runtime only after verifying no retained launch path depends on it. Preserve UART/device rules and container access needed by the retained hardware. Treat a smaller reliable image as the goal, not an absolute minimum image.

Keep `util/build`, unprivileged simulation and hardware container workflows. Update mounts to include retained firmware/tools and preserve settings across container recreation.

Replace the broad documentation site with concise Markdown covering setup, simulation, hardware launch, wiring, ROS interfaces, calibration, firmware and troubleshooting. Update README and AGENTS.md; remove obsolete documentation and its publishing workflow. Describe implemented arming/deadman behavior, the 60/40 responsive layout and automatic/manual camera-source behavior accurately.

Implement in this order:

1. Confirm the ADS1115 I²C bus/address and validate shared A0/A3 sampling feasibility; locate the ESP encoder firmware and trace its downstream connection. Capture passing tests, GUI behavior and dependency references.
2. Inventory all required basic vehicle and sensor support. Disable optional launch paths by default; extract or delete packages only where the reduction is low-risk and worthwhile.
3. Adapt `gnss_viewer` to ROS and the GUI WebSocket flow, complete detector integration, implement the 60/40 layout and camera auto-selection, and migrate configuration/preferences.
4. Delete unused code and slim Docker/dependencies.
5. Replace documentation and verify from a clean build.

## 5. Acceptance tests

- Clean Docker and native workspace builds resolve every retained dependency and installed asset.
- Simulation exercises driving, arm/probe behavior, ADS1115 A0 probe contact, ADS1115 A3 detector measurements and GNSS status without opening physical devices.
- Browser and Pi-controller paths retain their input priorities, control permissions and timeout behavior.
- Disconnects, stale commands, RC override, permission loss and driver restarts stop motion without automatic resumption.
- ArbotiX firmware watchdog tests and probe stationary/homing protections still pass.
- Encoder tests preserve count/direction/scaling behavior and check disconnect, stale-data and recovery handling through the actual downstream interface.
- Basic SVEA launch and teleoperation work without localization or map services. Hardware bringup starts the adapted GNSS ROS node; loss of GNSS fix, NTRIP or internet is clearly reported while safe manual control remains available. Existing steering/throttle, gear/differential controls, RC override, IMU, encoder/wheel and power/health telemetry pass regression checks.
- The retained localization package still builds and its explicit opt-in launch receives a smoke test, but it never starts from the default operator launch and never competes with `gnss_viewer` for the GNSS serial port.
- Probe stationary checks use real fresh velocity without localization, and block motion when that telemetry is missing/stale. Missing pose does not produce false map positions or positioned detections.
- Detector tests cover ADC read failures, invalid/stale samples, recovery, required sampling performance, calibration, plots and CSV units.
- Channel-routing tests prove A0 drives probe contact and A3 drives metal detection, with no swapped channels or calibration leakage. For ADS1115, verify single-ended MUX indices 4/7, shared acquisition timing and configuration migration.
- Adapted `gnss_viewer` tests cover serial reconnect, checked NMEA parsing, UBX accuracy, correction routing, NTRIP failure/recovery, fix quality/staleness and simulated input. End-to-end tests verify `gnss/fix`, `gnss/state`, the WebSocket `gnss` object and integrated GUI/map/chart parity without a separate HTTP polling service. Verify optional localization consumes the fix topic and exactly one node owns the ZED-F9P serial port.
- Dashboard features and settings persistence pass regression checks.
- At desktop widths, measured usable workspace columns are 60% camera and 40% functions within normal rounding; controls do not clip or become unreachable. At the existing responsive breakpoint, the layout stacks and remains keyboard/touch usable.
- Camera tests cover GUI opening with an already-ready camera, camera setup while the GUI is open, hot-plug, WebSocket reconnection, stale/disconnected stream and recovery. Auto mode switches as soon as a usable real stream is available without a reload or extra click, and falls back to labelled virtual when unavailable. Verify one-time shared/browser preference migration, late settings updates, persisted auto policy and explicit manual opt-out; browser cameras are never activated without permission.
- Only after simulation passes: supervised hardware verification, with the vehicle lifted/clear and physical RC available.

Success means a smaller source tree **and** smaller dependency/image footprint, with every retained operator feature working—not merely fewer files.

## Execution record — 2026-10-05

Implemented the four-package runtime, moved the required twist consumer to
`svea_core`, preserved ArbotiX firmware/prerequisites/watchdog tests under
`firmware/`, and moved bounded calibration tools into `tools/`. Removed unused
examples, mocap, autonomous controllers, the standalone GNSS service, legacy
servo demo GUIs and broad documentation publishing. The legacy base-image
workflow remains intact; automatic approval review rejected removing it.

Integrated the checked GNSS/NTRIP backend with relative ROS fix/state topics,
existing WebSocket snapshots and the original map/chart/diagnostics assets.
GNSS serial ownership is exclusive, optional localization uses the external fix,
and physical sensor transforms no longer imply a stationary vehicle pose in the
operator path. Fresh wheel speed is displayed independently of localization.

Implemented responsive 60/40 columns, robot-camera image probes, preferred
forward/fallback selection, stale recovery, persistent auto policy and manual
opt-out. Added one-time browser/shared migrations. Shared ADC defaults now use
A0/MUX 4 for probe and A3/MUX 7 for detector, with backed-up configuration
migration and independent new voltage calibration. Hardware A3 detector mode is
opt-in until physical acceptance; simulation exercises it by default.

Verification: 160 Python/ROS/browser tests passed, 3 environment-dependent tests
skipped; 7 retained PID/tool tests passed. Both default simulation launch commands
and the explicit localization opt-in started and stopped without runtime
exceptions. Known SIGINT exits from C++ transforms/nested localization launch are
classified only after requested shutdown. Clean Jazzy image builds resolve the
retained dependency graph and installed assets.

Outstanding physical acceptance: confirm ADS1115 bus/address; measure shared
A0/A3 rates against the real detector signal; calibrate A3 voltage endpoints;
verify detection performance and supervised vehicle/payload behavior. The legacy
detector and uncertain ADC hardware tools remain available until these pass.
ESP inspection/migration/removal is excluded by the user’s follow-up instruction.
Do not claim those hardware acceptance tests passed from simulation alone.
