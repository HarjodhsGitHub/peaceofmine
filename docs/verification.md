# Verification and troubleshooting

Build and test outside generated repository output when doing cleanup audits:

```bash
colcon --log-base /tmp/pom-log build --symlink-install \
  --build-base /tmp/pom-build --install-base /tmp/pom-install
source /tmp/pom-install/setup.bash
colcon --log-base /tmp/pom-test-log test --build-base /tmp/pom-build \
  --install-base /tmp/pom-install --event-handlers console_direct+
colcon test-result --test-result-base /tmp/pom-build --verbose
```

Run both operator simulation launch commands briefly and verify clean shutdown,
no ROS exceptions, no competing drivers and stopped motion after command loss.
Chromium/Playwright enable optional browser layout/input/map regressions. Source
ROS overlays before Python tests. Documentation is plain Markdown; there is no
site-generation or publishing workflow.

- Missing package or stale asset: rebuild and source the overlay in this shell.
- Port 8080 busy: stop the old gateway or select another launch port.
- Cannot drive: take the lease, satisfy RC permission and hold the configured
  deadman. Xbox: left stick steering, right trigger forward, left trigger
  reverse; configured trigger/bumper is the deadman. Keyboard: select WASD,
  focus forward camera, hold Shift.
- Remote browser controller unavailable: use HTTPS; robot USB `/joy` works over
  HTTP. Browser mapping settings do not change the robot USB controller.
- Probe blocked: home it and check fresh stationary wheel telemetry, RC permission,
  limits and faults. Missing velocity deliberately blocks motion.
- GNSS degraded: check UART wiring/ownership, fix age and correction diagnostics;
  credentials stay outside GUI telemetry.
- ADC failure: check bus/address, I²C device permissions and measured shared scan
  rates. Fresh samples and new voltage calibration are required for A3 detection.

Physical sampling, detector performance and supervised vehicle acceptance cannot
be inferred from a simulation pass. Keep the legacy detector until those pass.
