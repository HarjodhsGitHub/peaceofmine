# Firmware and calibration tools

The preserved ArbotiX sources, Arduino prerequisites and watchdog tests live in
[`firmware/`](../firmware/README.md). Build/flash instructions are in
[`safe_arm/README.md`](../firmware/safe_arm/README.md). The protocol and safety
logic were not changed by the move.

```bash
python3 firmware/safe_arm/build.py
PYTHONPATH=src/peaceofmine_operator python3 -m pytest \
  src/peaceofmine_operator/test/test_arm_firmware.py
python3 -m unittest discover -s tools -p test_tune_pid.py -v
```

`tools/` retains bounded PID tuning/verification, analysis of recorded sweeps and
explicit ADS1115 hardware checks. Hardware tuning requires its explicit `--run`
mode and RC protections. Stop acquisition before standalone I²C tests.

**ESP sources and related configuration are untouched at the user's request.**
Their deployed firmware ownership/transport remain unverified; do not infer or
prune that path. `DevTools/` remains for ESP and other hardware paths awaiting
acceptance. Legacy detector tools remain pending physical acceptance. The standalone GNSS
server has been retired after integrated parser/map/chart regressions passed.
