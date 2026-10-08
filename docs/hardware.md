# Hardware and wiring

Validate changes in simulation first. Keep the car lifted or clear and keep the
physical RC ready for supervised initial tests.

```bash
util/run
colcon build --symlink-install
source install/setup.bash
ros2 launch peaceofmine_operator operator.launch.xml is_sim:=false \
  tls_cert:=/tmp/operator-tls/cert.pem tls_key:=/tmp/operator-tls/key.pem
```

PX4/MAVROS keeps steering, throttle, gear/differentials, RC override, IMU,
wheel telemetry and power/health topics. ArbotiX owns the arm/probe motors.
ESP firmware, transport and settings are unchanged by this cleanup.

One Pi-connected **ADS1115** serves both analog inputs:

| Signal | Physical input | Single-ended MUX index |
|---|---|---|
| Probe force/contact | A0 | 4 |
| Metal detector | A3 | 7 |

MUX 0/3 are differential selections, not A0/A3. Current software defaults to
I²C bus 1, address `0x48`; **deployed bus/address remain unconfirmed**. Only the
shared ADS1115 node opens the ADC. Acquisition starts with the operator and
may be stopped/reconfigured in its settings. Contact thresholds are disabled
until calibrated. Readings are volts or calibrated scaled values, not newtons.
No new automatic stop/retract policy was introduced.

Hardware uses the legacy detector by default until A3 physical acceptance.
After confirming wiring, signal characteristics and sufficient shared sampling,
select `use_adc_detector:=true`; this disables the legacy detector process.
Stop the operator before running the independent measurement utility:

```bash
PYTHONPATH=src/peaceofmine_operator python3 tools/ads1115_sampling_check.py \
  --bus 1 --address 0x48 --required-a3-hz <measured-required-rate>
```

This reports actual A0/A3 rates and maximum gaps. It validates acquisition
throughput only; physical metal-detection performance still needs verification.
The A3 implementation interprets measured voltage and requires new calibration.
It does not assume an AC waveform, ESP JSON or an op-amp peak-to-peak value.

GNSS starts by default and exclusively owns `/dev/ttyAMA0` at 115200 baud.
Provide NTRIP credentials through container environment variables
`NTRIP_USERNAME` and `NTRIP_PASSWORD`; optional `NTRIP_HOST`, `NTRIP_PORT` and
`NTRIP_MOUNTPOINT` override the SWEPOS defaults. Use Docker environment
arguments/a private env-file, and keep secrets outside version control.
The GNSS dashboard uses the operator WebSocket; no port 8090 is required.
Receiver/internet/correction failures visibly degrade GNSS without introducing
an extra manual-driving interlock.

Localization defaults to `use_localization:=false`. Opt in explicitly; the
operator passes `external_gnss:=true` so the legacy RTK manager never opens a
second serial connection. Outdoor localization consumes `/<name>/gnss/fix`.
GNSS coordinates/course alone are not a fused vehicle pose. Missing fresh pose
shows position unavailable and suppresses new positioned detector hits. Probe
movement uses RC permission independently of wheel telemetry. Missing, stale or
nonzero wheel velocity does not block probe motion; fault, timeout, homing and
servo limits remain enforced.
