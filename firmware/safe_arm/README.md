# Safe arm firmware v2

Dedicated ArbotiX-M / ATmega644P firmware for the operator dashboard's MX-64
Protocol 1.0 arm. This is a replacement sketch, not a modification to the
archived upstream ROS gateway. It uses the vendored ArbotiX Arduino core and
Bioloid `ax12` bus library. No additional wiring is needed.

**Current deployment (2026-09-28):** endpoint-goal controller with 100 Hz
feedback and a bounded speed ramp, plus stopped-only PID calibration and
optional firmware-enforced manual test bounds. Native firmware simulation
passed, and all 10,756 flash bytes were read-back verified. The physical tuning
script verified capability register 79 before testing. Current HEX SHA-256:
`b168f64f255be8550e1a40d70f5b5cc4b6cc7fb9269406991195c98b033eabda`.

The recent streamed-position profiles showed tracking lag and stick/slip in
physical telemetry despite enabled torque and normal communication. Physical
PID calibration records are saved by `tools/tune_pid.py`; firmware
does not choose gains automatically or write them to EEPROM.

**Previous cosine deployment (2026-09-28):** all 10,002 bytes read-back verified.
HEX SHA-256:
`6f79e429cc6053af990c80e75c952884d40aa4a7ba65ed8dc3ea39632599ba7d`.

**Previous deployment (2026-09-28):** 100 Hz quintic sweep trajectory, built
and simulation-tested, flashed via FTDI with servo power disconnected. All
9,824 bytes were read-back verified against the HEX image. Physical sweep
smoothness and actual cycle timing remain unverified.

Quintic HEX SHA-256:
`a1348f77529f60b6a76d9d14622519a044d7239bdefed027a6e41fe3311fa734`.

The historical deployment record below describes an older image.

**Previous deployment:** flashed on 2026-09-21 via FTDI adapter AI049UTL after
checking ATmega644P/PA signature `0x1e960a`. All 9,030 flash bytes were read-back
verified. A read-only runtime check returned protocol 1, state 0 (stopped),
fault 0. The Arduino sensor and PX4 devices were not exposed to the programming
container. No torque-enable or physical motion test was performed; mechanical
validation remains pending. The matching ROS driver deliberately rejects the
legacy firmware. Deploy the firmware and ROS changes together.

Flashed HEX SHA-256:
`9ffd1c7f8263d216961f8826827770ff04bd6fa185b9cf823070dbd64b434886`.

This build adds a 1 ms settling interval before servo-bus reads and writes,
including forwarded telemetry reads, and accepts sweep speed registers 1..1023.
The preceding settling-interval build was tested with 500 cycles of bulk
telemetry, position reads and stop commands (1,500 transactions) completed in
24 seconds with zero errors and zero read retries. Final state, fault and
torque were all zero. This stopped-bus test does not validate loaded sweeps
or live speed changes on hardware; those remain unverified.

Sweep startup is staged at 20 ms intervals: speed, acceleration, holding goal,
profile readback, torque-enable, then confirmation before sending an endpoint.
Torque readback may take up to the bounded 250 ms startup deadline; it is not
required to change immediately after a write. Stop and watchdog checks remain
active throughout. Startup failures latch codes 6 (invalid profile), 7 (limits
or position checks), 8 (torque-limit read), or 9 (startup write confirmation).
The host captures the fault before cleanup sends stop and clears it.

This build uses single-servo sync writes internally, following the bundled
Bioloid pose writer. Servos configured for status return level 2 otherwise send
write acknowledgements which can collide with the next command in the sweep
startup burst. The controller still reads back torque after enabling it and
continues checking position/torque during the sweep. Servo EEPROM return-level
settings are not changed. Native tests emulate pending unicast write replies
and validate the emitted sync-write packet lengths and checksums.

## Motion and stopping

- ROS sends calibrated endpoints, peak speed and acceleration limits. The
  controller reads combined position/speed/torque feedback at a 100 Hz target.
  It sends one endpoint goal per leg, rather than streaming intermediate goals
  through the servo's position controller.
- A bounded speed ramp and remaining-distance braking ceiling limit commanded
  speed and acceleration. Reversal needs position within tolerance and measured
  speed at or below 2.052 degrees/s, with no fixed dwell. A 1.5 second settling
  timeout and a conservative whole-leg travel deadline stop unsuccessful moves.
  Slower live speed requests extend the travel deadline proportionally.
- Speed zero (unlimited) is never used during sweeping. Servo acceleration
  limiting and startup readback remain enabled. Physical tracking still needs
  supervised validation; simulation does not model the real load or friction.
- Only an explicit permission heartbeat renews the **350 ms communication
  watchdog**. Reads, repeated start commands, malformed packets and other
  writes do not. A heartbeat arriving at/after expiry is rejected. There is
  no background host heartbeat thread which could outlive the ROS safety gate.
- Stop/RC kill bypasses easing and disables torque. Timeout, failed sweep
  position reads, out-of-range position, disabled torque, zero torque limit,
  or failure to settle for 1.5 seconds
  cancels the sweep and disables torque. Recovery needs a new explicit lease
  and motion request; a heartbeat alone cannot restart motion.
- The former two-second encoder-progress cutoff is removed: displacement
  alone cannot reliably diagnose overload. A jam away from an endpoint no
  longer has that host-independent cutoff while permission remains fresh;
  servo shutdown, operator stop and permission expiry remain protections.
  A responding controller's motion fault does not require rediscovery: the
  ROS driver confirms torque-off and preserves the connection. Motion never
  resumes automatically. A failed stop confirmation still disconnects.
- On startup and every 100 ms while stopped, torque-off is broadcast to the
  entire attached Dynamixel bus. **This firmware owns that bus**: IDs 1 and 2
  can be discovered/selected, but other independently powered motions on the
  same bus are not supported.
- Calibration slider/jog commands retain the existing host behavior, including
  the slider's maximum-speed setting. They also require the watchdog lease.
  EEPROM writes, legacy sequence/base commands, and broadcast motion writes
  are rejected. Except for explicit stopped-only multi-turn setup (register 83), firmware never changes servo mode, EEPROM limits or torque
  limits automatically.

RC kill still travels through PX4 -> MAVROS -> the ROS arm driver -> USB. RC
override is allowed; kill, stale RC/state or lost owner permission stops the
arm and stops permission renewal. If ROS/Pi freezes or USB disconnects, the
controller attempts torque-off within 350 ms of the last accepted heartbeat,
plus bounded bus/loop handling. A missing heartbeat must not be confused with
a guarantee against a frozen ArbotiX, broken servo bus, actuator failure, or a
host which incorrectly continues granting permission. This is a communication
watchdog, not a certified hardware emergency stop or MCU reset watchdog.
Bootloader time before the sketch starts is also outside its supervision.
Torque-off can let a loaded arm fall: support/unload it for commissioning.

The speed/acceleration limits reduce reversal shock, but do not directly limit
mechanical torque. Linkage clearance, power supply, PID tuning and actual load
still need a physical check. Do not force a powered/stalled servo by hand.
Register units follow the [ROBOTIS MX-64 Protocol 1.0 manual](https://emanual.robotis.com/docs/en/dxl/mx/mx-64/).

## Build and test

From the repository root, with AVR GCC installed (the existing PlatformIO AVR
toolchain is detected automatically):

```sh
python3 firmware/safe_arm/build.py
python3 -m unittest discover -s src/peaceofmine_operator/test -p test_arm_firmware.py -v
PYTHONPATH=src/peaceofmine_operator python3 -m unittest discover -s src/peaceofmine_operator/test -p test_arm_servo.py -v
```

Build output defaults to `/tmp/peaceofmine-safe-arm/safe_arm.hex`. Options:
`--toolchain /path/to/bin` and `--output /path/to/output`. The script does not
upload, change fuses or access hardware. The native test compiles the actual
sketch with fake serial/servo IO and checks packet parsing, time wraparound,
late/corrupt heartbeats, faults, command rejection and simulated full sweeps.

For an explicitly supervised upload, first stop ROS and all servo demos,
support the arm and remove actuator power while keeping the controller's
programming connection powered. Verify the adapter by-id path. The archived
board definition specifies ATmega644P, Arduino bootloader, 38,400 baud:

```sh
avrdude -p m644p -c arduino -P /dev/serial/by-id/<verified-ArbotiX-adapter> \
  -b 38400 -D -U flash:w:/tmp/peaceofmine-safe-arm/safe_arm.hex:i
```

Rebuild/source `peaceofmine_operator`, then restart its launch. A physical
acceptance check is still required: start at low sweep speed unloaded, verify
gentle turnarounds on both sides, RC kill in mid-sweep and at a corner, owner
loss, and USB disconnect. Confirm torque stays off after reconnect until a
new request; do not put the loaded mechanism into service on simulated tests
alone. The older standalone demos target the legacy firmware and cannot
enable motion on this firmware without adopting the lease protocol.

## Protocol

Dynamixel Protocol 1.0 framing, 115200 baud host, fixed 1 Mbps servo bus.
Gateway ID 253. One- or two-byte writes only; little endian. Reads do not
renew permission. Gateway reads are one byte; servo telemetry reads are
bounded to 24 bytes. Errors are returned as Protocol 1.0 status packets.

| Gateway register | Size | Access | Meaning |
| --- | --- | --- | --- |
| 0 | 1 | Read | Legacy discovery signature 44 |
| 2, 80 | 1 | Read | Safe arm firmware/protocol version 2 |
| 79 | 1 | Read | Calibration capability 1: stopped PID writes and manual test bounds |
| 81 | 1 | Read/write | Selected servo ID 0..252; change only while stopped |
| 82 | 1 | Write | 0 stop, 1 explicit new lease, 2 renew existing lease |
| 83 | 1 | Write | 1: explicitly enable selected MX-64 multi-turn mode, stopped with torque off; sets limits 4095/4095 and divider 1 |
| 84 | 2 | Write | Sweep minimum, 0..4095 |
| 86 | 2 | Write | Sweep maximum, 0..4095 |
| 88 | 2 | Write | Sweep cruise speed 1..1023, units 0.684 degrees/s |
| 90 | 1 | Write | Sweep acceleration 1..254, units 8.583 degrees/s squared |
| 91 | 2 | Write | Endpoint tolerance, at most a quarter of the range |
| 94 | 1 | Write | 0 stop, 1 start/idempotent repeat |
| 95 | 1 | Write | Stopped-only manual bounds: 1 enables, 0 disables; selection clears |
| 96 | 1 | Read | 0 stopped, 1 permitted, 2 sweeping, 3 fault |
| 98 | 1 | Read | 0 none, 2 watchdog, 3 position/read, 4 torque, 5 legacy progress fault (no longer emitted), 6..9 startup checks (above), 10 settling timeout (1.5 s) |
| 99 | 1 | Read | 0: no independent wired kill input |

After stop, select the servo, acquire permission, then configure/start motion.
Acquiring permission validates MX-64 joint or multi-turn mode (divider 1) and initializes a holding goal
without enabling torque. During a sweep, only matching endpoint/tolerance
writes and valid speed/acceleration updates are accepted. Direct writes to
selected servo RAM registers 24/30/32/73 require permission and are rejected
while sweeping, except torque-off which always stops the whole arm bus.


## Probe multi-turn travel (protocol v2)

The probe supports signed positions from -28672 to +28672 ticks (−7 to +7
shaft revolutions at divider 1), including travel through zero. Wheel mode
remains unsupported. Firmware-owned arm sweeps remain joint-mode only.
Install the newly built v2 firmware before using **Enable multi-turn travel**
in Settings → Probe. This explicit setup keeps torque off, writes EEPROM,
and verifies limits/divider readback. It clears the current probe presets and
home reference; mode persists in the servo, but the home reference does not.
No firmware upload or physical test is performed by the build script.

**Hold to home at top** retracts at approximately 50°/s with a 10° outstanding
goal limit. The host checks the absolute signed Present Load (register 40)
against the selected threshold and releases torque on the first reading at or
above it, regardless of encoder movement. Both motion-loop and telemetry reads
can trigger this stop; a 20% threshold includes −20% and +20%. Releasing the button,
losing permission, the 60-second timeout, or reaching the supported position
range stops the search. Repeated hold packets cannot restart a completed search.
The existing 350 ms firmware watchdog remains independent of the host loop.
The upper rail stop supplies contact resistance; the initial 30% threshold
still needs mechanical validation. Load is an estimate, not calibrated force.
Home again after power cycling or reconnecting; do not assume turn-count
continuity across a power cycle.

Both actuator settings panels plot signed load using the bundled Chart.js
library. Histories cover the last 60 seconds, stay separate by actuator, and
clear on disconnection or servo-ID changes. Repeated status publications do
not fabricate samples between telemetry reads.

## Measured PID calibration

`tools/tune_pid.py` runs bounded bidirectional step tests using the
current arm ID and position limits in `operator_config.json`. Running without
`--run` prints the plan without opening hardware. Hardware mode requires fresh
MAVROS state/RC telemetry, exclusive servo USB access, and this firmware's
calibration capability (gateway register 79 = 1). Stop the operator servo driver
before starting. The test itself never publishes vehicle motion commands.

```sh
python3 tools/tune_pid.py
python3 -m unittest discover -s tools -p test_tune_pid.py -v
# With normal MAVROS RC telemetry available and the mechanism clear:
python3 tools/tune_pid.py --run --keep-best
```

Each run writes `samples.csv` and `report.json` into its own results directory.
The report records configuration hash, original gains, candidate scores and
restoration status. Tests retain the 350 ms firmware watchdog and abort on RC
loss, changed configuration, failed telemetry, excessive current/load/temperature,
invalid voltage, position guard violation or failed progress. Targets have
interior margins. Mechanical overshoot cannot be prevented solely by checking
targets; measured position is also monitored in the host and firmware.

The search compares P=40/48/64 with D=0/8, keeping I=0 initially. It tests I=1
only for a stable residual error. A candidate must improve the score by 10%
without excessive overshoot, settling noise or current, then pass two larger
validation trials against a repeated baseline. This is a conservative local
search, not proof of an optimal controller or performance at every speed/load.
Without `--keep-best`, original gains are restored even after a successful run.
An abort restores original gains if communication permits and leaves torque off.
**Gains are volatile RAM values and are lost when servo power is cycled.**
The operator does not automatically load arbitrary reports. An explicitly saved
`arm_pid` configuration section can reapply validated gains on connection, with
torque off and register readback; it never enables motion.

Firmware accepts one-byte P/I/D writes (addresses 28/27/26, values 0..254)
only for the selected MX-64 while permission is revoked and torque reads off.
Gateway register 95 enables test bounds from registers 84/86, only while stopped.
It prevents out-of-bounds manual targets, checks position every 10 ms when the
loop can run, rejects arming outside the bounds, and locks bounds during a lease.
Selecting a servo again clears this optional mode; normal sweep bounds and
watchdog checks are unchanged.

### Sweep jitter audit

Step accuracy is insufficient for choosing smooth sweep gains. Run
`compare_sweep_pid.py --run --output <new-directory>` in the same ROS environment
to compare the stopped-only gain candidates on recorded slow sweeps. It restores
the entering PID values after the comparison. Candidate selection can be narrowed
with repeated `--candidate P,I,D` arguments; `--reversals 6` extends validation.
Communication, RC and health failures abort the suite. An endpoint/travel
settling fault rejects that candidate and is retained in the report.

`plot_sweep.py <samples.csv> <output.png>` produces position, raw velocity and
encoder-derived velocity plots with cruise statistics. `plot_corners.py` overlays
unsmoothed turnarounds. Endpoint commands are discontinuous by design and must
not be interpreted as a continuous desired trajectory. These host captures are
about 25–27 Hz, not proof of high-frequency mechanical smoothness. Encoder slopes
use a short multi-sample window; the raw velocity trace remains visible alongside
it. Speed-register quantization and sampling affect the raw ripple statistic.

After inspecting the jitter graphs, `validate_pid.py --gains P,I,D --run
--save --output <new-directory>` compares the chosen gains against factory gains
on repeated larger steps and runs six wide-sweep reversals at each of three
speeds. Repositioning uses small, slower steps. It saves `arm_pid` atomically
only after validation, preserving the arm's existing position limits and other
settings. Any failure restores the entering gains and stops. The driver reapplies
this explicit section at connection with torque off and verifies all registers;
a changed PID section forces a stopped reconnect. Simulation ignores the hardware
PID profile. This validates the tested load and range, not all possible loads.
