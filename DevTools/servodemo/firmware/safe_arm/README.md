# Safe arm firmware v1

Dedicated ArbotiX-M / ATmega644P firmware for the operator dashboard's MX-64
Protocol 1.0 arm. This is a replacement sketch, not a modification to the
archived upstream ROS gateway. It uses the vendored ArbotiX Arduino core and
Bioloid `ax12` bus library. No additional wiring is needed.

**Deployment status:** flashed on 2026-09-21 via FTDI adapter AI049UTL after
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

- ROS sends the calibrated endpoints, cruise speed and acceleration once.
  The controller runs the sweep locally at 50 Hz. Duplicate starts do not
  restart it. Unchanged profiles are not resent by ROS; speed changes can be
  applied without dropping torque or restarting the sweep.
- The servo receives full endpoint goals, not tiny position increments.
  Its speed limit uses a braking-distance ceiling and acceleration ramp.
  The former extra smoothstep envelope was removed because it commanded
  minimum-speed crawling before reaching the endpoint tolerance.
- Within the configured endpoint tolerance (default about 2 degrees), the
  controller holds the measured position. It waits for measured speed to stay
  at or below approximately 2.052 degrees/s and position within two encoder
  ticks of a fixed anchor for 200 ms before reversing, then ramps up again.
  This tolerates low-speed telemetry jitter without allowing ongoing position
  drift. This intentionally trades a small corner pause for gentler
  reversals. It never writes speed zero during sweeping: zero means unlimited
  speed on the MX-64, not stop.
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
  are rejected. Firmware never changes servo mode, EEPROM limits or torque
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
python3 DevTools/servodemo/firmware/safe_arm/build.py
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
| 2, 80 | 1 | Read | Safe arm firmware/protocol version 1 |
| 81 | 1 | Read/write | Selected servo ID 0..252; change only while stopped |
| 82 | 1 | Write | 0 stop, 1 explicit new lease, 2 renew existing lease |
| 84 | 2 | Write | Sweep minimum, 0..4095 |
| 86 | 2 | Write | Sweep maximum, 0..4095 |
| 88 | 2 | Write | Sweep cruise speed 1..1023, units 0.684 degrees/s |
| 90 | 1 | Write | Sweep acceleration 1..254, units 8.583 degrees/s squared |
| 91 | 2 | Write | Endpoint tolerance, at most a quarter of the range |
| 94 | 1 | Write | 0 stop, 1 start/idempotent repeat |
| 96 | 1 | Read | 0 stopped, 1 permitted, 2 sweeping, 3 fault |
| 98 | 1 | Read | 0 none, 2 watchdog, 3 position/read, 4 torque, 5 legacy progress fault (no longer emitted), 6..9 startup checks (above), 10 settling timeout (1.5 s) |
| 99 | 1 | Read | 0: no independent wired kill input |

After stop, select the servo, acquire permission, then configure/start motion.
Acquiring permission validates MX-64 joint mode and initializes a holding goal
without enabling torque. During a sweep, only matching endpoint/tolerance
writes and valid speed/acceleration updates are accepted. Direct writes to
selected servo RAM registers 24/30/32/73 require permission and are rejected
while sweeping, except torque-off which always stops the whole arm bus.
