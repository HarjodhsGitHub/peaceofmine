# Arm servo and PX4 safety audit

Audited firmware: [nilskiefer/SVEA-PX4-Autopilot revision 53453cddf93ad8f691a6032849447c116cdceb7c](https://github.com/nilskiefer/SVEA-PX4-Autopilot/tree/53453cddf93ad8f691a6032849447c116cdceb7c).
This describes source behavior, not verification of the firmware currently flashed on the car.

**Current rollout:** the operator reported working jogging/sweeping but rough
turnarounds. The new [safe arm firmware](../../DevTools/servodemo/firmware/safe_arm/README.md)
adds locally eased sweeps and a communication watchdog. It was flashed and
read-back verified on 2026-09-21; runtime reports stopped with no fault.
Motion behavior is tested with fake IO, not yet physically validated. Deploy it with
the matching ROS driver; legacy firmware is rejected before enabling motion.

## What ROS mode actually means

`ActuationInterface` sends `mavros_msgs/ManualControl` on
`/self/mavros/manual_control/send`. The fork's
`src/modules/mavlink/mavlink_receiver.cpp::handle_message_manual_control`
accepts it only when `manual_control_switches.mode_slot == 1`.
This is a custom RC authority gate, not PX4 Offboard mode.

The [firmware control documentation](https://github.com/nilskiefer/SVEA-PX4-Autopilot/blob/53453cddf93ad8f691a6032849447c116cdceb7c/docs/en/svea/controlling-the-car.md)
maps SWB/CH5 low (~1000 us) to ROS, middle (~1500 us) to RC override,
and high (~2000 us) to kill. SWD/CH7 toggles PX4 arming. All these switch
positions use Manual mode, so `mavros/state.mode` alone cannot distinguish them.

## Existing MAVROS safety telemetry — no firmware changes

The gateway subscribes to:

- `/self/mavros/state`: connection, `armed`, and `system_status` from HEARTBEAT.
- `/self/mavros/rc/in`: physical channel values, including SWB on CH5.

The fork's `src/modules/mavlink/streams/HEARTBEAT.hpp` already maps
`actuator_armed.kill`, `termination`, or non-HIL `lockdown` to
`MAV_STATE_FLIGHT_TERMINATION` (8). Armed failsafe maps to CRITICAL (5).
Vehicle drive requires connected, armed, ACTIVE (4), and fresh RC in the low
CH5 position. The servo independently subscribes to both state and RC input:
it requires fresh connected status **4** and fresh RC with CH5 low or middle.
Every other status, including 0 and 8, blocks servo motion. It does not
additionally require the `armed` flag or ROS mode on CH5. No PX4 patch, uORB safety
streams or schema overrides are required. The generic uORB tunnel
remains available for its existing power telemetry.

CH5 acceptance uses a conservative 800–1100 us low band within mode slot 1;
1300–1700 us indicates override, and 1800–2200 us indicates kill.
Intermediate, missing, or invalid channel values deny motion. These bands
assume the documented board mapping and normal transmitter calibration.
If that mapping is changed, update the ROS interlock to match before use.

Local timeouts are 1.5 s for state and 0.4 s for RC. Repeated ROS message
stamps do not refresh permission; disconnect clears permission immediately.
The stock HEARTBEAT rate is 1 Hz and onboard RC_CHANNELS is 20 Hz, so a PX4
arming-state change can take roughly one heartbeat period to reach ROS.
Vehicle drive also checks kill/override through the faster CH5 stream.
Servo permission also checks kill/RC loss through the faster RC stream. This mirrors
the reported PX4 state, not the instantaneous internal power-gate timing.
`RCIn` does not expose every receiver failsafe flag; PX4 heartbeat failsafe,
RC freshness, and zero RSSI inhibit control. RSSI 255 means unknown, not loss.
Returning from kill never replays a servo target.

## Bring up the actual PX4 connection

The XML launch defaults to this vehicle: hardware mode, the verified FTDI
adapter, and servo ID 1. Limits remain unset until calibration. For the USB-connected vehicle:

```sh
ls -l /dev/serial/by-id/*SVEA*
ros2 launch peaceofmine_operator operator.launch.xml
ros2 topic echo /self/mavros/state --once
ros2 topic echo /self/mavros/rc/in --once
```

Set `lli_serial_device:=<the actual by-id path>` if it differs from the default.
`lsusb` confirms enumeration, not an open MAVLink connection. MAVROS startup
and connection diagnostics are visible in the launch output. A topic named
`mavros/manual_control/send` can exist solely because the drive node publishes
it; check `state.connected` to verify the FCU connection.

The XML uses a native ROS launch backend for MAVROS or `sim_svea`. Nesting
BetterLaunch's Python launch description inside ROS XML did not execute vehicle
bringup, and its CLI adapter can treat the string `false` as true. Native launch
actions avoid these issues for the hardware/simulation selection.

## Servo behavior

The Protocol 1.0 transport uses ArbotiX gateway ID 253, host baud 115200,
gateway model byte 44, safe arm protocol version 1 at register 80, and fixed
Dynamixel bus baud 1000000. It is not a transparent USB adapter. The driver requires
MX-64 model 310 in joint position-control mode and never rewrites EEPROM angle limits or mode. It addresses one
explicit servo ID; no automatic actuator selection or full-range move occurs.

The servo independently checks PX4 safety plus the gateway's short-lived
website permission. Any PX4 status other than 4 disables torque. RC override alone does not block
servo motion or disable torque. RC kill, RC loss and website
lease-loss paths stop the servo too. This is not a separate physical servo
arming mode. Calibration movement requires a held command refreshed within
0.25 s; gateway permission expires within 0.3 s. A stopped target is discarded.
Preset moves use a bounded goal lead; the settings slider sends its requested
position directly. Sweeping runs locally in the ArbotiX using full endpoint
goals, eased speed and a low-speed settling period before reversal. Every
motion requires a renewable 350 ms firmware lease. Unchanged sweep profiles
are not resent, and ordinary reads cannot refresh that lease.

**Hardware limit:** the new controller watchdog attempts torque-off if the Pi
crashes, loses power or USB fails, without needing additional wiring. RC kill
still travels through ROS. A frozen controller or broken servo bus cannot be
made safe by this communication watchdog alone; an independent cutoff would be
needed for that guarantee. The fork's
`svea_power_gate` uses `armed && !kill && !lockdown && !termination`; verify the
actual wiring and rail behavior before a loaded test. Torque-off can let a
loaded arm fall, so support it during setup. No physical servo motion has been
performed as part of these changes.

## Calibration and launch

1. Start `operator.launch.xml`; it already enables the verified adapter and ID 1.
   Stop the standalone demo first; one process owns serial.
2. Open Settings → Arm calibration and take control. The selected servo reports
   its shaft angle in degrees automatically; raw ticks remain in the details
   and exported XML (4096 ticks = 360°). Selecting a different ID receives an
   explicit driver confirmation or an error. All calibration commands show
   pending, confirmed, failed, or timed-out feedback.
3. Support/unload the linkage and ensure clear space. With PX4 status 4,
   enable slow jogging, then hold left/right. Jogging is approximately 2°/s,
   with at most 16 ticks (1.4°) outstanding per step. The motor speed register
   limits travel speed; a two-tick goal offset can be too small to overcome
   position deadband or load. Before limits exist, each hold
   has a fixed ±56-tick window (about 5°) around its starting position, clipped
   to servo EEPROM limits. Repeated heartbeat commands cannot extend that
   window; release and check clearance before the next hold. EEPROM limits
   are not a guarantee of mechanical clearance.
4. Release the button to stop and release torque, then record minimum, centre,
   or maximum. Alternatively gently position the supported arm by hand while
   torque is off. Recording is allowed during calibration only while stopped.
   Left decreases sensor angle; mounting determines physical direction.
5. Apply recorded limits: `0 <= minimum < center < maximum <= 4095`, inside the
   EEPROM range. Jogging thereafter stays inside this saved range. Hold-to-move
   preset buttons and automated sweeping still require complete calibration.
6. Check kill/status loss and button-release stopping. Copy the generated XML
   defaults into `operator.launch.xml` to persist calibration across restarts.
   These changes were tested using a fake servo bus, not physical jogging.

Hardware XML launch also accepts `use_arm_servo`, `arm_serial_port`,
`arm_servo_id`, `arm_minimum`, `arm_center`, and `arm_maximum`. Simulation
launch explicitly enables simulated safety **only in the gateway**. The real
serial servo node has no simulation bypass. Do not enable a real servo alongside
`simulate_payload`; both would otherwise publish fixture state.

Servo register definitions and speed units follow the [ROBOTIS MX-64 manual](https://emanual.robotis.com/docs/en/dxl/mx/mx-64/).

## Telemetry table

The panel shows hardware torque enable, goal/actual position, speed and speed
limit, estimated signed load %, RAM torque limit %, maximum torque setting,
current, voltage, temperature, movement flag, firmware, and PX4 permission.
The driver reads registers 24–46 in one packet and current (68) separately.
A zero torque limit blocks new motion with an explicit diagnostic; the driver
never resets that limit automatically after a servo shutdown.

As described in the [ROBOTIS manual](https://emanual.robotis.com/docs/en/dxl/mx/mx-64/#present-load-40),
load is inferred from internal output, not measured by a torque sensor.
Current uses 4.5 mA per count relative to 2048. Percent load and percent torque
limit are different quantities and are displayed separately.
