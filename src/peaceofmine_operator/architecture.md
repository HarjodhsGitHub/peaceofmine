# Operator architecture

Both public launch commands compose the same nodes. `operator_sim.launch.py`
includes `operator.launch.xml` with simulation defaults. Simulation replaces device
access, not the arm/probe behavior implementations.

See [the ROS graph](ros_graph.md) for interfaces and publisher/subscriber diagrams.

## Ownership

| Component | Responsibility |
| --- | --- |
| `operator_web_node.py`, `web_bridge.py` | HTTP/TLS, WebSocket, ROS state cache, camera proxy; no hardware or persistence writes |
| `operator_control_node.py` | Browser leases, explicit arming, drive/deadman timeout, RC gates, authorization and dispatch |
| `servo_driver_node.py` | One serial connection, discovery, telemetry, independent RC/permission gate and command watchdog |
| `servo_runtime.py`, `servo_motor.py`, `servo_transport.py` | Serialized hardware transactions, bounded motion primitives, protective load stop and firmware protocol |
| `arm_controller_node.py` | Calibration, positions, sweeps and arm action lifecycle |
| `probe_controller_node.py` | Homing, calibrated depth/holding, permission/fault interlocks and contact interpretation |
| `actuator_controller.py` | Nonblocking common lifecycle, typed commands/status, action execution and settings client |
| `actuator_protocol.py` | Browser optional-field conversion to typed actuator messages |
| `ads1115_node.py`, `adc_acquisition.py`, `ads1115.py` | Acquisition and configuration application; timestamped typed ADC samples and scalar measurements |
| `adc_contact.py`, `probe_contact.py` | ADC threshold/hysteresis/debounce and session-only motor contact baseline |
| `sensor_serial_node.py`, `sensor_serial.py` | Arduino detector acquisition and calibration application |
| `detector_node.py` | Detector interpretation, calibrated plot geometry and georeferenced detections |
| `operator_settings_node.py`, `configuration.py` | Sole runtime file writer; validation, optimistic revisions, migration and atomic persistence |
| `settings_client.py` | Asynchronous saves; no extra executor/thread and no blocking control-loop wait |
| Camera drivers / `camera_registry_node.py` | Capture / lightweight camera metadata discovery |
| `power_node.py` | Battery and ESC telemetry |
| `simulated_servo_bus.py` | Fake device registers/plant under the production driver |
| `simulated_payload.py` | Simulated detector/environment signals, including simulated probe load input |

`peaceofmine_interfaces` is a separate ament/rosidl package containing the motor,
actuator, ADC, contact, configuration and action contracts. Web-oriented aggregate
telemetry and configuration documents retain JSON at the presentation boundary.

## Motor contract and timing

The driver accepts a whole `ServoCommand`: target/profile, bounds, nonzero speed,
acceleration, optional protective load stop, servo ID, owner and operation ID.
It checks the saved speed/acceleration limits for every request. Register-level RPC
and its per-controller executor threads have been removed.

The driver publishes `ServoState` with a coherent per-servo snapshot. Controllers
read the latest received state; they never wait for a register reply. Driver-owned
register transactions are bounded by the existing serial timeout. The firmware
watchdog remains independent of the host process and ROS executor.

Commands echo the driver session and a recent driver-issued challenge. The driver
checks challenge age using its own monotonic clock, so clocks on different hosts
are not compared. Sequence numbers reject duplicate/reordered renewals. A 250 ms
command watchdog or permission loss retires the operation ID before torque-off;
renewing that ID can never restart motion. A new explicit operation is required.
Browser-to-controller commands also carry a timestamp and expire after 250 ms;
ROS hosts using this interface must have synchronized wall clocks.

The current firmware supports one active actuator. The driver rejects a competing
operation. An idle controller cannot release another controller's servo. Hardware
configuration services require the bus to be stopped. A driver reconnect changes
its session and invalidates outstanding commands and probe home position.

Motion topics use volatile, bounded queues. No latched motion is restored at
startup. Safety does not depend on an action client's cancellation packet arriving:
controller interlocks, permission freshness and driver/firmware watchdogs remain.

## Behaviors

Probe homing, jogging and target movement continuously enforce RC permission,
fresh driver telemetry and no probe fault. Wheel velocity is not a movement gate:
missing, stale or nonzero velocity does not block probe commands. Homing, servo
limits, command timeouts and driver/firmware watchdogs remain enforced.

`arm/execute` supports `sweep` and `position`; `probe/execute` supports `home` and
`target`. Actions expose feedback, cancellation and final success/failure. Homing
uses the generic driver protective load stop but only the probe controller assigns
meaning to that stop and establishes a home reference. Visually verify the top:
resistance alone cannot identify its physical cause.

The browser's probe target retains its existing bounded hold behavior: hold the
requested depth until Stop, permission/interlock loss, or the 60-second timeout.
A ROS `target` action completes at depth and releases torque. Motor-contact zeroing
remains session-only and requires a stable hold. Neither mode automatically retracts
on ADC contact. ADC trigger and homing motor-load threshold are separate settings.

Probe ADC contact consumes `adc/samples`, not a plot. Disabled, unavailable,
configuration-mismatched or stale measurements produce unknown contact. Hysteresis
and debounce avoid threshold chatter; plot routing never changes detection input.

## Persistence

`operator_config.json` has `schema_version`, a document `revision`, and
`section_revisions`. Existing unversioned calibration files load as schema 1.
The old `calibration.json` is imported only when the new file is missing, and remains
untouched. Atomic replace, file locking, ownership preservation and fsync protect
saves. The settings service rejects stale revisions and preserves other sections.

Persistent sections include arm/probe calibration, arm/probe motion limits, ADC
hardware configuration/scaling, plot routing, probe ADC threshold, detector
calibration and robot camera defaults. ADC simulation uses `adc_simulation`.
Simulated actuator saves use `arm_simulation`, `probe_simulation`,
`arm_motion_simulation` and `probe_motion_simulation`. Hardware calibration is a
read-only fallback until a simulation-specific section exists.
The probe trigger remains nested in the ADC settings document for existing file/UI
compatibility; its evaluator belongs to the probe controller.

Device nodes validate/apply their settings and use the asynchronous settings service
for persistence. Saved revision and applied/readback state are distinct. Driver
configuration refresh stops active motion when actuator settings change. Camera
capture changes require restart; display defaults update through ROS. A failed or
timed-out save is visible to the client, never silently reported as applied.

Live arming, leases, active operations, acquisition-running state and probe home
position are never restored. Browser controller mappings and laptop-camera IDs stay
local. Ports, namespaces, TLS paths and hardware-enable flags remain launch options.

## Verification and remaining physical validation

Regression tests exercise watchdog latching, duplicate/reordered operations,
exclusive ownership, bounded speed, probe motion/fault/velocity interlocks,
homing/reconnect behavior, actions and cancellation, ADC freshness/hysteresis,
revision conflicts, browser/ROS integration and shutdown. The production controllers
are tested with the production driver and an in-memory bus. Both launch entry
points must start and stop without ROS exceptions in the isolated container.

These checks do not establish physical timing margins, real homing accuracy, motor
loads or wheel-velocity availability on a particular PX4. Confirm those on a
supported/lifted vehicle with RC override ready after simulation checks pass.
ROS command access uses the deployment's existing trusted-network model.
