# PeaceOfMine project audit — 2026-11-05

## RC supervision follow-up

The operator chose physical RC supervision as the motion authority. The earlier
snapshot below predates that decision: its stationary-velocity, PX4-status and
probe-stow prerequisites are superseded. Fresh CH5 input now permits or denies
motion; stale/lost RC and kill still stop it. Probe home/jog/targets and detector
calibration do not require odometry. Sweep, alignment and marking do not require
probe calibration or stow; active-operation conflicts remain. Pose logging still
requires real localization. Settings Stop now stops only its selected actuator
without releasing dashboard control. A shallow probe cycle must actually descend
and return before reporting completion.

Validation: 174 operator tests passed, two Chrome browser suites passed separately,
controller-input checks passed, and the updated launch and desktop/phone UI were
checked locally. Physical movement was not exercised in this follow-up. The
original core-suite limitation remains open.

## Earlier audit snapshot

What this repo does: this ROS 2 Jazzy workspace controls the SVEA vehicle, detector arm, probe and marking pump through a browser dashboard. The audit assumes one physical vehicle, one active operator and several observers on a trusted private network.

Kent and upstream SVEA were merged into local `main` before this work. At validation time, the audit fixes were uncommitted and nothing had been pushed; subsequent publication is recorded in Git history. Existing local `.agents/` and `skills-lock.json` were left untouched. Existing uncommitted car changes and calibration were preserved.

## Result

13 confirmed issues fixed, with two remaining findings. There are no remaining P0 findings in the inspected flows. The remaining hardware prerequisite is P1; the obsolete core test suite is P2. This is a risk-based repository audit, not a claim that every file or physical operation has been verified.

The most important fixes are the lost alignment angle, the missing cycle state in the drive interlock, the probe cycle cancelling itself during retraction, and the pump remaining active if its command process disappears.

## Must fix before physical motion verification

1. **[P1, open] The car cannot currently establish safe stationary motion or a usable position.**
   - What this is: probe, alignment and marking require fresh measured velocity; pose logging requires fresh localization.
   - Problem: during the real-car checks, PX4 reported `system_status == 0`, wheel velocity produced no sample, and localization was unavailable. RC input and both servos were present, so a connection indicator alone would misleadingly suggest readiness.
   - Fix needed: resolve PX4 active state and supply a real, fresh velocity source through `probe_velocity_topic`; configure localization before georeferenced logging. The updated control node uses the same selected velocity source as the probe. No RC-neutral substitute or safety bypass was added.
   - If skipped: the new gates deliberately keep motion blocked. Probe homing, physical probing, alignment, pump delivery and driving have not been accepted as working on hardware.

## Confirmed defects fixed

2. **[P1, fixed] Alignment requests lost their angle.** `ActuatorCommand.msg` had no `angle_deg`, so `command_message()` discarded the requested angle and the arm used zero. The typed field now survives dispatch; a real ROS command regression checks a -15° request.

3. **[P1, fixed] Cycle state never reached the drive safety gate.** `ActuatorStatus.msg` omitted `cycle_active`; the JSON dashboard could display it, but the typed control subscriber could not enforce it. Cycle state now crosses that boundary. Browser drive, onboard ROS joystick input and the periodic output path all respect the payload block. A deployed probe also prevents arm/sweep motion.

4. **[P1, fixed] Retraction cancelled the automated probe cycle.** `start_target(0)` replaced the existing operation through `start()`, which called the probe's `stop()` and marked the cycle aborted. Retraction now retargets the same operation to the saved home position. A regression exercises the production controller and driver with a simulated servo bus, rather than mocking away the replacement behavior. The ROS action path also dispatches `probe_cycle` to the cycle implementation.

5. **[P1, fixed] Missing contact data could produce a clear result.** The cycle read raw latched attributes even when the ADC was disabled or stale. It now requires enabled, fresh, initially clear contact input and aborts if that input disappears during descent. A spike now expires even while steady samples continue. Tests cover disabled/stale input, an already triggered input, loss during descent and continuous-sample expiry. An aborted cycle releases torque; it does not claim automatic retraction.

6. **[P1, fixed] The marking pump lacked an independent downstream timeout.** The timer lived only in the operator process, while `twist_consumer` latched the last pump value. The control watchdog now sends a heartbeat; the downstream consumer turns the pump off after 250 ms without renewal. New requests also require fresh stationary velocity and a freshly measured stowed probe. Shutdown and the independent pump watchdog are tested.

7. **[P1, fixed] Detector input and scaling could disagree with settings.** Kent hard-coded A3 and subscribed to both sample batches and scalar scaled values, processing the same signal twice with different conversions. The detector now uses exactly one enabled input assigned to Detector/Both, uses timestamped voltage samples once, and rejects stale/reordered samples. Conflicting/missing saved assignments and invalid calibration produce an explicit error. Repository and empty-configuration defaults are **metal detector A2 / MUX 6**, **probe A0 / MUX 4**. Saved user settings remain authoritative.

8. **[P1, fixed] Another website could open a browser control WebSocket.** The server accepted arbitrary browser origins. It now rejects a supplied Origin whose host or scheme differs from the operator request. A regression checks rejection of a foreign origin and acceptance of the same origin. This is a browser-origin boundary, not user authentication; the established trusted-network deployment assumption still applies.

9. **[P2, fixed] The browser silently changed probe depth.** The UI clamped requests to 10–120 mm and substituted 80 mm for invalid input, independently of calibration. It now uses the reported calibrated range and native validation and sends the entered valid value unchanged. Missing depth, calibration, stationary telemetry, contact freshness, manual holds and running operations have visible blocking reasons.

10. **[P2, fixed] The log claimed physical success before acceptance.** A pump click immediately created “PHYSICALLY MARKED,” even if the server rejected it. Logging now waits for an observed pump activation and says “PUMP ACTIVATED,” without claiming paint delivery. Logs require a fresh real pose, use safe text nodes and escaped CSV, and disclose their browser-session lifetime. Missing localization no longer appears as a real `(0, 0)` position or rover marker.

11. **[P2, fixed] Alignment could use an old target.** The UI fell back to historical detections or beam values after the live peak was unavailable. It now requires a fresh explicit peak. The backend expires peaks after 30 seconds, invalidates them after movement or configuration changes, and requires a recent arm angle when recording a peak.

12. **[P2, fixed] Operators had to discover controls and infer why they were unavailable.** The header now has a persistent Stop control. Search actions show their blocking reasons directly. Existing input/probe setup screens have direct entry points. Errors are outside the camera and remain visible until superseded. Servo blocks use the actual servo safety reason rather than the permissive RC-override drive message. Settings tabs support keyboard navigation and the dialog has an accessible name. Contact, aborted and timeout messages describe sensor evidence and stopped state without claiming a verified object or successful retraction. Starting another cycle clears the previous result banner.

13. **[P2, fixed] Kent's tests replaced ROS modules globally.** Test collection inserted mock modules into `sys.modules`, contaminating unrelated real ROS tests. The baseline produced 16 errors, 21 failures and 2 browser skips. The suite now keeps real ROS modules; the relevant tests check actual typed fields and production lifecycle paths.

14. **[P2, fixed] Layout hid information or wasted the usable viewport.** Control panels grow with their content, action groups wrap, and camera/map sizes are bounded. Final measurement found a 409 px document in a 390 px viewport; the camera's intrinsic minimum was constrained, giving a 390 px document afterward. Desktop grid sizing also removed the large gap below the camera. This was confirmed with rendered DOM geometry, not inferred from CSS alone.

## Should fix

15. **[P2, open] The inherited core model tests cannot be collected.**
   - What this is: `src/svea_core/svea_core/tests/test_models.py` is the inherited vehicle-state/control-value test suite.
   - Problem: it imports missing `svea_core.models.states`, references removed message types, and retains a ROS 1 `rostest` entry point. Collection fails with `ModuleNotFoundError` before exercising models.
   - Fix needed: migrate the suite to the current bicycle/localization/control interfaces and current ROS 2 messages. Adding a compatibility shim would hide obsolete assumptions, so none was added.
   - If skipped: the passing operator suite does not establish general vehicle-model correctness.

## UIAudit technical assessment

Implementation integrity: the interface remains a coherent vehicle-operator dashboard using its existing native HTML, stylesheet and controls. Safety state and feedback now have explicit sources. No component framework, dependency or parallel design system was added.

| Dimension | Provisional score | Evidence and limit |
| --- | ---: | --- |
| Accessibility | 3/4 | Native labeled controls, visible focus, named dialog, keyboard tabs, persistent alert feedback. No full screen-reader or WCAG conformance audit. |
| Performance | 3/4 | Bounded histories, existing camera fanout and input throttling retained. No CPU/frame-time profile under prolonged multi-viewer load. |
| Responsive design | 3/4 | Rendered desktop and 390 px checks; overflow corrected. No physical phone/touch-controller test. |
| Theming | 1/4 | Existing dark stylesheet and repeated literal values retained; no token system or alternate theme. This is a maintenance gap, not a reason to introduce a new visual system in a bug fix. |
| Implementation integrity | 3/4 | Verified command/state boundaries and truthful action feedback. Hardware prerequisites and inherited core tests remain open. |
| Total | **13/20** | Acceptable; significant verification/maintenance work remains. |

The bundled detector ran once. Its missing camera-src warnings were false positives for hidden elements populated when a camera is selected; its flat-heading warning could not resolve `/assets/style.css`; em-dash warnings were unavailable-data placeholders. Existing Inter typography and the calibration feedback accent are incumbent style. The existing meter-width transition remains an optimization candidate, not a measured performance defect. These were not used to justify a redesign.

The single-page ui-consistency checklist used the existing page's own patterns, not a claimed multi-page convention. An independent reviewer checked the page twice; the resulting state inconsistencies were fixed and covered by browser regressions. Final layout measurement was performed by the implementing agent. File/search counts are estimates: roughly 80 project files and 65 searches across mapping, implementation and review; not every source file was read end to end.

## Validation and real-car evidence

- Built all five operator dependencies locally and on the car using the existing `peaceofmine:latest` image.
- **172 operator tests passed; 2 browser tests were skipped only in the ROS container because Chromium is absent there.** Both were run separately in installed Chrome and passed. The existing Node gamepad checks also passed.
- The operator tests cover typed command/status transport, freshness, motion/RC/lease gates, the production probe transition, watchdogs, shutdown, persistence, camera behavior and the WebSocket origin boundary. Python compilation and `git diff --check` passed.
- On `nils@svea-mine.tailead4c5.ts.net`, MAVROS connected to PX4; RC override, both servo telemetry paths and battery telemetry were visible. Live ADS1115 input was verified on **A0 and A2** through the actual dashboard. Example readings were approximately 0.0047 V on A0 and 0.0853 V on A2; these are observations, not calibration endpoints.
- Fresh position logging was correctly unavailable; the final UI displayed `PX4 status 0: servo movement disabled`. Wheel velocity did not yield a sample and local-position velocity was not published. No drive, sweep, alignment, home, probe-cycle or pump-on request was issued on the physical car.
- The real car reported negative battery draw. The repository follows the ROS BatteryState sign convention; the [MAVROS sys_status implementation](https://github.com/mavlink/mavros/blob/ros2/mavros/src/plugins/sys_status.cpp) also negates MAVLink current. A physical current/charger check is needed before attributing that sign to software or calibration. It is not counted as a confirmed defect here.
- Car source snapshot: `/home/nils/peaceofmine-audit-9815cf804ce4`. Approved archive: `/home/nils/peaceofmine-audit-final-9815cf804ce4.tar.gz`, SHA-256 `9815cf804ce4927a82189ab0e537916f120e2899e132475df7f5f8e51323ee7c`. The two subsequently approved CSS rules were applied to that test copy. The existing car configuration was copied into the isolated snapshot, with the probe trigger input set to A0; the original file was not modified. The snapshot is a test source copy, not a replacement Git checkout.
- Hardware launches were time bounded. Test containers were stopped after validation. No deployment or firmware flashing was performed.

## Coverage and next steps

The deep audit covered the operator dashboard, web/ROS boundaries, settings, ADC, detector, arm/probe controllers, servo runtime, RC/lease safety, pump/twist output and their tests. Repository mapping also covered launch/package/build configuration, core actuation, localization/MoCap entry points, representative navigation code and CI. Hardware CAD/electronics, every legacy DevTool, MPC tuning, RTK/MoCap calibration and every navigation algorithm were not exhaustively reviewed or physically tested.

First restore a valid physical motion/localization source and active PX4 state, then perform supervised actuator checks with the physical RC ready. Calibrate the detector and probe contact from measured readings; do not infer safe endpoints from this audit. Migrate the obsolete core suite separately. UIAudit follow-up commands would be `/impeccable harden` for any newly observed device failures, `/impeccable adapt` for physical-phone evidence, and `/impeccable polish` last. The authorized fixes above are already implemented; no additional command invocation is needed to apply them.

Verdict: the confirmed control, sensor and feedback defects are fixed and regression-tested. Physical motion remains unverified because the actual car did not meet its safety prerequisites; the gates remain intact.

Not checked: physical driving/probing/pump delivery, live cameras, real controller enumeration, full WCAG/screen-reader behavior, prolonged multi-viewer performance, firmware/CAD correctness and the obsolete core suite's intended model assertions.


---

The following is a historical snapshot. Its running-container and deployment claims do not describe the current car.

## Historical audit — 2026-09-14

Workspace: `/home/mine/Documents/nils/PeaceOfMine`.

The dashboard at `http://svea-mine.tailead4c5.ts.net:8080/` now runs this
workspace in simulation mode, in container `peaceofmine-nils`. The other
container used `/home/mine/Documents/peaceofmine/src`; it was stopped with
user approval. The new container uses `ROS_DOMAIN_ID=81` to isolate simulation
traffic. Vehicle and payload are simulated; the front USB camera is real.
No hardware driving was performed during the audit.

## Fixed issues

- Global `.active` styling colored entire Settings panels red. Alert colors
  now apply only to badges.
- Short windows clipped controls; headers and device names could overflow.
  Panels grow with content, and narrow screens stack the controls.
- Camera images were cropped by `object-fit: cover`. Both previews now retain
  the full image. Inset labels follow the actual inset size.
- Camera labels intercepted clicks intended to focus keyboard driving.
- The field map was aligned to one edge, leaving an unexplained blank area.
  The lane is now centered and the grid fills the viewport.
- Discovery discarded laptop camera options, duplicated them on refresh,
  reset missing saved cameras, and overrode explicit virtual-camera choices.
  Options are rebuilt from both device lists while preserving selections.
- Camera permission results could replace a newer selection and leak tracks.
  Per-slot generations reject late results and stop obsolete captures.
- Camera errors had no automatic retry. Network previews now retry; local
  camera failures are labeled rather than silently showing a simulation.
- Slow camera viewers could accumulate buffered video and each viewer opened
  another upstream encoder. The gateway now shares one upstream stream per
  topic, keeps one latest frame per viewer, and disconnects stalled writes.
  The ROS stream uses the `sensor_data` QoS profile supported by
  [web_video_server](https://github.com/RobotWebTools/web_video_server).
- Sliders could discard their final value during throttling; release always
  sends the final value. Focus alone no longer suppresses telemetry updates.
- Keyboard, controller and wheel steering signs were opposite ROS yaw signs.
  Left now produces positive yaw; right produces negative yaw.
- Retrying a connection retained stale ownership until the old socket closed.
  Retry now stops input and clears the local lease immediately; superseded
  socket messages are ignored. Missing initial telemetry also times out.
- Spectators sent drive-stop commands when opening Settings or losing focus,
  producing avoidable server errors. Stop commands now require ownership.
- Instrument canvases redrew at display refresh rate despite 10 Hz telemetry.
  They now draw at 10 Hz, with a separate 30 Hz virtual-camera limit; hidden
  virtual cameras and hidden pages do not waste drawing work.
- An inherited `DEBUG=release` caused shell numeric-comparison errors in
  `util/build` and `util/run`. Boolean flags now use string comparison.

## Validation

- `util/build` and a container `colcon build --symlink-install
  --packages-select peaceofmine_operator` succeeded.
- Four unittest methods pass: browser controls/settings/camera races, stream
  fanout, latest-frame buffering, and unavailable-upstream recovery.
- Chromium inspected the live URL at 1440×900, 1024×768, 800×600 and 390×844.
  All Settings tabs opened; no JavaScript exceptions, horizontal page overflow,
  or clipped panel contents were detected.
- The live front preview decoded 640×480 MJPEG. CameraInfo reported roughly
  30 source FPS. A 61-frame proxy sample measured 26.2 received FPS and 55.4 ms
  to first frame. Another sample during the image rebuild measured 23.6 FPS
  and 19.1 ms to first frame. These are local-host measurements via the
  Tailscale hostname, not measurements across a remote operator's network.
- Source frame age, source FPS and WebSocket RTT are explicitly labeled.
  They do not measure camera-to-display latency. A synchronized visual target
  is needed for that measurement.

Run regressions with Chromium and aiohttp installed:

```bash
python3 -m unittest discover -s src/peaceofmine_operator/test -v
```

The audit used a temporary virtual environment at `/tmp/pom-audit-venv` for
browser automation and aiohttp, leaving system Python unchanged.

## Running instance

Inspect the launch log:

```bash
docker exec peaceofmine-nils tail -60 /tmp/pom-operator.log
```

The active launch is equivalent to:

```bash
source /opt/svea/jazzy/setup.bash
source /svea_ws/install/setup.bash
ros2 launch peaceofmine_operator operator.launch.xml is_sim:=true port:=8080
```

Use Settings → Cameras to select the discovered front camera. Saved camera
choices remain selected across reloads and discovery changes.

## Remaining limits

- Remote gamepads and laptop-camera permission require HTTPS; this requested
  HTTP URL supports network cameras and keyboard input. The UI now explains
  the gamepad restriction directly in the Drive panel.
- The auxiliary camera remains disabled by the existing launch configuration;
  only the front physical camera was validated. Fanout was also tested with
  two synthetic viewers sharing the same source.
- Real wheel enumeration, hardware driving, and remote-network video latency
  were not tested.
- Git HEAD is corrupt: object `665178ea887d023a9cd7470dc979746dadc69e2b`
  is empty. Changes remain in the working files; Git history was not modified.
- Docker build output reports a missing Zenoh repository signing key
  (`829768EDD9BD8B8F`) inherited from the base image. It did not prevent the
  build; repository trust configuration was not changed by this UI audit.
