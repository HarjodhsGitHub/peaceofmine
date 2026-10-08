# Calibration and settings

Settings are in `src/peaceofmine_operator/operator_config.json` or the absolute
`config_file` launch override. A one-time ADC/camera migration keeps an original
`*.pre-lean-v1.bak` backup. A0 and A3 are enabled and routed to probe and detector;
old channel scale/offset calibration is cleared, probe trigger moves to A0 and
is disabled until recalibrated. Legacy 0–255 detector calibration is retained
for the legacy driver and is never converted into A3 voltage calibration.

Simulation uses separate `adc_simulation`, actuator and detector calibration
sections, preserving hardware calibration. Configure probe threshold direction,
hysteresis and debounce in ADC settings. ArbotiX motor load protection remains
separate from external analog contact sensing. Set new detector zero and trigger
voltages using fresh A3 readings and verify performance against the real signal.

Main camera policy defaults to **Automatic**. It prefers a forward robot stream,
then stable sorted camera IDs, and displays a real source only after an image
probe succeeds. Freshness loss falls back to a labelled virtual view; recovery
returns automatically. Browser/laptop cameras require explicit permission.
The source policy stays `auto` separately from the resolved live camera.

Old missing/virtual defaults migrate once to auto. Explicit off, named robot and
laptop selections remain explicit. After migration, choosing a manual source,
including virtual, persists until choosing auto. Manual choices survive late
shared settings. Browser preferences contain controller mapping and local device
IDs; portable robot defaults travel through the settings service.

Take control before changing calibration. Respect the dashboard's motion and
RC gates. Firmware homing, limits, torque/load protection and RC permission checks
remain active.
