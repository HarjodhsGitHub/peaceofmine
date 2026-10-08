# Servo motion audit — 2026-09-28

Calibration did **not** pass final acceptance. Original arm gains P=32/I=0/D=0
were restored and read back; both arm and probe torque registers read zero.
`operator_config.json` remains unchanged from the start of this audit (SHA-256
`8f6d79011762d1b5ef9a09dfbc38ec30c6200f5dadb3e0124714f32cac3a3f14`).
The configured arm limits are 511..2555, center 1514, servo ID 1. Wide tests
commanded 591..2475, leaving an 80-tick margin at each side.

## Measurements

- P=64/I=0/D=0 completed 18 interior and 18 wide sweep reversals, at nominal
  speed settings of 19.836, 39.672 and 79.344 degrees/s. Completion alone did
  not establish smooth motion. At the slow setting, raw cruise-speed standard
  deviation was about 6.3 degrees/s, with visible encoder fluctuations.
- P=48/I=0/D=16 reduced slow-sweep raw speed standard deviation to about
  4.7 degrees/s in repeated validation. A four-interval encoder velocity estimate
  fell from about 2.1 to 1.2 degrees/s standard deviation. Results varied between
  trials and direction; short-window estimates must not conceal raw jitter.
- The candidate's near-stationary intervals after reversal were approximately
  0.23..0.39 seconds. Damping improved travel but did not fix endpoint hesitation.
  The opposite endpoint command was already present during these intervals.
- In repeated 40-degree step tests, factory gains had about 34 ticks (3.0 degrees)
  average settled error. P=48/I=0/D=16 had about 16 ticks (1.4 degrees), no measured
  overshoot, and lower peak current in those particular tests.
- The final candidate sweep stopped at position 2301 after current increased to
  1.611 A, above the test's unchanged 1.5 A cutoff. Current rose over several
  samples as motion became irregular; it was not simply an isolated bad point.
  Earlier passes through that region were lower-current. The cause of this
  variability remains unresolved. Do not treat the candidate as commissioned.
- One earlier comparison stopped on a firmware permission watchdog expiration.
  Another factory-gain sweep failed to settle. Both failures remain in the logs.

Host captures were approximately 25–27 Hz. Speed-register quantization and
sampling affect raw velocity statistics. Encoder-derived speed and unsmoothed
position traces were also inspected. No claim is made about unobserved
high-frequency vibration or optimal PID gains.

## Local artifacts

Generated results are ignored by Git but retained under `results/`:

- `pid-jitter-comparison.png` and `.json`: undamped P64 versus candidate P48/D16,
  matched cruise segments and raw turnaround overlays.
- `current-abort.png`: final validation's current rise and irregular movement.
- `sweep-20260928-wide/motion-audit.png`: full position/velocity traces at three speeds.
- `sweep-20260928-wide/corners.png`: raw turnaround overlays at three speeds.
- `final-pid-20260928-02/validation.json`: failed final acceptance and gain restoration.
- `final-stopped-state.json`: read-back of original gains and both torque-off registers.
- Each experiment directory retains its CSV samples and JSON reports, including failures.

## Implemented tools and verification

`tune_pid.py` provides bounded step-response tuning with RC checks, exclusive
serial ownership, firmware watchdog renewal in the main test loop, position and
health guards, gain readback, rollback and CSV capture. `compare_sweep_pid.py`
adds sweep jitter comparisons; `validate_pid.py` gates optional persistent
configuration on repeated steps and wide sweeps. Plot scripts retain raw traces.

The flashed firmware adds stopped-only PID register writes and optional manual
position bounds; 10,756 bytes were read-back verified. The firmware and scripts
were exercised in simulation before physical motion. Native firmware tests,
seven tuning/audit tests, four PID-application tests, nine arm-node tests,
eight driver tests and the ROS simulation pipeline passed. The affected operator
package builds. Documentation builds with one pre-existing missing-page warning.

The driver supports an explicit `arm_pid` configuration section, applied only
while stopped with readback. No such section was saved during this audit.
