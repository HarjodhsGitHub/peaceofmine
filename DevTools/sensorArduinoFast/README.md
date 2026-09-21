# Fast A0 amplitude sampler

PlatformIO firmware for an Arduino Uno (ATmega328P). It samples `A0` in ADC
free-running mode at about 76.9 kS/s, rectifies the signal in the ADC ISR, and
low-pass filters that magnitude into a sine-wave peak-amplitude estimate.

## Quick start

From the repository root, enter this project:

```sh
cd DevTools/sensorArduinoFast
```

Create the project-local virtual environment and install PlatformIO:

```sh
python3 -m venv .venv
.venv/bin/python -m pip install --upgrade pip platformio
```

Activate the environment for subsequent commands:

```sh
source .venv/bin/activate
```

Build the firmware:

```sh
pio run
```

Connect an Arduino Uno, then upload and monitor it:

```sh
pio run --target upload
pio device monitor
```

The monitor runs at 115200 baud. Leave it with `Ctrl+C`, then deactivate the
environment with:

```sh
deactivate
```

If upload fails with `Permission denied` for `/dev/ttyACM0`, add your user to
the serial-device group and start a new login session:

```sh
sudo usermod -aG dialout "$USER"
```

You can check the detected board and port with:

```sh
pio device list
```

The serial monitor prints one compact JSON object every 50 ms, using
ArduinoJson (installed automatically by PlatformIO):

```json
{"v":1,"seq":42,"uptime_ms":2100,"amplitude_adc":87}
```

Each object ends in a newline (CRLF is accepted). `v` is the protocol version;
`seq` and `uptime_ms` are unsigned 32-bit counters that wrap and reset on reboot.
The time is device uptime at capture, not ROS time. `amplitude_adc` is the
estimated sine peak in 8-bit ADC counts (`0` to `255`). With the default AVcc
reference, input peak voltage is approximately `amplitude_adc * Vcc / 255`.
Add optional named fields with explicit units for future sensors. Existing
readers ignore unknown fields; changing existing field meanings requires a new
protocol version. Keep each line within 512 bytes and keep debug text off this
port. No acknowledgement or commands are needed for this telemetry stream.

## ROS bridge

Build and source `peaceofmine_operator` in the ROS environment, then start the
standalone bridge (no vehicle or motion nodes):

```sh
colcon build --symlink-install --packages-up-to peaceofmine_operator
source install/setup.bash
ros2 launch peaceofmine_operator sensor_serial.launch.xml serial_port:=/dev/ttyACM0
```

Prefer a stable `/dev/serial/by-id/...` path when available. Close the serial
monitor before starting ROS. Opening the port may reset the Uno; the bridge
waits for valid messages and retries disconnected ports once a second.
The package uses `python3-serial` (pyserial), already in the workspace Python
requirements. Install package dependencies with rosdep or rebuild the Docker
image after this dependency change.

Published relative topics:

| Topic | Message | Meaning |
| --- | --- | --- |
| `detector/amplitude_adc` | `std_msgs/UInt16` | Raw peak amplitude |
| `detector/signal_ratio` | `std_msgs/Float32` | Calibrated, clamped 0..1 response |
| `detector/fresh` | `std_msgs/Bool` | Valid telemetry received within 0.5 s |

Set `baseline_adc` and `full_response_adc` launch arguments to measured detector
endpoints. Defaults (0 and 255) only scale the encoded range; they are not a
detector calibration. Descending endpoints are supported for an inverse sensor.
The node exposes `stale_timeout` and `reconnect_interval` as ROS parameters.
Malformed messages are discarded and never refresh the freshness timer.
Measurements stop on stale/disconnected input. Consumers must monitor freshness;
the existing dashboard does not yet display this new status and can retain its
last reading. Sequence and uptime are validated but not published on the scalar
topics; add a stamped aggregate ROS message when synchronized recording is needed.

The real operator launch enables the sensor by default. Use
`ros2 launch peaceofmine_operator operator.launch.xml` with
`sensor_serial_port:=/dev/serial/by-id/...` for another board and optionally
`sensor_baseline_adc:=... sensor_full_response_adc:=...`. The XML excludes the
real sensor when `is_sim:=true` or `simulate_payload:=true` to avoid competing
publishers. Set `use_sensor_serial:=false` to disable it in hardware mode.
The launch defaults to the configured Uno's stable by-id path;
override it for another board. Topics inherit the operator namespace (default `/self`).

The input must stay between GND and AVcc. Bias an AC-only source around about
Vcc/2 before connecting it to A0; never drive A0 negative. Although 60 kHz is
above the Uno ADC's Nyquist frequency at this rate, it aliases to a lower
frequency without changing its amplitude. The software measures that aliased
signal's envelope rather than attempting to reconstruct its waveform.
