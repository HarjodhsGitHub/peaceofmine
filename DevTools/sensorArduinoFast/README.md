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

The serial monitor prints one integer every 50 ms:
the estimated sine peak in 8-bit ADC counts (`0` to `255`). With the default
AVcc reference, the corresponding input peak voltage is approximately
`printed_value * Vcc / 255`.

The input must stay between GND and AVcc. Bias an AC-only source around about
Vcc/2 before connecting it to A0; never drive A0 negative. Although 60 kHz is
above the Uno ADC's Nyquist frequency at this rate, it aliases to a lower
frequency without changing its amplitude. The software measures that aliased
signal's envelope rather than attempting to reconstruct its waveform.
