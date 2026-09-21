# Fast A0 amplitude sampler

PlatformIO firmware for an Arduino Uno (ATmega328P). It samples `A0` in ADC
free-running mode at about 76.9 kS/s, rectifies the signal in the ADC ISR, and
low-pass filters that magnitude into a sine-wave peak-amplitude estimate.

Upload and monitor with:

```sh
pio run --target upload
pio device monitor
```

The serial monitor runs at 115200 baud. It prints one integer every 50 ms:
the estimated sine peak in 8-bit ADC counts (`0` to `255`). With the default
AVcc reference, the corresponding input peak voltage is approximately
`printed_value * Vcc / 255`.

The input must stay between GND and AVcc. Bias an AC-only source around about
Vcc/2 before connecting it to A0; never drive A0 negative. Although 60 kHz is
above the Uno ADC's Nyquist frequency at this rate, it aliases to a lower
frequency without changing its amplitude. The software measures that aliased
signal's envelope rather than attempting to reconstruct its waveform.
