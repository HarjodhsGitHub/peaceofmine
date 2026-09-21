# XIAO ESP32-C6 sensor ADC test

This is a first hardware test for the buffered metal-sensor signal. It uses the XIAO ESP32-C6's internal ADC1 in continuous DMA mode at 80 kSPS and prints one summary per second over USB serial. The stable ESP-IDF driver used by this project limits continuous C6 sampling to below 83,333 S/s.

It is deliberately a measurement test, not the final metal-classification firmware.

## Wiring used by this project

```text
Sensor output (0-5 V)
  -> divider / level shift
  -> 3.3 V rail-to-rail buffer
  -> XIAO D0 (GPIO0 / ADC1)

All circuit grounds -> XIAO GND
```

The XIAO ADC pin must never receive a voltage below ground or above 3.3 V. The buffer must be powered by 3.3 V and GND, and must be a rail-to-rail input/output, unity-gain-stable op-amp.

`platformio.ini` currently selects XIAO **D0 / GPIO0**. If the wire is on D1 or D2 instead, change `ADC_INPUT_GPIO` to `1` or `2` in that file.

## Build and upload in VS Code

1. Install the PlatformIO IDE extension if it is not already installed.
2. In VS Code, choose **File -> Open Folder** and open this folder:

   ```text
   firmware/xiao_esp32c6_adc_test
   ```

3. Connect the XIAO by USB.
4. Click the PlatformIO **Build** checkmark. The first build downloads the ESP-IDF toolchain, so it can take several minutes.
5. Click the PlatformIO **Upload** arrow.
6. Open the serial monitor and set its speed to **115200 baud**.

The C6 uses its continuous ADC/DMA driver. Do not add Wi-Fi, Bluetooth, Zigbee, or Thread code while making this first measurement, because radio activity can add noise to the ADC result.

## Expected serial output

Every second, the monitor prints an amplitude summary similar to:

```text
I (...) sensor_adc: rate=80000 S/s samples=80000 raw[min=...] amplitude=... ...
```

Useful checks:

- `rate` should be close to `80000 S/s`.
- `p2p` is the peak-to-peak size of the waveform in raw ADC counts.
- `amplitude` is `p2p / 2`: the approximate peak amplitude around the waveform's centre line.
- `ac_rms` is a convenient measure of the sine-wave strength after its DC baseline is removed.
- `clips[low=... high=...]` should remain zero. If either rises, stop and check the divider/buffer output before continuing.
- `pin_est` is only an approximate voltage at the XIAO ADC pin. It is not yet a calibrated reading and is not converted back to the original sensor-output voltage.

For the metal test, record the `p2p` and `ac_rms` values with no target, aluminium, and iron at the same distance. That will show whether the C6's internal ADC is adequate before adding the ADS7042.

Every five seconds, the program also prints a CSV waveform block containing 256 consecutive samples. At 80 kSPS, that block covers 3.2 ms (about 21 cycles of your 6.71 kHz waveform). Copy everything from `# waveform_begin` through `# waveform_end` into a text file or spreadsheet to plot it. With no signal wired in yet, expect only a mostly flat/noisy reading rather than a waveform.

The ADC is paused only while this short block is printed, then restarts automatically. To alter the frequency or number of printed samples, change `WAVEFORM_REPORT_INTERVAL_MS` or `WAVEFORM_SAMPLE_COUNT` in `platformio.ini`.
