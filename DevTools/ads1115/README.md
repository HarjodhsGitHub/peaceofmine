# ADS1115 ADC bench

Standalone Raspberry Pi I2C development tool with live Chart.js plots, raw signed
counts, input voltage, software scaling, clipping detection, rolling statistics,
measured reading rate, CSV download, and JSON configuration import/export.
No ROS, npm build, or internet connection is needed at runtime.

## Run on the Pi

```bash
cd DevTools/ads1115
python3 -m venv .venv
. .venv/bin/activate
python -m pip install -r requirements.txt
python server.py --bus 1 --address 0x48 --port 8091
```

Open `http://<pi-ip>:8091`. The service configures the chip on connection and
starts with acquisition stopped. Press Start to collect readings. Stop puts the
ADC in power-down and disables the comparator; Start resumes the selected settings.
Use only one program to configure this ADC at a time. This service binds to all
interfaces for a trusted lab LAN; use `--host 127.0.0.1` for local-only access.
There is no authentication. Cross-origin browser writes are rejected.

The launching account must have access to `/dev/i2c-1`, normally through the
`i2c` group. Run from the same account where `i2cdetect -y 1` succeeds. A permission
or disconnected-device error is displayed in the UI and retried every second.
No fake readings are substituted when real hardware fails.

For permanent access, an administrator can run `sudo usermod -aG i2c nils`
(substitute the launching account), then start a fresh login session or reboot.
Existing sessions do not acquire the new supplementary group automatically.

For development without hardware:

```bash
python server.py --demo --port 8092
```

Demo data is visibly labelled. The simulator exercises acquisition and scaling;
it does not simulate the electrical ALERT pin or comparator hysteresis.

## Configuration

- All eight MUX options: four single-ended inputs plus AIN0-AIN1, AIN0-AIN3,
  AIN1-AIN3, and AIN2-AIN3. Enable any subset for sequential single-shot scanning.
- All eight data rates, 8 through 860 SPS. This is the chip conversion rate,
  shared across selected inputs, not the achieved per-channel browser rate.
- Six distinct PGA ranges, +/-6.144 V through +/-0.256 V. PGA bit patterns 6 and
  7 duplicate range 5 and are not offered separately.
- Per-input software multiplier and offset: `scaled = volts * multiplier + offset`.
  These do not change chip gain or the voltage/count conversion.
- Single-shot scanning or continuous conversion with exactly one enabled input.
  Optional pause between complete scans supports duty cycling in single-shot mode.
- ALERT disabled, traditional/window comparator, or conversion-ready output;
  polarity, latch, queue of 1/2/4 conversions, and signed raw thresholds.
  Threshold voltage equivalents update with PGA. Thresholds remain raw counts
  when changing gain. Comparator testing requires one input to avoid mixing signals.
- Configuration register and threshold readback, with decoded MUX/PGA/rate/mode.
  Apply is pending until the device acknowledges and readback succeeds.

Settings are held for this server session. Export JSON to retain configurations;
import loads an editable draft and Apply programs it. Configuration changes clear
the current history. CSV exports up to the latest 60 seconds / 30,000 samples
received by this browser, with timestamps, input, raw, volts, scaled, and clipping.
Charts and statistics use the selected 10/30/60-second window. Browser polling
fetches batches of readings, not just the last value. There is no persistent logger;
long disconnects may lose samples and produce a missed-readings notice.

Graphs scroll on browser animation frames independently of incoming batches, with foreground
polling targeting 60 Hz. The measured ADC samples/s and display fps appear above
the graphs. They are different: a 60 Hz display can show up to 60 frames/s, each
containing multiple ADC readings. All received samples remain in browser history;
The plotted line preserves bucket extrema while limiting points to the available
pixels; statistics refresh at 10 Hz. Measured rates use
the last two seconds so changes appear promptly. Two inputs at 64 SPS have an ideal ceiling of 32 readings/s
each, reduced by conversion-start, readiness polling, and I2C transfer overhead.
Configuration/threshold diagnostics are read once per second during acquisition
to avoid slowing every sample. Hidden pages reduce polling and skip chart work.

The Pi GPIO I2C clock can be raised from 100 kHz to the ADS1115 fast-mode limit of
400 kHz using `dtparam=i2c_arm_baudrate=400000` in `/boot/firmware/config.txt`, then
rebooting. This changes the whole bus; attached devices and wiring must support
that clock. It reduces transfer overhead, not the ADC conversion time. ADS1115
high-speed mode requires a special protocol; simply setting a clock above 400 kHz
is not equivalent. Recheck real acquisition and bus errors after changing it.

## Datasheet decisions and limits

Based on the supplied **Texas Instruments ADS1113/ADS1114/ADS1115 SBAS444E,
revised December 2024**, reviewed including the application, layout, and packaging
sections. [TI datasheet](https://www.ti.com/lit/ds/symlink/ads1115.pdf).

- Sections 5.3 and 7.3.1: each analog pin must remain between GND and VDD during
  normal operation, even for differential readings or a +/-6.144 V PGA range.
  The supply range is 2.0-5.5 V. Pi GPIO and I2C pull-ups must use Pi-compatible
  3.3 V levels. Never infer permitted pin voltage from the selected PGA range.
- Sections 7.3.3 and 7.5.4: `volts = signed_count * positive_full_scale / 32768`.
  Negative counts near zero are preserved for single-ended inputs. Endpoint
  counts are flagged as clipped, although an endpoint alone cannot prove overdrive.
- Sections 7.4.2 and 8.1.3: single-shot conversion polls OS with a bounded timeout.
  Configuration changes first stop the previous conversion. There is one ADC;
  channels are sequential, not simultaneous. Continuous mode uses conservative
  timed reads accounting for -10% oscillator tolerance, without a required RDY GPIO.
  Some continuous conversions can be skipped, especially at 860 SPS; measured
  reading rate is reported rather than promising lossless acquisition.
- Sections 7.3.7-7.3.9 and 8.1.4: reading conversion data clears latched alerts.
  Ready mode sets low threshold to 0x0000, high to 0x8000, and enables COMP_QUE.
  ALERT/RDY is an open-drain physical output requiring a pull-up. This tool
  configures it but does not sample a GPIO or claim to show the pin state.
- Sections 7.5.1 and 7.5.2: registers use MSB-first transfers. Addresses 0x48-0x4B
  follow the physical ADDR wiring; selecting an address does not readdress a chip.
  Address 0x48 alone is not proof of the model: ADS1115 has no chip-ID register.
  I2C clock speed is configured by the host, not an ADS1115 register.
- Sections 6, 9-11: lower data rates improve noise performance. Floating inputs
  are not meaningful voltage sources; ground an input for a baseline noise test.
  Source impedance, filtering, decoupling and wiring affect accuracy. Use the
  recommended 0.1 uF supply bypass close to the device.

No general-call reset is sent because it can reset other devices on the bus.
Shutdown powers down the selected ADC and disables ALERT; previous settings are
not restored. Demo mode is the default choice for automated tests.

## Verification

```bash
python -m unittest -v test_adc.py
python -m pip install playwright
python -m unittest -v test_browser.py
```

The browser test uses system Chromium, synthetic readings, and a temporary port.
To verify actual hardware, stop other ADC services, then run:

```bash
python hardware_test.py --bus 1 --address 0x48
ADS1115_HARDWARE_TEST=1 python -m unittest -v test_browser.py
```

The explicit hardware test checks all 384 MUX/PGA/rate combinations, 40 continuous
reads, and 48 ALERT register configurations, then restores the original registers.
It writes `/tmp/ads1115-hardware-results.json`. The browser hardware test changes
device settings and leaves the ADC powered down. These tests validate real I2C
communication, conversion completion, register readback and the browser workflow.
They do not establish absolute voltage accuracy without a known voltage source,
or verify the electrical ALERT output without a connected measurement instrument.

Bundled Chart.js 4.5.1 is MIT licensed; Lucide 0.468.0 is ISC licensed. License
files are included in `web/vendor`.
