# Arm servo demos

There are two standalone demos for MX-64 servos connected through the ArbotiX
FTDI USB adapter. Neither requires ROS or a firmware change to run against the
existing ArbotiX ROS firmware.

| Demo | Requirements | Where to run |
| --- | --- | --- |
| Python desktop (`mx64_slider.py`) | Python virtual environment and `requirements.txt` | Desktop session on the computer connected to the FTDI USB adapter |
| Browser (`web/index.html`) | Chrome/Chromium or Edge with Web Serial; no pip dependencies | Browser on the computer connected to the FTDI USB adapter |

## Before connecting

Stop the operator ROS launch with **Ctrl+C**, and close other servo tools so only
one program owns the FTDI port. Support the arm and leave clearance for motion.
These standalone demos do **not** enforce the operator dashboard's PX4 status
interlock. Do not rely on the RC switch to stop commands from a demo.

## Python desktop demo on the Pi

Run this on the **Pi's desktop, outside Docker**. The pygame window needs a
graphical desktop session; a plain SSH terminal is not enough.

### 1. Enter the demo directory

Run this from the repository root. If your prompt already ends in
`DevTools/servodemo`, do not run it again.

```bash
cd DevTools/servodemo
```

### 2. Set up the environment once

Only run the first command if `.venv` does not exist. Re-running it on the
existing environment can report an `activate.csh` permission error.

```bash
python3 -m venv .venv
```

Then activate the environment and install the dependencies:

```bash
source .venv/bin/activate
python -m pip install -r requirements.txt
```

If creating the environment reports that `venv` or `ensurepip` is unavailable,
install `python3-venv` with `sudo apt install python3-venv`, then run the setup
again.

### 3. Start the demo

Run this while your prompt is in `DevTools/servodemo` and shows `(.venv)`:

```bash
python mx64_slider.py \
  --port /dev/serial/by-id/usb-FTDI_FT232R_USB_UART_AI049UTL-if00-port0 \
  --id 1
```

On later runs, only repeat steps 1 and 3. If you are already in the demo
directory, skip step 1.

The demo waits for ArbotiX startup and scans servo IDs at 1 Mbps and 57,600 baud.
Wait for **Connected** and check that only the intended servo is selected.
`--id 1` is an initial selection, not a restriction on discovery: if that ID is
absent, the demo selects the discovered servos instead.

For a position-controlled servo, set the speed slider low, then use **−10°** or
**+10°** only if there is clearance for that travel. Each click enables torque
and commands a relative move; these are not hold-to-jog buttons.
**E-STOP ALL** releases torque on discovered servos. Press it before closing
the window; closing the Python demo does not explicitly release torque.

The **JOINT MODE** and **WHEEL MODE** buttons change persistent servo settings.
They are not required just to test an already configured joint servo.

## Browser demo

The browser demo is a static site, so do not activate the Python virtual
environment or install `requirements.txt` for it. Run this from the repository
root on the Pi:

```bash
python3 -m http.server 8080 --bind 127.0.0.1 --directory DevTools/servodemo/web
```

Open <http://localhost:8080> in Chrome/Chromium or Edge **on the Pi**, click
**Connect ArbotiX**, and choose the FTDI adapter. A browser on a remote PC
cannot access the Pi's USB through this static web server. For remote control,
run the Python desktop demo on the Pi, or add a serial proxy backend. You can alternatively open
`web/index.html` directly if the browser permits Web Serial from local files.
See [the browser README](web/README.md) for details.

## Troubleshooting

- **Wrong/missing port:** run `ls -l /dev/serial/by-id/` and supply the FTDI path
  with `--port`. The SVEA PX4 USB device is not the servo adapter.
- **Permission denied:** check `ls -l /dev/ttyUSB0` and `groups`. On systems
  where the port belongs to `dialout`, use `sudo usermod -aG dialout "$USER"`,
  then log out and back in. Run the demo as your normal user.
- **Port busy or intermittent responses:** stop ROS and other serial tools,
  including browser tabs connected to the adapter.
- **No servo discovered:** check servo power as well as USB; USB detection
  alone does not prove the servo has power. Check the displayed scan error.
- **Position updates but no motion:** note the mode, torque state, voltage,
  load, and any error shown when a jog is clicked. Do not force the linkage or
  change persistent settings blindly.

Load percentage is the servo's estimated signed load, not a measured physical
torque percentage. After testing, release torque and close the demo before
restarting the operator launch.
