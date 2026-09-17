# ArbotiX-M Servo Toolbox

This is a standalone static website. It has no backend, package install, or
virtual environment requirement. The browser connects directly to the ArbotiX
FTDI device through the Web Serial API.

## Start for local testing

From the repository root, start a local static server. Do not activate the
servodemo Python virtual environment or install `requirements.txt`; those are
only needed for the Python desktop demo:

```bash
cd DevTools/servodemo
python3 -m http.server 8080 --directory web
```

Open <http://localhost:8080> in Chrome or Edge on the same computer connected
to the servo adapter. Keep the terminal running while testing the site; press
**Ctrl+C** to stop the server.

If Python reports `OSError: [Errno 98] Address already in use`, a server may
already be running on port 8080, so try <http://localhost:8080> first. To use
another port instead, start the server with a matching directory and URL:

```bash
python3 -m http.server 8081 --directory web
```

Then open <http://localhost:8081>.

## Use

1. Close any Python/terminal program that has the ArbotiX serial port open.
2. Open `index.html` in current Chrome or Edge on desktop.
3. Click **Connect ArbotiX** and choose the FTDI serial device in the browser
   permission picker.

If a browser blocks `file://` access in its local policy, host this directory
with any static HTTPS host or a local static-file server. The host must not
interfere with the serial connection; all DYNAMIXEL traffic stays in the
browser.

Web Serial is not supported by Safari or Firefox. It requires a secure context
and a user-initiated permission request. The serial connection belongs to the
browser computer: opening this site from a remote PC does not give it access to
the Raspberry Pi's USB devices.

For remote testing with the servo adapter connected to the Raspberry Pi, run
the Python desktop demo on the Pi instead. The static browser demo would need a
serial proxy backend on the Pi before a remote browser could control that
hardware.

## Live telemetry

The toolbox polls each selected servo and plots its MX-64 **Present Load**
register over the most recent 60 seconds. The value is a signed controller load
estimate displayed as -100% to +100%; it is not calibrated physical force,
pressure, or output-shaft torque.

The chart uses uPlot 1.6.32. Its JavaScript, stylesheet, and MIT license are
vendored in `web/vendor/`, so the page has no CDN or runtime dependency.
