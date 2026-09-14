# GNSS remote viewer

A small Raspberry Pi service for the MikroE ZED-F9P. It reads checked NMEA
sentences from the Pi 5 GPIO UART and serves a remote browser dashboard.
The serial parser also handles mixed UBX/NMEA output from the SVEA RTK manager.

## Install and run

The ZED-F9P in this setup outputs NMEA on `/dev/ttyAMA0` at `115200` baud.
Only one program can read the UART at a time.

```bash
cd DevTools/gnss_viewer
python3 -m venv .venv
. .venv/bin/activate
python -m pip install -r requirements.txt
python gnss_viewer.py
```

## RTK corrections via SWEPOS/NTRIP

The receiver only reaches RTK float/fixed when it receives RTCM correction
data. Copy the template and fill it with the caster details and account issued
by Lantmäteriet/SWEPOS:

```bash
cp .env.example .env
${EDITOR:-nano} .env
```

The existing SVEA defaults are already built in: host `nrtk-swepos.lm.se`,
port `80`, and mountpoint `MSM_GNSS`. With a normal SVEA/SWEPOS account, only
set `NTRIP_USERNAME` and `NTRIP_PASSWORD` in `.env`.
Use exactly one assignment `=`: `NTRIP_PASSWORD="your_password"`.
An extra `=` becomes part of the password and causes authentication to fail.
Numeric passwords stay strings; the ROS launch file's YAML quoting workaround
is not needed in this standalone Python demo. Exported environment variables
take precedence over `.env`, so unset old `NTRIP_*` exports when changing accounts.

The viewer connects to the configured NTRIP mountpoint, sends the receiver's
latest valid GGA position every 10 seconds, and forwards received RTCM bytes to the
ZED-F9P. The dashboard shows the NTRIP connection and correction byte count.
Only configure credentials on the Pi; `.env` is ignored by Git.

If your account uses a different caster or mountpoint, override the defaults in
`.env`. If the credentials are absent, the viewer still displays ordinary GNSS
telemetry and reports that credentials are not configured for RTCM.

The demo waits for a valid GGA fix before connecting. It handles both the
single-line NTRIP v1 `ICY` response and HTTP responses, including corrections
arriving in the same packet as the response. It reconnects if corrections stop
for 30 seconds, or if the receiver's GGA becomes stale.

The caster, port, mountpoint, and baud defaults match `svea_charging` on
`bt_outdoor_charging_integration`. That branch uses an Arduino USB-to-UART1
bridge; this demo defaults to the Pi GPIO UART. Set `--device` for your actual
connection. The demo reads the receiver's existing configuration: NMEA GGA
output and RTCM3 input must be enabled on the connected receiver interface.
Stop the ROS RTK manager before using the demo so they do not compete for serial
data. Other dashboard fields depend on which NMEA messages are enabled; the
working ROS setup disables most sentences other than GGA.

Open `http://<raspberry-pi-ip>:8090` from another computer on the same network.
The server binds to all interfaces by default. Use `--host 127.0.0.1` if it
should only be accessible on the Pi, or put it behind an authenticated reverse
proxy before exposing it outside a trusted network.

Options:

```bash
python gnss_viewer.py --device /dev/ttyAMA0 --baud 115200 --port 8090
```

The dashboard displays the latest valid GGA/RMC data, including RTK fixed and
RTK float status. A position fix still requires antenna sky view; RTK fixed
requires RTCM corrections from a base station or NTRIP service.

## Map and rolling history

The dashboard keeps 30 seconds of samples in the browser, sampled approximately
once per second. Numeric readings use interactive Chart.js graphs with their observed min/max;
connection and fix states remain compact status indicators. PDOP and VDOP share a chart.
Correction throughput is measured in bytes per second. Static device settings,
UTC, and message names remain text. History starts when the page opens and resets
on reload. Missing/stale readings leave gaps; charts scale independently and DOP
is not an accuracy measurement in metres.

The Leaflet map shows valid positions, a rolling 30-second trail, and a follow
toggle. Dragging pauses following. A lost fix leaves a grey last-known marker;
before the first fix there is no marker. OpenStreetMap tiles load directly in
the browser and require internet; no API key is needed. Tile requests identify
the viewed map area to OpenStreetMap. Telemetry and charts work without tiles.
The bundled Leaflet 1.9.4 library is BSD-2-Clause licensed; its license is in
`web/vendor/leaflet/LICENSE`. Use the map in accordance with the
[OpenStreetMap tile policy](https://operations.osmfoundation.org/policies/tiles/).

Refresh the browser after frontend updates. Restart the Python viewer after
backend updates to enable per-field freshness tracking (so, for example, new
satellite messages cannot make an old position appear fresh).

Run the offline regression tests after installing dependencies:

```bash
python -m unittest -v test_gnss_viewer.py
```

### Dashboard dependencies and troubleshooting

Chart.js 4.5.1 and Leaflet 1.9.4 are bundled locally, so charts do not depend on
CDN access or an npm build on the Pi. Chart.js uses the MIT license, included in
`web/vendor/chartjs/LICENSE.md`. Chart configuration follows the
[Chart.js line chart documentation](https://www.chartjs.org/docs/latest/charts/line.html).

Missing chart readings now name the required NMEA messages: GGA provides position,
altitude, satellites and HDOP; GSA provides PDOP/VDOP and fix mode; RMC or VTG
provides speed/course; GSV provides satellites in view. A GGA-only receiver will
not populate the other fields. Stale readings and viewer/receiver disconnections
are shown explicitly, and polling recovers automatically. History is still local
to the browser and starts on page load. Valid zero coordinates/altitude are retained.

Restart the Python service and reload the browser to apply this update. Static
assets now require cache revalidation to avoid mixing old and new scripts.

Optional browser regression (requires system Chromium):

```bash
python -m pip install playwright
python -m unittest -v test_dashboard.py
```

This test uses a temporary localhost server and synthetic GGA data, blocks
external tile requests, and checks charts, missing-message explanations, mobile
layout, stale positions, HTTP failures, and recovery without real hardware.

### Position accuracy circle

The blue map circle uses the receiver's horizontal accuracy estimate as its
radius in metres. A numeric readout and rolling chart accompany it. It is an
estimate, not a guaranteed error bound; no 95% confidence is implied. Small RTK
radii may be smaller than the position marker at the current map zoom.

The viewer polls UBX NAV-PVT once per second over the existing UART without
changing saved receiver configuration. UBX input/output must be available on
that interface. NAV-PVT hAcc is converted from millimetres to metres following
the [u-blox interface description](https://cdn.sparkfun.com/assets/learn_tutorials/8/5/6/ZED-F9P_UBX_NMEA_and_RTCM_protocols.pdf).
NMEA GST is also supported: the radius is sqrt(latitude_stddev² + longitude_stddev²),
labelled horizontal RMS. HDOP is never treated as metres. Missing or stale
accuracy, a lost fix, or disconnection removes the circle.

Restart the Python viewer after this update, then reload the browser to begin
requesting accuracy data. Older running backends cannot supply this new field.

The map also compares the receiver with an editable approximate reference,
disabled by default; no fixed offset or coordinate correction is applied. Orange marks the reference
and the connecting line; the displayed distance is an offset to that reference,
not a surveyed error measurement. Clear the reference when moving the antenna
or enter its new known position. Reference changes apply to the current page.
A warning identifies references outside the receiver uncertainty circle; the
circle is not enlarged to disguise an overconfident receiver estimate.

The dashboard separates receiver connectivity, NTRIP service connection, recent
RTCM delivery and the receiver's reported position solution. The large summary
identifies Regular GPS, Differential GPS, RTK float, RTK fixed, missing fix or
stale/offline data. Correction delivery is not proof of an RTK fixed solution;
float remains amber and explicitly notes that position can drift. These status
indicators use existing telemetry and need only a browser reload.
