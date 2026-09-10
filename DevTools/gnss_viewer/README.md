# GNSS remote viewer

A small Raspberry Pi service for the MikroE ZED-F9P. It reads checked NMEA
sentences from the Pi 5 GPIO UART and serves a remote browser dashboard.

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

The viewer connects to the configured NTRIP mountpoint, sends the receiver's
latest GGA position every 10 seconds, and forwards received RTCM bytes to the
ZED-F9P. The dashboard shows the NTRIP connection and correction byte count.
Only configure credentials on the Pi; `.env` is ignored by Git.

If your account uses a different caster or mountpoint, override the defaults in
`.env`. If the credentials are absent, the viewer still displays ordinary GNSS
telemetry and reports that credentials are not configured for RTCM.

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
