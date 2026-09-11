# Accuracy investigation — 2026-09-11

No fixed position offset is applied. The preset reference was removed; optional
reference comparison starts disabled. The user reports antenna placement near a
window. The supplied expected coordinates are approximate and their source is
not yet established.

## Observations

- Displayed coordinates match decoded receiver NMEA position sentences.
- pyubx2 NAV-PVT hAcc is millimetres; 1043 decodes to 1.043 m correctly.
- A 30-s live capture stayed RTK FLOAT, with 12 satellites reported by GGA.
- Corrections grew by 32,034 bytes; sampled correction ages were 0.1–0.4 s.
- Receiver RXM-RTCM reports explicitly mark 1005, 1074, 1084, 1094, 1124,
  1230 and 1033 as used (msgUsed=2). No captured report indicated a CRC failure.
  This verifies correction acceptance, not the correctness of the reference
  station coordinates or every aspect of receiver configuration.
- Reported hAcc ranged 0.179–0.222 m during that capture. This is an internal
  uncertainty estimate, not an independent measurement of true position error.
- GSV shows a broad spread of signal levels, including weak signals. These
  messages alone do not prove multipath or identify a hardware fault.

## Working hypothesis and discriminating test

Restricted sky visibility and reflected/non-line-of-sight signals at the window
are the leading hypothesis, not a confirmed root cause. The u-blox integration
manual requires good signal levels and as wide a sky view as possible:
https://content.u-blox.com/sites/default/files/ZED-F9P_IntegrationManual_UBX-18010802.pdf

Move the antenna outdoors with clear sky, away from walls, using its appropriate
mounting/ground plane. Keep receiver settings and correction service unchanged.
Collect several minutes of fix state, signal levels, receiver uncertainty and
positions. Because moving the antenna changes its real position, do not compare
against the old coordinates; use an independently established reference at the
new location to assess absolute error. Stable RTK FIXED and improved agreement
would support the reception-environment hypothesis. Persistence outdoors warrants
checking antenna/cable/power, interference, receiver configuration and firmware,
and correction/reference-frame setup. Do not inflate the circle or calibrate a
constant offset from this single window observation.
