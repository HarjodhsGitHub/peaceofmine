#!/usr/bin/env python3
"""ZED-F9P acquisition, checked parsing and NTRIP correction backend."""

from __future__ import annotations

import argparse
import base64
import json
import math
import os
import socket
import threading
import time
from collections import deque
from pathlib import Path
from typing import Any

import pynmea2
import serial
from serial import SerialException
from pyubx2 import UBXReader, UBXMessage, POLL


WEB_ROOT = Path(__file__).with_name("web")


class GnssState:
    def __init__(self, device: str, baud: int) -> None:
        self.device = device
        self.baud = baud
        self.stop_event = threading.Event()
        self.lock = threading.Lock()
        self.field_updated: dict[str, float] = {}
        self.port: serial.Serial | None = None
        self.data: dict[str, Any] = {
            "connected": False,
            "device": device,
            "baud": baud,
            "last_update": None,
            "last_message": "Waiting for GNSS data",
            "latitude": None,
            "longitude": None,
            "fix_quality": 0,
            "fix_label": "NO FIX",
            "satellites": None,
            "satellites_in_view": None,
            "fix_mode": "NO FIX",
            "altitude_m": None,
            "horizontal_accuracy_m": None,
            "accuracy_source": None,
            "hdop": None,
            "vdop": None,
            "pdop": None,
            "speed_mps": None,
            "course_deg": None,
            "utc": None,
            "ntrip_enabled": False,
            "ntrip_status": "Not configured",
            "corrections_bytes": 0,
            "last_correction": None,
        }
        self.gga_lock = threading.Lock()
        self.latest_gga: str | None = None
        self.gga_time = 0.0
        self.log_lock = threading.Lock()
        self.raw_log: deque[str] = deque(maxlen=200)
        self.event_log: deque[dict[str, str]] = deque(maxlen=100)

    def update(self, **values: Any) -> None:
        with self.lock:
            was_connected = self.data["connected"]
            old_fix = self.data["fix_label"]
            old_fix_mode = self.data["fix_mode"]
            self.data.update(values)
            self.field_updated.update({key: time.time() for key in values})
            self.data["last_update"] = time.time()
            self.data["connected"] = True
            new_fix = self.data["fix_label"]
            new_fix_mode = self.data["fix_mode"]
        if not was_connected:
            self.add_event("Receiver UART connection restored", "success")
        if new_fix != old_fix:
            self.add_event(f"Position status changed: {old_fix} -> {new_fix}", "success" if new_fix != "NO FIX" else "warning")
        if new_fix_mode != old_fix_mode and new_fix_mode != "NO FIX":
            self.add_event(f"Fix mode changed: {old_fix_mode} -> {new_fix_mode}", "info")

    def connection_error(self, message: str) -> None:
        with self.lock:
            was_connected = self.data["connected"]
            self.data["connected"] = False
            self.data["last_message"] = message
        if was_connected:
            self.add_event(f"Receiver UART connection lost: {message}", "error")

    def set_port(self, port: serial.Serial | None) -> None:
        with self.lock:
            self.port = port
        if port is None:
            with self.gga_lock:
                self.latest_gga = None

    def write_corrections(self, data: bytes) -> bool:
        with self.lock:
            if self.port is None or not self.port.is_open:
                return False
            if self.port.write(data) != len(data):
                raise RuntimeError("Incomplete RTCM write to GNSS UART")
            return True

    def set_gga(self, sentence: str | None) -> None:
        with self.gga_lock:
            self.latest_gga = sentence.strip() if sentence else None
            self.gga_time = time.monotonic()

    def get_gga(self) -> str | None:
        with self.gga_lock:
            return self.latest_gga if time.monotonic() - self.gga_time < 15 else None

    def append_raw_log(self, raw_line: bytes) -> None:
        timestamp = time.strftime("%H:%M:%S")
        if raw_line.startswith(b"$"):
            text = raw_line.decode("ascii", errors="replace").strip()
        else:
            preview = raw_line[:32].hex(" ")
            suffix = " ..." if len(raw_line) > 32 else ""
            text = f"[binary {len(raw_line)} bytes] {preview}{suffix}"
        with self.log_lock:
            self.raw_log.append(f"{timestamp}  {text}")

    def add_event(self, message: str, kind: str = "info") -> None:
        with self.log_lock:
            self.event_log.append({
                "time": time.strftime("%H:%M:%S"),
                "message": message,
                "kind": kind,
            })

    def set_ntrip(self, status: str, correction_bytes: int | None = None) -> None:
        with self.lock:
            old_status = self.data["ntrip_status"]
            self.data["ntrip_status"] = status
            if correction_bytes is not None:
                self.data["corrections_bytes"] += correction_bytes
                self.data["last_correction"] = time.time()
        if status != old_status and not status.startswith("Receiving RTCM"):
            kind = "error" if "error" in status.lower() else "info"
            self.add_event(f"RTK corrections: {status}", kind)

    def snapshot(self) -> dict[str, Any]:
        with self.lock:
            result = dict(self.data)
            result["field_age_s"] = {
                key: round(max(0.0, time.time() - updated), 1)
                for key, updated in self.field_updated.items()
            }
        with self.log_lock:
            result["raw_log"] = list(self.raw_log)
            result["event_log"] = list(self.event_log)
        if result["last_update"] is not None:
            result["age_s"] = round(max(0.0, time.time() - result["last_update"]), 1)
        else:
            result["age_s"] = None
        if result["last_correction"] is not None:
            result["correction_age_s"] = round(max(0.0, time.time() - result["last_correction"]), 1)
        else:
            result["correction_age_s"] = None
        result.pop("last_correction", None)
        return result


def fix_label(quality: int) -> str:
    return {
        0: "NO FIX",
        1: "GPS FIX",
        2: "DGPS",
        4: "RTK FIXED",
        5: "RTK FLOAT",
    }.get(quality, f"FIX {quality}")


def serial_reader(state: GnssState) -> None:
    while not state.stop_event.is_set():
        try:
            with serial.Serial(state.device, state.baud, timeout=1,
                               write_timeout=2, exclusive=True) as port:
                state.set_port(port)
                state.update(last_message="Serial connected")
                reader = UBXReader(port)
                last_accuracy_poll = 0.0
                while not state.stop_event.is_set():
                    if time.monotonic() - last_accuracy_poll >= 1:
                        # Poll only: do not change receiver configuration. Serialize
                        # with RTCM writes so packets cannot interleave on the UART.
                        with state.lock:
                            port.write(UBXMessage('NAV', 'NAV-PVT', POLL).serialize())
                        last_accuracy_poll = time.monotonic()
                    raw_line, parsed = reader.read()
                    if not raw_line:
                        continue
                    state.append_raw_log(raw_line)
                    if not raw_line.startswith(b"$"):
                        process_ubx(state, parsed)
                        continue
                    try:
                        sentence = pynmea2.parse(raw_line.decode("ascii", errors="ignore"), check=True)
                        process_sentence(state, sentence, raw_line.decode("ascii"))
                    except (pynmea2.ParseError, UnicodeError, ValueError, TypeError):
                        continue
        except (SerialException, OSError) as error:
            state.set_port(None)
            state.connection_error(f"Serial unavailable: {error}")
            state.stop_event.wait(2)


def process_ubx(state: GnssState, message: Any) -> None:
    if getattr(message, 'identity', None) != 'NAV-PVT':
        return
    valid = message.gnssFixOk and message.fixType in (2, 3, 4)
    radius = float(message.hAcc) / 1000 if valid else None
    state.update(horizontal_accuracy_m=radius, accuracy_source='UBX NAV-PVT')


def process_sentence(state: GnssState, sentence: Any, raw_line: str = "") -> None:
    values: dict[str, Any] = {"last_message": sentence.sentence_type}
    if sentence.sentence_type == "GGA":
        quality = int(sentence.gps_qual or 0)
        state.set_gga(raw_line if quality and sentence.lat and sentence.lon else None)
        values.update(
            latitude=sentence.latitude if sentence.lat else None,
            longitude=sentence.longitude if sentence.lon else None,
            fix_quality=quality,
            fix_label=fix_label(quality),
            satellites=int(sentence.num_sats) if sentence.num_sats else None,
            altitude_m=float(sentence.altitude) if sentence.altitude is not None else None,
            hdop=float(sentence.horizontal_dil) if sentence.horizontal_dil else None,
            utc=sentence.timestamp.isoformat() if sentence.timestamp else None,
        )
    elif sentence.sentence_type == "RMC":
        if sentence.status == "A":
            values.update(
                latitude=sentence.latitude if sentence.lat else None,
                longitude=sentence.longitude if sentence.lon else None,
                speed_mps=round(float(sentence.spd_over_grnd or 0) * 0.514444, 2),
                course_deg=float(sentence.true_course) if sentence.true_course else None,
                utc=sentence.timestamp.isoformat() if sentence.timestamp else None,
            )
        elif sentence.status == "V":
            values["last_message"] = "RMC (no fix)"
    elif sentence.sentence_type == "GSA":
        mode = getattr(sentence, "mode_fix_type", "")
        values.update(
            fix_mode={"1": "NO FIX", "2": "2D FIX", "3": "3D FIX"}.get(str(mode), "UNKNOWN"),
            pdop=float(sentence.pdop) if getattr(sentence, "pdop", "") else None,
            hdop=float(sentence.hdop) if getattr(sentence, "hdop", "") else None,
            vdop=float(sentence.vdop) if getattr(sentence, "vdop", "") else None,
        )
    elif sentence.sentence_type == "GST":
        lat = sentence.std_dev_latitude
        lon = sentence.std_dev_longitude
        radius = math.hypot(lat, lon) if lat is not None and lon is not None and lat >= 0 and lon >= 0 else None
        values.update(horizontal_accuracy_m=radius if radius is not None and math.isfinite(radius) else None,
                      accuracy_source='NMEA GST horizontal RMS')
    elif sentence.sentence_type == "GSV":
        satellites_in_view = getattr(sentence, "num_sv_in_view", "")
        if satellites_in_view:
            values["satellites_in_view"] = int(satellites_in_view)
    elif sentence.sentence_type == "GLL" and sentence.status == "A":
        values.update(
            latitude=sentence.latitude if sentence.lat else None,
            longitude=sentence.longitude if sentence.lon else None,
            utc=sentence.timestamp.isoformat() if sentence.timestamp else None,
        )
    elif sentence.sentence_type == "VTG":
        values.update(
            speed_mps=round(float(sentence.spd_over_grnd_kmph or 0) / 3.6, 2),
            course_deg=float(sentence.true_track) if sentence.true_track else None,
        )
    state.update(**values)


def read_ntrip_response(caster: socket.socket) -> bytes:
    """Consume HTTP headers or the single-line NTRIP v1 ICY response.

    Return any correction bytes already received with the response.
    """
    response = b""
    while b"\r\n" not in response:
        chunk = caster.recv(1024)
        if not chunk:
            raise RuntimeError("NTRIP caster closed during response")
        response += chunk
        if len(response) > 8192:
            raise RuntimeError("NTRIP response headers too large")
    first_line, remainder = response.split(b"\r\n", 1)
    parts = first_line.split()
    if len(parts) < 2 or parts[0] not in (b"ICY", b"HTTP/1.0", b"HTTP/1.1") or parts[1] != b"200":
        if len(parts) > 1 and parts[1] in (b"401", b"403"):
            raise RuntimeError("Caster rejected credentials or mountpoint access (check .env)")
        raise RuntimeError("Caster rejected stream: " + first_line.decode("ascii", errors="replace"))
    if parts[0] == b"ICY":
        return remainder
    while b"\r\n\r\n" not in response:
        chunk = caster.recv(1024)
        if not chunk:
            raise RuntimeError("NTRIP caster closed during headers")
        response += chunk
        if len(response) > 8192:
            raise RuntimeError("NTRIP response headers too large")
    headers, remainder = response.split(b"\r\n\r\n", 1)
    if b"transfer-encoding:" in headers.lower():
        raise RuntimeError("Unsupported transfer encoding; caster must support NTRIP v1")
    if b"sourcetable" in headers.lower() or b"text/html" in headers.lower():
        raise RuntimeError("Caster returned a page or sourcetable; check NTRIP_MOUNTPOINT")
    return remainder


def stream_corrections(state: GnssState, caster: socket.socket, initial: bytes) -> None:
    last_gga = 0.0
    last_correction = time.monotonic()
    corrections = initial
    while not state.stop_event.is_set():
        now = time.monotonic()
        gga = state.get_gga()
        if not gga:
            raise RuntimeError("No fresh GNSS position for NTRIP; waiting for a valid GGA fix")
        if now - last_gga >= 10:
            caster.sendall((gga + "\r\n").encode("ascii"))
            last_gga = now
        if corrections:
            if not state.write_corrections(corrections):
                raise RuntimeError("GNSS UART is not available")
            state.set_ntrip("Receiving RTCM corrections", len(corrections))
            last_correction = now
        if now - last_correction > 30:
            raise RuntimeError("No RTCM received for 30 seconds")
        try:
            corrections = caster.recv(4096)
        except socket.timeout:
            corrections = b""
            continue
        if not corrections:
            raise RuntimeError("NTRIP caster closed the connection")


def ntrip_reader(state: GnssState, host: str, port: int, mountpoint: str,
                 username: str, password: str) -> None:
    state.data["ntrip_enabled"] = True
    credentials = base64.b64encode(f"{username}:{password}".encode()).decode()
    while not state.stop_event.is_set():
        try:
            state.set_ntrip("Waiting for GNSS UART")
            while state.port is None and not state.stop_event.is_set():
                state.stop_event.wait(1)
            state.set_ntrip("Waiting for valid GNSS position (GGA)")
            while not state.get_gga() and not state.stop_event.is_set():
                state.stop_event.wait(1)
            if state.stop_event.is_set():
                return
            with socket.create_connection((host, port), timeout=10) as caster:
                request = (
                    f"GET /{mountpoint.lstrip('/')} HTTP/1.0\r\n"
                    f"Host: {host}:{port}\r\n"
                    "User-Agent: NTRIP PeaceOfMine/1.0\r\n"
                    f"Authorization: Basic {credentials}\r\n"
                    "Accept: */*\r\nConnection: close\r\n\r\n"
                ).encode("ascii")
                caster.sendall(request)
                initial = read_ntrip_response(caster)
                caster.settimeout(1)
                state.set_ntrip("Connected, waiting for RTCM")
                stream_corrections(state, caster, initial)
        except (OSError, RuntimeError) as error:
            state.set_ntrip(f"NTRIP error: {error}")
            state.stop_event.wait(5)


