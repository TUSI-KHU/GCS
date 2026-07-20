# -*- coding: utf-8 -*-
from __future__ import annotations

import argparse
import csv
import math
import os
import re
import sys
import threading
import time
import webbrowser
from datetime import datetime

from flask import Flask, jsonify, request, send_file

from uplink import PyroUplink
import protocol as p


HERE = os.path.dirname(os.path.abspath(__file__))
LOG_DIR = os.path.join(HERE, "logs")
HTML_FILENAME = "mission_control.html"

app = Flask(__name__)
uplink: PyroUplink | None = None

FLIGHT_STATE_NAMES = {
    p.FLIGHT_INIT: "INIT",
    p.FLIGHT_SAFE: "SAFE",
    p.FLIGHT_ARMED: "ARMED",
    p.FLIGHT_LAUNCH: "LAUNCH",
    p.FLIGHT_COAST: "COAST",
    p.FLIGHT_APOGEE: "APOGEE",
    p.FLIGHT_DROGUE: "DROGUE",
    p.FLIGHT_DEPLOY: "DEPLOY",
    p.FLIGHT_GROUND: "GROUND",
    p.FLIGHT_FAULT: "FAULT",
}


def avionics_downlink_only() -> bool | None:
    """Return the adjacent avionics source setting when it is available."""
    override = os.getenv("NURA_AVIONICS_DOWNLINK_ONLY")
    if override in {"0", "1"}:
        return override == "1"
    constants_path = os.getenv(
        "NURA_AVIONICS_CONSTANTS_PATH",
        os.path.abspath(os.path.join(HERE, "..", "2026-nura-avionics", "include", "nura_constants.h")),
    )
    try:
        with open(constants_path, "r", encoding="utf-8") as handle:
            source = handle.read()
    except OSError:
        return None
    match = re.search(r"\bkFlightDownlinkOnly\s*=\s*(true|false)\s*;", source)
    return None if match is None else match.group(1) == "true"


class TelemetryLogger:
    def __init__(self) -> None:
        os.makedirs(LOG_DIR, exist_ok=True)
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.path = os.path.join(LOG_DIR, f"flight_log_{ts}.csv")
        self.file = open(self.path, "w", newline="", encoding="utf-8")
        self.writer = csv.DictWriter(
            self.file,
            fieldnames=["ts", "alt", "lat", "lng", "pitch", "roll", "yaw", "accel"],
        )
        self.writer.writeheader()

    def log(self, row: dict) -> None:
        self.writer.writerow(row)
        self.file.flush()

    def close(self) -> None:
        if not self.file.closed:
            self.file.close()


class HardwareTelemetryLogger:
    fieldnames = [
        "received_at", "packet_id", "packet_type", "seq", "payload_hex", "ts",
        "alt", "baro_alt_agl_m", "gps_alt_m", "lat", "lng",
        "pitch", "roll", "yaw", "accel", "accel_x_g", "accel_y_g", "accel_z_g",
        "gyro_x", "gyro_y", "gyro_z", "yaw_rate_dps", "baro_dp_2pa",
        "batt_mv", "health", "state", "state_code", "status_word",
        "gps_fix", "gps_fix_flags", "satellites", "hdop", "gps_age_s",
        "speed_mps", "course_deg", "rssi", "snr",
        "sequence_gaps", "duplicate_frames", "out_of_order_frames", "sequence_resets",
    ]

    def __init__(self) -> None:
        os.makedirs(LOG_DIR, exist_ok=True)
        ts = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        self.path = os.path.join(LOG_DIR, f"hardware_log_{ts}.csv")
        self.file = open(self.path, "w", newline="", encoding="utf-8")
        self.writer = csv.DictWriter(self.file, fieldnames=self.fieldnames, extrasaction="ignore")
        self.writer.writeheader()

    def log(self, row: dict, packet_type: str, seq: int, payload_hex: str) -> None:
        output = dict(row)
        output["received_at"] = datetime.now().astimezone().isoformat(timespec="milliseconds")
        output["packet_type"] = packet_type
        output["seq"] = seq
        output["payload_hex"] = payload_hex
        self.writer.writerow(output)
        self.file.flush()

    def close(self) -> None:
        if not self.file.closed:
            self.file.close()


class TelemetrySimulator:
    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.logger: TelemetryLogger | None = None
        self.index = 0
        self.launch_site = (34.4258, 127.5211)
        self.apogee_pos: tuple[float, float] | None = None

    def reset(self) -> None:
        with self.lock:
            if self.logger is not None:
                self.logger.close()
            self.logger = TelemetryLogger()
            self.index = 0
            self.apogee_pos = None

    def close(self) -> None:
        with self.lock:
            if self.logger is not None:
                self.logger.close()
                self.logger = None

    def next_point(self) -> dict:
        with self.lock:
            if self.logger is None:
                self.logger = TelemetryLogger()
            row = self._generate_point(self.index)
            self.index += 1
            self.logger.log(row)
            return row

    def _generate_point(self, index: int) -> dict:
        t = index * 0.2
        launch_lat, launch_lng = self.launch_site
        wind_lat = 0.000012
        wind_lng = 0.000020
        alt = accel = pitch = roll = yaw = 0.0
        lat = launch_lat
        lng = launch_lng

        if t < 0.5:
            pass
        elif t <= 2.41:
            tb = t - 0.5
            frac = tb / 1.91
            accel = 42.94 + frac * (56.35 - 42.94) + math.sin(t * 9.0) * 0.7
            alt = 90 * pow(frac, 1.6)
            pitch = math.sin(t * 7.0) * 1.25
            roll = math.cos(t * 6.0) * 1.5
            yaw = math.sin(t * 5.0)
            lat = launch_lat + tb * wind_lat * 0.3
            lng = launch_lng + tb * wind_lng * 0.3
        elif t <= 10.11:
            tc = t - 2.41
            frac = tc / (10.11 - 2.41)
            alt = 90 + (400 - 90) * math.sin(frac * math.pi * 0.5)
            accel = -9.8 * (1 - frac * 0.3) + math.sin(t * 4.0) * 0.4
            pitch = frac * 8 + math.sin(t * 2.0) * 0.75
            roll = frac * 15 + math.cos(t * 2.4)
            yaw = frac * 5 + math.sin(t * 1.7) * 0.75
            lat = launch_lat + (2.41 - 0.5) * wind_lat * 0.3 + tc * wind_lat
            lng = launch_lng + (2.41 - 0.5) * wind_lng * 0.3 + tc * wind_lng
        elif t <= 11.14:
            ta = t - 10.11
            frac = ta / (11.14 - 10.11)
            alt = 400 - (400 - 395.52) * frac
            accel = -9.8 + math.sin(t * 12.0)
            pitch = math.sin(frac * math.pi) * 25 + math.sin(t * 8.0) * 2.5
            roll = 8 + ta * 40 + math.cos(t * 9.0) * 2.5
            yaw = ta * 20 + math.sin(t * 6.0) * 2.0
            if self.apogee_pos is None:
                self.apogee_pos = (
                    launch_lat + (2.41 - 0.5) * wind_lat * 0.3 + (10.11 - 2.41) * wind_lat,
                    launch_lng + (2.41 - 0.5) * wind_lng * 0.3 + (10.11 - 2.41) * wind_lng,
                )
            lat = self.apogee_pos[0] + ta * wind_lat * 0.5
            lng = self.apogee_pos[1] + ta * wind_lng * 0.5
        elif t <= 25.23:
            td = t - 11.14
            frac = td / (25.23 - 11.14)
            alt = 395.52 - (395.52 - 150) * frac
            accel = -2.5 + math.sin(t * 3.0) * 0.25
            pitch = math.sin(td * 0.8) * 6 + math.sin(t * 5.0)
            roll = math.sin(td * 0.4) * 8 + math.cos(t * 3.0) * 1.5
            yaw = math.sin(td * 0.5) * 5 + math.sin(t * 2.0)
            base = self.apogee_pos or self.launch_site
            lat = base[0] + (td + 1.03) * wind_lat * 0.5
            lng = base[1] + (td + 1.03) * wind_lng * 0.5
        elif t <= 43:
            tm = t - 25.23
            alt = max(0.0, 150 - 8.17 * tm)
            accel = -0.5 + math.sin(t * 2.0) * 0.15
            pitch = math.sin(tm * 0.4) * 3 + math.sin(t * 3.0) * 0.75
            roll = math.sin(tm * 0.3) * 4 + math.cos(t * 2.0) * 0.75
            yaw = math.sin(tm * 0.25) * 3 + math.sin(t * 1.7) * 0.75
            base = self.apogee_pos or self.launch_site
            lat = base[0] + (tm + 14.09 + 1.03) * wind_lat * 0.4
            lng = base[1] + (tm + 14.09 + 1.03) * wind_lng * 0.4
        else:
            base = self.apogee_pos or self.launch_site
            lat = base[0] + (43 - 25.23 + 14.09 + 1.03) * wind_lat * 0.4
            lng = base[1] + (43 - 25.23 + 14.09 + 1.03) * wind_lng * 0.4

        return {
            "ts": round(t, 2),
            "alt": round(max(0.0, alt), 2),
            "lat": round(lat, 7),
            "lng": round(lng, 7),
            "pitch": round(pitch, 2),
            "roll": round(roll, 2),
            "yaw": round(yaw, 2),
            "accel": round(accel, 3),
        }


telemetry = TelemetrySimulator()


class HardwareTelemetry:
    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.logger: HardwareTelemetryLogger | None = None
        self.logging_enabled = False
        self._reset_state()

    def _reset_state(self) -> None:
        self.latest: dict | None = None
        self.packet_id = 0
        self.fast_count = 0
        self.gps_count = 0
        self.control_count = 0
        self.last_downlink_seq: int | None = None
        self.last_fast_seq: int | None = None
        self.last_gps_seq: int | None = None
        self.sequence_gaps = 0
        self.duplicate_frames = 0
        self.out_of_order_frames = 0
        self.sequence_resets = 0
        self.last_fast_payload_hex = ""
        self.last_gps_payload_hex = ""
        self.last_packet_at: float | None = None
        self.lat = 34.4258
        self.lng = 127.5211
        self.alt = 0.0
        self.baro_alt_agl_m = 0.0
        self.gps_alt_m: float | None = None
        self.base_boot_ms: int | None = None
        self.last_boot_ms: int | None = None
        self.state = "UNKNOWN"
        self.state_code: int | None = None
        self.status_word: str | None = None
        self.gps_fix = False
        self.gps_fix_flags = 0
        self.satellites: int | None = None
        self.hdop: float | None = None
        self.gps_age_s: float | None = None
        self.speed_mps: float | None = None
        self.course_deg: float | None = None
        self.rssi: int | None = None
        self.snr: float | None = None
        self.receiver_status: dict[str, str] = {}
        self.last_receiver_status_at: float | None = None

    def start_logging(self) -> str:
        with self.lock:
            self.logging_enabled = True
            if self.logger is None:
                self.logger = HardwareTelemetryLogger()
            return self.logger.path

    def close_logging(self) -> None:
        with self.lock:
            if self.logger is not None:
                self.logger.close()
                self.logger = None
            self.logging_enabled = False

    def reset(self) -> str | None:
        with self.lock:
            if self.logger is not None:
                self.logger.close()
                self.logger = None
            self._reset_state()
            if self.logging_enabled:
                self.logger = HardwareTelemetryLogger()
            return None if self.logger is None else self.logger.path

    def _default_latest(self) -> dict:
        return {
            "ts": 0.0,
            "alt": round(self.baro_alt_agl_m, 2),
            "baro_alt_agl_m": round(self.baro_alt_agl_m, 2),
            "gps_alt_m": self.gps_alt_m,
            "lat": round(self.lat, 7),
            "lng": round(self.lng, 7),
            "pitch": 0.0,
            "roll": 0.0,
            "yaw": 0.0,
            "accel": 0.0,
        }

    def _row_locked(self, source: str) -> dict:
        row = dict(self.latest) if self.latest is not None else self._default_latest()
        age = None if self.last_packet_at is None else round(time.monotonic() - self.last_packet_at, 3)
        row.update({
            "source": source,
            "packet_id": self.packet_id,
            "fast_count": self.fast_count,
            "gps_count": self.gps_count,
            "control_count": self.control_count,
            "state": self.state,
            "state_code": self.state_code,
            "status_word": self.status_word,
            "gps_fix": self.gps_fix,
            "gps_fix_flags": self.gps_fix_flags,
            "satellites": self.satellites,
            "hdop": self.hdop,
            "gps_age_s": self.gps_age_s,
            "speed_mps": self.speed_mps,
            "course_deg": self.course_deg,
            "rssi": self.rssi,
            "snr": self.snr,
            "baro_alt_agl_m": round(self.baro_alt_agl_m, 2),
            "gps_alt_m": None if self.gps_alt_m is None else round(self.gps_alt_m, 2),
            "last_packet_age_s": age,
            "sequence_gaps": self.sequence_gaps,
            "duplicate_frames": self.duplicate_frames,
            "out_of_order_frames": self.out_of_order_frames,
            "sequence_resets": self.sequence_resets,
            "receiver_status": dict(self.receiver_status),
        })
        return row

    def waiting_point(self) -> dict:
        with self.lock:
            return self._row_locked("hardware_waiting")

    def current_point(self) -> dict:
        with self.lock:
            age = None if self.last_packet_at is None else time.monotonic() - self.last_packet_at
            source = "hardware" if self.latest is not None and age is not None and age <= 10.0 else "hardware_waiting"
            return self._row_locked(source)

    def disconnected_point(self) -> dict:
        with self.lock:
            row = self._row_locked("hardware_disconnected")
            row["connected"] = False
            return row

    def status(self, link: PyroUplink | None) -> dict:
        link_diagnostics = None if link is None or link.simulate else link.diagnostics()
        with self.lock:
            age = None if self.last_packet_at is None else time.monotonic() - self.last_packet_at
            point_source = "hardware" if self.latest is not None and age is not None and age <= 10.0 else "hardware_waiting"
            row = self._row_locked(point_source)
            return {
                "connected": bool(link is not None and link.is_open()),
                "simulate": bool(link is not None and link.simulate),
                "port": None if link is None else link.port,
                "serial_mode": None if link is None else link.serial_mode,
                "source": "simulate" if link is not None and link.simulate else row["source"],
                "packet_id": self.packet_id,
                "fast_count": self.fast_count,
                "gps_count": self.gps_count,
                "control_count": self.control_count,
                "last_downlink_seq": self.last_downlink_seq,
                "last_fast_seq": self.last_fast_seq,
                "last_gps_seq": self.last_gps_seq,
                "last_packet_age_s": row["last_packet_age_s"],
                "sequence_gaps": self.sequence_gaps,
                "duplicate_frames": self.duplicate_frames,
                "out_of_order_frames": self.out_of_order_frames,
                "sequence_resets": self.sequence_resets,
                "last_fast_payload_hex": self.last_fast_payload_hex,
                "last_gps_payload_hex": self.last_gps_payload_hex,
                "receiver_status": dict(self.receiver_status),
                "log_path": None if self.logger is None else self.logger.path,
                "transport": link_diagnostics,
                "latest": None if self.latest is None else dict(self.latest),
            }

    def next_point(self, link: PyroUplink | None) -> dict | None:
        if link is None or link.simulate or not link.is_open():
            return None

        frames, lines = link.read_frames_and_lines(4096)
        new_telemetry = False
        with self.lock:
            for frame in frames:
                if frame.msg_type == p.MESSAGE_FAST_TLM:
                    new_telemetry = self._apply_fast(frame.seq, frame.payload) or new_telemetry
                elif frame.msg_type == p.MESSAGE_GPS_TLM:
                    new_telemetry = self._apply_gps(frame.seq, frame.payload) or new_telemetry
                elif frame.msg_type == p.MESSAGE_CONTROL and self._accept_sequence(frame.seq):
                    self.control_count += 1
                    self._log_latest("CONTROL", frame.seq, frame.payload.hex(" "))

            # In raw mode these are only NURA_BRIDGE diagnostics. In text mode
            # they are decoded receiver telemetry. Frames are never discarded.
            for line in lines:
                new_telemetry = self._apply_receiver_line(line) or new_telemetry

            if self.latest is None or not new_telemetry:
                return None
            return self._row_locked("hardware")

    def _prepare_boot_clock(self, boot_ms: int) -> float:
        if self.last_boot_ms is not None:
            wrapped = self.last_boot_ms > 0xF0000000 and boot_ms < 0x0FFFFFFF
            if not wrapped and boot_ms + 5000 < self.last_boot_ms:
                self.base_boot_ms = boot_ms
                self.last_downlink_seq = None
                self.sequence_resets += 1
        if self.base_boot_ms is None:
            self.base_boot_ms = boot_ms
        self.last_boot_ms = boot_ms
        return ((boot_ms - self.base_boot_ms) & 0xFFFFFFFF) / 1000.0

    def _accept_sequence(self, seq: int) -> bool:
        seq &= 0xFFFF
        if self.last_downlink_seq is None:
            self.last_downlink_seq = seq
            return True
        delta = (seq - self.last_downlink_seq) & 0xFFFF
        if delta == 0:
            self.duplicate_frames += 1
            return False
        if delta >= 0x8000:
            self.out_of_order_frames += 1
            return False
        if delta > 1:
            self.sequence_gaps += delta - 1
        self.last_downlink_seq = seq
        return True

    def _apply_receiver_line(self, line: str) -> bool:
        if line.startswith("rx type=FAST "):
            return self._apply_receiver_fast_line(line)
        if line.startswith("rx type=GPS "):
            return self._apply_receiver_gps_line(line)
        if line.startswith("rx type=CONTROL "):
            match = re.search(r"frame_seq=(\d+)", line)
            if match is None or self._accept_sequence(int(match.group(1))):
                self.control_count += 1
            return False
        if line.startswith("status ") or line.startswith("NURA_BRIDGE "):
            status = {}
            for token in line.split()[1:]:
                if "=" in token:
                    key, value = token.split("=", 1)
                    status[key] = value
            if status:
                self.receiver_status.update(status)
                self.last_receiver_status_at = time.monotonic()
                try:
                    if "last_rssi" in status:
                        self.rssi = int(status["last_rssi"])
                    if "last_snr" in status:
                        self.snr = float(status["last_snr"])
                except ValueError:
                    pass
            return False
        return False

    def _apply_receiver_fast_line(self, line: str) -> bool:
        match = re.search(
            r"seq=(?P<seq>\d+).*?boot_ms=(?P<boot_ms>\d+).*?"
            r"state=(?P<state>\S+).*?state_code=(?P<state_code>\d+).*?"
            r"status=(?P<status>0x[0-9A-Fa-f]+).*?"
            r"accel_g=\((?P<accel>[^)]*)\).*?"
            r"gyro_dps=\((?P<gyro>[^)]*)\).*?"
            r"batt_mv=(?P<batt>\d+).*?"
            r"health=(?P<health>\S+).*?"
            r"rssi=(?P<rssi>-?\d+).*?snr=(?P<snr>-?\d+(?:\.\d+)?)",
            line,
        )
        if match is None:
            return False
        accel_parts = self._parse_float_tuple(match.group("accel"), 3)
        gyro_parts = self._parse_float_tuple(match.group("gyro"), 3)
        if accel_parts is None or gyro_parts is None:
            return False

        boot_ms = int(match.group("boot_ms"))
        session_s = self._prepare_boot_clock(boot_ms)
        seq = int(match.group("seq"))
        if not self._accept_sequence(seq):
            return False
        ax, ay, az = accel_parts
        gx, gy, gz = gyro_parts
        accel = math.sqrt(ax * ax + ay * ay + az * az)
        pitch = math.degrees(math.atan2(ax, math.sqrt(ay * ay + az * az)))
        roll = math.degrees(math.atan2(ay, az)) if az != 0.0 else 0.0
        baro_dp_2pa_match = re.search(r"baro_dp_2pa=(?P<baro>-?\d+)", line)
        baro_dp_2pa = int(baro_dp_2pa_match.group("baro")) if baro_dp_2pa_match else None
        baro_alt_m = self.alt
        if baro_dp_2pa is not None:
            baro_alt_m = max(0.0, -(baro_dp_2pa * 2.0) / 12.0)
        self.alt = baro_alt_m
        self.baro_alt_agl_m = baro_alt_m

        self.packet_id += 1
        self.fast_count += 1
        self.last_fast_seq = seq
        self.last_fast_payload_hex = line
        self.last_packet_at = time.monotonic()
        self.state = match.group("state")
        self.state_code = int(match.group("state_code"))
        self.status_word = match.group("status")
        self.rssi = int(match.group("rssi"))
        self.snr = float(match.group("snr"))
        self.latest = {
            "ts": round(session_s, 2),
            "alt": round(baro_alt_m, 2),
            "baro_alt_agl_m": round(baro_alt_m, 2),
            "gps_alt_m": None if self.gps_alt_m is None else round(self.gps_alt_m, 2),
            "lat": round(self.lat, 7),
            "lng": round(self.lng, 7),
            "pitch": round(pitch, 2),
            "pitch_source": "accel_tilt_estimate",
            "roll": round(roll, 2),
            "roll_source": "accel_tilt_estimate",
            "yaw": 0.0,
            "yaw_source": "unavailable",
            "accel": round(accel, 3),
            "accel_x_g": round(ax, 3),
            "accel_y_g": round(ay, 3),
            "accel_z_g": round(az, 3),
            "gyro_x": round(gx, 2),
            "gyro_y": round(gy, 2),
            "gyro_z": round(gz, 2),
            "yaw_rate_dps": round(gz, 2),
            "baro_dp_2pa": baro_dp_2pa,
            "batt_mv": int(match.group("batt")),
            "health": match.group("health"),
        }
        self._log_latest("FAST", seq, line)
        return True

    def _apply_receiver_gps_line(self, line: str) -> bool:
        match = re.search(
            r"seq=(?P<seq>\d+).*?fix=(?P<fix>yes|no).*?"
            r"lat_deg=(?P<lat>-?\d+(?:\.\d+)?).*?"
            r"lon_deg=(?P<lng>-?\d+(?:\.\d+)?).*?"
            r"alt_m=(?P<alt>-?\d+(?:\.\d+)?).*?"
            r"speed_mps=(?P<speed>-?\d+(?:\.\d+)?).*?"
            r"course_deg=(?P<course>-?\d+(?:\.\d+)?).*?"
            r"hdop=(?P<hdop>-?\d+(?:\.\d+)?).*?"
            r"sats=(?P<sats>\d+).*?"
            r"rssi=(?P<rssi>-?\d+).*?snr=(?P<snr>-?\d+(?:\.\d+)?)",
            line,
        )
        if match is None:
            return False

        seq = int(match.group("seq"))
        if not self._accept_sequence(seq):
            return False

        self.packet_id += 1
        self.gps_count += 1
        self.last_gps_seq = seq
        self.last_gps_payload_hex = line
        self.last_packet_at = time.monotonic()
        self.gps_fix = match.group("fix") == "yes"
        self.gps_fix_flags = 0x02 if self.gps_fix else 0
        self.satellites = int(match.group("sats"))
        self.hdop = float(match.group("hdop"))
        age_match = re.search(r"age_s=(?P<age>-?\d+(?:\.\d+)?)", line)
        self.gps_age_s = float(age_match.group("age")) if age_match else None
        self.speed_mps = float(match.group("speed"))
        self.course_deg = float(match.group("course"))
        self.rssi = int(match.group("rssi"))
        self.snr = float(match.group("snr"))

        lat = float(match.group("lat"))
        lng = float(match.group("lng"))
        self.gps_alt_m = float(match.group("alt"))
        if self.gps_fix or lat != 0.0 or lng != 0.0:
            self.lat = lat
            self.lng = lng
        if self.latest is None:
            self.latest = self._default_latest()
        self.latest.update({
            "lat": round(self.lat, 7),
            "lng": round(self.lng, 7),
            "gps_alt_m": round(self.gps_alt_m, 2),
            "gps_fix": self.gps_fix,
            "gps_fix_flags": self.gps_fix_flags,
            "satellites": self.satellites,
            "hdop": self.hdop,
            "gps_age_s": self.gps_age_s,
            "speed_mps": self.speed_mps,
            "course_deg": self.course_deg,
        })
        self._log_latest("GPS", seq, line)
        return True

    @staticmethod
    def _parse_float_tuple(text: str, expected_len: int) -> tuple[float, ...] | None:
        try:
            values = tuple(float(part.strip()) for part in text.split(","))
        except ValueError:
            return None
        if len(values) != expected_len:
            return None
        return values

    def _apply_fast(self, seq: int, payload: bytes) -> bool:
        if len(payload) != p.FAST_PAYLOAD_LEN:
            return False
        status_word = int.from_bytes(payload[0:2], "little")
        state_code = (status_word >> 8) & 0x0F
        boot_ms = int.from_bytes(payload[2:6], "little")
        session_s = self._prepare_boot_clock(boot_ms)
        if not self._accept_sequence(seq):
            return False
        ax = int.from_bytes(payload[8:10], "little", signed=True) / 100.0
        ay = int.from_bytes(payload[10:12], "little", signed=True) / 100.0
        az = int.from_bytes(payload[12:14], "little", signed=True) / 100.0
        gx = int.from_bytes(payload[14:16], "little", signed=True) / 10.0
        gy = int.from_bytes(payload[16:18], "little", signed=True) / 10.0
        gz = int.from_bytes(payload[18:20], "little", signed=True) / 10.0
        accel = math.sqrt(ax * ax + ay * ay + az * az)
        pitch = math.degrees(math.atan2(ax, math.sqrt(ay * ay + az * az)))
        roll = math.degrees(math.atan2(ay, az)) if az != 0.0 else 0.0
        baro_dp_2pa = int.from_bytes(payload[6:8], "little", signed=True)
        baro_alt_m = max(0.0, -(baro_dp_2pa * 2.0) / 12.0)
        self.alt = baro_alt_m
        self.baro_alt_agl_m = baro_alt_m
        self.packet_id += 1
        self.fast_count += 1
        self.last_fast_seq = seq
        self.last_fast_payload_hex = payload.hex(" ")
        self.last_packet_at = time.monotonic()
        self.state_code = state_code
        self.state = FLIGHT_STATE_NAMES.get(state_code, f"UNKNOWN({state_code})")
        self.status_word = f"0x{status_word:04X}"
        self.latest = {
            "ts": round(session_s, 2),
            "alt": round(baro_alt_m, 2),
            "baro_alt_agl_m": round(baro_alt_m, 2),
            "gps_alt_m": None if self.gps_alt_m is None else round(self.gps_alt_m, 2),
            "lat": round(self.lat, 7),
            "lng": round(self.lng, 7),
            "pitch": round(pitch, 2),
            "pitch_source": "accel_tilt_estimate",
            "roll": round(roll, 2),
            "roll_source": "accel_tilt_estimate",
            "yaw": 0.0,
            "yaw_source": "unavailable",
            "accel": round(accel, 3),
            "accel_x_g": round(ax, 3),
            "accel_y_g": round(ay, 3),
            "accel_z_g": round(az, 3),
            "gyro_x": round(gx, 2),
            "gyro_y": round(gy, 2),
            "gyro_z": round(gz, 2),
            "yaw_rate_dps": round(gz, 2),
            "baro_dp_2pa": baro_dp_2pa,
            "batt_mv": int.from_bytes(payload[20:22], "little"),
        }
        self._log_latest("FAST", seq, payload.hex(" "))
        return True

    def _apply_gps(self, seq: int, payload: bytes) -> bool:
        if len(payload) != p.GPS_PAYLOAD_LEN:
            return False
        if not self._accept_sequence(seq):
            return False
        gps = p.decode_gps_payload(payload)
        self.gps_alt_m = gps.altitude_m
        self.speed_mps = gps.speed_mps
        self.course_deg = gps.course_deg
        self.hdop = gps.hdop
        self.satellites = gps.satellites
        self.gps_fix_flags = gps.fix_flags
        self.gps_fix = gps.has_fix
        self.gps_age_s = gps.age_s
        if self.gps_fix or gps.latitude_deg != 0.0 or gps.longitude_deg != 0.0:
            self.lat = gps.latitude_deg
            self.lng = gps.longitude_deg
        self.packet_id += 1
        self.gps_count += 1
        self.last_gps_seq = seq
        self.last_gps_payload_hex = payload.hex(" ")
        self.last_packet_at = time.monotonic()
        if self.latest is None:
            self.latest = self._default_latest()
        self.latest.update({
            "lat": round(self.lat, 7),
            "lng": round(self.lng, 7),
            "gps_alt_m": round(self.gps_alt_m, 2),
            "gps_fix": self.gps_fix,
            "gps_fix_flags": self.gps_fix_flags,
            "satellites": self.satellites,
            "hdop": self.hdop,
            "gps_age_s": self.gps_age_s,
            "speed_mps": self.speed_mps,
            "course_deg": self.course_deg,
        })
        self._log_latest("GPS", seq, payload.hex(" "))
        return True

    def _log_latest(self, packet_type: str, seq: int, payload_hex: str) -> None:
        if self.logger is None:
            return
        self.logger.log(self._row_locked("hardware"), packet_type, seq, payload_hex)


hardware_telemetry = HardwareTelemetry()


class HardwareTelemetryReader:
    def __init__(self) -> None:
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()
        self._link: PyroUplink | None = None
        self.last_error: str | None = None
        self.reconnect_count = 0

    def start(self, link: PyroUplink | None) -> None:
        if link is None or link.simulate:
            return
        self._link = link
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="telemetry-reader", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        thread = self._thread
        if thread is not None and thread.is_alive() and thread is not threading.current_thread():
            thread.join(timeout=1.0)

    def status(self) -> dict:
        return {
            "running": bool(self._thread is not None and self._thread.is_alive()),
            "last_error": self.last_error,
            "reconnect_count": self.reconnect_count,
        }

    def _run(self) -> None:
        while not self._stop.is_set():
            link = self._link
            if link is None:
                time.sleep(0.1)
                continue
            if not link.is_open():
                try:
                    link.open()
                    self.reconnect_count += 1
                    self.last_error = None
                except RuntimeError as exc:
                    self.last_error = str(exc)
                    time.sleep(0.5)
                    continue
            try:
                hardware_telemetry.next_point(link)
                link.request_bridge_status()
                self.last_error = None
            except Exception as exc:  # keep diagnostics alive after an unexpected decoder error
                self.last_error = f"{type(exc).__name__}: {exc}"
                link.close()
                time.sleep(0.5)
                continue
            time.sleep(0.005)


hardware_reader = HardwareTelemetryReader()


def mission_control_html_path() -> str:
    return os.path.join(HERE, HTML_FILENAME)


@app.route("/")
def index():
    return send_file(mission_control_html_path())


@app.after_request
def add_no_cache_headers(response):
    if request.path.startswith("/api/"):
        response.headers["Cache-Control"] = "no-store, max-age=0"
    return response


@app.route("/api/telemetry/next", methods=["GET"])
def telemetry_next():
    if uplink is not None and uplink.simulate:
        row = telemetry.next_point()
        row["source"] = "simulate"
        return jsonify(row)

    if uplink is None or not uplink.is_open():
        return jsonify(hardware_telemetry.disconnected_point())

    return jsonify(hardware_telemetry.current_point())


@app.route("/api/telemetry/reset", methods=["POST"])
def telemetry_reset():
    if uplink is not None and uplink.simulate:
        telemetry.reset()
        log_path = None if telemetry.logger is None else telemetry.logger.path
    else:
        log_path = hardware_telemetry.reset()
    return jsonify({"ok": True, "log": log_path})


@app.route("/api/telemetry/status", methods=["GET"])
def telemetry_status():
    status = hardware_telemetry.status(uplink)
    status["reader"] = hardware_reader.status()
    return jsonify(status)


@app.route("/api/pyro/deploy", methods=["POST"])
def pyro_deploy():
    data = request.get_json(silent=True) or {}
    if data.get("confirm") != "DEPLOY":
        return jsonify({
            "success": False,
            "message": 'Confirmation token required: {"confirm":"DEPLOY"}',
        }), 400

    if uplink is not None and not uplink.simulate and avionics_downlink_only() is True:
        return jsonify({
            "success": False,
            "message": (
                "Adjacent avionics source has kFlightDownlinkOnly=true; "
                "PYRO uplink is intentionally blocked until avionics uplink is explicitly enabled."
            ),
        }), 409

    if uplink is None or not uplink.is_open():
        return jsonify({
            "success": False,
            "message": "PYRO uplink is not connected.",
        }), 503

    result = uplink.force_deploy()
    status_code = 200 if result.success else 502
    return jsonify(result.to_dict()), status_code


@app.route("/api/pyro/status", methods=["GET"])
def pyro_status():
    if uplink is None:
        return jsonify({
            "connected": False,
            "simulate": False,
            "port": None,
            "telemetry_source": "hardware_disconnected",
        })
    return jsonify({
        "connected": uplink.is_open(),
        "simulate": uplink.simulate,
        "port": uplink.port,
        "serial_mode": uplink.serial_mode,
        "telemetry_source": "simulate" if uplink.simulate else "hardware",
        "fast_count": hardware_telemetry.fast_count,
        "gps_count": hardware_telemetry.gps_count,
        "control_count": hardware_telemetry.control_count,
        "packet_id": hardware_telemetry.packet_id,
        "bridge_status": uplink.diagnostics().get("bridge_status") if not uplink.simulate else {},
        "avionics_downlink_only": avionics_downlink_only(),
    })


def main() -> int:
    parser = argparse.ArgumentParser(description="NURA Mission Control server")
    parser.add_argument("--serial-port", default=None, help="Ground-station Teensy serial port (required in hardware mode)")
    parser.add_argument(
        "--serial-mode", choices=("raw", "text"), default="raw",
        help="raw transparent bridge (default) or legacy receiver text output",
    )
    parser.add_argument("--simulate", action="store_true", help="Run without hardware")
    parser.add_argument("--host", default="127.0.0.1", help="HTTP bind host")
    parser.add_argument("--http-port", type=int, default=8080, help="HTTP bind port")
    parser.add_argument("--no-browser", action="store_true", help="Do not open a browser")
    args = parser.parse_args()
    if not args.simulate and not args.serial_port:
        parser.error("--serial-port is required unless --simulate is used")
    if not 1 <= args.http_port <= 65535:
        parser.error("--http-port must be between 1 and 65535")

    global uplink
    uplink = PyroUplink(
        port=args.serial_port,
        simulate=args.simulate,
        serial_mode=args.serial_mode,
    )
    try:
        uplink.open()
    except RuntimeError as exc:
        print(f"[ERROR] Ground-station serial connection failed: {exc}", file=sys.stderr)
        return 2

    hardware_log_path = None
    if not args.simulate:
        hardware_log_path = hardware_telemetry.start_logging()
        if args.serial_mode == "raw":
            deadline = time.monotonic() + 6.0
            while time.monotonic() < deadline:
                hardware_telemetry.next_point(uplink)
                bridge_radio = uplink.diagnostics()["bridge_status"].get("radio")
                if bridge_radio in {"ready", "failed"}:
                    break
                time.sleep(0.05)
            bridge_radio = uplink.diagnostics()["bridge_status"].get("radio")
            if bridge_radio == "failed":
                print("[ERROR] Ground-station bridge reported radio initialization failure.", file=sys.stderr)
                hardware_telemetry.close_logging()
                uplink.close()
                return 3
            if bridge_radio != "ready":
                print("[WARN] Bridge startup status was not received; continuing with frame diagnostics.", file=sys.stderr)
    hardware_reader.start(uplink)

    mode = "simulate" if args.simulate else f"serial:{uplink.port} mode:{uplink.serial_mode}"
    url = f"http://{args.host}:{args.http_port}/"
    print(f"[NURA] Mission Control server started ({mode})")
    print(f"[NURA] Open in browser: {url}")
    print(f"[NURA] Serving HTML: {mission_control_html_path()}")
    if hardware_log_path is not None:
        print(f"[NURA] Hardware CSV log: {hardware_log_path}")
    if not args.simulate and not p.RADIO_IDENTITY_PROVISIONED:
        print("[WARN] Public bench radio identity is active; do not use it for flight.", file=sys.stderr)
    if not args.simulate and avionics_downlink_only() is True:
        print(
            "[WARN] Avionics source has kFlightDownlinkOnly=true; the PYRO API is blocked.",
            file=sys.stderr,
        )

    if not args.no_browser:
        threading.Timer(1.5, lambda: webbrowser.open(url)).start()

    try:
        app.run(host=args.host, port=args.http_port, threaded=True)
    finally:
        hardware_reader.stop()
        hardware_telemetry.close_logging()
        telemetry.close()
        uplink.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
