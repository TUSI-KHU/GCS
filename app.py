# -*- coding: utf-8 -*-
from __future__ import annotations

import argparse
import csv
import math
import os
import sys
import threading
import time
import webbrowser
from datetime import datetime

from flask import Flask, jsonify, request, send_file

from uplink import PyroUplink


HERE = os.path.dirname(os.path.abspath(__file__))
LOG_DIR = os.path.join(HERE, "logs")
HTML_FILENAME = "mission_control.html"

app = Flask(__name__)
uplink: PyroUplink | None = None


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
        self.file.close()


class TelemetrySimulator:
    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.logger = TelemetryLogger()
        self.index = 0
        self.launch_site = (34.4258, 127.5211)
        self.apogee_pos: tuple[float, float] | None = None

    def reset(self) -> None:
        with self.lock:
            self.logger.close()
            self.logger = TelemetryLogger()
            self.index = 0
            self.apogee_pos = None

    def next_point(self) -> dict:
        with self.lock:
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


def mission_control_html_path() -> str:
    return os.path.join(HERE, HTML_FILENAME)


@app.route("/")
def index():
    return send_file(mission_control_html_path())


@app.route("/api/telemetry/next", methods=["GET"])
def telemetry_next():
    return jsonify(telemetry.next_point())


@app.route("/api/telemetry/reset", methods=["POST"])
def telemetry_reset():
    telemetry.reset()
    return jsonify({"ok": True, "log": telemetry.logger.path})


@app.route("/api/pyro/deploy", methods=["POST"])
def pyro_deploy():
    data = request.get_json(silent=True) or {}
    if data.get("confirm") != "DEPLOY":
        return jsonify({
            "success": False,
            "message": 'Confirmation token required: {"confirm":"DEPLOY"}',
        }), 400

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
        return jsonify({"connected": False, "simulate": False, "port": None})
    return jsonify({
        "connected": uplink.is_open(),
        "simulate": uplink.simulate,
        "port": uplink.port,
    })


def main() -> int:
    parser = argparse.ArgumentParser(description="NURA Mission Control server")
    parser.add_argument("--serial-port", default=None, help="Teensy serial port")
    parser.add_argument("--simulate", action="store_true", help="Run without hardware")
    parser.add_argument("--host", default="127.0.0.1", help="HTTP bind host")
    parser.add_argument("--http-port", type=int, default=8080, help="HTTP bind port")
    parser.add_argument("--no-browser", action="store_true", help="Do not open a browser")
    args = parser.parse_args()

    global uplink
    uplink = PyroUplink(port=args.serial_port, simulate=args.simulate)
    try:
        uplink.open()
    except RuntimeError as exc:
        print(f"[WARN] PYRO uplink connection failed: {exc}", file=sys.stderr)
        print("       Use --simulate to test without hardware.", file=sys.stderr)

    mode = "simulate" if args.simulate else f"serial:{uplink.port}"
    url = f"http://{args.host}:{args.http_port}/"
    print(f"[NURA] Mission Control server started ({mode})")
    print(f"[NURA] Open in browser: {url}")
    print(f"[NURA] Serving HTML: {mission_control_html_path()}")

    if not args.no_browser:
        threading.Timer(1.5, lambda: webbrowser.open(url)).start()

    app.run(host=args.host, port=args.http_port, threaded=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
