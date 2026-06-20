# -*- coding: utf-8 -*-
"""
Non-invasive integration entrypoint for the NURA GCS project.

Usage:
  1. Put this file next to the existing GCS files, or run with:
       python gcs_integrated_server.py --gcs-root C:\\path\\to\\GCS-main --simulate
  2. Existing app.py/protocol.py/uplink.py/mission_control.html are not edited.

What it adds:
  - CSV logging with battery voltage columns.
  - Binary FAST telemetry battery decode from payload[20:22].
  - Dynamic HTML injection for a battery tile without changing mission_control.html.
"""

from __future__ import annotations

import argparse
import csv
import os
import sys
import threading
import time
import webbrowser
from datetime import datetime
from types import MethodType


def _import_legacy(gcs_root: str):
    root = os.path.abspath(gcs_root)
    if root not in sys.path:
        sys.path.insert(0, root)
    import app as legacy_app  # type: ignore
    from uplink import PyroUplink  # type: ignore

    return legacy_app, PyroUplink


class BatteryTelemetryLogger:
    fieldnames = [
        "timestamp",
        "packet_number",
        "altitude",
        "accel",
        "gps_temp",
        "roll",
        "pitch",
        "yaw",
        "lat",
        "lng",
        "battery_mv",
        "battery_v",
    ]

    def __init__(self, log_dir: str) -> None:
        os.makedirs(log_dir, exist_ok=True)
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.path = os.path.join(log_dir, f"flight_log_integrated_{ts}.csv")
        self.file = open(self.path, "w", newline="", encoding="utf-8")
        self.writer = csv.DictWriter(self.file, fieldnames=self.fieldnames, extrasaction="ignore")
        self.writer.writeheader()

    def log(self, row: dict) -> None:
        batt_mv = _coerce_float(row.get("batt_mv", row.get("battery_mv")))
        out = {
            "timestamp": row.get("ts", row.get("timestamp", time.time())),
            "packet_number": row.get("packet_id", row.get("packet_number", "")),
            "altitude": row.get("alt", row.get("altitude", "")),
            "accel": row.get("accel", ""),
            "gps_temp": row.get("gps_temp", ""),
            "roll": row.get("roll", ""),
            "pitch": row.get("pitch", ""),
            "yaw": row.get("yaw", ""),
            "lat": row.get("lat", ""),
            "lng": row.get("lng", ""),
            "battery_mv": "" if batt_mv is None else int(round(batt_mv)),
            "battery_v": "" if batt_mv is None else round(batt_mv / 1000.0, 3),
        }
        self.writer.writerow(out)
        self.file.flush()

    def close(self) -> None:
        self.file.close()


def _coerce_float(value):
    try:
        if value is None or value == "":
            return None
        return float(value)
    except (TypeError, ValueError):
        return None


def _normalize_battery(row: dict) -> dict:
    batt_mv = _coerce_float(row.get("batt_mv", row.get("battery_mv")))
    if batt_mv is not None:
        row["batt_mv"] = int(round(batt_mv))
        row["battery_mv"] = int(round(batt_mv))
        row["battery_v"] = round(batt_mv / 1000.0, 3)
    elif "battery_v" not in row:
        row["battery_v"] = None
    return row


def install_battery_integration(legacy) -> None:
    """Patch legacy runtime objects without editing their source files."""

    class LoggerAdapter(BatteryTelemetryLogger):
        def __init__(self) -> None:
            super().__init__(legacy.LOG_DIR)

    legacy.TelemetryLogger = LoggerAdapter
    try:
        legacy.telemetry.logger.close()
    except Exception:
        pass
    legacy.telemetry.logger = LoggerAdapter()

    original_generate = legacy.TelemetrySimulator._generate_point

    def generate_with_battery(self, index: int) -> dict:
        row = original_generate(self, index)
        # Simple simulated battery sag: 12.6 V at pad, slowly falling in flight.
        row["batt_mv"] = max(10800, int(12600 - index * 3))
        return _normalize_battery(row)

    legacy.TelemetrySimulator._generate_point = generate_with_battery

    original_binary_fast = legacy.HardwareTelemetry._apply_fast

    def apply_fast_with_battery(self, seq: int, payload: bytes) -> None:
        original_binary_fast(self, seq, payload)
        if self.latest is not None and len(payload) >= 22:
            # FAST payload is 22 bytes; bytes 20..21 are the natural u16 slot
            # for battery millivolts in the current Python/C++ layout.
            self.latest["batt_mv"] = int.from_bytes(payload[20:22], "little", signed=False)
            _normalize_battery(self.latest)

    legacy.HardwareTelemetry._apply_fast = apply_fast_with_battery

    for name in ("waiting_point", "disconnected_point", "next_point"):
        original = getattr(legacy.hardware_telemetry, name)

        def wrapper(self, *args, __original=original, **kwargs):
            row = __original(*args, **kwargs)
            return None if row is None else _normalize_battery(row)

        setattr(legacy.hardware_telemetry, name, MethodType(wrapper, legacy.hardware_telemetry))

    original_receiver_fast = legacy.hardware_telemetry._apply_receiver_fast_line

    def receiver_fast_line_with_battery(self, line: str) -> bool:
        ok = original_receiver_fast(line)
        if ok and self.latest is not None:
            _normalize_battery(self.latest)
        return ok

    legacy.hardware_telemetry._apply_receiver_fast_line = MethodType(
        receiver_fast_line_with_battery, legacy.hardware_telemetry
    )

    def integrated_index():
        with open(legacy.mission_control_html_path(), "r", encoding="utf-8") as f:
            html = f.read()
        html = _inject_battery_tile(html)
        html = _inject_battery_script(html)
        return legacy.app.response_class(html, mimetype="text/html")

    legacy.app.view_functions["index"] = integrated_index


def _inject_battery_tile(html: str) -> str:
    if 'id="v-batt"' in html:
        return html
    marker = '<div class="metric-value"><span id="v-accel">0.0</span><span class="metric-unit">G</span></div>'
    insert = marker + """
        </div>
        <div class="metric-cell highlight">
          <div class="metric-label">Battery</div>
          <div class="metric-value"><span id="v-batt">--</span><span class="metric-unit">V</span></div>"""
    return html.replace(marker, insert, 1)


def _inject_battery_script(html: str) -> str:
    if "function updateBatteryVoltage" in html:
        return html
    script = """
<script>
function updateBatteryVoltage(row) {
  const el = document.getElementById('v-batt');
  if (!el) return;
  const mv = Number(row.batt_mv ?? row.battery_mv);
  const v = Number(row.battery_v);
  const volts = Number.isFinite(v) ? v : (Number.isFinite(mv) ? mv / 1000 : NaN);
  el.textContent = Number.isFinite(volts) ? volts.toFixed(2) : '--';
}
const __nuraUpdateUI = window.updateUI;
window.updateUI = function(row) {
  __nuraUpdateUI(row);
  updateBatteryVoltage(row);
};
</script>
</body>"""
    return html.replace("</body>", script, 1)


def main() -> int:
    parser = argparse.ArgumentParser(description="NURA non-invasive integrated Flask server")
    parser.add_argument("--gcs-root", default=os.path.dirname(os.path.abspath(__file__)))
    parser.add_argument("--serial-port", default=None)
    parser.add_argument("--simulate", action="store_true")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--http-port", type=int, default=8080)
    parser.add_argument("--no-browser", action="store_true")
    args = parser.parse_args()

    legacy, PyroUplink = _import_legacy(args.gcs_root)
    install_battery_integration(legacy)

    legacy.uplink = PyroUplink(port=args.serial_port, simulate=args.simulate)
    try:
        legacy.uplink.open()
    except RuntimeError as exc:
        print(f"[WARN] uplink connection failed: {exc}", file=sys.stderr)

    url = f"http://{args.host}:{args.http_port}/"
    print(f"[NURA] Integrated server started: {url}")
    print(f"[NURA] Original GCS root: {os.path.abspath(args.gcs_root)}")
    print(f"[NURA] CSV log: {legacy.telemetry.logger.path}")
    if not args.no_browser:
        threading.Timer(1.5, lambda: webbrowser.open(url)).start()
    legacy.app.run(host=args.host, port=args.http_port, threaded=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
