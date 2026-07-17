#!/usr/bin/env python3
"""Continuously monitor NURA GCS serial traffic.

Shows raw byte counts, frame sync counts, parsed packet counts, and the latest
decoded packet status. This is intended for field checks where the ground
station Teensy is connected over USB serial.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

try:
    import serial
    from serial.tools import list_ports
except ImportError as exc:
    raise SystemExit("pyserial is required: python3 -m pip install pyserial") from exc

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import protocol as p  # noqa: E402


TYPE_NAMES = {
    p.MESSAGE_FAST_TLM: "FAST",
    p.MESSAGE_GPS_TLM: "GPS",
    p.MESSAGE_CONTROL: "CONTROL",
}


def find_teensy_port() -> str | None:
    for port in list_ports.comports():
        haystack = " ".join(
            str(value or "")
            for value in (port.device, port.description, port.manufacturer, port.hwid)
        ).lower()
        if "teensy" in haystack or "16c0:0483" in haystack:
            return port.device
    return None


def printable_preview(data: bytes, limit: int = 64) -> str:
    preview = data[:limit]
    return " ".join(f"{byte:02X}" for byte in preview)


def type_name(msg_type: int) -> str:
    return TYPE_NAMES.get(msg_type, f"0x{msg_type:02X}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Monitor NURA GCS serial packets")
    parser.add_argument("--port", help="Serial port, e.g. /dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--report-interval", type=float, default=1.0)
    parser.add_argument("--read-size", type=int, default=512)
    parser.add_argument(
        "--show-raw",
        action="store_true",
        help="Print a hex preview for every non-empty serial read",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    parser = p.FrameParser(
        vehicle_id=p.VEHICLE_ID,
        direction=p.FRAME_DIRECTION_DOWNLINK,
        key=p.AUTH_KEY,
    )

    raw_total = 0
    sync_total = 0
    parsed_total = 0
    counts = {p.MESSAGE_FAST_TLM: 0, p.MESSAGE_GPS_TLM: 0, p.MESSAGE_CONTROL: 0}
    line_buffer = bytearray()
    latest = "none"
    last_byte_at: float | None = None
    last_report = time.monotonic()
    start = last_report

    print("[wait] listening for serial bytes and NURA downlink frames")

    try:
        while True:
            port = args.port or find_teensy_port()
            if port is None:
                print("[reconnect] no Teensy serial port found; retrying in 1s", flush=True)
                time.sleep(1.0)
                continue

            print(f"[open] port={port} baud={args.baud}", flush=True)
            try:
                with serial.Serial(port, args.baud, timeout=0.2) as ser:
                    ser.dtr = True
                    time.sleep(0.3)
                    parser.reset()
                    line_buffer.clear()

                    while True:
                        now = time.monotonic()
                        try:
                            chunk = ser.read(args.read_size)
                        except serial.SerialException as exc:
                            latest = f"serial_error={exc}"
                            print(f"[reconnect] {exc}; retrying in 1s", flush=True)
                            break

                        if chunk:
                            now = time.monotonic()
                            last_byte_at = now
                            raw_total += len(chunk)
                            sync_total += chunk.count(bytes((p.SYNC0, p.SYNC1)))

                            if args.show_raw:
                                print(f"[raw] +{len(chunk)} bytes hex={printable_preview(chunk)}")

                            frames = parser.feed_bytes(chunk)
                            for frame in frames:
                                parsed_total += 1
                                counts[frame.msg_type] = counts.get(frame.msg_type, 0) + 1
                                latest = (
                                    f"type={type_name(frame.msg_type)} seq={frame.seq} "
                                    f"payload_len={len(frame.payload)} "
                                    f"payload={printable_preview(frame.payload, 32)}"
                                )
                                print(f"[packet] {latest}")

                            line_buffer.extend(chunk)
                            while b"\n" in line_buffer:
                                raw_line, _, rest = line_buffer.partition(b"\n")
                                line_buffer = bytearray(rest)
                                text = raw_line.rstrip(b"\r").decode("utf-8", "replace").strip()
                                if text:
                                    latest = f"line={text}"
                                    print(f"[line] {text}")

                            if len(line_buffer) > 2048:
                                line_buffer.clear()

                        if now - last_report >= args.report_interval:
                            elapsed = now - start
                            last_age = "never" if last_byte_at is None else f"{now - last_byte_at:.1f}s"
                            print(
                                "[status] "
                                f"elapsed={elapsed:.0f}s raw={raw_total} sync={sync_total} "
                                f"parsed={parsed_total} fast={counts.get(p.MESSAGE_FAST_TLM, 0)} "
                                f"gps={counts.get(p.MESSAGE_GPS_TLM, 0)} "
                                f"control={counts.get(p.MESSAGE_CONTROL, 0)} "
                                f"last_byte_age={last_age} latest={latest}",
                                flush=True,
                            )
                            last_report = now
            except serial.SerialException as exc:
                latest = f"open_error={exc}"
                print(f"[reconnect] cannot open {port}: {exc}; retrying in 1s", flush=True)

            time.sleep(1.0)
    except KeyboardInterrupt:
        print("\n[stop] monitor interrupted")
        return 130


if __name__ == "__main__":
    raise SystemExit(main())
