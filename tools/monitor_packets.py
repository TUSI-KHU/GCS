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
except ImportError as exc:
    raise SystemExit("pyserial is required (Ubuntu: sudo apt install python3-serial)") from exc

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import protocol as p  # noqa: E402


TYPE_NAMES = {
    p.MESSAGE_FAST_TLM: "FAST",
    p.MESSAGE_GPS_TLM: "GPS",
    p.MESSAGE_CONTROL: "CONTROL",
}


def printable_preview(data: bytes, limit: int = 64) -> str:
    preview = data[:limit]
    return " ".join(f"{byte:02X}" for byte in preview)


def type_name(msg_type: int) -> str:
    return TYPE_NAMES.get(msg_type, f"0x{msg_type:02X}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Monitor NURA GCS serial packets")
    parser.add_argument("--port", required=True, help="Ground Teensy port, e.g. /dev/ttyACM1")
    parser.add_argument(
        "--serial-mode", choices=("raw", "text"), default="raw",
        help="raw bridge frames (default) or legacy receiver text output",
    )
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--report-interval", type=float, default=1.0)
    parser.add_argument("--read-size", type=int, default=512)
    parser.add_argument(
        "--show-raw",
        action="store_true",
        help="Print a hex preview for every non-empty serial read",
    )
    args = parser.parse_args()
    if args.report_interval <= 0:
        parser.error("--report-interval must be positive")
    if args.read_size <= 0:
        parser.error("--read-size must be positive")
    return args


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
    diagnostic_buffer = bytearray()
    latest = "none"
    last_byte_at: float | None = None
    last_report = time.monotonic()
    start = last_report

    print("[wait] listening for serial bytes and NURA downlink frames")

    try:
        while True:
            port = args.port

            print(f"[open] port={port} baud={args.baud}", flush=True)
            try:
                with serial.Serial(port, args.baud, timeout=0.2) as ser:
                    ser.dtr = True
                    time.sleep(0.3)
                    parser.reset()
                    line_buffer.clear()
                    diagnostic_buffer.clear()
                    if args.serial_mode == "raw":
                        ser.write(b"NURA_STATUS\n")
                        ser.flush()

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

                            if args.serial_mode == "raw":
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

                                for byte in chunk:
                                    if byte == 0x0A:
                                        text = diagnostic_buffer.rstrip(b"\r").decode("ascii", "ignore").strip()
                                        diagnostic_buffer.clear()
                                        if text.startswith("NURA_BRIDGE "):
                                            latest = f"bridge={text}"
                                            print(f"[bridge] {text}")
                                    elif byte == 0x0D or 0x20 <= byte <= 0x7E:
                                        if len(diagnostic_buffer) < 512:
                                            diagnostic_buffer.append(byte)
                                        else:
                                            diagnostic_buffer.clear()
                                    else:
                                        diagnostic_buffer.clear()
                            else:
                                line_buffer.extend(chunk)
                                while b"\n" in line_buffer:
                                    raw_line, _, rest = line_buffer.partition(b"\n")
                                    line_buffer = bytearray(rest)
                                    text = raw_line.rstrip(b"\r").decode("utf-8", "replace").strip()
                                    if not text:
                                        continue
                                    for msg_type, prefix in (
                                        (p.MESSAGE_FAST_TLM, "rx type=FAST "),
                                        (p.MESSAGE_GPS_TLM, "rx type=GPS "),
                                        (p.MESSAGE_CONTROL, "rx type=CONTROL "),
                                    ):
                                        if text.startswith(prefix):
                                            parsed_total += 1
                                            counts[msg_type] = counts.get(msg_type, 0) + 1
                                            break
                                    latest = f"line={text}"
                                    print(f"[line] {text}")
                                if len(line_buffer) > 4096:
                                    line_buffer.clear()

                        if now - last_report >= args.report_interval:
                            if args.serial_mode == "raw":
                                ser.write(b"NURA_STATUS\n")
                                ser.flush()
                            elapsed = now - start
                            last_age = "never" if last_byte_at is None else f"{now - last_byte_at:.1f}s"
                            parser_rejects = sum(parser.stats()["reject_counts"].values())
                            print(
                                "[status] "
                                f"mode={args.serial_mode} elapsed={elapsed:.0f}s raw={raw_total} sync={sync_total} "
                                f"parsed={parsed_total} fast={counts.get(p.MESSAGE_FAST_TLM, 0)} "
                                f"gps={counts.get(p.MESSAGE_GPS_TLM, 0)} "
                                f"control={counts.get(p.MESSAGE_CONTROL, 0)} "
                                f"frame_rejects={parser_rejects} "
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
