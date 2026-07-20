#!/usr/bin/env python3
"""Cache integrity-pinned browser libraries for offline mission-control use."""

from __future__ import annotations

import hashlib
import os
import sys
import tempfile
import urllib.error
import urllib.request
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
VENDOR_DIR = ROOT / "static" / "vendor"
ASSETS = {
    "chart.umd.js": (
        "https://cdnjs.cloudflare.com/ajax/libs/Chart.js/4.4.1/chart.umd.js",
        "74401d738dd3e03ee5dfb3b6841210fe2c4ead8a960c4011ca4ba0b78a9fd8f3",
    ),
    "leaflet.js": (
        "https://unpkg.com/leaflet@1.9.4/dist/leaflet.js",
        "db49d009c841f5ca34a888c96511ae936fd9f5533e90d8b2c4d57596f4e5641a",
    ),
    "leaflet.css": (
        "https://unpkg.com/leaflet@1.9.4/dist/leaflet.css",
        "a7837102824184820dfa198d1ebcd109ff6d0ff9a2672a074b9a1b4d147d04c6",
    ),
}


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(128 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def prepare_asset(name: str, url: str, expected_hash: str) -> None:
    destination = VENDOR_DIR / name
    if destination.is_file() and sha256(destination) == expected_hash:
        destination.chmod(0o644)
        print(f"[NURA] Web asset ready: {name}")
        return

    VENDOR_DIR.mkdir(parents=True, exist_ok=True)
    fd, temporary_name = tempfile.mkstemp(prefix=f".{name}.", dir=VENDOR_DIR)
    os.close(fd)
    temporary = Path(temporary_name)
    try:
        request = urllib.request.Request(url, headers={"User-Agent": "NURA-GCS/1.0"})
        try:
            with urllib.request.urlopen(request, timeout=30) as response, temporary.open("wb") as output:
                while chunk := response.read(128 * 1024):
                    output.write(chunk)
        except (OSError, urllib.error.URLError) as exc:
            raise RuntimeError(
                f"cannot download {name}; connect once to the internet or restore {destination}: {exc}"
            ) from exc

        actual_hash = sha256(temporary)
        if actual_hash != expected_hash:
            raise RuntimeError(
                f"integrity check failed for {name}: expected {expected_hash}, got {actual_hash}"
            )
        os.replace(temporary, destination)
        destination.chmod(0o644)
        print(f"[NURA] Cached web asset: {name}")
    finally:
        temporary.unlink(missing_ok=True)


def main() -> int:
    try:
        for name, (url, expected_hash) in ASSETS.items():
            prepare_asset(name, url, expected_hash)
    except RuntimeError as exc:
        print(f"[ERROR] {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
