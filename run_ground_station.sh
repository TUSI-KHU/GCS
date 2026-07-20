#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
FIRMWARE_DIR="$SCRIPT_DIR/firmware/lora_serial_bridge"
SERIAL_PORT=""
FIRMWARE_ENV="sx1276_ground"
HTTP_HOST="127.0.0.1"
HTTP_PORT="8080"
NO_BROWSER=0

usage() {
    cat <<'EOF'
Usage:
  ./run_ground_station.sh --port <Teensy serial port> [options]

Required:
  --port PORT             Ground-station Teensy, e.g. /dev/ttyACM1 or
                          /dev/serial/by-id/usb-Teensyduino_...

Options:
  --firmware-env ENV      PlatformIO environment (default: sx1276_ground)
  --host HOST             Flask bind host (default: 127.0.0.1)
  --http-port PORT        Flask port (default: 8080)
  --no-browser            Do not open the browser automatically
  -h, --help              Show this help

Environment overrides:
  GCS_PYTHON              Base Python (default: /usr/bin/python3)
  PLATFORMIO_CMD          PlatformIO executable (default: pio from PATH)
EOF
}

die() {
    printf '[ERROR] %s\n' "$*" >&2
    exit 1
}

while (($# > 0)); do
    case "$1" in
        --port)
            (($# >= 2)) || die "--port requires a value"
            SERIAL_PORT="$2"
            shift 2
            ;;
        --firmware-env)
            (($# >= 2)) || die "--firmware-env requires a value"
            FIRMWARE_ENV="$2"
            shift 2
            ;;
        --host)
            (($# >= 2)) || die "--host requires a value"
            HTTP_HOST="$2"
            shift 2
            ;;
        --http-port)
            (($# >= 2)) || die "--http-port requires a value"
            HTTP_PORT="$2"
            shift 2
            ;;
        --no-browser)
            NO_BROWSER=1
            shift
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            die "unknown argument: $1 (use --help)"
            ;;
    esac
done

[[ -n "$SERIAL_PORT" ]] || {
    usage >&2
    die "ground-station Teensy port must be specified with --port"
}
[[ "$HTTP_PORT" =~ ^[0-9]+$ ]] && ((HTTP_PORT >= 1 && HTTP_PORT <= 65535)) \
    || die "--http-port must be between 1 and 65535"
[[ "$FIRMWARE_ENV" =~ ^[A-Za-z0-9_.-]+$ ]] || die "invalid PlatformIO environment name"

TEENSY_TOOLS="$HOME/.platformio/packages/tool-teensy"
PORTS_TOOL="$TEENSY_TOOLS/teensy_ports"
REBOOT_TOOL="$TEENSY_TOOLS/teensy_reboot"
LOADER_TOOL="$TEENSY_TOOLS/teensy_loader_cli"
for tool in "$PORTS_TOOL" "$REBOOT_TOOL" "$LOADER_TOOL"; do
    [[ -x "$tool" ]] || die "required Teensy upload tool is missing: $tool"
done

command -v udevadm >/dev/null 2>&1 || die "udevadm is required to verify the selected Teensy"

TARGET_MODE="serial"
RESOLVED_PORT=""
STABLE_PORT=""
RUNTIME_PORT=""
SERIAL_ID=""
BOOTLOADER_USB_LOCATION=""
BOOTLOADER_SERIAL_HEX=""

if [[ -e "$SERIAL_PORT" ]]; then
    RESOLVED_PORT="$(readlink -f -- "$SERIAL_PORT")"
    [[ -c "$RESOLVED_PORT" ]] || die "not a character device: $SERIAL_PORT"

    UDEV_PROPERTIES="$(udevadm info --query=property --name="$RESOLVED_PORT" 2>/dev/null || true)"
    VID="$(sed -n 's/^ID_VENDOR_ID=//p' <<<"$UDEV_PROPERTIES" | head -n1)"
    PID="$(sed -n 's/^ID_MODEL_ID=//p' <<<"$UDEV_PROPERTIES" | head -n1)"
    SERIAL_ID="$(sed -n 's/^ID_SERIAL=//p' <<<"$UDEV_PROPERTIES" | head -n1)"
    if [[ "${VID,,}:${PID,,}" != "16c0:0483" ]]; then
        die "$SERIAL_PORT is not a Teensy USB serial device (VID:PID=${VID:-unknown}:${PID:-unknown})"
    fi

    for candidate in /dev/serial/by-id/*; do
        [[ -e "$candidate" ]] || continue
        if [[ "$(readlink -f -- "$candidate")" == "$RESOLVED_PORT" ]]; then
            STABLE_PORT="$candidate"
            break
        fi
    done
    RUNTIME_PORT="${STABLE_PORT:-$RESOLVED_PORT}"

    if command -v fuser >/dev/null 2>&1; then
        PORT_USERS="$(fuser "$RESOLVED_PORT" 2>/dev/null || true)"
        [[ -z "$PORT_USERS" ]] || die "$RESOLVED_PORT is already in use by PID(s):$PORT_USERS"
    fi
else
    case "$SERIAL_PORT" in
        /dev/ttyACM[0-9]*|/dev/serial/by-id/usb-Teensyduino_USB_Serial_*-if*) ;;
        *) die "serial port does not exist: $SERIAL_PORT" ;;
    esac

    PORT_LIST="$($PORTS_TOOL -L 2>/dev/null || true)"
    mapfile -t BOOTLOADER_LINES < <(
        awk '/\(Teensy 4\.1\) Bootloader$/ { print }' <<<"$PORT_LIST"
    )
    if ((${#BOOTLOADER_LINES[@]} == 0)); then
        die "serial port does not exist and no Teensy 4.1 bootloader was found: $SERIAL_PORT"
    fi
    if ((${#BOOTLOADER_LINES[@]} > 1)); then
        die "serial port is absent and multiple Teensy bootloaders are present; disconnect all but the ground Teensy"
    fi

    TARGET_MODE="bootloader"
    BOOTLOADER_USB_LOCATION="$(awk '{ print $1 }' <<<"${BOOTLOADER_LINES[0]}")"
    [[ -r "$BOOTLOADER_USB_LOCATION/serial" ]] \
        || die "cannot read the Teensy bootloader identity at $BOOTLOADER_USB_LOCATION"
    BOOTLOADER_SERIAL_HEX="$(tr -d '[:space:]' < "$BOOTLOADER_USB_LOCATION/serial")"
    [[ "$BOOTLOADER_SERIAL_HEX" =~ ^[0-9A-Fa-f]+$ ]] \
        || die "invalid Teensy bootloader identity: $BOOTLOADER_SERIAL_HEX"
    BOOTLOADER_SERIAL_DEC=$((16#$BOOTLOADER_SERIAL_HEX))
    RUNTIME_SERIAL_NUMBER=$((BOOTLOADER_SERIAL_DEC * 10))

    SERIAL_BASENAME="$(basename -- "$SERIAL_PORT")"
    if [[ "$SERIAL_BASENAME" =~ ^usb-Teensyduino_USB_Serial_([0-9]+)-if[0-9]+$ ]]; then
        REQUESTED_SERIAL_NUMBER="${BASH_REMATCH[1]}"
        [[ "$REQUESTED_SERIAL_NUMBER" == "$RUNTIME_SERIAL_NUMBER" ]] \
            || die "the only bootloader is Teensy serial $RUNTIME_SERIAL_NUMBER, not requested serial $REQUESTED_SERIAL_NUMBER"
    else
        printf '[WARN] %s is absent; recovering the only connected Teensy 4.1 bootloader\n' "$SERIAL_PORT" >&2
    fi

    SERIAL_ID="Teensyduino_USB_Serial_${RUNTIME_SERIAL_NUMBER}"
    STABLE_PORT="/dev/serial/by-id/usb-${SERIAL_ID}-if00"
    RUNTIME_PORT="$STABLE_PORT"
fi

PIO="${PLATFORMIO_CMD:-}"
if [[ -z "$PIO" ]]; then
    PIO="$(command -v pio || true)"
fi
if [[ -z "$PIO" && -x "$HOME/.local/bin/pio" ]]; then
    PIO="$HOME/.local/bin/pio"
fi
[[ -n "$PIO" && -x "$PIO" ]] || die "PlatformIO 'pio' was not found"

BASE_PYTHON="${GCS_PYTHON:-/usr/bin/python3}"
[[ -x "$BASE_PYTHON" ]] || die "Python executable was not found: $BASE_PYTHON"

if [[ "$TARGET_MODE" == "serial" ]]; then
    printf '[NURA] Ground Teensy: %s -> %s\n' "$SERIAL_PORT" "$RESOLVED_PORT"
else
    printf '[NURA] Ground Teensy: %s (already in HalfKay bootloader)\n' "$SERIAL_PORT"
    printf '[NURA] Bootloader location: %s\n' "$BOOTLOADER_USB_LOCATION"
fi
printf '[NURA] USB identity: %s\n' "${SERIAL_ID:-unknown}"
printf '[NURA] Firmware environment: %s\n' "$FIRMWARE_ENV"

VENV_DIR="$SCRIPT_DIR/.venv"
if [[ ! -x "$VENV_DIR/bin/python" ]]; then
    printf '[NURA] Creating isolated Python environment (system pyserial is reused)\n'
    "$BASE_PYTHON" -m venv --system-site-packages "$VENV_DIR"
fi
if ! "$VENV_DIR/bin/python" -c 'import flask, serial' >/dev/null 2>&1; then
    printf '[NURA] Installing web-only Python dependencies into %s\n' "$VENV_DIR"
    "$VENV_DIR/bin/python" -m pip install --disable-pip-version-check \
        -r "$SCRIPT_DIR/requirements-web.txt"
fi
"$VENV_DIR/bin/python" -c 'import importlib.metadata as m, serial; print("[NURA] Python ready: Flask %s, pyserial %s" % (m.version("flask"), serial.VERSION))'
"$VENV_DIR/bin/python" "$SCRIPT_DIR/tools/prepare_web_assets.py"
"$VENV_DIR/bin/python" - "$HTTP_HOST" "$HTTP_PORT" <<'PY'
import socket
import sys

host, port_text = sys.argv[1:]
errors = []
for family, socktype, proto, _canonname, address in socket.getaddrinfo(
    host, int(port_text), type=socket.SOCK_STREAM, flags=socket.AI_PASSIVE
):
    probe = socket.socket(family, socktype, proto)
    try:
        probe.bind(address)
    except OSError as exc:
        errors.append(str(exc))
    else:
        probe.close()
        break
    probe.close()
else:
    raise SystemExit(f"[ERROR] Flask listen address is unavailable: {host}:{port_text} ({'; '.join(errors)})")
print(f"[NURA] Flask listen address available: {host}:{port_text}")
PY

printf '[NURA] Building Teensy firmware\n'
"$PIO" run -d "$FIRMWARE_DIR" -e "$FIRMWARE_ENV"
HEX_PATH="$FIRMWARE_DIR/.pio/build/$FIRMWARE_ENV/firmware.hex"
[[ -s "$HEX_PATH" ]] || die "firmware image was not produced: $HEX_PATH"

if [[ "$TARGET_MODE" == "serial" ]]; then
    # Re-resolve the stable identity immediately before flashing in case ttyACM
    # numbering changed while the firmware was being built.
    if [[ -n "$STABLE_PORT" ]]; then
        [[ -e "$STABLE_PORT" ]] || die "selected Teensy disappeared: $STABLE_PORT"
        RESOLVED_PORT="$(readlink -f -- "$STABLE_PORT")"
    fi
    PORT_LIST="$("$PORTS_TOOL" -L 2>/dev/null)"
    if grep -qi 'bootloader' <<<"$PORT_LIST"; then
        die "another Teensy is already in bootloader mode; disconnect it before targeted upload"
    fi
    SELECTED_PORT_LINE="$(awk -v port="$RESOLVED_PORT" '$2 == port { print; exit }' <<<"$PORT_LIST")"
    USB_LOCATION="$(awk '{ print $1 }' <<<"$SELECTED_PORT_LINE")"
    [[ -n "$USB_LOCATION" ]] || die "Teensy upload tools cannot map $RESOLVED_PORT to a USB location"
    [[ "$SELECTED_PORT_LINE" == *"(Teensy 4.1)"* ]] \
        || die "selected device is not reported as a Teensy 4.1: $SELECTED_PORT_LINE"

    printf '[NURA] Upload target locked: %s (%s)\n' "$RESOLVED_PORT" "$USB_LOCATION"
    printf '[NURA] Rebooting only the selected Teensy into bootloader mode\n'
    "$REBOOT_TOOL" -s \
        "-port=$USB_LOCATION" \
        "-portlabel=$RESOLVED_PORT (Teensy 4.1) Serial" \
        "-portprotocol=Teensy"
else
    PORT_LIST="$("$PORTS_TOOL" -L 2>/dev/null || true)"
    mapfile -t BOOTLOADER_LINES < <(
        awk '/\(Teensy 4\.1\) Bootloader$/ { print }' <<<"$PORT_LIST"
    )
    ((${#BOOTLOADER_LINES[@]} == 1)) \
        || die "ground Teensy bootloader disappeared or another bootloader was connected during the build"
    CURRENT_USB_LOCATION="$(awk '{ print $1 }' <<<"${BOOTLOADER_LINES[0]}")"
    [[ "$CURRENT_USB_LOCATION" == "$BOOTLOADER_USB_LOCATION" ]] \
        || die "ground Teensy bootloader USB location changed during the build"
    CURRENT_BOOTLOADER_SERIAL_HEX="$(tr -d '[:space:]' < "$CURRENT_USB_LOCATION/serial")"
    [[ "${CURRENT_BOOTLOADER_SERIAL_HEX^^}" == "${BOOTLOADER_SERIAL_HEX^^}" ]] \
        || die "ground Teensy bootloader identity changed during the build"
    printf '[NURA] Upload target locked: HalfKay %s (%s)\n' \
        "$RUNTIME_SERIAL_NUMBER" "$BOOTLOADER_USB_LOCATION"
    printf '[NURA] Selected Teensy is already in bootloader mode; skipping reboot\n'
fi

printf '[NURA] Uploading %s\n' "$HEX_PATH"
timeout 45s "$LOADER_TOOL" --mcu=TEENSY41 -w -v "$HEX_PATH" \
    || die "Teensy firmware upload failed or timed out"

printf '[NURA] Waiting for the uploaded Teensy serial port\n'
for _ in $(seq 1 100); do
    [[ -e "$RUNTIME_PORT" ]] && break
    sleep 0.1
done
[[ -e "$RUNTIME_PORT" ]] || die "uploaded Teensy did not reappear at $RUNTIME_PORT"

APP_ARGS=(
    "$SCRIPT_DIR/app.py"
    --serial-port "$RUNTIME_PORT"
    --serial-mode raw
    --host "$HTTP_HOST"
    --http-port "$HTTP_PORT"
)
if ((NO_BROWSER)); then
    APP_ARGS+=(--no-browser)
fi

printf '[NURA] Starting Flask mission control on http://%s:%s/\n' "$HTTP_HOST" "$HTTP_PORT"
cd "$SCRIPT_DIR"
exec "$VENV_DIR/bin/python" "${APP_ARGS[@]}"
