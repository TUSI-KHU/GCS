# NURA Ground Control System

Integrated web and desktop ground-control tools using authenticated NURA V2
Lite frames.

## Radio Identity

Hardware mode reads the vehicle identity from environment variables:

```text
NURA_RADIO_VEHICLE_ID=0x........
NURA_RADIO_AUTH_KEY_HEX=<32 hexadecimal characters>
```

Both variables must match the avionics `include/nura_radio_secrets.h`. When the
variables are absent, the app uses the public bench identity and is unsafe for
flight.

## Structure

```text
app.py                 Flask web server and PYRO API
desktop.py             Legacy/helper PyQt desktop GCS
mission_control.html   Web mission-control UI
protocol.py            NURA frame/protocol helpers
uplink.py              Serial/LoRa PYRO uplink
firmware/              Teensy LoRa serial bridge
logs/                  Flight logs
```

## Ground Station Flow

This project is organized around one main web-based ground station.

```text
mission_control.html
  -> app.py
  -> uplink.py
  -> protocol.py
  -> USB Serial
  -> firmware/lora_serial_bridge on Teensy
  -> LoRa
  -> rocket
```

| Process | Role | Output |
| --- | --- | --- |
| Data logging | `app.py` generates/receives telemetry and writes CSV logs | `logs/flight_log_*.csv`, `logs/hardware_log_*.csv` |
| Packet decoding | `protocol.py` verifies NURA V2 vehicle ID, direction, MAC, CRC, and payloads | authenticated telemetry/control frames |
| Graphs | `mission_control.html` receives telemetry from `app.py` | Chart.js graphs |
| Map | `mission_control.html` plots GPS telemetry | Leaflet map |
| Frontend to backend | UI buttons call Flask APIs | `/api/telemetry/next`, `/api/pyro/deploy` |
| Backend to frontend | `app.py` provides telemetry JSON | graph/map updates |
| Uplink | `uplink.py` builds and sends PYRO commands | serial bytes to Teensy |
| Hardware bridge | Teensy firmware forwards PC serial frames over LoRa | LoRa packet bridge |

`desktop.py` is not the main ground station anymore. It is kept as a PyQt helper/legacy view. The main operator flow is the web UI served by `app.py`.

## One-command Linux launch

The hardware path requires an explicit ground-station Teensy port. The command
below builds the bridge, uploads it only to that selected Teensy, creates a
project-local Python virtual environment, and starts Flask and the browser:

```bash
./run_ground_station.sh --port /dev/ttyACM1
```

A stable device path is safer than `ttyACM` numbering when both avionics and
ground Teensys are connected:

```bash
./run_ground_station.sh \
  --port /dev/serial/by-id/usb-Teensyduino_USB_Serial_19957540-if00
```

`--port` is mandatory; neither the launcher nor hardware-mode `app.py`
auto-selects among multiple Teensys. The launcher resolves the selected serial
port to its USB physical location before calling the Teensy reboot tool, so the
avionics Teensy is not selected by PlatformIO's global auto-search. Flask is
started only after the firmware build and upload succeed and the device has
reappeared.

If the selected ground Teensy is already in HalfKay bootloader mode, its
`ttyACM` node is temporarily absent. In that state the same command still
recovers and uploads it when exactly one Teensy 4.1 bootloader is connected.
A missing `/dev/serial/by-id` path is checked against the HalfKay hardware
serial; a missing `/dev/ttyACM*` path uses the sole-bootloader fallback. The
launcher refuses to flash when multiple bootloaders are present.

The default firmware environment is `sx1276_ground`. Select another defined
bridge explicitly when the radio hardware differs:

```bash
./run_ground_station.sh --port /dev/ttyACM1 --firmware-env teensy41
./run_ground_station.sh --port /dev/ttyACM1 --firmware-env lr900f_teensy41
```

The script defaults to `/usr/bin/python3` and creates `.venv` with
`--system-site-packages`, so Ubuntu's installed `python3-serial`/pyserial is
reused. Only the web dependencies in `requirements-web.txt` are installed into
the virtual environment; no system Python package is modified.

Chart.js and Leaflet are downloaded once with fixed SHA-256 checks and cached
under `static/vendor/`; subsequent launches use the local copies. The control
UI and charts therefore load without internet after the first successful
preparation. Online map tile imagery still depends on the selected tile server.

## LR900-F Teensy Bridge

For an LR900-F ground radio, upload the UART bridge firmware instead of the
SX127x SPI bridge:

```bash
pio run -d firmware/lora_serial_bridge -e lr900f_teensy41 -t upload
```

Default wiring uses Teensy 4.1 `Serial1`:

| LR900-F JST-GH pin | Teensy 4.1 pin |
| --- | --- |
| `G` / GND | GND |
| `V` / VCC | 5V/VIN |
| `R` / RX | TX1, pin 1 |
| `T` / TX | RX1, pin 0 |

The PC-facing USB serial stays at `115200`. The LR900-F JST-GH UART side uses
the LR900-F default `57600` baud.

## SparkFun 1W SX1276 ground-radio pin map

The launcher's default `sx1276_ground` profile targets the SparkFun LoRa 1W
Breakout SPX-18572 / E19-915M30S. Its MCU wiring is locked to the adjacent
avionics repository's final `BoardPinMap::Sx1276BreakoutLoRa` SPI1 map:

| SparkFun breakout signal | Teensy 4.1 connection |
| --- | --- |
| `CIPO` / `MISO` | `MISO1`, pin 1 |
| `COPI` / `MOSI` | `MOSI1`, pin 26 |
| `SCK` | `SCK1`, pin 27 |
| `NSS` / `CS` | pin 9 |
| `RST` | pin 24 |
| `DIO0` | 32 |
| `RXEN` | pin 30 |
| `TXEN` | pin 31 |
| `DIO1` | not used by this polling bridge |
| `3.3V` | regulated 3.3 V logic supply |
| `5V` | regulated 5 V PA/module supply |
| `GND` | common ground |

Both the 5 V module/PA rail and the 3.3 V logic rail must be connected. RXEN
and TXEN are active-high RF-path controls: receive is `(1,0)`, transmit is
`(0,1)`, and idle is `(0,0)`. Always attach a suitable 915 MHz antenna before
transmitting.

Hardware references: [SparkFun SPX-18572 product page](https://www.sparkfun.com/lora-1w-breakout-915m30s.html),
[official schematic](https://cdn.sparkfun.com/assets/0/b/c/6/0/LoRa_1W_Breakout.pdf),
and [SparkFun RF-switch example](https://github.com/sparkfunX/LoRa_1W_Breakout/blob/main/Firmware/SX127x_Transmit/SX127x_Transmit.ino).

The former SPI0 harness remains available only for already-wired legacy units:

```bash
./run_ground_station.sh --port /dev/ttyACM1 \
  --firmware-env sx1276_ground_legacy_spi0
```

## Install

Web only (normally handled automatically by `run_ground_station.sh`):

```bash
/usr/bin/python3 -m venv --system-site-packages .venv
.venv/bin/python -m pip install -r requirements-web.txt
.venv/bin/python tools/prepare_web_assets.py
```

Desktop and all optional tools:

```bash
pip install -r requirements.txt
```

## Run

Web mission control with simulated PYRO uplink:

```bash
run_web_simulate.bat
```

Web mission control with an explicitly selected Teensy (Windows helper):

```bash
run_web_hardware.bat COM5
```

Desktop PyQt GCS with simulated PYRO uplink:

```bash
run_desktop_simulate.bat
```

The web server serves `mission_control.html`. Its eject button calls `POST /api/pyro/deploy`, which uses the same `PyroUplink` code as the desktop GCS. An
`EXECUTED` ACK confirms the avionics recovery execution path, not independent
electrical continuity or physical parachute deployment feedback.

When the adjacent avionics source reports `kFlightDownlinkOnly=true`, the Flask
PYRO endpoint and confirmation button are intentionally blocked. The GCS does
not silently change this avionics safety setting; uplink must be enabled and
validated explicitly in the avionics firmware before command testing.

## Field diagnostics

The web API exposes transport, frame validation, sequence-gap, receiver/bridge,
reader-thread, and CSV log state:

```bash
curl http://127.0.0.1:8080/api/telemetry/status
```

For a terminal-only check, select both the port and its output format:

```bash
.venv/bin/python tools/monitor_packets.py --port /dev/ttyACM1 --serial-mode raw
.venv/bin/python tools/monitor_packets.py --port /dev/ttyACM1 --serial-mode text
```

Raw bridge bytes are never parsed as newline-delimited receiver text. Hardware
telemetry is logged to `logs/hardware_log_*.csv`, including the full GPS fields,
packet sequence diagnostics, barometric AGL, and GPS absolute altitude.
