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
| Data logging | `app.py` generates/receives telemetry and writes CSV logs | `logs/flight_log_*.csv` |
| Packet decoding | `protocol.py` verifies NURA V2 vehicle ID, direction, MAC, CRC, and payloads | authenticated telemetry/control frames |
| Graphs | `mission_control.html` receives telemetry from `app.py` | Chart.js graphs |
| Map | `mission_control.html` plots GPS telemetry | Leaflet map |
| Frontend to backend | UI buttons call Flask APIs | `/api/telemetry/next`, `/api/pyro/deploy` |
| Backend to frontend | `app.py` provides telemetry JSON | graph/map updates |
| Uplink | `uplink.py` builds and sends PYRO commands | serial bytes to Teensy |
| Hardware bridge | Teensy firmware forwards PC serial frames over LoRa | LoRa packet bridge |

`desktop.py` is not the main ground station anymore. It is kept as a PyQt helper/legacy view. The main operator flow is the web UI served by `app.py`.

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

## SX127x SPI Ground LoRa Pin Map

The default `teensy41` and `dev_sx1278` bridge firmware uses the ground station
LoRa module on Teensy 4.1 `SPI1`:

| LoRa signal | Teensy 4.1 pin |
| --- | --- |
| `MISO1` | 1 |
| `MOSI1` | 26 |
| `SCK1` | 27 |
| `RXE` | 30 |
| `TXE` | 31 |
| `DIO0` | 32 |
| `DIO1` | 8 |
| `RST` | 24 |
| `CS` | 9 |

## Install

```bash
pip install -r requirements.txt
```

## Run

Web mission control with simulated PYRO uplink:

```bash
run_web_simulate.bat
```

Web mission control with Teensy hardware auto-detect:

```bash
run_web_hardware.bat
```

Desktop PyQt GCS with simulated PYRO uplink:

```bash
run_desktop_simulate.bat
```

The web server serves `mission_control.html`. Its eject button calls `POST /api/pyro/deploy`, which uses the same `PyroUplink` code as the desktop GCS. An
`EXECUTED` ACK confirms the avionics recovery execution path, not independent
electrical continuity or physical parachute deployment feedback.
