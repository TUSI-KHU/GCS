# Avionics LoRa and GCS Integration Status - 2026-06-23

작성 시각: 2026-06-23 17:15 KST

## 기준 브랜치

| Repository | GitHub main 기준 | 로컬 상태 |
| --- | --- | --- |
| `TUSI-KHU/2026-nura-avionics` | `origin/main` `77ce2fd` | 로컬 수정 있음 |
| `TUSI-KHU/GCS` | `origin/main` `02e6f4a` | 이 문서 추가 전 깨끗함 |

이 문서는 GCS repo에 기록한다. Avionics repo의 코드 수정은 현장 디버깅을 위한 로컬 변경이며, 별도 요청 전까지 avionics GitHub에는 업로드하지 않았다.

## 현재 가능한 것

- GCS 웹 UI 실행 가능: `http://127.0.0.1:8080/`
- GCS 서버가 지상국 Teensy serial을 열 수 있음.
- 지상국 LoRa SX1276 SPI 인식 가능:
  - `radio=ready`
  - `m0=0x12`
  - `spi_mode=0`
- 지상국 Teensy와 GCS serial 경로 연결 가능.
- Avionics standalone SX1262 bench sender에서는 LoRa 송신 가능했음:
  - Avionics 로그: `SX1262_INIT_OK`, `TX_OK seq=...`
  - GCS API: `source=hardware`, `fast_count` 증가
  - 예시 수신 품질: RSSI 약 `-99` to `-110 dBm`, SNR 약 `5` to `9 dB`
- 실제 FSM/sensor/telemetry 빌드도 한때 GCS까지 실제 센서 값을 전달했음:
  - 예시: `state=LAUNCH`, `roll=83.09`, `accel=0.997`, `batt_mv=12271`

## 현재 막힌 것

실제 FSM 빌드에서 LoRa 송신이 지속되지 않는다.

확인된 실패 로그:

```text
SX1262_TX_STATE=-2 RX_STATE=0
[0] WARN telemetry: lora tx failed
```

RadioLib 기준 `-2`는 다음 오류다.

```text
RADIOLIB_ERR_CHIP_NOT_FOUND
```

즉 주파수, sync word, GCS UI, packet decoder 문제가 아니라, 실제 앱 실행 중 Avionics SX1262가 SPI에서 응답하지 않는 상태로 떨어진다.

지상국 직접 확인 결과:

```text
status radio=ready spi_mode=0 m0=0x12 ... phy_rx=526 decode_fail=0 rssi_now_dbm=-157
```

`phy_rx`가 더 이상 증가하지 않아 GCS 문제가 아니라 무선 송신이 새로 들어오지 않는 상태로 판단했다.

## GitHub main 대비 Avionics 로컬 수정 내역

`TUSI-KHU/2026-nura-avionics` 기준 `origin/main` 대비 변경:

```text
platformio.ini                    |  4 ++++
src/app/app_config.cpp            |  4 ++++
src/app/flight_controller_app.cpp |  2 ++
src/hal/sx1262_lora_hal.cpp       | 22 +++++++++++++++++++++-
src/hal/sx1262_lora_hal.h         |  2 ++
5 files changed, 33 insertions(+), 1 deletion(-)
```

파일별 목적:

| File | 변경 목적 |
| --- | --- |
| `platformio.ini` | `debug_radio_bench`에 벤치 전용 LoRa 복구/저부하 플래그 추가 |
| `src/app/app_config.cpp` | 벤치 빌드에서 FAST telemetry 주기를 1000 ms로 낮출 수 있게 함 |
| `src/app/flight_controller_app.cpp` | 벤치 빌드에서 flight log task를 제외할 수 있게 함 |
| `src/hal/sx1262_lora_hal.cpp` | TX 실패 시 벤치 전용 SX1262 재초기화/재송신 시도 추가, minimal init 옵션 추가 |
| `src/hal/sx1262_lora_hal.h` | 재초기화를 위해 마지막 LoRa config 저장 |

추가된 벤치 전용 build flags:

```ini
-D NURA_BENCH_RADIO_REINIT_ON_TX_FAIL=1
-D NURA_BENCH_TELEMETRY_FAST_PERIOD_MS=1000
-D NURA_BENCH_RADIO_USE_MINIMAL_INIT=1
-D NURA_BENCH_DISABLE_FLIGHT_LOG_TASK=1
```

이 변경은 `debug_radio_bench` 환경에만 적용되도록 작성했다. `main` 비행 빌드의 state-machine, pyro, deployment threshold 자체를 바꾸지는 않았다.

## 시도한 것과 결과

| 시도 | 결과 |
| --- | --- |
| 지상국 SX1276 전원/핀맵/레지스터 확인 | 성공. `m0=0x12`, `radio=ready` |
| 지상국 receiver firmware 재업로드 | 성공 |
| GCS 웹 UI 실행 및 serial 연결 | 성공 |
| Avionics standalone LoRa bench sender 업로드 | 성공 |
| Bench sender -> Ground -> GCS 통신 | 성공. `fast_count` 증가 |
| 실제 FSM `debug_radio_bench` 업로드 | 빌드/업로드 성공 |
| 실제 FSM telemetry 수신 | 일부 성공 후 중단. 실제 센서값이 GCS에 들어온 뒤 stale |
| SX1262 TX 실패 시 재초기화 추가 | 실패. `RADIOLIB_ERR_CHIP_NOT_FOUND` 지속 |
| FAST 주기 200 ms -> 1000 ms 감소 | 실패. 새 packet count 증가 없음 |
| bench sender와 동일하게 minimal RadioLib init 사용 | 실패 |
| 벤치 빌드에서 flight log task 제외 | 실패. LoRa TX가 계속 `-2` |

## 판단

현재 가장 가능성이 높은 원인은 Avionics SX1262 하드웨어 쪽이다.

근거:

- standalone bench sender는 동작했으나 실제 FSM 구동 중에는 SX1262가 SPI에서 사라진다.
- 실패 코드가 packet/protocol 오류가 아니라 `RADIOLIB_ERR_CHIP_NOT_FOUND`이다.
- Avionics PCB에는 MCU가 제어하는 SX1262 `NRESET`이 현재 코드상 없다 (`RADIOLIB_NC`).
- NRESET을 소프트웨어로 당길 수 없어, SX1262가 한 번 stuck되면 펌웨어만으로 복구하기 어렵다.
- 실제 FSM은 센서, logging, scheduler 부하가 같이 걸리므로 전원/리셋/보드 상태가 더 민감하게 드러나는 것으로 보인다.

## 다음 액션 제안

1. Avionics SX1262 전원 레일을 실제 FSM 부팅/송신 순간에 오실로스코프 또는 멀티미터로 확인한다.
2. SX1262 `NRESET` pad 또는 `LORA_RST JP2`가 실제 reset net인지 회로도/연속성으로 확인한다.
3. 가능하면 SX1262 NRESET을 Teensy GPIO에 연결한 리비전 또는 점퍼 테스트를 준비한다.
4. 현재 보드에서는 전원 완전 차단 후 부팅 순서를 바꿔가며 반복 테스트한다.
5. GCS/UI 검증만 필요할 때는 standalone `avionics_radio_bench` sender를 사용한다.
6. 실제 FSM telemetry 검증은 SX1262 reset/power 문제가 해결된 뒤 다시 진행한다.

## 현재 실행 상태

마지막 확인 시점:

- GCS server는 `http://127.0.0.1:8080/`에서 실행됨.
- GCS serial port는 지상국 `/dev/ttyACM1`.
- 지상국 receiver는 `radio=ready`.
- 실제 FSM Avionics는 LoRa TX 실패 상태.

