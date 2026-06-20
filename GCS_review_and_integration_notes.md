# GCS 코드 리뷰 및 통합 판단

## 1. 각 코드는 목적에 맞게 잘 만들어졌는가?

대체로 목적에 맞게 작성되어 있다.

- `protocol.py`: 프레임 구조, CRC16, SipHash, ControlPayload, FrameParser가 분리되어 있어 통신 프로토콜 모듈로 적절하다. 업링크 명령 프레임을 만드는 책임도 명확하다.
- `uplink.py`: Teensy 포트 자동 탐색, 시리얼 open/close, 재전송, ACK 판정이 한 클래스에 모여 있어 PYRO 업링크 목적에 맞다.
- `app.py`: Flask API, 시뮬레이션 텔레메트리, 하드웨어 텔레메트리 수신, UI 서빙이 이미 연결되어 있어 현재 GCS 실행용으로는 동작 가능한 구조다.
- `mission_control.html`: Carbon 스타일 UI, 지도, 그래프, 2D 로켓 시각화, PYRO 사출 버튼이 한 화면에 구현되어 있다.

다만 완성도 관점의 문제도 있다.

- `app.py`가 Flask route, 시뮬레이터, 하드웨어 수신, CSV 저장을 모두 가진다. 지금은 편하지만 팀원이 각자 수정하면 충돌이 커진다.
- CSV 컬럼은 현재 `ts, alt, lat, lng, pitch, roll, yaw, accel`이고 요구사항의 `timestamp, packet_number, altitude, accel, gps_temp, roll, pitch, yaw`와 다르다.
- 배터리 전압은 텍스트 수신 라인에서는 `batt_mv`로 파싱되지만, 바이너리 FAST 프레임 경로와 UI/CSV에는 완전 반영되어 있지 않다.
- 요구사항에 있는 Abort, Cairo/Pyro 명령은 UI/백엔드가 모두 일반화되어 있지 않고, 실제 API는 `/api/pyro/deploy` 중심이다.
- `mission_control.html`은 파일 하나가 매우 커서 UI 수정이 누적될수록 유지보수가 어려워진다.

## 2. 기존 코드를 수정하지 않고 연결하는 통합 코드를 만들 수 있는가?

가능하다. 단, 가장 현실적인 방식은 완전 독립 실행이 아니라 `새 엔트리포인트 + 어댑터/패치 계층`이다.

`outputs/gcs_integrated_server.py`는 기존 파일을 수정하지 않고 다음을 수행한다.

- 기존 `app.py`, `uplink.py`, `protocol.py`를 그대로 import한다.
- 새 CSV logger를 주입해 배터리 컬럼까지 저장한다.
- 바이너리 FAST payload의 마지막 2바이트를 배터리 mV로 해석해 JSON에 추가한다.
- 기존 `mission_control.html`을 디스크에서 바꾸지 않고, Flask 응답 시점에 배터리 UI와 JS만 동적으로 삽입한다.

## 3. 2번 요구는 적절한가?

적절하다. 특히 지금처럼 팀원이 UI, 수신, 디코딩, 업링크를 나눠 작업하는 경우 기존 파일을 직접 자주 고치는 방식은 충돌과 회귀가 생기기 쉽다.

하지만 장기적으로는 `app.py` 내부의 큰 클래스를 그대로 monkey patch하는 방식보다, 아래 구조로 리팩터링하는 것이 더 좋다.

```text
gcs/
  server.py          Flask app factory
  telemetry/
    serial_reader.py
    decoder.py
    csv_logger.py
  commands/
    uplink.py
    command_service.py
  web/
    routes.py
    static/
    templates/
```

지금 단계에서는 새 통합 코드로 연결하고, 발사 전 안정화 단계에서 모듈 경계를 정리하는 순서가 안전하다.

## 적용 방법

1. `GCS-main.zip`을 풀어 `GCS-main` 폴더를 준비한다.
2. `outputs/gcs_integrated_server.py`를 `GCS-main` 폴더 안에 복사하거나, `--gcs-root`로 경로를 지정해 실행한다.

시뮬레이션:

```bat
python outputs\gcs_integrated_server.py --gcs-root C:\path\to\GCS-main --simulate
```

하드웨어:

```bat
python outputs\gcs_integrated_server.py --gcs-root C:\path\to\GCS-main --serial-port COM3
```

포트 자동 탐색:

```bat
python outputs\gcs_integrated_server.py --gcs-root C:\path\to\GCS-main
```

## 추가로 해야 할 일

- 통신팀과 FAST payload의 배터리 위치를 확정해야 한다. 현재 통합 코드는 22바이트 FAST payload의 `payload[20:22]`를 `uint16 battery_mv`로 본다.
- Abort/Cairo/Pyro를 모두 같은 명령 서비스로 묶으려면 command id 표가 필요하다.
- 엔드투엔드 테스트는 가상 FAST/GPS 프레임을 만들어 `/api/telemetry/next`까지 흘려보내는 방식으로 추가하는 것이 좋다.
