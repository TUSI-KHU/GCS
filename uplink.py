# -*- coding: utf-8 -*-
"""
nura/uplink.py
==============
강제 사출(FORCE_DEPLOY) 명령을 LoRa 로 쏴 보내는 업링크 모듈.

[전체 통신 경로]

   브라우저 (PYRO 버튼)
        |  HTTP POST /api/pyro/deploy
        v
   Flask 서버 (app.py)
        |  PyroUplink.force_deploy()
        v
   PC USB 시리얼
        |  raw nura 프레임 바이트
        v
   Teensy "LoRa 시리얼 브리지" (firmware/lora_serial_bridge)
        |  LoRa 송신 (선택한 펌웨어 프로필의 주파수)
        v
   로켓 비행컴퓨터 (sender 펌웨어) ── COMMAND_FORCE_DEPLOY_RECOVERY 수신
        |  deployFired = true  → 낙하산 사출!
        v  LoRa ACK 송신 (ACCEPTED → EXECUTED)
   다시 브리지 → PC → Flask → 브라우저로 결과 표시

[재전송 로직]
  receiver/src/main.cpp 의 PendingCommand / serviceCommandSender 로직을
  그대로 Python 으로 옮겨왔어:
   - 250ms 간격으로 재전송
   - 최대 8회 시도
   - ACK 의 stage 가 EXECUTED + result OK 면 성공
   - stage 가 REJECTED 면 즉시 실패
"""

from __future__ import annotations
from collections import deque
import time
import threading

try:
    from . import protocol as p
except ImportError:
    import protocol as p

# pyserial 은 선택 의존성. 없으면 시뮬레이션 모드만 사용 가능.
try:
    import serial            # type: ignore
    _HAS_SERIAL = True
except ImportError:  # pragma: no cover
    serial = None
    _HAS_SERIAL = False

# receiver/src/main.cpp 의 상수와 동일
COMMAND_RETRY_INTERVAL_S = 0.25     # kCommandRetryIntervalMs = 250
COMMAND_MAX_ATTEMPTS = 8            # kCommandMaxAttempts = 8
SERIAL_BAUD = 115200                # kSerialBaud

class DeployResult:
    """강제 사출 명령의 최종 결과."""

    def __init__(self):
        self.success = False          # EXECUTED + OK 로 끝났는가
        self.stage = None             # 마지막으로 받은 ACK 단계
        self.result = None            # 마지막 ResultCode
        self.reason = None            # REJECT 사유
        self.flight_state = None      # 로켓이 알려준 비행 상태
        self.attempts = 0             # 실제 송신 횟수
        self.command_seq = None
        self.message = ""             # 사람이 읽을 요약
        self.acks = []                # 받은 ACK 들의 로그

    def to_dict(self):
        return {
            "success": self.success,
            "stage": p.stage_name(self.stage) if self.stage is not None else None,
            "result": p.result_name(self.result) if self.result is not None else None,
            "reason": p.reason_name(self.reason) if self.reason is not None else None,
            "flight_state": self.flight_state,
            "attempts": self.attempts,
            "command_seq": self.command_seq,
            "message": self.message,
            "acks": self.acks,
        }


class PyroUplink:
    """
    LoRa 강제 사출 업링크.

    사용 예:
        up = PyroUplink(port="/dev/ttyACM1", serial_mode="raw")
        up.open()
        result = up.force_deploy()
        print(result.to_dict())
        up.close()

    simulate=True 로 만들면 하드웨어 없이 동작을 흉내냄(개발/HTML 테스트용).
    """

    def __init__(self, port: str | None = None, baud: int = SERIAL_BAUD,
                 auth_key: bytes = p.AUTH_KEY,
                 vehicle_id: int = p.VEHICLE_ID,
                 simulate: bool = False,
                 serial_mode: str = "raw"):
        if serial_mode not in {"raw", "text"}:
            raise ValueError("serial_mode must be 'raw' or 'text'")
        self.port = port
        self.baud = baud
        self.auth_key = auth_key
        self.vehicle_id = vehicle_id
        self.simulate = simulate
        self.serial_mode = serial_mode
        self._ser = None
        self._lock = threading.Lock()       # 동시 명령 방지
        # 시퀀스 카운터 (receiver 펌웨어의 nextCommandSeq / uplinkFrameSeq)
        self._next_command_seq = 1
        self._next_frame_seq = 0
        self._parser = p.FrameParser(vehicle_id=vehicle_id,
                                     direction=p.FRAME_DIRECTION_DOWNLINK,
                                     key=auth_key)
        self._line_buffer = bytearray()
        self._diagnostic_buffer = bytearray()
        self._bridge_diagnostics: list[str] = []
        self._bridge_status: dict[str, str] = {}
        self._pending_frames = deque(maxlen=1024)
        self._serial_errors = 0
        self._last_status_request = 0.0

    # ── 연결 관리 ──────────────────────────────
    def open(self):
        """시리얼 포트를 연다. simulate 모드면 아무것도 안 함."""
        if self.simulate:
            return
        if not _HAS_SERIAL:
            raise RuntimeError(
                "선택한 Python에서 pyserial을 불러올 수 없음. Ubuntu의 python3-serial을 "
                "사용하거나 requirements-web.txt를 가상환경에 설치해줘."
            )
        if self.port is None:
            raise RuntimeError(
                "Teensy 시리얼 포트가 지정되지 않음. --serial-port 또는 port를 반드시 지정해줘."
            )
        if self._ser is not None:
            self.close()
        try:
            self._ser = serial.Serial(self.port, self.baud, timeout=0.05)
            self._ser.dtr = True
            self._parser.reset()
            self._line_buffer.clear()
            self._diagnostic_buffer.clear()
            self._pending_frames.clear()
            time.sleep(0.3)                 # Teensy 가 깨어날 시간
            if self.serial_mode == "raw":
                # 브리지는 이 ASCII 요청만 별도로 알아듣고 NURA_BRIDGE 한 줄로
                # 응답한다. 프레임 파서는 ASCII를 안전하게 건너뛴다.
                self._ser.write(b"NURA_STATUS\n")
                self._ser.flush()
        except (serial.SerialException, OSError) as exc:
            if self._ser is not None:
                self._ser.close()
            self._ser = None
            raise RuntimeError(f"시리얼 포트를 열 수 없음: {exc}") from exc

    def close(self):
        if self._ser is not None:
            self._ser.close()
            self._ser = None

    def is_open(self) -> bool:
        return self.simulate or (self._ser is not None and self._ser.is_open)

    def read_frames(self, max_bytes: int = 1024):
        """Read currently buffered frames from the owned serial port."""
        frames, _lines = self.read_frames_and_lines(max_bytes)
        return frames

    def read_frames_and_lines(self, max_bytes: int = 1024):
        """Read one transport without interpreting the same bytes in two formats.

        raw mode parses authenticated frames and only extracts NURA_BRIDGE
        diagnostic lines. text mode parses receiver firmware lines only.
        """
        if self.simulate or self._ser is None or not self._ser.is_open:
            return [], []
        with self._lock:
            pending = list(self._pending_frames)
            self._pending_frames.clear()
            try:
                chunk = self._ser.read(max_bytes)
            except (serial.SerialException, OSError):
                self._serial_errors += 1
                self.close()
                return pending, []
            if not chunk:
                return pending, []
            if self.serial_mode == "raw":
                frames = pending + self._parser.feed_bytes(chunk)
                return frames, self._extract_bridge_diagnostics(chunk)
            return [], self._extract_text_lines(chunk)

    def _extract_text_lines(self, chunk: bytes) -> list[str]:
        lines = []
        self._line_buffer.extend(chunk)
        while b"\n" in self._line_buffer:
            raw, _, rest = self._line_buffer.partition(b"\n")
            self._line_buffer = bytearray(rest)
            text = raw.rstrip(b"\r").decode("utf-8", "replace").strip()
            if text:
                lines.append(text)
        if len(self._line_buffer) > 4096:
            self._line_buffer.clear()
        return lines

    def _extract_bridge_diagnostics(self, chunk: bytes) -> list[str]:
        lines = []
        for byte in chunk:
            if byte == 0x0A:
                text = self._diagnostic_buffer.rstrip(b"\r").decode("ascii", "ignore").strip()
                self._diagnostic_buffer.clear()
                if text.startswith("NURA_BRIDGE "):
                    lines.append(text)
                    self._bridge_diagnostics.append(text)
                    self._bridge_diagnostics = self._bridge_diagnostics[-20:]
                    for token in text.split()[1:]:
                        if "=" in token:
                            key, value = token.split("=", 1)
                            self._bridge_status[key] = value
                continue
            if byte == 0x0D or 0x20 <= byte <= 0x7E:
                if len(self._diagnostic_buffer) < 512:
                    self._diagnostic_buffer.append(byte)
                else:
                    self._diagnostic_buffer.clear()
            else:
                # Binary frame data must never become a receiver text line.
                self._diagnostic_buffer.clear()
        return lines

    def diagnostics(self) -> dict:
        return {
            "serial_mode": self.serial_mode,
            "serial_errors": self._serial_errors,
            "parser": self._parser.stats(),
            "bridge_status": dict(self._bridge_status),
            "bridge_diagnostics": list(self._bridge_diagnostics),
        }

    def request_bridge_status(self, min_interval_s: float = 1.0) -> None:
        if self.simulate or self.serial_mode != "raw" or not self.is_open():
            return
        now = time.monotonic()
        if now - self._last_status_request < min_interval_s:
            return
        with self._lock:
            if self._ser is None or not self._ser.is_open:
                return
            try:
                self._ser.write(b"NURA_STATUS\n")
                self._ser.flush()
                self._last_status_request = now
            except (serial.SerialException, OSError):
                self._serial_errors += 1
                self.close()

    # ── 시퀀스 번호 ────────────────────────────
    def _make_nonce(self, command_seq: int) -> int:
        # receiver/src/main.cpp startCommand 와 동일한 방식
        millis = int(time.monotonic() * 1000) & 0xFFFFFFFF
        return (0x4E550000 ^ ((command_seq & 0xFFFF) << 8) ^ millis) & 0xFFFFFFFF

    # ── 핵심: 강제 사출 ────────────────────────
    def force_deploy(self, timeout_s: float = 3.0) -> DeployResult:
        """
        강제 사출 명령을 보내고, 로켓의 ACK 를 받을 때까지 (재전송하며) 기다림.

        반환값: DeployResult
          - success=True  : 로켓이 recovery 실행 경로의 ACK_EXECUTED + RESULT_OK 로 응답
          - success=False : 거부됐거나, 8회 재전송에도 응답 없음
        """
        with self._lock:                # 버튼 연타 등 동시 호출 방어
            return self._force_deploy_locked(timeout_s)

    def _force_deploy_locked(self, timeout_s: float) -> DeployResult:
        result = DeployResult()

        if not self.is_open():
            result.message = "시리얼 포트가 안 열려 있음. open() 먼저 호출해줘."
            return result

        if not self.simulate and self.serial_mode != "raw":
            result.message = "텍스트 receiver 펌웨어에서는 PC PYRO 업링크를 지원하지 않음. raw bridge를 업로드해줘."
            return result

        if not self.simulate and self._bridge_status.get("radio") == "failed":
            result.message = "지상국 LoRa 브리지의 radio 초기화가 실패해서 명령을 보내지 않음."
            return result

        command_seq = self._next_command_seq
        self._next_command_seq = (self._next_command_seq + 1) & 0xFFFF
        frame_seq = self._next_frame_seq
        self._next_frame_seq = (self._next_frame_seq + 1) & 0xFFFF
        nonce = self._make_nonce(command_seq)
        result.command_seq = command_seq

        # ── 시뮬레이션 모드 ──
        if self.simulate:
            time.sleep(0.4)
            result.success = True
            result.stage = p.ACK_EXECUTED
            result.result = p.RESULT_OK
            result.reason = p.REJECT_NONE
            result.flight_state = p.FLIGHT_DESCENT
            result.attempts = 1
            result.message = "[시뮬레이션] 강제 사출 명령 실행 완료 (EXECUTED/OK)"
            result.acks = ["[SIM] stage=ACCEPTED result=OK", "[SIM] stage=EXECUTED result=OK"]
            return result

        # ── 실제 LoRa 송신 ──
        frame = p.build_force_deploy_frame(command_seq,
                                           frame_seq,
                                           nonce,
                                           self.auth_key,
                                           self.vehicle_id)

        deadline = time.monotonic() + timeout_s
        last_tx = 0.0
        attempts = 0
        got_accepted = False

        while time.monotonic() < deadline:
            now = time.monotonic()

            # 재전송 타이밍 (250ms 마다, 최대 8회)
            if (now - last_tx) >= COMMAND_RETRY_INTERVAL_S and attempts < COMMAND_MAX_ATTEMPTS:
                try:
                    self._ser.write(frame)
                    self._ser.flush()
                except (serial.SerialException, OSError) as exc:
                    self._serial_errors += 1
                    self.close()
                    result.message = f"명령 송신 중 시리얼 연결이 끊김: {exc}"
                    return result
                attempts += 1
                last_tx = now
                result.attempts = attempts

            # 시리얼에서 들어온 바이트 → 프레임 파싱
            try:
                chunk = self._ser.read(256)
            except (serial.SerialException, OSError) as exc:
                self._serial_errors += 1
                self.close()
                result.message = f"ACK 대기 중 시리얼 연결이 끊김: {exc}"
                return result
            if chunk:
                parsed_frames = self._parser.feed_bytes(chunk)
                # ACK 처리 중 함께 들어온 FAST/GPS/CONTROL도 reader가
                # 나중에 소비하도록 보존한다.
                self._pending_frames.extend(parsed_frames)
                for parsed in parsed_frames:
                    ack = self._handle_frame(parsed, command_seq, nonce)
                    if ack is None:
                        continue
                    stage, res, reason, fstate = ack
                    result.stage = stage
                    result.result = res
                    result.reason = reason
                    result.flight_state = fstate
                    result.acks.append(
                        f"stage={p.stage_name(stage)} result={p.result_name(res)} "
                        f"reason={p.reason_name(reason)}"
                    )

                    if stage == p.ACK_ACCEPTED and res == p.RESULT_OK:
                        got_accepted = True

                    if stage == p.ACK_EXECUTED and res == p.RESULT_OK:
                        result.success = True
                        result.message = (
                            f"recovery 실행 ACK 확인 (EXECUTED/OK, {attempts}회 송신)"
                        )
                        return result

                    if stage == p.ACK_REJECTED:
                        result.message = (
                            f"로켓이 명령을 거부함: {p.reason_name(reason)} "
                            f"({p.result_name(res)})"
                        )
                        return result

                    if stage == p.ACK_DUPLICATE:
                        # 이미 처리된 명령. 이전에 실행됐다는 뜻.
                        result.success = True
                        result.message = "이미 실행된 명령 (DUPLICATE/ALREADY_DONE)"
                        return result

            # 8회 다 보냈고 응답 없으면 더 기다릴 필요 없음
            if attempts >= COMMAND_MAX_ATTEMPTS and (now - last_tx) > COMMAND_RETRY_INTERVAL_S:
                if not result.acks:
                    break

            time.sleep(0.01)

        # 타임아웃
        if got_accepted:
            result.message = (
                f"ACCEPTED 까지는 받았지만 EXECUTED 확인 실패 ({attempts}회 송신). "
                "재시도 권장."
            )
        else:
            result.message = (
                f"로켓 응답 없음 ({attempts}회 송신, 타임아웃). "
                "LoRa 링크/안테나/거리 확인 필요."
            )
        return result

    def _handle_frame(self, frame: p.ParsedFrame, expected_seq: int, expected_nonce: int):
        """
        받은 프레임이 우리가 기다리는 ACK 인지 확인.
        맞으면 (stage, result, reason, flight_state) 튜플, 아니면 None.
        (FAST/GPS 텔레메트리 프레임은 여기서 무시 — 그건 app.py 쪽에서 따로 다룸)
        """
        if frame.msg_type != p.MESSAGE_CONTROL:
            return None
        try:
            ctrl = p.ControlPayload.decode(frame.payload)
        except ValueError:
            return None
        if ctrl.subtype != p.CONTROL_ACK:
            return None
        if ctrl.command_id != p.COMMAND_FORCE_DEPLOY_RECOVERY:
            return None
        if ctrl.command_seq != expected_seq:
            return None      # 다른 명령에 대한 ACK
        if ctrl.nonce != expected_nonce:
            return None
        stage = ctrl.auth_or_ack[0]
        res = ctrl.auth_or_ack[1]
        reason = ctrl.auth_or_ack[2]
        fstate = ctrl.auth_or_ack[3]
        return (stage, res, reason, fstate)
