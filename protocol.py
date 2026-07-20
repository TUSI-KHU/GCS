# -*- coding: utf-8 -*-
"""
nura/protocol.py
================
NURA V2 Lite 인증 통신 프로토콜의 Python 포팅.

원본: protocol/include/nura_protocol_v1_lite.h (C++)
이 파일은 그 C++ 헤더와 "바이트 단위로 동일한" 프레임을 만들어내도록 작성됐어.
로켓(sender) 펌웨어가 SipHash 인증 태그를 검사하기 때문에, 1비트라도 다르면
명령이 통째로 거부(REJECT_AUTH_TAG_MISMATCH)돼. 그래서 정확도가 생명임.

프레임 구조 (총 19 + payloadLen 바이트):
  [0]      Sync0   = 0xAA
  [1]      Sync1   = 0x55
  [2]      VerType = (version<<4) | type   (version=2)
  [3..6]   Vehicle ID (little-endian u32)
  [7..8]   Seq (little-endian u16)
  [9..N]   Payload
  [N+1..8] SipHash-2-4 frame authentication tag
  [N+9..10] CRC16-CCITT-FALSE
"""

from __future__ import annotations
from dataclasses import dataclass, field
import os

# ─────────────────────────────────────────────
#  상수 (C++ 헤더의 static constexpr 값들)
# ─────────────────────────────────────────────
SYNC0 = 0xAA
SYNC1 = 0x55
VERSION = 2

FAST_PAYLOAD_LEN = 22
GPS_PAYLOAD_LEN = 18
CONTROL_PAYLOAD_LEN = 24
FRAME_HEADER_LEN = 9
FRAME_AUTH_TAG_LEN = 8
FRAME_CRC_LEN = 2
FRAME_OVERHEAD = FRAME_HEADER_LEN + FRAME_AUTH_TAG_LEN + FRAME_CRC_LEN
MAX_PAYLOAD_LEN = CONTROL_PAYLOAD_LEN
MAX_FRAME_LEN = FRAME_OVERHEAD + MAX_PAYLOAD_LEN

FRAME_DIRECTION_UPLINK = 0x55
FRAME_DIRECTION_DOWNLINK = 0x44

# MessageType
MESSAGE_FAST_TLM = 0x1
MESSAGE_GPS_TLM = 0x2
MESSAGE_CONTROL = 0x3

# ControlSubtype
CONTROL_CMD = 0x01
CONTROL_ACK = 0x81

# CommandId  ── 강제 사출은 COMMAND_FORCE_DEPLOY_RECOVERY
COMMAND_FORCE_DEPLOY_RECOVERY = 0x01
COMMAND_ABORT_PROPULSION_DEPRECATED = 0x02   # 더 이상 지원 안 함(로켓이 REJECT 함)
COMMAND_SET_TELEMETRY_PROFILE = 0x03

# AckStage  ── 로켓이 보내주는 ACK의 단계
ACK_RECEIVED = 0
ACK_ACCEPTED = 1
ACK_EXECUTED = 2     # ★ 여기까지 와야 "실제로 사출됨"
ACK_REJECTED = 3
ACK_DUPLICATE = 4

# ResultCode
RESULT_OK = 0
RESULT_AUTH_FAILED = 1
RESULT_EXPIRED = 2
RESULT_BAD_FORMAT = 3
RESULT_BAD_STATE = 4
RESULT_NOT_ARMED = 5
RESULT_ALREADY_DONE = 6
RESULT_NOT_SUPPORTED = 7
RESULT_ACTUATOR_FAULT = 8
RESULT_INTERNAL_ERROR = 9

# RejectReason
REJECT_NONE = 0
REJECT_COMMAND_EXPIRED = 1
REJECT_UNKNOWN_COMMAND = 2
REJECT_AUTH_TAG_MISMATCH = 3
REJECT_DUPLICATE_OLDER_COMMAND = 4
REJECT_DEPLOYMENT_INHIBITED = 5
REJECT_CONTINUITY_BAD = 6
REJECT_STATE_REJECTED = 7
REJECT_DEPRECATED_COMMAND = 8
REJECT_PROFILE_REJECTED = 9

# FlightStateCode
FLIGHT_INIT = 0
FLIGHT_SAFE = 1
FLIGHT_ARMED = 2
FLIGHT_LAUNCH = 3
FLIGHT_COAST = 4
FLIGHT_APOGEE = 5
FLIGHT_DROGUE = 6
FLIGHT_DEPLOY = 7
FLIGHT_GROUND = 8
FLIGHT_FAULT = 9
FLIGHT_BOOT = FLIGHT_INIT
FLIGHT_IDLE = FLIGHT_SAFE
FLIGHT_DESCENT = FLIGHT_DROGUE

# 인증 키 (sender/receiver main.cpp 의 kAuthKey 와 동일, ASCII "NURA-V1LITE-TEST")
# !! 실제 발사 전에는 반드시 비밀 키로 교체할 것 !!
AUTH_KEY = bytes([
    0x4e, 0x55, 0x52, 0x41, 0x2d, 0x56, 0x31, 0x4c,
    0x49, 0x54, 0x45, 0x2d, 0x54, 0x45, 0x53, 0x54,
])
VEHICLE_ID = 0x4E555241

_vehicle_id_text = os.getenv("NURA_RADIO_VEHICLE_ID")
_auth_key_hex = os.getenv("NURA_RADIO_AUTH_KEY_HEX")
if (_vehicle_id_text is None) != (_auth_key_hex is None):
    raise RuntimeError("NURA_RADIO_VEHICLE_ID와 NURA_RADIO_AUTH_KEY_HEX를 함께 설정해야 함")
if _vehicle_id_text is not None:
    VEHICLE_ID = int(_vehicle_id_text, 0)
    AUTH_KEY = bytes.fromhex(_auth_key_hex)
    if not 0 <= VEHICLE_ID <= 0xFFFFFFFF or len(AUTH_KEY) != 16:
        raise RuntimeError("vehicle ID는 u32, radio auth key는 16바이트여야 함")
RADIO_IDENTITY_PROVISIONED = _vehicle_id_text is not None

# 사람이 읽기 좋은 이름 매핑 (로그 출력용)
_STAGE_NAMES = {0: "RECEIVED", 1: "ACCEPTED", 2: "EXECUTED", 3: "REJECTED", 4: "DUPLICATE"}
_RESULT_NAMES = {
    0: "OK", 1: "AUTH_FAILED", 2: "EXPIRED", 3: "BAD_FORMAT", 4: "BAD_STATE",
    5: "NOT_ARMED", 6: "ALREADY_DONE", 7: "NOT_SUPPORTED", 8: "ACTUATOR_FAULT",
    9: "INTERNAL_ERROR",
}
_REASON_NAMES = {
    0: "NONE", 1: "COMMAND_EXPIRED", 2: "UNKNOWN_COMMAND", 3: "AUTH_TAG_MISMATCH",
    4: "DUPLICATE_OLDER_COMMAND", 5: "DEPLOYMENT_INHIBITED", 6: "CONTINUITY_BAD",
    7: "STATE_REJECTED", 8: "DEPRECATED_COMMAND", 9: "PROFILE_REJECTED",
}


def stage_name(v: int) -> str:
    return _STAGE_NAMES.get(v, f"UNKNOWN({v})")


def result_name(v: int) -> str:
    return _RESULT_NAMES.get(v, f"UNKNOWN({v})")


def reason_name(v: int) -> str:
    return _REASON_NAMES.get(v, f"UNKNOWN({v})")


# ─────────────────────────────────────────────
#  CRC16-CCITT-FALSE  (poly 0x1021, init 0xFFFF)
#  C++ 헤더의 crc16CcittFalse 와 동일
# ─────────────────────────────────────────────
def crc16_ccitt_false(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= (byte << 8)
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc & 0xFFFF


# ─────────────────────────────────────────────
#  SipHash-2-4  (C++ 헤더의 sipHash24 와 동일)
#  ControlPayload 인증 태그(8바이트) 생성에 사용
# ─────────────────────────────────────────────
_MASK64 = (1 << 64) - 1


def _rotl64(value: int, shift: int) -> int:
    value &= _MASK64
    return ((value << shift) | (value >> (64 - shift))) & _MASK64


def _read_u64_le(data: bytes, offset: int = 0) -> int:
    return int.from_bytes(data[offset:offset + 8], "little")


def _sip_round(v0: int, v1: int, v2: int, v3: int):
    v0 = (v0 + v1) & _MASK64
    v1 = _rotl64(v1, 13)
    v1 ^= v0
    v0 = _rotl64(v0, 32)
    v2 = (v2 + v3) & _MASK64
    v3 = _rotl64(v3, 16)
    v3 ^= v2
    v0 = (v0 + v3) & _MASK64
    v3 = _rotl64(v3, 21)
    v3 ^= v0
    v2 = (v2 + v1) & _MASK64
    v1 = _rotl64(v1, 17)
    v1 ^= v2
    v2 = _rotl64(v2, 32)
    return v0, v1, v2, v3


def siphash24(data: bytes, key: bytes) -> int:
    """SipHash-2-4. data 임의 길이, key 16바이트. 64비트 정수 반환."""
    if len(key) != 16:
        raise ValueError("SipHash 키는 16바이트여야 함")

    k0 = _read_u64_le(key, 0)
    k1 = _read_u64_le(key, 8)
    v0 = 0x736F6D6570736575 ^ k0
    v1 = 0x646F72616E646F6D ^ k1
    v2 = 0x6C7967656E657261 ^ k0
    v3 = 0x7465646279746573 ^ k1

    length = len(data)
    offset = 0
    while offset + 8 <= length:
        m = _read_u64_le(data, offset)
        v3 ^= m
        v0, v1, v2, v3 = _sip_round(v0, v1, v2, v3)
        v0, v1, v2, v3 = _sip_round(v0, v1, v2, v3)
        v0 ^= m
        offset += 8

    b = (length & 0xFF) << 56
    remaining = length - offset
    for i in range(remaining):
        b |= data[offset + i] << (8 * i)
    b &= _MASK64

    v3 ^= b
    v0, v1, v2, v3 = _sip_round(v0, v1, v2, v3)
    v0, v1, v2, v3 = _sip_round(v0, v1, v2, v3)
    v0 ^= b
    v2 ^= 0xFF
    for _ in range(4):
        v0, v1, v2, v3 = _sip_round(v0, v1, v2, v3)

    return (v0 ^ v1 ^ v2 ^ v3) & _MASK64


# ─────────────────────────────────────────────
#  헬퍼: VerType
# ─────────────────────────────────────────────
def make_ver_type(msg_type: int) -> int:
    return ((VERSION << 4) | (msg_type & 0x0F)) & 0xFF


def frame_version(ver_type: int) -> int:
    return (ver_type >> 4) & 0x0F


def frame_type(ver_type: int) -> int:
    return ver_type & 0x0F


def payload_length_for_type(msg_type: int) -> int:
    return {
        MESSAGE_FAST_TLM: FAST_PAYLOAD_LEN,
        MESSAGE_GPS_TLM: GPS_PAYLOAD_LEN,
        MESSAGE_CONTROL: CONTROL_PAYLOAD_LEN,
    }.get(msg_type, 0)


# ─────────────────────────────────────────────
#  ControlPayload  (24바이트)
# ─────────────────────────────────────────────
@dataclass
class ControlPayload:
    subtype: int = 0
    command_id: int = 0
    command_seq: int = 0
    nonce: int = 0
    valid_until_ms: int = 0
    param0: int = 0
    param1: int = 0
    auth_or_ack: bytes = field(default_factory=lambda: bytes(8))

    def encode(self) -> bytes:
        """C++ encodeControlPayload 와 동일한 24바이트 직렬화."""
        auth = (self.auth_or_ack + bytes(8))[:8]
        out = bytearray(CONTROL_PAYLOAD_LEN)
        out[0] = self.subtype & 0xFF
        out[1] = self.command_id & 0xFF
        out[2:4] = (self.command_seq & 0xFFFF).to_bytes(2, "little")
        out[4:8] = (self.nonce & 0xFFFFFFFF).to_bytes(4, "little")
        out[8:12] = (self.valid_until_ms & 0xFFFFFFFF).to_bytes(4, "little")
        out[12:14] = (self.param0 & 0xFFFF).to_bytes(2, "little", signed=False)
        out[14:16] = (self.param1 & 0xFFFF).to_bytes(2, "little", signed=False)
        out[16:24] = auth
        return bytes(out)

    @classmethod
    def decode(cls, data: bytes) -> "ControlPayload":
        """C++ decodeControlPayload 와 동일."""
        if len(data) != CONTROL_PAYLOAD_LEN:
            raise ValueError(f"ControlPayload 길이가 {CONTROL_PAYLOAD_LEN}이 아님: {len(data)}")
        return cls(
            subtype=data[0],
            command_id=data[1],
            command_seq=int.from_bytes(data[2:4], "little"),
            nonce=int.from_bytes(data[4:8], "little"),
            valid_until_ms=int.from_bytes(data[8:12], "little"),
            param0=int.from_bytes(data[12:14], "little", signed=True),
            param1=int.from_bytes(data[14:16], "little", signed=True),
            auth_or_ack=bytes(data[16:24]),
        )


def make_control_auth_tag(control: ControlPayload, frame_seq: int, key: bytes = AUTH_KEY) -> bytes:
    """
    C++ makeControlAuthTag 와 동일.
    19바이트 입력에 SipHash-2-4 를 적용하고 결과를 little-endian 8바이트로 반환.
    """
    inp = bytearray(19)
    inp[0] = make_ver_type(MESSAGE_CONTROL)
    inp[1:3] = (frame_seq & 0xFFFF).to_bytes(2, "little")
    inp[3] = control.subtype & 0xFF
    inp[4] = control.command_id & 0xFF
    inp[5:7] = (control.command_seq & 0xFFFF).to_bytes(2, "little")
    inp[7:11] = (control.nonce & 0xFFFFFFFF).to_bytes(4, "little")
    inp[11:15] = (control.valid_until_ms & 0xFFFFFFFF).to_bytes(4, "little")
    inp[15:17] = (control.param0 & 0xFFFF).to_bytes(2, "little", signed=False)
    inp[17:19] = (control.param1 & 0xFFFF).to_bytes(2, "little", signed=False)
    return siphash24(bytes(inp), key).to_bytes(8, "little")


# ─────────────────────────────────────────────
#  V2 인증 프레임 인코딩 / 파싱
# ─────────────────────────────────────────────
def _frame_auth_tag(frame: bytes, payload_len: int, direction: int, key: bytes) -> bytes:
    authenticated = bytes([direction & 0xFF]) + frame[2:9 + payload_len]
    return siphash24(authenticated, key).to_bytes(8, "little")


def encode_frame(msg_type: int, seq: int, payload: bytes, *,
                 vehicle_id: int = VEHICLE_ID,
                 direction: int = FRAME_DIRECTION_UPLINK,
                 key: bytes = AUTH_KEY) -> bytes:
    """
    C++ encodeFrame 과 동일. 완성된 송신 프레임(bytes)을 반환.
    payload 길이는 msg_type 에 맞아야 함.
    """
    expected = payload_length_for_type(msg_type)
    if expected == 0 or len(payload) != expected:
        raise ValueError(f"payload 길이가 타입과 안 맞음 (expected {expected}, got {len(payload)})")

    out = bytearray(FRAME_OVERHEAD + expected)
    out[0] = SYNC0
    out[1] = SYNC1
    out[2] = make_ver_type(msg_type)
    out[3:7] = (vehicle_id & 0xFFFFFFFF).to_bytes(4, "little")
    out[7:9] = (seq & 0xFFFF).to_bytes(2, "little")
    out[FRAME_HEADER_LEN:FRAME_HEADER_LEN + expected] = payload
    tag_offset = FRAME_HEADER_LEN + expected
    out[tag_offset:tag_offset + FRAME_AUTH_TAG_LEN] = _frame_auth_tag(
        bytes(out), expected, direction, key
    )
    crc = crc16_ccitt_false(bytes(out[2:tag_offset + FRAME_AUTH_TAG_LEN]))
    out[-FRAME_CRC_LEN:] = (crc & 0xFFFF).to_bytes(2, "little")
    return bytes(out)


@dataclass
class ParsedFrame:
    msg_type: int
    vehicle_id: int
    seq: int
    payload: bytes


@dataclass
class GpsTelemetry:
    latitude_deg: float
    longitude_deg: float
    altitude_m: float
    speed_mps: float
    course_deg: float
    hdop: float
    satellites: int
    fix_flags: int
    age_s: float

    @property
    def has_fix(self) -> bool:
        return bool(self.fix_flags & 0x02)


def decode_gps_payload(payload: bytes) -> GpsTelemetry:
    """Decode the complete 18-byte C++ GpsTelemetry payload."""
    if len(payload) != GPS_PAYLOAD_LEN:
        raise ValueError(f"GPS payload length must be {GPS_PAYLOAD_LEN}, got {len(payload)}")
    return GpsTelemetry(
        latitude_deg=int.from_bytes(payload[0:4], "little", signed=True) / 10_000_000.0,
        longitude_deg=int.from_bytes(payload[4:8], "little", signed=True) / 10_000_000.0,
        altitude_m=int.from_bytes(payload[8:10], "little", signed=True) / 10.0,
        speed_mps=int.from_bytes(payload[10:12], "little") / 100.0,
        course_deg=int.from_bytes(payload[12:14], "little") / 100.0,
        hdop=payload[14] / 10.0,
        satellites=payload[15],
        fix_flags=payload[16],
        age_s=payload[17] / 10.0,
    )


def decode_frame(frame: bytes, *, vehicle_id: int = VEHICLE_ID,
                 direction: int = FRAME_DIRECTION_DOWNLINK,
                 key: bytes = AUTH_KEY) -> ParsedFrame | None:
    parsed, _reason = decode_frame_diagnostic(
        frame, vehicle_id=vehicle_id, direction=direction, key=key
    )
    return parsed


def decode_frame_diagnostic(frame: bytes, *, vehicle_id: int = VEHICLE_ID,
                            direction: int = FRAME_DIRECTION_DOWNLINK,
                            key: bytes = AUTH_KEY) -> tuple[ParsedFrame | None, str | None]:
    """Decode one frame and return a stable reject reason for diagnostics."""
    if len(frame) < FRAME_OVERHEAD or frame[0:2] != bytes([SYNC0, SYNC1]):
        return None, "sync_or_short"
    ver_type = frame[2]
    msg_type = frame_type(ver_type)
    payload_len = payload_length_for_type(msg_type)
    if frame_version(ver_type) != VERSION:
        return None, "version"
    if payload_len == 0:
        return None, "message_type"
    if len(frame) != FRAME_OVERHEAD + payload_len:
        return None, "length"
    received_vehicle_id = int.from_bytes(frame[3:7], "little")
    if received_vehicle_id != vehicle_id:
        return None, "vehicle_id"
    received_crc = int.from_bytes(frame[-FRAME_CRC_LEN:], "little")
    if received_crc != crc16_ccitt_false(frame[2:-FRAME_CRC_LEN]):
        return None, "crc"
    tag_offset = FRAME_HEADER_LEN + payload_len
    expected_tag = _frame_auth_tag(frame, payload_len, direction, key)
    if expected_tag != frame[tag_offset:tag_offset + FRAME_AUTH_TAG_LEN]:
        return None, "auth"
    return ParsedFrame(
        msg_type=msg_type,
        vehicle_id=received_vehicle_id,
        seq=int.from_bytes(frame[7:9], "little"),
        payload=bytes(frame[FRAME_HEADER_LEN:tag_offset]),
    ), None


class FrameParser:
    """
    USB serial stream에서 V2 프레임 경계를 찾은 뒤 전체 인증을 검증한다.
    """

    def __init__(self, *, vehicle_id: int = VEHICLE_ID,
                 direction: int = FRAME_DIRECTION_DOWNLINK,
                 key: bytes = AUTH_KEY):
        self.vehicle_id = vehicle_id
        self.direction = direction
        self.key = key
        self.reset_stats()
        self.reset()

    def reset(self):
        self._buf = bytearray()
        self._expected_len = 0

    def reset_stats(self):
        self.bytes_received = 0
        self.sync_candidates = 0
        self.frame_candidates = 0
        self.frames_ok = 0
        self.reject_counts = {
            "header": 0,
            "sync_or_short": 0,
            "version": 0,
            "message_type": 0,
            "length": 0,
            "vehicle_id": 0,
            "crc": 0,
            "auth": 0,
            "oversize": 0,
        }

    def stats(self) -> dict:
        return {
            "bytes_received": self.bytes_received,
            "sync_candidates": self.sync_candidates,
            "frame_candidates": self.frame_candidates,
            "frames_ok": self.frames_ok,
            "reject_counts": dict(self.reject_counts),
        }

    def feed(self, byte: int):
        b = byte & 0xFF
        if not self._buf:
            if b == SYNC0:
                self._buf.append(b)
                self.sync_candidates += 1
            return None
        if len(self._buf) == 1:
            if b == SYNC1:
                self._buf.append(b)
            elif b != SYNC0:
                self.reset()
            return None

        self._buf.append(b)
        if len(self._buf) == 3:
            payload_len = payload_length_for_type(frame_type(b))
            if frame_version(b) != VERSION or payload_len == 0:
                self.reject_counts["header"] += 1
                self.reset()
                return None
            self._expected_len = FRAME_OVERHEAD + payload_len

        if self._expected_len and len(self._buf) == self._expected_len:
            raw = bytes(self._buf)
            self.reset()
            self.frame_candidates += 1
            parsed, reason = decode_frame_diagnostic(
                raw,
                vehicle_id=self.vehicle_id,
                direction=self.direction,
                key=self.key,
            )
            if parsed is not None:
                self.frames_ok += 1
            elif reason is not None:
                self.reject_counts[reason] = self.reject_counts.get(reason, 0) + 1
            return parsed
        if len(self._buf) > MAX_FRAME_LEN:
            self.reject_counts["oversize"] += 1
            self.reset()
        return None

    def feed_bytes(self, data: bytes):
        """여러 바이트를 한 번에 먹이고, 완성된 프레임들을 리스트로 반환."""
        self.bytes_received += len(data)
        frames = []
        for byte in data:
            frame = self.feed(byte)
            if frame is not None:
                frames.append(frame)
        return frames


# ─────────────────────────────────────────────
#  강제 사출 프레임 빌더 (가장 중요한 함수)
# ─────────────────────────────────────────────
def build_force_deploy_frame(command_seq: int, frame_seq: int, nonce: int,
                              key: bytes = AUTH_KEY,
                              vehicle_id: int = VEHICLE_ID) -> bytes:
    """
    "강제 사출(FORCE_DEPLOY_RECOVERY)" CONTROL 프레임을 통째로 만들어서 bytes 로 반환.

    sender 펌웨어(로켓)의 handleCommand() 가 검사하는 조건들:
      - SipHash 인증 태그 일치해야 함            → make_control_auth_tag 로 채움
      - param0 == 0 이어야 함 (0 아니면 사출 금지) → param0 = 0 고정
      - command_seq/nonce 가 최근에 본 적 없어야 함 (중복 방지)

    이 조건을 만족하면 로켓은 ACK_ACCEPTED → deployFired=true → ACK_EXECUTED 순으로
    응답을 보냄.
    """
    control = ControlPayload(
        subtype=CONTROL_CMD,
        command_id=COMMAND_FORCE_DEPLOY_RECOVERY,
        command_seq=command_seq,
        nonce=nonce,
        valid_until_ms=0,     # 0 = 만료 검사 안 함
        param0=0,             # ★ 반드시 0 (0이 아니면 로켓이 거부함)
        param1=0,
    )
    control.auth_or_ack = make_control_auth_tag(control, frame_seq, key)
    payload = control.encode()
    return encode_frame(MESSAGE_CONTROL, frame_seq, payload,
                        vehicle_id=vehicle_id,
                        direction=FRAME_DIRECTION_UPLINK,
                        key=key)
