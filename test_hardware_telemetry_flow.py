import csv
import math
import os
import tempfile
import unittest
from unittest import mock

import app
import protocol as p
from uplink import PyroUplink


FAST_1 = (
    "rx type=FAST seq=10 boot_ms=1000 state=SAFE state_code=1 status=0x8181 "
    "baro_dp_2pa=0 accel_g=(0.01,-0.01,0.26) gyro_dps=(-0.1,0.0,0.0) "
    "batt_mv=0 health=imu,-,-,radio rssi=-31 snr=9.50"
)

FAST_2 = (
    "rx type=FAST seq=11 boot_ms=1200 state=SAFE state_code=1 status=0x8181 "
    "baro_dp_2pa=0 accel_g=(0.50,0.00,0.87) gyro_dps=(1.2,2.3,3.4) "
    "batt_mv=12345 health=imu,-,-,radio rssi=-30 snr=10.25"
)

GPS_1 = (
    "rx type=GPS seq=12 fix=no lat_deg=34.4258000 lon_deg=127.5211000 "
    "alt_m=12.3 speed_mps=0.4 course_deg=90.0 hdop=25.5 sats=0 rssi=-32 snr=9.75"
)


class HardwareTelemetryFlowTest(unittest.TestCase):
    def setUp(self):
        self.telemetry = app.HardwareTelemetry()

    def test_fast_line_updates_attitude_when_accel_changes(self):
        self.assertTrue(self.telemetry._apply_receiver_fast_line(FAST_1))
        first = dict(self.telemetry.latest)

        self.assertTrue(self.telemetry._apply_receiver_fast_line(FAST_2))
        second = dict(self.telemetry.latest)

        self.assertNotEqual(first["pitch"], second["pitch"])
        self.assertNotEqual(first["roll"], second["roll"])
        self.assertNotEqual(first["accel"], second["accel"])
        self.assertAlmostEqual(second["accel"], math.sqrt(0.50**2 + 0.87**2), places=3)
        self.assertEqual(second["batt_mv"], 12345)
        self.assertEqual(second["gyro_z"], 3.4)

    def test_waiting_point_preserves_latest_value(self):
        self.assertTrue(self.telemetry._apply_receiver_fast_line(FAST_2))
        latest = dict(self.telemetry.latest)

        waiting = self.telemetry.waiting_point()

        self.assertEqual(waiting["source"], "hardware_waiting")
        self.assertEqual(waiting["pitch"], latest["pitch"])
        self.assertEqual(waiting["roll"], latest["roll"])
        self.assertEqual(waiting["accel"], latest["accel"])
        self.assertEqual(waiting["packet_id"], 1)

    def test_gps_line_updates_position_without_overwriting_baro_altitude_or_attitude(self):
        self.assertTrue(self.telemetry._apply_receiver_fast_line(FAST_2))
        pitch = self.telemetry.latest["pitch"]
        roll = self.telemetry.latest["roll"]
        accel = self.telemetry.latest["accel"]

        self.assertTrue(self.telemetry._apply_receiver_gps_line(GPS_1))

        self.assertEqual(self.telemetry.latest["pitch"], pitch)
        self.assertEqual(self.telemetry.latest["roll"], roll)
        self.assertEqual(self.telemetry.latest["accel"], accel)
        self.assertEqual(self.telemetry.latest["lat"], 34.4258)
        self.assertEqual(self.telemetry.latest["lng"], 127.5211)
        self.assertEqual(self.telemetry.latest["alt"], 0.0)
        self.assertEqual(self.telemetry.latest["gps_alt_m"], 12.3)
        self.assertEqual(self.telemetry.latest["speed_mps"], 0.4)
        self.assertEqual(self.telemetry.latest["course_deg"], 90.0)

    def test_reset_discards_latest_hardware_sample(self):
        self.assertTrue(self.telemetry._apply_receiver_fast_line(FAST_2))

        self.telemetry.reset()
        waiting = self.telemetry.waiting_point()

        self.assertEqual(waiting["source"], "hardware_waiting")
        self.assertEqual(waiting["pitch"], 0.0)
        self.assertEqual(waiting["roll"], 0.0)
        self.assertEqual(waiting["accel"], 0.0)
        self.assertEqual(waiting["packet_id"], 0)

    def test_binary_gps_decodes_every_protocol_field(self):
        payload = bytearray(p.GPS_PAYLOAD_LEN)
        payload[0:4] = int(34.4258 * 10_000_000).to_bytes(4, "little", signed=True)
        payload[4:8] = int(127.5211 * 10_000_000).to_bytes(4, "little", signed=True)
        payload[8:10] = int(123).to_bytes(2, "little", signed=True)
        payload[10:12] = int(456).to_bytes(2, "little")
        payload[12:14] = int(9012).to_bytes(2, "little")
        payload[14:18] = bytes((17, 9, 0x02, 4))

        self.assertTrue(self.telemetry._apply_gps(7, bytes(payload)))

        self.assertTrue(self.telemetry.gps_fix)
        self.assertEqual(self.telemetry.satellites, 9)
        self.assertEqual(self.telemetry.hdop, 1.7)
        self.assertEqual(self.telemetry.gps_age_s, 0.4)
        self.assertEqual(self.telemetry.speed_mps, 4.56)
        self.assertEqual(self.telemetry.course_deg, 90.12)
        self.assertEqual(self.telemetry.latest["gps_alt_m"], 12.3)
        self.assertEqual(self.telemetry.latest["alt"], 0.0)

    def test_shared_downlink_sequence_tracks_gap_and_rejects_duplicate(self):
        self.assertTrue(self.telemetry._apply_receiver_fast_line(FAST_1))
        self.assertTrue(self.telemetry._apply_receiver_gps_line(GPS_1))
        self.assertEqual(self.telemetry.sequence_gaps, 1)
        packet_id = self.telemetry.packet_id

        self.assertFalse(self.telemetry._apply_receiver_gps_line(GPS_1))

        self.assertEqual(self.telemetry.packet_id, packet_id)
        self.assertEqual(self.telemetry.duplicate_frames, 1)

    def test_receiver_and_bridge_status_are_exposed(self):
        self.assertFalse(self.telemetry._apply_receiver_line(
            "status radio=ready phy_rx=12 decode_fail=3 rssi=-41 snr=7.5"
        ))
        self.assertFalse(self.telemetry._apply_receiver_line(
            "NURA_BRIDGE radio=ready profile=sx1276_ground "
            "hardware=sparkfun_spx18572_915m30s_1w "
            "pins=miso:1,mosi:26,sck:27,cs:9,rst:24,dio0:32,rxen:30,txen:31 "
            "spi_mode=0 reg42_m0=0x12 last_rssi=-39 last_snr=8.25"
        ))

        waiting = self.telemetry.waiting_point()
        self.assertEqual(waiting["receiver_status"]["profile"], "sx1276_ground")
        self.assertEqual(
            waiting["receiver_status"]["hardware"],
            "sparkfun_spx18572_915m30s_1w",
        )
        self.assertEqual(waiting["receiver_status"]["spi_mode"], "0")
        self.assertEqual(waiting["receiver_status"]["reg42_m0"], "0x12")
        self.assertEqual(waiting["rssi"], -39)
        self.assertEqual(waiting["snr"], 8.25)

    def test_hardware_packet_is_written_to_hardware_csv(self):
        old_log_dir = app.LOG_DIR
        try:
            with tempfile.TemporaryDirectory() as temp_dir:
                app.LOG_DIR = temp_dir
                path = self.telemetry.start_logging()
                self.assertTrue(self.telemetry._apply_receiver_fast_line(FAST_1))
                self.telemetry.close_logging()

                with open(path, newline="", encoding="utf-8") as handle:
                    rows = list(csv.DictReader(handle))
                self.assertEqual(len(rows), 1)
                self.assertEqual(rows[0]["packet_type"], "FAST")
                self.assertEqual(rows[0]["seq"], "10")
                self.assertEqual(rows[0]["payload_hex"], FAST_1)
        finally:
            app.LOG_DIR = old_log_dir


class RawTransportTest(unittest.TestCase):
    class FakeSerial:
        def __init__(self, chunk: bytes):
            self.chunk = chunk
            self.is_open = True

        def read(self, _max_bytes: int) -> bytes:
            chunk, self.chunk = self.chunk, b""
            return chunk

        def close(self) -> None:
            self.is_open = False

    def test_newline_inside_authenticated_frame_is_not_a_text_line(self):
        payload = bytearray(p.FAST_PAYLOAD_LEN)
        payload[0:2] = (0x0101).to_bytes(2, "little")
        payload[2:6] = (1000).to_bytes(4, "little")
        payload[8] = 0x0A
        frame = p.encode_frame(
            p.MESSAGE_FAST_TLM,
            10,
            bytes(payload),
            direction=p.FRAME_DIRECTION_DOWNLINK,
        )
        link = PyroUplink(port="fake", serial_mode="raw")
        link._ser = self.FakeSerial(frame)

        frames, lines = link.read_frames_and_lines()

        self.assertEqual(len(frames), 1)
        self.assertEqual(frames[0].seq, 10)
        self.assertEqual(lines, [])
        self.assertEqual(link.diagnostics()["parser"]["frames_ok"], 1)

    def test_frame_parser_reports_crc_rejection(self):
        payload = bytes(p.GPS_PAYLOAD_LEN)
        frame = bytearray(p.encode_frame(
            p.MESSAGE_GPS_TLM,
            1,
            payload,
            direction=p.FRAME_DIRECTION_DOWNLINK,
        ))
        frame[-1] ^= 0x01
        parser = p.FrameParser()

        self.assertEqual(parser.feed_bytes(bytes(frame)), [])
        self.assertEqual(parser.stats()["reject_counts"]["crc"], 1)

    def test_frames_buffered_during_pyro_ack_wait_are_returned_to_reader(self):
        frame = p.ParsedFrame(
            msg_type=p.MESSAGE_FAST_TLM,
            vehicle_id=p.VEHICLE_ID,
            seq=22,
            payload=bytes(p.FAST_PAYLOAD_LEN),
        )
        link = PyroUplink(port="fake", serial_mode="raw")
        link._ser = self.FakeSerial(b"")
        link._pending_frames.append(frame)

        frames, lines = link.read_frames_and_lines()

        self.assertEqual(frames, [frame])
        self.assertEqual(lines, [])


class PyroSafetyTest(unittest.TestCase):
    def test_downlink_only_override_is_detected(self):
        with mock.patch.dict(os.environ, {"NURA_AVIONICS_DOWNLINK_ONLY": "1"}):
            self.assertTrue(app.avionics_downlink_only())
        with mock.patch.dict(os.environ, {"NURA_AVIONICS_DOWNLINK_ONLY": "0"}):
            self.assertFalse(app.avionics_downlink_only())

    def test_simulation_pyro_is_not_blocked_by_hardware_source_setting(self):
        original_uplink = app.uplink
        try:
            app.uplink = PyroUplink(simulate=True)
            with mock.patch.dict(os.environ, {"NURA_AVIONICS_DOWNLINK_ONLY": "1"}):
                response = app.app.test_client().post(
                    "/api/pyro/deploy", json={"confirm": "DEPLOY"}
                )
            self.assertEqual(response.status_code, 200)
            self.assertTrue(response.get_json()["success"])
        finally:
            app.uplink = original_uplink


if __name__ == "__main__":
    unittest.main()
