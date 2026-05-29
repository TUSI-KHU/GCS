import math
import unittest

import app


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

    def test_gps_line_updates_latest_position_without_overwriting_attitude(self):
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
        self.assertEqual(self.telemetry.latest["alt"], 12.3)

    def test_reset_discards_latest_hardware_sample(self):
        self.assertTrue(self.telemetry._apply_receiver_fast_line(FAST_2))

        self.telemetry.reset()
        waiting = self.telemetry.waiting_point()

        self.assertEqual(waiting["source"], "hardware_waiting")
        self.assertEqual(waiting["pitch"], 0.0)
        self.assertEqual(waiting["roll"], 0.0)
        self.assertEqual(waiting["accel"], 0.0)
        self.assertEqual(waiting["packet_id"], 0)


if __name__ == "__main__":
    unittest.main()
