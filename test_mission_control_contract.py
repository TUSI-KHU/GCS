from pathlib import Path
import re
import unittest


HTML = Path(__file__).with_name("mission_control.html").read_text(encoding="utf-8")


class MissionControlContractTest(unittest.TestCase):
    def _toggle_stream_body(self) -> str:
        match = re.search(r"function toggleStream\(\) \{(?P<body>.*?)\nfunction resetAll\(\)", HTML, re.S)
        self.assertIsNotNone(match)
        return match.group("body")

    def test_connect_does_not_reset_hardware_backend_state(self):
        body = self._toggle_stream_body()
        self.assertNotIn("/api/telemetry/reset", body)

    def test_hardware_waiting_still_updates_ui_with_latest_sample(self):
        self.assertIsNotNone(
            re.search(
                r"if \(p\.source === 'hardware_waiting'\) \{.*?updateUI\(p\);.*?return;",
                HTML,
                re.S,
            )
        )

    def test_hardware_zero_altitude_does_not_auto_stop_stream(self):
        body = self._toggle_stream_body()
        self.assertIn("p.source === 'simulate' && p.alt <= 0 && p.ts > 30", body)

    def test_reset_button_is_the_only_ui_path_that_resets_backend(self):
        match = re.search(r"function resetAll\(\) \{(?P<body>.*?)\n\}", HTML, re.S)
        self.assertIsNotNone(match)
        self.assertIn("/api/telemetry/reset", match.group("body"))


if __name__ == "__main__":
    unittest.main()
