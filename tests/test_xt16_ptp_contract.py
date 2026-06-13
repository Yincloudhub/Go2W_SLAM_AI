from __future__ import annotations

from pathlib import Path
import unittest


REPO_ROOT = Path(__file__).resolve().parents[1]
SCRIPT = REPO_ROOT / "scripts" / "go2w_xt16_ptp.sh"


class Xt16PtpContractTests(unittest.TestCase):
    def test_ptp_manager_is_bounded_and_does_not_start_motion_stack(self) -> None:
        text = SCRIPT.read_text(encoding="utf-8")
        self.assertIn('ptp4l -i "${PTP_IFACE}" -S -4 -E', text)
        self.assertIn("clock_source&value=1", text)
        self.assertIn('"PTPStatus":"Locked', text)
        self.assertIn("running_unverified", text)
        self.assertIn("clock_source&value=0", text)
        self.assertNotIn("start_go2w_slam_stack", text)
        self.assertNotIn("go2w_slam_gateway", text)
        self.assertNotIn("unitree_sdk", text.lower())


if __name__ == "__main__":
    unittest.main()
