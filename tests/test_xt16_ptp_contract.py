from __future__ import annotations

from pathlib import Path
import unittest


REPO_ROOT = Path(__file__).resolve().parents[1]
SCRIPT = REPO_ROOT / "scripts" / "go2w_xt16_ptp.sh"


class Xt16PtpContractTests(unittest.TestCase):
    def test_ptp_manager_is_bounded_and_does_not_start_motion_stack(self) -> None:
        text = SCRIPT.read_text(encoding="utf-8")
        self.assertIn('ptp4l -i "${PTP_IFACE}" -S -4 -E', text)
        self.assertIn('--tx_timestamp_timeout "${TX_TIMESTAMP_TIMEOUT_MS}"', text)
        self.assertIn("clock_source&value=1", text)
        self.assertIn('"PTPStatus":"(Locked|Tracking)', text)
        self.assertIn("ptp_stably_healthy", text)
        self.assertIn("stop_existing_process", text)
        self.assertIn("xt16_ptp=healthy", text)
        self.assertIn("running_unverified", text)
        self.assertIn("clock_source&value=0", text)
        self.assertNotIn("start_go2w_slam_stack", text)
        self.assertNotIn("go2w_slam_gateway", text)
        self.assertNotIn("unitree_sdk", text.lower())

    def test_slam_startup_fails_closed_before_xt16_driver(self) -> None:
        slam_script = (
            REPO_ROOT / "scripts" / "start_go2w_slam_stack.sh"
        ).read_text(encoding="utf-8")
        check = 'bash "${SCRIPT_DIR}/go2w_xt16_ptp.sh" check'
        driver = "run_unitree_binary xt16_driver"
        self.assertIn(check, slam_script)
        self.assertLess(slam_script.index(check), slam_script.index(driver))
        self.assertIn('REQUIRE_XT16_PTP="${GO2W_REQUIRE_XT16_PTP:-1}"', slam_script)


if __name__ == "__main__":
    unittest.main()
