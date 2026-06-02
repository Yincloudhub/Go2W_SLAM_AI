import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]


class RuntimeResourceProfileTests(unittest.TestCase):
    def test_deepyolo_sidecar_has_bounded_profiles(self):
        text = (REPO_ROOT / "scripts" / "go2w_deepyolo_sidecar.sh").read_text(encoding="utf-8")
        self.assertIn('PROFILE="${GO2W_DEEPYOLO_PROFILE:-resident}"', text)
        self.assertIn("resident)", text)
        self.assertIn("balanced)", text)
        self.assertIn("diagnostic)", text)
        self.assertIn("DEFAULT_CAPTURE_EVERY_N=5", text)
        self.assertIn("DEFAULT_INFERENCE_INTERVAL_MS=333", text)
        self.assertIn('RENDER_OVERLAY="${GO2W_DEEPYOLO_RENDER_OVERLAY:-0}"', text)
        self.assertIn("summary_health()", text)
        self.assertIn("restart-if-stale)", text)
        self.assertIn('healthy_statuses = {"fresh", "event_only_idle", "depth_insufficient"}', text)

    def test_deepyolo_headless_skips_overlay_by_default(self):
        text = (REPO_ROOT / "scripts" / "build_deepyolo_headless.sh").read_text(encoding="utf-8")
        self.assertIn('std::getenv("GO2W_DEEPYOLO_RENDER_OVERLAY")', text)
        self.assertIn("if (render_overlay) {", text)

    def test_runtime_resource_snapshot_is_read_only(self):
        text = (REPO_ROOT / "scripts" / "snapshot_go2w_runtime_resources.sh").read_text(encoding="utf-8")
        self.assertIn("Read-only resource snapshot", text)
        self.assertIn('labels = ["xt16_driver", "unitree_slam", "stereo_depth", "deepyolo_detector", "deepyolo_bridge", "operator_web"]', text)
        self.assertNotIn(" -> int | None", text)
        self.assertNotIn(" -> dict[", text)
        for forbidden in ("kill ", "pkill", "systemctl", "subprocess", "os.system"):
            self.assertNotIn(forbidden, text)


if __name__ == "__main__":
    unittest.main()
