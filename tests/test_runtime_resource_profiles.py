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
        self.assertIn('labels = ["xt16_driver", "unitree_slam", "stereo_depth", "xt16_geometry", "deepyolo_detector", "deepyolo_bridge", "operator_web"]', text)
        self.assertNotIn(" -> int | None", text)
        self.assertNotIn(" -> dict[", text)
        for forbidden in ("kill ", "pkill", "systemctl", "subprocess", "os.system"):
            self.assertNotIn(forbidden, text)

    def test_short_acceptance_wrapper_has_no_navigation_execution_action(self):
        text = (REPO_ROOT / "scripts" / "go2w_accept.sh").read_text(encoding="utf-8")
        self.assertIn("status)", text)
        self.assertIn("relocate)", text)
        self.assertIn("verify)", text)
        self.assertIn("check)", text)
        self.assertIn("snapshot)", text)
        self.assertIn("relocate ANCHOR confirm", text)
        self.assertIn("verify ANCHOR", text)
        self.assertIn("check TARGET", text)
        self.assertIn("snapshot TEST_ID [ANCHOR]", text)
        self.assertIn("xt16-baseline OPERATOR FRONT_M LEFT_M RIGHT_M REAR_M", text)
        self.assertIn("xt16-scene SCENE OPERATOR MEASURED_M", text)
        self.assertIn('confirmation="${3:-}"', text)
        self.assertIn('[[ -z "${anchor}" || "${confirmation}" != "confirm" ]]', text)
        self.assertNotIn('${2:-mapping_origin}', text)
        self.assertNotIn("--execute", text)
        self.assertNotIn("navigate_to", text)
        self.assertIn("capture_go2w_field_acceptance.py", text)
        self.assertIn("capture_xt16_calibration_scene.py", text)

    def test_xt16_calibrated_mode_requires_repository_record_guard(self):
        text = (REPO_ROOT / "scripts" / "go2w_xt16_geometry_sidecar.sh").read_text(encoding="utf-8")
        self.assertIn("xt16_calibration_guard.py", text)
        self.assertIn("calibration_rejected", text)
        self.assertIn("extra_args_not_allowed", text)
        self.assertIn("--calibration-id", text)
        self.assertIn('SAFETY_STALE_MS="${GO2W_LIDAR_GEOMETRY_STALE_MS:-1000}"', text)

        producer = (REPO_ROOT / "scripts" / "xt16_lidar_geometry_summary.py").read_text(encoding="utf-8")
        self.assertIn("ReliabilityPolicy.BEST_EFFORT", producer)
        self.assertIn("depth=1", producer)
        self.assertIn("--footprint-front-m\", type=float, default=0.25", producer)
        self.assertIn("--footprint-rear-m\", type=float, default=0.50", producer)
        self.assertIn("--footprint-half-width-m\", type=float, default=0.30", producer)

    def test_slam_startup_self_heals_silent_xt16_once(self):
        text = (REPO_ROOT / "scripts" / "start_go2w_slam_stack.sh").read_text(encoding="utf-8")
        self.assertIn('RESTART_STALE_PROCESSES="${GO2W_RESTART_STALE_PROCESSES:-1}"', text)
        self.assertIn("restart_unitree_binary()", text)
        self.assertIn("require_driver_topic xt16_driver /unitree/slam_lidar/points 6", text)
        self.assertIn("--qos-reliability best_effort", text)
        self.assertIn("/slam_info may remain silent until a relocation request", text)

    def test_gateway_retries_pending_pause_until_accepted(self):
        text = (
            REPO_ROOT / "robot" / "slam_gateway_refactor" / "src" / "llm_command_main.cpp"
        ).read_text(encoding="utf-8")
        self.assertIn("bool pause_pending{false};", text)
        self.assertIn('"type", "navigation_pause_retry"', text)
        self.assertIn("lease.pause_pending && loop_now_ms >= lease.next_pause_retry_ms", text)
        self.assertIn('{"pause_pending", !paused.result.ok}', text)


if __name__ == "__main__":
    unittest.main()
