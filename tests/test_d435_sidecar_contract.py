import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]


class D435SidecarContractTests(unittest.TestCase):
    def test_legacy_sidecars_only_forward_to_unified_manager(self):
        for name in ("go2w_deepyolo_sidecar.sh", "go2w_stereo_depth_sidecar.sh"):
            text = (REPO_ROOT / "scripts" / name).read_text(encoding="utf-8")
            self.assertIn("go2w_d435_perception_sidecar.sh", text)
            self.assertNotIn("realsense_depth_summary.py", text)
            self.assertNotIn("yolo_test_realsense_headless", text)

    def test_unified_manager_refuses_known_competing_owners(self):
        text = (REPO_ROOT / "scripts" / "go2w_d435_perception_sidecar.sh").read_text(encoding="utf-8")
        self.assertIn("refuse_competing_owner", text)
        self.assertIn('command -v fuser', text)
        self.assertIn("D435/video device already held", text)
        self.assertIn('/comm"', text)
        self.assertIn("realsense_depth_summary.py", text)
        self.assertIn("yolo_test_realsense_headless", text)
        self.assertEqual(text.count('"${HEADLESS_BIN}" > "${CAPTURE_LOG}"'), 1)

    def test_manager_serializes_mutations_and_requires_fresh_startup(self):
        text = (REPO_ROOT / "scripts" / "go2w_d435_perception_sidecar.sh").read_text(encoding="utf-8")
        self.assertIn("acquire_operation_lock", text)
        self.assertIn("flock -n 9", text)
        self.assertIn("summary_health", text)
        self.assertIn("if ! health_sidecar; then", text)
        self.assertIn('owner.get("pid") == int(sys.argv[2])', text)

    def test_manager_runs_one_bounded_health_supervisor(self):
        text = (REPO_ROOT / "scripts" / "go2w_d435_perception_sidecar.sh").read_text(encoding="utf-8")
        self.assertIn('SUPERVISOR_SCRIPT="${SCRIPT_DIR}/go2w_d435_supervisor.py"', text)
        self.assertIn("start_supervisor()", text)
        self.assertIn("stop_supervisor()", text)
        self.assertIn("--failure-threshold", text)
        self.assertIn("--cooldown-s", text)
        self.assertIn("restart-if-stale)", text)
        self.assertEqual(text.count('nohup python3 "${SUPERVISOR_SCRIPT}"'), 1)

    def test_stop_attempts_reducer_and_capture_owner(self):
        text = (REPO_ROOT / "scripts" / "go2w_d435_perception_sidecar.sh").read_text(encoding="utf-8")
        stop_start = text.index("stop_sidecar()")
        stop_end = text.index("\n}\n", stop_start)
        stop_body = text[stop_start:stop_end]
        self.assertIn('stop_one "reducer"', stop_body)
        self.assertIn('stop_one "capture_owner"', stop_body)
        self.assertIn('return "${rc}"', stop_body)

    def test_perception_snapshot_reads_unified_pid_files(self):
        text = (REPO_ROOT / "scripts" / "snapshot_go2w_perception_sidecar.sh").read_text(encoding="utf-8")
        self.assertIn("go2w_d435_perception_sidecar.sh", text)
        self.assertIn("capture_owner.pid", text)
        self.assertIn("summary_reducer.pid", text)
        self.assertNotIn("detector.pid", text)
        self.assertNotIn("bridge.pid", text)

    def test_capture_generator_publishes_depth_before_yolo_frame_drop(self):
        text = (REPO_ROOT / "scripts" / "build_deepyolo_headless.sh").read_text(encoding="utf-8")
        depth_publish = text.index("go2wWriteDepthPacket(go2w_capture_depth")
        yolo_drop = text.index("capture_frame_count % capture_every_n")
        self.assertLess(depth_publish, yolo_drop)
        self.assertIn("go2wAttachCaptureMetadata", text)
        self.assertIn("get_frame_number()", text)
        self.assertIn("get_timestamp()", text)

    def test_capture_owner_atomically_publishes_latest_color_frame(self):
        generator = (REPO_ROOT / "scripts" / "build_deepyolo_headless.sh").read_text(encoding="utf-8")
        manager = (REPO_ROOT / "scripts" / "go2w_d435_perception_sidecar.sh").read_text(encoding="utf-8")
        self.assertIn("go2wWriteLatestColorFrame", generator)
        self.assertIn('output_path + ".tmp.jpg"', generator)
        self.assertIn("std::rename(tmp_image_path.c_str(), output_path.c_str())", generator)
        self.assertIn('std::getenv("GO2W_D435_LATEST_COLOR_PATH")', generator)
        self.assertIn('GO2W_D435_LATEST_COLOR_PATH="${LATEST_COLOR_PATH}"', manager)
        self.assertIn('GO2W_D435_LATEST_COLOR_INTERVAL_MS="${LATEST_COLOR_INTERVAL_MS}"', manager)


if __name__ == "__main__":
    unittest.main()
