import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

from edge_autonomy.perception_context import (
    build_live_perception_context,
    load_perception_context_file,
    write_perception_context_file,
)


REPO_ROOT = Path(__file__).resolve().parents[1]
SERVICE_SCRIPT = REPO_ROOT / "scripts" / "perception_context_service.py"
MANAGER_SCRIPT = REPO_ROOT / "scripts" / "go2w_perception_context_sidecar.sh"
SLAM_START_SCRIPT = REPO_ROOT / "scripts" / "start_go2w_slam_stack.sh"
WEB_START_SCRIPT = REPO_ROOT / "scripts" / "run_go2w_operator_web.sh"
SNAPSHOT_SCRIPT = REPO_ROOT / "scripts" / "snapshot_go2w_perception_sidecar.sh"


class PerceptionContextServiceTests(unittest.TestCase):
    def test_live_builder_exposes_all_sources_fail_closed(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            context = build_live_perception_context(root, generated_at_ms=10_000)

            self.assertEqual(
                [source["source_id"] for source in context["sources"]],
                [
                    "xt16_geometry",
                    "d435_depth",
                    "d435_yolo",
                    "ti_nx_radar",
                    "xt16_imu_motion",
                    "unitree_odometry_motion",
                ],
            )
            self.assertTrue(all(source["status"] in {"offline", "invalid"} for source in context["sources"]))
            self.assertEqual(context["degraded_capabilities"], sorted(source["source_id"] for source in context["sources"]))

    def test_context_file_loader_rejects_stale_artifact(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / "context.json"
            context = build_live_perception_context(Path(temp), generated_at_ms=10_000)
            write_perception_context_file(path, context)

            self.assertEqual(load_perception_context_file(path, current_time_ms=10_500)["context_id"], context["context_id"])
            self.assertIsNone(load_perception_context_file(path, current_time_ms=11_001))
            self.assertTrue(json.loads(path.read_text(encoding="utf-8"))["policy"]["llm_direct_motion"] is False)

    def test_one_shot_service_writes_single_context_artifact(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            output = root / "artifacts" / "perception_context_v1.json"
            result = subprocess.run(
                [
                    sys.executable,
                    str(SERVICE_SCRIPT),
                    "--repo-root",
                    str(root),
                    "--output",
                    str(output),
                    "--once",
                ],
                cwd=REPO_ROOT,
                capture_output=True,
                text=True,
                timeout=20,
            )
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertIsNotNone(load_perception_context_file(output))

    def test_manager_never_starts_sensor_or_motion_services(self) -> None:
        text = MANAGER_SCRIPT.read_text(encoding="utf-8")
        self.assertIn("perception_context_service.py", text)
        self.assertNotIn("go2w_d435_perception_sidecar.sh start", text)
        self.assertNotIn("go2w_xt16_geometry_sidecar.sh start", text)
        self.assertNotIn("start_go2w_slam_stack.sh", text)
        self.assertNotIn("slam_llm_command_client", text)

    def test_runtime_entrypoints_use_single_context_sidecar(self) -> None:
        slam_text = SLAM_START_SCRIPT.read_text(encoding="utf-8")
        web_text = WEB_START_SCRIPT.read_text(encoding="utf-8")
        snapshot_text = SNAPSHOT_SCRIPT.read_text(encoding="utf-8")

        self.assertIn("go2w_perception_context_sidecar.sh", slam_text)
        self.assertIn("go2w_perception_context_sidecar.sh", web_text)
        self.assertIn("--perception-context-path", web_text)
        self.assertNotIn("GO2W_STEREO_SUMMARY_PATH", web_text)
        self.assertNotIn("GO2W_LIDAR_GEOMETRY_SUMMARY_PATH", web_text)
        self.assertIn("go2w_perception_context_sidecar.sh", snapshot_text)


if __name__ == "__main__":
    unittest.main()
