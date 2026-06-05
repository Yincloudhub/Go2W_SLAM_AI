from __future__ import annotations

import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

from edge_autonomy.xt16_geometry import Xt16GeometryConfig, build_xt16_geometry_summary


REPO_ROOT = Path(__file__).resolve().parents[1]
SCRIPT = REPO_ROOT / "scripts" / "xt16_lidar_geometry_summary.py"


def repeated_point(x: float, y: float, z: float = 0.2, count: int = 10) -> list[tuple[float, float, float]]:
    return [(x, y, z) for _ in range(count)]


def clear_roi_points() -> list[tuple[float, float, float]]:
    points: list[tuple[float, float, float]] = []
    points.extend(repeated_point(4.0, 0.0))
    points.extend(repeated_point(0.0, 2.0))
    points.extend(repeated_point(0.0, -2.0))
    points.extend(repeated_point(-2.0, 0.0))
    return points


class Xt16GeometryTests(unittest.TestCase):
    def test_front_obstacle_blocks_when_calibrated(self) -> None:
        points = clear_roi_points()
        points.extend(repeated_point(0.9, 0.0))

        summary = build_xt16_geometry_summary(
            points,
            config=Xt16GeometryConfig(calibrated=True, min_points_per_roi=5),
            timestamp_ms=1,
        )

        self.assertFalse(summary["stale"])
        self.assertEqual(summary["source"], "lidar_pointcloud")
        self.assertLess(summary["front_clearance_m"], 0.8)
        self.assertIn("front", summary["blocked_directions"])
        self.assertEqual(summary["recommended_action"], "pause")

    def test_footprint_returns_are_ignored(self) -> None:
        points = clear_roi_points()
        points.extend(repeated_point(0.05, 0.05, count=30))

        summary = build_xt16_geometry_summary(
            points,
            config=Xt16GeometryConfig(calibrated=True, min_points_per_roi=5),
            timestamp_ms=1,
        )

        self.assertFalse(summary["stale"])
        self.assertGreater(summary["front_clearance_m"], 3.0)
        self.assertGreater(summary["summary"]["points_excluded_footprint"], 0)

    def test_uncalibrated_output_is_stale_even_with_good_roi(self) -> None:
        summary = build_xt16_geometry_summary(
            clear_roi_points(),
            config=Xt16GeometryConfig(calibrated=False, min_points_per_roi=5),
            timestamp_ms=1,
        )

        self.assertTrue(summary["stale"])
        self.assertIn("uncalibrated_xt16_geometry", summary["stale_reasons"])

    def test_missing_required_roi_is_stale(self) -> None:
        points = repeated_point(4.0, 0.0)

        summary = build_xt16_geometry_summary(
            points,
            config=Xt16GeometryConfig(calibrated=True, min_points_per_roi=5),
            timestamp_ms=1,
        )

        self.assertTrue(summary["stale"])
        self.assertIn("missing_required_roi:left,right", summary["stale_reasons"])
        self.assertIsNone(summary["left_clearance_m"])
        self.assertIsNone(summary["right_clearance_m"])

    def test_offline_script_writes_summary_json(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            input_path = root / "points.json"
            output_path = root / "summary.json"
            input_path.write_text(
                json.dumps({"frame_id": "rslidar", "points": clear_roi_points()}),
                encoding="utf-8",
            )

            result = subprocess.run(
                [
                    sys.executable,
                    str(SCRIPT),
                    "--input-json",
                    str(input_path),
                    "--output",
                    str(output_path),
                    "--calibrated",
                    "--min-points-per-roi",
                    "5",
                ],
                cwd=REPO_ROOT,
                text=True,
                capture_output=True,
                check=False,
            )

            self.assertEqual(result.returncode, 0, result.stderr)
            summary = json.loads(output_path.read_text(encoding="utf-8"))
            self.assertEqual(summary["source"], "lidar_pointcloud")
            self.assertFalse(summary["stale"])


if __name__ == "__main__":
    unittest.main()
