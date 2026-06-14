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


def repeated_point(
    x: float,
    y: float,
    z: float = 0.2,
    count: int = 10,
    spread_m: float = 0.06,
) -> list[tuple[float, float, float]]:
    return [
        (
            x + ((index % 3) - 1) * spread_m,
            y + (((index // 3) % 3) - 1) * spread_m,
            z + ((index % 2) * spread_m),
        )
        for index in range(count)
    ]


def clear_roi_points() -> list[tuple[float, float, float]]:
    points: list[tuple[float, float, float]] = []
    points.extend(repeated_point(0.0, -4.0))
    points.extend(repeated_point(2.0, 0.0))
    points.extend(repeated_point(-2.0, 0.0))
    points.extend(repeated_point(0.0, 2.0))
    return points


class Xt16GeometryTests(unittest.TestCase):
    def test_default_footprint_is_symmetric_thirty_centimeters(self) -> None:
        config = Xt16GeometryConfig()

        self.assertEqual(config.footprint_front_m, 0.30)
        self.assertEqual(config.footprint_rear_m, 0.30)
        self.assertEqual(config.footprint_half_width_m, 0.30)
        self.assertEqual(config.footprint_filter_margin_m, 0.05)
        self.assertEqual(config.footprint_lateral_filter_margin_m, 0.0)

    def test_corridor_rear_self_return_is_removed_by_default_margin(self) -> None:
        points = clear_roi_points()
        points.extend(repeated_point(0.11, 0.325, z=-0.09, count=30, spread_m=0.015))
        points.extend(repeated_point(-0.11, 0.325, z=-0.09, count=30, spread_m=0.015))

        summary = build_xt16_geometry_summary(
            points,
            config=Xt16GeometryConfig(
                calibrated=True,
                calibration_id="corridor-regression",
                min_points_per_roi=5,
            ),
            timestamp_ms=1,
        )

        self.assertGreater(summary["rear_clearance_m"], 1.5)
        self.assertGreaterEqual(summary["summary"]["points_excluded_footprint"], 60)
        self.assertEqual(
            summary["summary"]["footprint_m"]["filter_margin_applies_to"],
            "body_height_only_per_axis",
        )
        self.assertEqual(summary["summary"]["footprint_m"]["filter_margin"], 0.05)
        self.assertEqual(
            summary["summary"]["footprint_m"]["longitudinal_filter_margin"],
            0.05,
        )
        self.assertEqual(
            summary["summary"]["footprint_m"]["lateral_filter_margin"],
            0.0,
        )

    def test_lateral_margin_does_not_hide_close_side_obstacle(self) -> None:
        points = clear_roi_points()
        points.extend(repeated_point(0.33, 0.0, z=0.05, count=30, spread_m=0.01))

        summary = build_xt16_geometry_summary(
            points,
            config=Xt16GeometryConfig(
                calibrated=True,
                calibration_id="side-obstacle-regression",
                min_points_per_roi=5,
            ),
            timestamp_ms=1,
        )

        self.assertLess(summary["left_clearance_m"], 0.05)
        self.assertIn("left", summary["blocked_directions"])

    def test_body_margin_does_not_hide_low_hazard_outside_nominal_footprint(self) -> None:
        points = clear_roi_points()
        points.extend(repeated_point(0.0, 0.33, z=-0.20, count=12, spread_m=0.01))

        summary = build_xt16_geometry_summary(
            points,
            config=Xt16GeometryConfig(
                calibrated=True,
                calibration_id="low-hazard-regression",
                min_points_per_roi=5,
            ),
            timestamp_ms=1,
        )

        self.assertAlmostEqual(summary["low_hazard_clearance_m"]["rear"], 0.03, delta=0.03)
        self.assertIn("rear", summary["blocked_directions"])

    def test_front_obstacle_blocks_when_calibrated(self) -> None:
        points = clear_roi_points()
        points.extend(repeated_point(0.0, -0.9))

        summary = build_xt16_geometry_summary(
            points,
            config=Xt16GeometryConfig(calibrated=True, calibration_id="test-calibration", min_points_per_roi=5),
            timestamp_ms=1,
        )

        self.assertFalse(summary["stale"])
        self.assertEqual(summary["source"], "lidar_pointcloud")
        self.assertEqual(summary["schema_version"], 2)
        self.assertTrue(summary["parameters"]["calibrated"])
        self.assertEqual(summary["parameters"]["calibration_id"], "test-calibration")
        self.assertLess(summary["front_clearance_m"], 0.8)
        self.assertIn("front", summary["blocked_directions"])
        self.assertEqual(summary["recommended_action"], "pause")

    def test_corridor_side_wall_slows_without_blocking_navigation(self) -> None:
        points = clear_roi_points()
        points.extend(repeated_point(0.80, 0.0, count=30, spread_m=0.04))

        summary = build_xt16_geometry_summary(
            points,
            config=Xt16GeometryConfig(
                calibrated=True,
                calibration_id="corridor-policy",
                min_points_per_roi=5,
            ),
            timestamp_ms=1,
        )

        self.assertGreater(summary["left_clearance_m"], 0.2)
        self.assertLess(summary["left_clearance_m"], 0.6)
        self.assertNotIn("left", summary["blocked_directions"])
        self.assertEqual(summary["recommended_action"], "go_slow")

    def test_extremely_close_side_wall_blocks_navigation(self) -> None:
        points = clear_roi_points()
        points.extend(repeated_point(0.44, 0.0, count=30, spread_m=0.03))

        summary = build_xt16_geometry_summary(
            points,
            config=Xt16GeometryConfig(
                calibrated=True,
                calibration_id="corridor-policy",
                min_points_per_roi=5,
            ),
            timestamp_ms=1,
        )

        self.assertLess(summary["left_clearance_m"], 0.2)
        self.assertIn("left", summary["blocked_directions"])
        self.assertEqual(summary["recommended_action"], "pause")

    def test_footprint_returns_are_ignored(self) -> None:
        points = clear_roi_points()
        points.extend(repeated_point(0.05, -0.05, count=30))

        summary = build_xt16_geometry_summary(
            points,
            config=Xt16GeometryConfig(calibrated=True, calibration_id="test-calibration", min_points_per_roi=5),
            timestamp_ms=1,
        )

        self.assertFalse(summary["stale"])
        self.assertGreater(summary["front_clearance_m"], 3.0)
        self.assertGreater(summary["summary"]["points_excluded_footprint"], 0)

    def test_filter_margin_removes_boundary_self_return_without_shifting_clearance(self) -> None:
        points = clear_roi_points()
        points.extend(repeated_point(-0.31, 0.0, count=30, spread_m=0.0))
        points.extend(repeated_point(-0.50, 0.0, count=30, spread_m=0.04))

        summary = build_xt16_geometry_summary(
            points,
            config=Xt16GeometryConfig(
                calibrated=True,
                calibration_id="test-calibration",
                min_points_per_roi=5,
                footprint_half_width_m=0.30,
                footprint_filter_margin_m=0.02,
            ),
            timestamp_ms=1,
        )

        self.assertAlmostEqual(summary["right_clearance_m"], 0.18, delta=0.08)
        self.assertEqual(summary["summary"]["footprint_m"]["half_width"], 0.30)
        self.assertEqual(
            summary["summary"]["footprint_m"]["longitudinal_filter_margin"],
            0.02,
        )
        self.assertEqual(
            summary["summary"]["footprint_m"]["lateral_filter_margin"],
            0.0,
        )

    def test_uncalibrated_output_is_stale_even_with_good_roi(self) -> None:
        summary = build_xt16_geometry_summary(
            clear_roi_points(),
            config=Xt16GeometryConfig(calibrated=False, min_points_per_roi=5),
            timestamp_ms=1,
        )

        self.assertTrue(summary["stale"])
        self.assertIn("uncalibrated_xt16_geometry", summary["stale_reasons"])

    def test_calibrated_flag_without_record_id_fails_closed(self) -> None:
        summary = build_xt16_geometry_summary(
            clear_roi_points(),
            config=Xt16GeometryConfig(calibrated=True, min_points_per_roi=5),
            timestamp_ms=1,
        )

        self.assertTrue(summary["stale"])
        self.assertFalse(summary["summary"]["calibrated"])
        self.assertIn("missing_xt16_calibration_id", summary["stale_reasons"])

    def test_missing_required_roi_is_stale(self) -> None:
        points = repeated_point(0.0, -4.0)

        summary = build_xt16_geometry_summary(
            points,
            config=Xt16GeometryConfig(calibrated=True, calibration_id="test-calibration", min_points_per_roi=5),
            timestamp_ms=1,
        )

        self.assertTrue(summary["stale"])
        self.assertIn("missing_required_roi:left,right,rear", summary["stale_reasons"])
        self.assertIsNone(summary["left_clearance_m"])
        self.assertIsNone(summary["right_clearance_m"])
        self.assertIsNone(summary["rear_clearance_m"])

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
                    "--calibration-id",
                    "test-calibration",
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

    def test_default_axes_match_static_robot_calibration(self) -> None:
        summary = build_xt16_geometry_summary(
            clear_roi_points(),
            config=Xt16GeometryConfig(calibrated=True, calibration_id="test-calibration", min_points_per_roi=5),
            timestamp_ms=1,
        )

        axes = summary["summary"]["axes"]
        self.assertEqual(axes["forward"], "y")
        self.assertEqual(axes["forward_sign"], -1.0)
        self.assertEqual(axes["lateral"], "x")
        self.assertEqual(axes["lateral_sign"], 1.0)

    def test_low_rear_hazard_is_preserved_separately_from_body_clearance(self) -> None:
        points = clear_roi_points()
        points.extend(repeated_point(0.0, 0.53, z=-0.2, count=12))
        points.extend(repeated_point(0.0, 1.12, z=0.2, count=30))

        summary = build_xt16_geometry_summary(
            points,
            config=Xt16GeometryConfig(
                calibrated=True,
                calibration_id="test-calibration",
                min_points_per_roi=5,
                footprint_rear_m=0.50,
            ),
            timestamp_ms=1,
        )

        self.assertAlmostEqual(summary["low_hazard_clearance_m"]["rear"], 0.08, delta=0.08)
        self.assertAlmostEqual(summary["body_clearance_m"]["rear"], 0.56, delta=0.08)
        self.assertLess(summary["rear_clearance_m"], 0.2)
        self.assertIn("rear", summary["low_hazard_directions"])
        self.assertIn("rear", summary["blocked_directions"])
        self.assertEqual(summary["recommended_action"], "pause")

    def test_sparse_low_returns_do_not_override_supported_body_cluster(self) -> None:
        points = clear_roi_points()
        points.extend(repeated_point(0.0, 0.68, z=-0.2, count=3))
        points.extend(repeated_point(0.0, 1.12, z=0.2, count=30))

        summary = build_xt16_geometry_summary(
            points,
            config=Xt16GeometryConfig(
                calibrated=True,
                calibration_id="test-calibration",
                min_points_per_roi=5,
                footprint_rear_m=0.50,
            ),
            timestamp_ms=1,
        )

        self.assertIsNone(summary["low_hazard_clearance_m"]["rear"])
        self.assertAlmostEqual(summary["body_clearance_m"]["rear"], 0.56, delta=0.08)
        self.assertAlmostEqual(summary["rear_clearance_m"], 0.56, delta=0.08)
        self.assertIn("rear", summary["pending_low_hazard_directions"])
        self.assertTrue(summary["stale"])

    def test_nearest_supported_cluster_survives_far_low_background(self) -> None:
        points = clear_roi_points()
        points.extend(repeated_point(0.0, 0.53, z=-0.2, count=12))
        points.extend(repeated_point(0.0, 2.0, z=-0.2, count=120))
        points.extend(repeated_point(0.0, 1.12, z=0.2, count=30))

        summary = build_xt16_geometry_summary(
            points,
            config=Xt16GeometryConfig(
                calibrated=True,
                calibration_id="test-calibration",
                min_points_per_roi=5,
                footprint_rear_m=0.50,
            ),
            timestamp_ms=1,
        )

        self.assertAlmostEqual(summary["low_hazard_clearance_m"]["rear"], 0.08, delta=0.08)
        self.assertAlmostEqual(summary["body_clearance_m"]["rear"], 0.56, delta=0.08)
        self.assertLess(summary["rear_clearance_m"], 0.2)

    def test_duplicate_points_do_not_create_supported_cluster(self) -> None:
        points = clear_roi_points()
        points.extend([(0.0, 0.53, -0.2)] * 30)

        summary = build_xt16_geometry_summary(
            points,
            config=Xt16GeometryConfig(calibrated=True, calibration_id="test-calibration", min_points_per_roi=5),
            timestamp_ms=1,
        )

        self.assertIsNone(summary["low_hazard_clearance_m"]["rear"])
        self.assertEqual(summary["low_hazard_roi_confidence"]["rear"], 0.0)
        self.assertIn("rear", summary["pending_low_hazard_directions"])
        self.assertTrue(summary["stale"])

    def test_healthy_cloud_with_no_directional_return_is_clear_to_range(self) -> None:
        points: list[tuple[float, float, float]] = []
        points.extend(repeated_point(0.0, -4.0))
        points.extend(repeated_point(2.0, 0.0))
        points.extend(repeated_point(0.0, 2.0))
        points.extend(repeated_point(-7.0, 0.0))
        points.extend(repeated_point(8.0, 8.0, z=2.0, count=1000))

        summary = build_xt16_geometry_summary(
            points,
            config=Xt16GeometryConfig(calibrated=True, calibration_id="test-calibration", min_points_per_roi=5),
            timestamp_ms=1,
        )

        self.assertEqual(summary["right_clearance_m"], 6.0)
        self.assertIn("right", summary["summary"]["body_no_return_directions"])

    def test_unrelated_dense_cloud_cannot_clear_missing_direction(self) -> None:
        points: list[tuple[float, float, float]] = []
        points.extend(repeated_point(0.0, -4.0))
        points.extend(repeated_point(2.0, 0.0))
        points.extend(repeated_point(0.0, 2.0))
        points.extend(repeated_point(8.0, 8.0, z=2.0, count=1000))

        summary = build_xt16_geometry_summary(
            points,
            config=Xt16GeometryConfig(
                calibrated=True,
                calibration_id="test-calibration",
                min_points_per_roi=5,
            ),
            timestamp_ms=1,
        )

        self.assertIsNone(summary["right_clearance_m"])
        self.assertTrue(summary["stale"])
        self.assertIn("missing_required_roi:right", summary["stale_reasons"])


if __name__ == "__main__":
    unittest.main()
