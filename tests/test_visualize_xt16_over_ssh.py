from __future__ import annotations

import unittest

from scripts.visualize_xt16_over_ssh import (
    build_remote_command,
    canvas_point,
    classify_body_point,
)


class VisualizeXt16OverSshTests(unittest.TestCase):
    def test_classifies_current_footprint_and_height_bands(self) -> None:
        common = {
            "footprint_front_m": 0.30,
            "footprint_rear_m": 0.30,
            "footprint_half_width_m": 0.30,
            "footprint_filter_margin_m": 0.05,
            "min_z_m": -0.25,
            "body_min_z_m": -0.10,
            "max_z_m": 1.20,
        }

        self.assertEqual(
            classify_body_point(0.31, 0.0, 0.1, **common),
            "footprint_rejected",
        )
        self.assertEqual(
            classify_body_point(0.50, 0.0, 0.1, **common),
            "body_height",
        )
        self.assertEqual(
            classify_body_point(0.50, 0.0, -0.20, **common),
            "low_hazard",
        )
        self.assertEqual(
            classify_body_point(0.33, 0.0, -0.20, **common),
            "low_hazard",
        )
        self.assertEqual(
            classify_body_point(0.50, 0.0, -0.40, **common),
            "height_rejected",
        )

    def test_canvas_orientation_matches_robot_view(self) -> None:
        center = canvas_point(0.0, 0.0, width=800, height=600, range_m=4.0)
        front = canvas_point(1.0, 0.0, width=800, height=600, range_m=4.0)
        left = canvas_point(0.0, 1.0, width=800, height=600, range_m=4.0)

        self.assertLess(front[1], center[1])
        self.assertLess(left[0], center[0])

    def test_remote_command_contains_no_credentials(self) -> None:
        command = build_remote_command(
            {
                "topic": "/utlidar/cloud",
                "repo_root": "/home/unitree/Go2W_SLAM_AI",
                "range_m": 4.0,
            }
        )

        self.assertIn("base64 -d", command)
        self.assertNotIn("password", command.lower())
        self.assertNotIn("123", command)


if __name__ == "__main__":
    unittest.main()
