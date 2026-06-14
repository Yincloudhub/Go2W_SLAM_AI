from __future__ import annotations

import unittest

from scripts.visualize_xt16_over_ssh import (
    DEFAULT_FRAME_ID,
    DEFAULT_TOPIC,
    REMOTE_STREAMER,
    blend_hex,
    build_remote_command,
    canvas_point,
    classify_body_point,
    main,
    stable_trail_cells,
    trail_weight,
)


class VisualizeXt16OverSshTests(unittest.TestCase):
    def test_classifies_current_footprint_and_height_bands(self) -> None:
        common = {
            "footprint_front_m": 0.30,
            "footprint_rear_m": 0.30,
            "footprint_half_width_m": 0.30,
            "footprint_filter_margin_m": 0.05,
            "footprint_lateral_filter_margin_m": 0.0,
            "min_z_m": -0.25,
            "body_min_z_m": -0.10,
            "max_z_m": 1.20,
        }

        self.assertEqual(
            classify_body_point(0.31, 0.0, 0.1, **common),
            "footprint_rejected",
        )
        self.assertEqual(
            classify_body_point(0.0, 0.31, 0.1, **common),
            "body_height",
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

    def test_trail_colors_fade_toward_canvas_background(self) -> None:
        self.assertEqual(blend_hex("#22d3ee", "#030712", 1.0), "#22d3ee")
        self.assertEqual(blend_hex("#22d3ee", "#030712", 0.0), "#030712")
        self.assertGreater(trail_weight(0.0, 3.0), trail_weight(2.0, 3.0))
        self.assertAlmostEqual(trail_weight(3.0, 3.0), 0.12)

    def test_default_topic_is_formal_xt16_runtime_cloud(self) -> None:
        self.assertEqual(DEFAULT_TOPIC, "/unitree/slam_lidar/points")
        self.assertEqual(DEFAULT_FRAME_ID, "rslidar")
        self.assertIn(
            'frame_id != config["expected_frame_id"]',
            REMOTE_STREAMER,
        )

    def test_main_rejects_non_xt16_topic_before_connecting(self) -> None:
        with self.assertRaisesRegex(SystemExit, "pipeline/frame"):
            main(["--topic", "/utlidar/cloud"])

    def test_stable_trail_cells_drop_single_frame_noise(self) -> None:
        history = [
            (
                1.0,
                {
                    "plot_points": [
                        [1.0, 0.0, 0.2, "body_height"],
                        [2.0, 0.0, 0.2, "body_height"],
                        [0.5, 0.0, -0.2, "low_hazard"],
                    ]
                },
            ),
            (
                2.0,
                {
                    "plot_points": [
                        [1.02, 0.01, 0.2, "body_height"],
                    ]
                },
            ),
        ]
        cells = stable_trail_cells(
            history,
            voxel_m=0.1,
            min_hits=2,
            show_rejected=False,
            show_low_hazard=False,
            max_cells=100,
        )
        self.assertEqual(len(cells), 1)
        self.assertAlmostEqual(cells[0][0], 1.01)
        self.assertEqual(cells[0][2], "body_height")
        self.assertEqual(cells[0][4], 2)


if __name__ == "__main__":
    unittest.main()
