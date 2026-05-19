import math
import unittest

from edge_autonomy.runtime_state import (
    build_runtime_snapshot,
    parse_lidar_state,
    parse_odom_pose,
    parse_pointcloud_summary,
    parse_process_state,
)


class RuntimeStateParserTests(unittest.TestCase):
    def test_parse_process_state(self) -> None:
        state = parse_process_state(
            "3219 ./unitree_slam\n"
            "3519 ./xt16_driver\n"
            "3914 ./slam_keyboard_client eth0\n"
        )

        self.assertTrue(state.unitree_slam)
        self.assertTrue(state.xt16_driver)
        self.assertTrue(state.slam_keyboard_client)
        self.assertFalse(state.slam_llm_command_client)

    def test_parse_lidar_state(self) -> None:
        state = parse_lidar_state(
            "cloud_frequency: 15.36\n"
            "imu_frequency: 250.0\n"
            "cloud_packet_loss_rate: 0.0\n"
            "cloud_size: 46800\n"
            "error_state: 0\n"
        )

        self.assertTrue(state.alive)
        self.assertAlmostEqual(state.cloud_frequency_hz or 0.0, 15.36)
        self.assertAlmostEqual(state.imu_frequency_hz or 0.0, 250.0)
        self.assertEqual(state.cloud_size, 46800)
        self.assertEqual(state.error_state, 0)

    def test_parse_pointcloud_summary(self) -> None:
        summary = parse_pointcloud_summary(
            "header:\n"
            "  frame_id: rslidar\n"
            "height: 1\n"
            "width: 56831\n"
            "point_step: 24\n"
            "row_step: 1363944\n"
            "is_dense: false\n"
        )

        self.assertTrue(summary.alive)
        self.assertEqual(summary.frame_id, "rslidar")
        self.assertEqual(summary.width, 56831)
        self.assertEqual(summary.point_step, 24)
        self.assertFalse(summary.is_dense)

    def test_parse_odom_pose(self) -> None:
        pose = parse_odom_pose(
            "header:\n"
            "  frame_id: map\n"
            "child_frame_id: rslidar\n"
            "pose:\n"
            "  pose:\n"
            "    position:\n"
            "      x: 1.0\n"
            "      y: -2.0\n"
            "      z: 0.1\n"
            "    orientation:\n"
            "      x: 0.0\n"
            "      y: 0.0\n"
            f"      z: {math.sqrt(0.5)}\n"
            f"      w: {math.sqrt(0.5)}\n"
        )

        self.assertTrue(pose.alive)
        self.assertEqual(pose.frame_id, "map")
        self.assertEqual(pose.child_frame_id, "rslidar")
        self.assertAlmostEqual(pose.x or 0.0, 1.0)
        self.assertAlmostEqual(pose.yaw or 0.0, math.pi / 2, places=6)

    def test_build_runtime_snapshot_health_ok(self) -> None:
        snapshot = build_runtime_snapshot(
            {
                "processes": "3219 ./unitree_slam\n3519 ./xt16_driver\n",
                "lidar_state": "cloud_frequency: 15.0\nerror_state: 0\n",
                "live_pointcloud": "frame_id: rslidar\nwidth: 50000\n",
                "relocation_odom": "frame_id: map\nchild_frame_id: rslidar\n",
            },
            host="192.168.123.18",
            expected_map_id="test_current_main",
            timestamp_ms=123,
        )

        self.assertEqual(snapshot.timestamp_ms, 123)
        self.assertEqual(snapshot.health_status, "ok")
        self.assertEqual(snapshot.localization_status, "localized_or_tracking")


if __name__ == "__main__":
    unittest.main()
