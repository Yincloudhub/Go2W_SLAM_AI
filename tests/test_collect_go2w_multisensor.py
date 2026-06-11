from __future__ import annotations

import importlib.util
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch


SCRIPT = (
    Path(__file__).resolve().parents[1]
    / "robot"
    / "collection"
    / "collect_go2w_multisensor.py"
)
SPEC = importlib.util.spec_from_file_location("collect_go2w_multisensor", SCRIPT)
assert SPEC and SPEC.loader
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


class CollectGo2wMultisensorTests(unittest.TestCase):
    def test_parse_topic_list(self) -> None:
        topics = MODULE.parse_topic_list(
            "\n".join(
                [
                    "/frontvideostream [unitree_go/msg/WebRtcReq]",
                    "/utlidar/cloud [sensor_msgs/msg/PointCloud2]",
                    "/utlidar/robot_odom [nav_msgs/msg/Odometry]",
                ]
            )
        )
        self.assertEqual(topics["/utlidar/cloud"], "sensor_msgs/msg/PointCloud2")
        self.assertEqual(topics["/utlidar/robot_odom"], "nav_msgs/msg/Odometry")

    def test_select_topics_prefers_live_primary_topics(self) -> None:
        available = {
            "/frontvideostream": "video/type",
            "/utlidar/cloud": "sensor_msgs/msg/PointCloud2",
            "/utlidar/imu": "sensor_msgs/msg/Imu",
            "/utlidar/robot_odom": "nav_msgs/msg/Odometry",
            "/utlidar/robot_pose": "geometry_msgs/msg/PoseStamped",
        }
        selected, checks, missing = MODULE.select_topics(
            available,
            lambda topic: True,
            False,
        )
        self.assertEqual(
            selected,
            {
                "front_camera": "/frontvideostream",
                "pointcloud": "/utlidar/cloud",
                "imu": "/utlidar/imu",
                "odometry": "/utlidar/robot_odom",
                "attitude": "/utlidar/robot_pose",
            },
        )
        self.assertEqual(len(checks), 5)
        self.assertEqual(missing, [])

    def test_select_topics_uses_odometry_and_attitude_fallbacks(self) -> None:
        available = {
            "/frontvideostream": "video/type",
            "/utlidar/cloud": "sensor_msgs/msg/PointCloud2",
            "/utlidar/imu": "sensor_msgs/msg/Imu",
            "/uslam/localization/odom": "nav_msgs/msg/Odometry",
            "/sportmodestate": "unitree_go/msg/SportModeState",
        }
        selected, _, missing = MODULE.select_topics(
            available,
            lambda topic: True,
            False,
        )
        self.assertEqual(selected["odometry"], "/uslam/localization/odom")
        self.assertEqual(selected["attitude"], "/sportmodestate")
        self.assertEqual(missing, [])

    def test_select_topics_reports_listed_but_silent_imu_without_stopping(self) -> None:
        available = {
            "/frontvideostream": "video/type",
            "/utlidar/cloud": "sensor_msgs/msg/PointCloud2",
            "/utlidar/imu": "sensor_msgs/msg/Imu",
            "/utlidar/robot_odom": "nav_msgs/msg/Odometry",
            "/utlidar/robot_pose": "geometry_msgs/msg/PoseStamped",
        }
        selected, _, missing = MODULE.select_topics(
            available,
            lambda topic: topic != "/utlidar/imu",
            False,
        )
        self.assertNotIn("imu", selected)
        self.assertEqual(missing, ["imu"])

    def test_allow_missing_state_still_requires_sensor_topics(self) -> None:
        available = {
            "/frontvideostream": "video/type",
            "/utlidar/cloud": "sensor_msgs/msg/PointCloud2",
            "/utlidar/imu": "sensor_msgs/msg/Imu",
        }
        selected, _, missing = MODULE.select_topics(
            available,
            lambda topic: True,
            True,
        )
        self.assertEqual(set(selected), {"front_camera", "pointcloud", "imu"})
        self.assertEqual(missing, [])

    def test_build_bag_command_uses_sqlite3(self) -> None:
        command = MODULE.build_bag_command(
            Path("/tmp/bag"),
            ["/frontvideostream", "/utlidar/cloud"],
        )
        self.assertEqual(
            command,
            [
                "ros2",
                "bag",
                "record",
                "-s",
                "sqlite3",
                "-o",
                str(Path("/tmp/bag")),
                "/frontvideostream",
                "/utlidar/cloud",
            ],
        )

    def test_read_message_counts_without_yaml(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            metadata = Path(directory) / "metadata.yaml"
            metadata.write_text(
                """
rosbag2_bagfile_information:
  topics_with_message_count:
    - topic_metadata:
        name: /utlidar/cloud
        type: sensor_msgs/msg/PointCloud2
      message_count: 42
    - topic_metadata:
        name: /utlidar/imu
        type: sensor_msgs/msg/Imu
      message_count: 100
""",
                encoding="utf-8",
            )
            with patch.dict(sys.modules, {"yaml": None}):
                self.assertEqual(
                    MODULE.read_message_counts(metadata),
                    {
                        "/utlidar/cloud": 42,
                        "/utlidar/imu": 100,
                    },
                )


if __name__ == "__main__":
    unittest.main()
