import json
import math
import unittest

from edge_autonomy.slam_topics import SlamTopicParseError, parse_slam_ctrl_info, parse_slam_info, parse_slam_key_info, quaternion_to_yaw


class SlamTopicParserTests(unittest.TestCase):
    def test_quaternion_to_yaw(self) -> None:
        yaw = quaternion_to_yaw(0.0, 0.0, math.sqrt(0.5), math.sqrt(0.5))
        self.assertAlmostEqual(yaw, math.pi / 2, places=6)

    def test_parse_slam_info_pos_info(self) -> None:
        raw = json.dumps(
            {
                "errorCode": 0,
                "type": "pos_info",
                "data": {
                    "currentPose": {
                        "x": 1.2,
                        "y": -0.4,
                        "z": 0.1,
                        "q_x": 0.0,
                        "q_y": 0.0,
                        "q_z": math.sqrt(0.5),
                        "q_w": math.sqrt(0.5),
                    }
                },
            }
        )

        pose = parse_slam_info(raw, timestamp_ms=123, map_id="701")

        self.assertIsNotNone(pose)
        assert pose is not None
        self.assertEqual(pose.timestamp_ms, 123)
        self.assertEqual(pose.map_id, "701")
        self.assertAlmostEqual(pose.pose.x, 1.2)
        self.assertAlmostEqual(pose.pose.y, -0.4)
        self.assertAlmostEqual(pose.pose.yaw, math.pi / 2, places=6)
        self.assertAlmostEqual(pose.z, 0.1)

    def test_parse_slam_info_ignores_non_pose_messages(self) -> None:
        self.assertIsNone(parse_slam_info({"errorCode": 0, "type": "other"}, timestamp_ms=1))

    def test_parse_slam_info_rejects_missing_pose_fields(self) -> None:
        with self.assertRaises(SlamTopicParseError):
            parse_slam_info(
                {
                    "errorCode": 0,
                    "type": "pos_info",
                    "data": {"currentPose": {"x": 1.0, "y": 2.0}},
                },
                timestamp_ms=1,
            )

    def test_parse_slam_info_rejects_invalid_quaternion(self) -> None:
        with self.assertRaises(SlamTopicParseError):
            parse_slam_info(
                {
                    "errorCode": 0,
                    "type": "pos_info",
                    "data": {
                        "currentPose": {
                            "x": 1.0,
                            "y": 2.0,
                            "z": 0.0,
                            "q_x": 0.0,
                            "q_y": 0.0,
                            "q_z": 0.0,
                            "q_w": 0.0,
                        }
                    },
                },
                timestamp_ms=1,
            )

    def test_parse_slam_key_info_arrived(self) -> None:
        state = parse_slam_key_info(
            {"type": "task_result", "data": {"is_arrived": True, "targetNodeName": "701_door_inside"}},
            timestamp_ms=456,
        )

        self.assertIsNotNone(state)
        assert state is not None
        self.assertEqual(state.timestamp_ms, 456)
        self.assertEqual(state.state, "arrived")
        self.assertTrue(state.is_arrived)
        self.assertEqual(state.target_node, "701_door_inside")

    def test_parse_slam_key_info_error_maps_to_failed(self) -> None:
        state = parse_slam_key_info({"errorCode": 1, "info": "backend error"}, timestamp_ms=789)

        self.assertIsNotNone(state)
        assert state is not None
        self.assertEqual(state.state, "failed")
        self.assertEqual(state.failure_reason, "backend error")

    def test_parse_slam_ctrl_info_arrived(self) -> None:
        state = parse_slam_ctrl_info(
            {
                "type": "ctrl_info",
                "errorCode": 0,
                "info": "The navigation point has been reached. Node id is No.9999",
                "data": {
                    "is_arrived": True,
                    "targetNodeName": 9999,
                    "stateMachine": {"state": "FINISHED"},
                    "targetPose": {"x": 3.25, "y": -2.28, "yaw": -1.61},
                },
            },
            timestamp_ms=1001,
        )

        self.assertIsNotNone(state)
        assert state is not None
        self.assertEqual(state.state, "arrived")
        self.assertTrue(state.is_arrived)
        self.assertEqual(state.target_node, "9999")
        self.assertAlmostEqual(state.target_pose.x, 3.25)
        self.assertAlmostEqual(state.target_pose.y, -2.28)
        self.assertAlmostEqual(state.target_pose.yaw, -1.61)


if __name__ == "__main__":
    unittest.main()
