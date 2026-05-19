import json
import unittest

from edge_autonomy.models import NavigationGoal, Pose2D
from edge_autonomy.slam_adapter import NavigationStatus, ReplaySlamAdapter


class ReplaySlamAdapterTests(unittest.TestCase):
    def test_ingest_slam_info_updates_pose_and_localization(self) -> None:
        adapter = ReplaySlamAdapter(map_id="701")
        raw = json.dumps(
            {
                "type": "pos_info",
                "data": {
                    "currentPose": {
                        "x": 1.0,
                        "y": 2.0,
                        "z": 0.0,
                        "q_x": 0.0,
                        "q_y": 0.0,
                        "q_z": 0.0,
                        "q_w": 1.0,
                    }
                },
            }
        )

        pose = adapter.ingest_slam_info(raw, timestamp_ms=1000)

        self.assertIsNotNone(pose)
        self.assertEqual(adapter.get_robot_state().pose, Pose2D(1.0, 2.0, 0.0))
        self.assertEqual(adapter.get_localization_state().map_id, "701")

    def test_ingest_slam_key_info_updates_navigation_feedback(self) -> None:
        adapter = ReplaySlamAdapter(map_id="701")
        adapter.submit_navigation_goal(NavigationGoal(goal_id="701_door_inside", target_pose=Pose2D(1.0, 0.0, 0.0)))

        state = adapter.ingest_slam_key_info(
            json.dumps({"type": "task_result", "data": {"is_arrived": True, "targetNodeName": "701_door_inside"}}),
            timestamp_ms=2000,
        )

        self.assertIsNotNone(state)
        self.assertEqual(adapter.get_navigation_task_state().state, "arrived")
        self.assertEqual(adapter.get_feedback().status, NavigationStatus.GOAL_REACHED)
        self.assertEqual(adapter.get_feedback().distance_to_goal_m, 0.0)

    def test_ingest_slam_ctrl_info_updates_navigation_feedback(self) -> None:
        adapter = ReplaySlamAdapter(map_id="701")
        adapter.submit_navigation_goal(NavigationGoal(goal_id="wp_1", target_pose=Pose2D(3.25, -2.28, -1.61)))

        state = adapter.ingest_slam_ctrl_info(
            json.dumps(
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
                }
            ),
            timestamp_ms=3000,
        )

        self.assertIsNotNone(state)
        self.assertEqual(adapter.get_navigation_task_state().state, "arrived")
        self.assertEqual(adapter.get_feedback().status, NavigationStatus.GOAL_REACHED)
        self.assertEqual(adapter.get_feedback().distance_to_goal_m, 0.0)


if __name__ == "__main__":
    unittest.main()
