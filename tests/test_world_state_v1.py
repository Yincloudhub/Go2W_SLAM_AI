import json
import unittest
from pathlib import Path

from edge_autonomy.llm_context import build_planner_context
from edge_autonomy.map_registry import MapRegistry
from edge_autonomy.operator_display import build_operator_display_state
from edge_autonomy.runtime_log import build_runtime_log_record
from edge_autonomy.world_state_v1 import build_world_state_v1


REGISTRY_PATH = Path(__file__).resolve().parents[1] / "configs" / "maps" / "go2w_map_registry.example.json"


def make_snapshot(ok: bool = True) -> dict:
    return {
        "timestamp_ms": 123,
        "expected_map_id": "test_current_main",
        "health_status": "ok" if ok else "failed",
        "localization_status": "localized_or_tracking" if ok else "relocation_odom_missing",
        "processes": {"unitree_slam": ok, "xt16_driver": ok},
        "lidar_state": {"alive": ok, "cloud_frequency_hz": 15.0, "cloud_size": 56000, "error_state": 0},
        "live_pointcloud": {"alive": ok, "topic": "/unitree/slam_lidar/points", "width": 56000},
        "relocation_odom": {"alive": ok, "x": 3.26, "y": -2.27, "z": 0.0, "yaw": -1.5},
    }


class WorldStateV1Tests(unittest.TestCase):
    def test_builds_canonical_world_state_from_planner_context(self) -> None:
        registry = MapRegistry.from_file(REGISTRY_PATH)
        context = build_planner_context(make_snapshot(), registry, user_command="go to wp_1")

        world = build_world_state_v1(
            make_snapshot(),
            planner_context=context,
            task_phase="planning",
            network_level="weak",
            timestamp_ms=456,
        )

        self.assertEqual(world["schema_version"], 1)
        self.assertEqual(world["timestamp_ms"], 456)
        self.assertTrue(world["localized"])
        self.assertTrue(world["map_loaded"])
        self.assertEqual(world["current_node"], "nie_guoli_office_front")
        self.assertEqual(world["task_phase"], "planning")
        self.assertEqual(world["network_level"], "weak")
        self.assertIn("navigate", world["available_tools"])
        self.assertIn("request_relocalization", build_world_state_v1(make_snapshot(False))["available_tools"])

    def test_gateway_safety_blocks_motion_tools(self) -> None:
        gateway = {
            "timestamp_ms": 1000,
            "world_state": {
                "localization": {"status": "lost"},
                "slam_health": {"status": "failed"},
                "safety": {"allow_navigation": False, "reason": "localization lost"},
                "local_obstacle": {"front_clearance_m": 0.0, "confidence": 0.8, "stale": False},
            },
        }

        world = build_world_state_v1(gateway, task_phase="executing_navigation")

        self.assertFalse(world["localized"])
        self.assertFalse(world["motion_allowed"])
        self.assertEqual(world["task_phase"], "executing_navigation")
        self.assertEqual(world["obstacle_status"], "blocked")
        self.assertEqual(world["front_clearance_m"], 0.0)
        self.assertNotIn("navigate", world["available_tools"])

    def test_operator_display_and_runtime_log_are_bounded(self) -> None:
        world = build_world_state_v1(make_snapshot(), timestamp_ms=123)
        queue_execution = {
            "completed": False,
            "blocked_reason": "",
            "events": [
                {
                    "operator_feedback": [{"text": "moving to target"}],
                    "llm_feedback_results": [{"text": "I am moving to the target.", "source": "template"}],
                }
            ],
        }
        task_queue = {
            "queue_id": "q1",
            "mode": "sequential",
            "status": "running",
            "source": "operator_panel",
            "targets": ["wp_1"],
            "steps": [{"task_id": "nav_1", "action": "navigate", "status": "running", "target_node": "wp_1"}],
            "communication_policy": {"mode": "semantic_only", "send": [], "drop": []},
        }

        display = build_operator_display_state(world, task_queue=task_queue, queue_execution=queue_execution, user_command="go")
        record = build_runtime_log_record(world_state=world, operator_display=display, task_queue=task_queue, queue_execution=queue_execution, user_command="go")

        self.assertEqual(display["screen"]["current_target"], "wp_1")
        self.assertEqual(display["screen"]["llm_reply"], "I am moving to the target.")
        self.assertFalse(record["artifact_policy"]["allow_raw_video"])
        self.assertFalse(record["artifact_policy"]["allow_dense_pointcloud"])
        self.assertEqual(json.loads(json.dumps(record))["schema_version"], 1)


if __name__ == "__main__":
    unittest.main()
