import json
import unittest
from pathlib import Path

from jsonschema import Draft202012Validator
from referencing import Registry, Resource

from edge_autonomy.llm_context import build_planner_context
from edge_autonomy.map_registry import MapRegistry
from edge_autonomy.operator_display import build_operator_display_state
from edge_autonomy.perception_context import build_perception_context
from edge_autonomy.runtime_log import build_runtime_log_record
from edge_autonomy.world_state_v1 import build_world_state_v1


REGISTRY_PATH = Path(__file__).resolve().parents[1] / "configs" / "maps" / "go2w_map_registry.example.json"
REPO_ROOT = Path(__file__).resolve().parents[1]
SENSOR_SCHEMA = json.loads((REPO_ROOT / "schemas" / "sensor_envelope_v1.schema.json").read_text(encoding="utf-8"))
CONTEXT_SCHEMA = json.loads((REPO_ROOT / "schemas" / "perception_context_v1.schema.json").read_text(encoding="utf-8"))
WORLD_SCHEMA = json.loads((REPO_ROOT / "schemas" / "world_state_schema_v1.json").read_text(encoding="utf-8"))
SCHEMA_REGISTRY = (
    Registry()
    .with_resource(SENSOR_SCHEMA["$id"], Resource.from_contents(SENSOR_SCHEMA))
    .with_resource(CONTEXT_SCHEMA["$id"], Resource.from_contents(CONTEXT_SCHEMA))
)
WORLD_VALIDATOR = Draft202012Validator(WORLD_SCHEMA, registry=SCHEMA_REGISTRY)


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


def make_perception_context(timestamp_ms: int = 123) -> dict:
    source = {
        "schema_version": 1,
        "schema": "go2w_sensor_envelope_v1",
        "source_id": "d435_yolo",
        "source_kind": "visual_object_semantics",
        "timestamp_ms": timestamp_ms,
        "received_ms": timestamp_ms,
        "sequence": 42,
        "age_ms": 0,
        "stale_ms": 3000,
        "frame_id": "camera_color_optical_frame",
        "status": "fresh",
        "confidence": 0.9,
        "calibration_status": "unknown",
        "calibration_id": None,
        "producer": "d435_perception_sidecar",
        "producer_instance_id": "boot:123:456",
        "status_reasons": [],
        "payload": {
            "objects": [{"class_name": "person", "confidence": 0.9}],
            "generation_id": "generation-test",
        },
    }
    return build_perception_context([source], generated_at_ms=timestamp_ms, context_id="pc-world-test")


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
        self.assertIn("record_keyframe_event", world["available_tools"])
        self.assertNotIn("capture_keyframe", world["available_tools"])
        self.assertIn("request_relocalization", build_world_state_v1(make_snapshot(False))["available_tools"])
        self.assertIsNone(world["perception_context"])
        self.assertEqual(world["perception_summaries"], [])
        self.assertEqual(list(WORLD_VALIDATOR.iter_errors(world)), [])

    def test_capture_tool_requires_configured_command(self) -> None:
        registry = MapRegistry.from_file(REGISTRY_PATH)
        snapshot = make_snapshot()
        snapshot["capture_command_configured"] = True
        context = build_planner_context(snapshot, registry, user_command="photo")

        world = build_world_state_v1(snapshot, planner_context=context)

        self.assertIn("capture_keyframe", world["available_tools"])
        self.assertNotIn("record_keyframe_event", world["available_tools"])
        self.assertTrue(world["capture_keyframe"]["configured"])

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

    def test_world_state_consumes_only_fresh_perception_context(self) -> None:
        context = make_perception_context()
        world = build_world_state_v1(make_snapshot(), perception_context=context, timestamp_ms=123)

        self.assertEqual(world["perception_context"]["context_id"], "pc-world-test")
        self.assertEqual(world["perception_summaries"], context["sources"])
        self.assertEqual(world["detected_objects"][0]["class_name"], "person")
        self.assertEqual(world["source_health"]["perception_context_status"], "fresh")
        self.assertEqual(list(WORLD_VALIDATOR.iter_errors(world)), [])

        stale_world = build_world_state_v1(make_snapshot(), perception_context=context, timestamp_ms=1_124)
        self.assertIsNone(stale_world["perception_context"])
        self.assertEqual(stale_world["perception_summaries"], [])
        self.assertEqual(stale_world["detected_objects"], [])
        self.assertEqual(stale_world["source_health"]["perception_context_status"], "unavailable_or_stale")

        malformed_context = make_perception_context()
        malformed_context["sources"][0]["schema"] = "wrong"
        malformed_world = build_world_state_v1(
            make_snapshot(),
            perception_context=malformed_context,
            timestamp_ms=123,
        )
        self.assertIsNone(malformed_world["perception_context"])


if __name__ == "__main__":
    unittest.main()
