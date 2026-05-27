import unittest

from edge_autonomy.local_llm_planner import (
    apply_context_policy_overrides,
    build_lightweight_planner_context,
    build_task_queue_from_context,
    deterministic_intent_from_context,
    extract_json_object,
    task_queue_to_plan,
    validate_context_policy,
    validate_execution_contract,
    validate_local_llm_plan,
)
from edge_autonomy.task_queue import validate_task_queue


def make_plan() -> dict:
    return {
        "plan_id": "p1",
        "mode": "mapped_navigation",
        "confidence": 0.9,
        "reason": "safe mapped navigation",
        "steps": [
            {
                "step_id": "nav_1",
                "tool": "create_navigation_subgoal",
                "arguments": {"map_id": "test_current_main", "target_node": "nie_guoli_office_front"},
            },
            {
                "step_id": "wait_1",
                "tool": "wait_until",
                "arguments": {"condition": "navigation_finished"},
            },
        ],
        "communication_policy": {
            "mode": "semantic_only",
            "send": ["task_state", "navigation_feedback", "world_state_summary"],
            "drop": ["raw_video", "dense_pointcloud"],
            "reason": "weak link",
        },
        "requires_human_ack": False,
    }


class LocalLlmPlannerTests(unittest.TestCase):
    def test_extract_prefers_plan_object(self) -> None:
        text = '{"copied_input": true}\n{"plan_id":"p1","mode":"safe_hold","confidence":0.8,"reason":"stop","steps":[{"step_id":"h","tool":"hold_position","arguments":{}}],"communication_policy":{"mode":"normal","send":["task_state"],"drop":[]},"requires_human_ack":false}'

        plan = extract_json_object(text)

        self.assertEqual(plan["plan_id"], "p1")
        self.assertEqual(plan["mode"], "safe_hold")

    def test_validate_plan_and_execution_contract(self) -> None:
        plan = make_plan()

        validate_local_llm_plan(plan)
        validate_execution_contract(plan)

    def test_execution_contract_rejects_target_node_id(self) -> None:
        plan = make_plan()
        plan["steps"][0]["arguments"] = {"map_id": "test_current_main", "target_node_id": "nie_guoli_office_front"}

        validate_local_llm_plan(plan)
        with self.assertRaises(ValueError):
            validate_execution_contract(plan)

    def test_context_policy_forces_hold_when_already_near_target(self) -> None:
        plan = make_plan()
        context = {
            "user_command": "go to office",
            "world_state_summary": {
                "map": {"map_id": "test_current_main"},
                "robot": {"localized": True},
                "slam": {"health_status": "ok"},
                "topology": {
                    "available_nodes": [
                        {
                            "node_id": "nie_guoli_office_front",
                            "name": "office",
                            "aliases": ["office"],
                            "tags": [],
                            "distance_from_robot_m": 0.1,
                        }
                    ]
                },
            },
        }

        fixed = apply_context_policy_overrides(plan, context)

        self.assertEqual(fixed["mode"], "safe_hold")
        self.assertEqual(fixed["steps"][0]["tool"], "hold_position")
        validate_local_llm_plan(fixed)
        validate_context_policy(fixed, context)

    def test_target_matching_uses_command_order_not_registry_order(self) -> None:
        context = {
            "user_command": "go room701 then return station",
            "world_state_summary": {
                "map": {"map_id": "test_current_main"},
                "robot": {"localized": True},
                "slam": {"health_status": "ok"},
                "topology": {
                    "available_nodes": [
                        {
                            "node_id": "station",
                            "name": "station",
                            "aliases": ["station"],
                            "tags": [],
                            "distance_from_robot_m": 0.05,
                        },
                        {
                            "node_id": "room701",
                            "name": "room701",
                            "aliases": ["room701"],
                            "tags": ["photo_required"],
                            "distance_from_robot_m": 5.0,
                        },
                    ]
                },
            },
        }

        light = build_lightweight_planner_context(context)

        self.assertEqual(light["requested_target_guess"], "room701")
        self.assertTrue(light["multi_target"])
        self.assertEqual([item["node_id"] for item in light["matched_targets"]], ["room701", "station"])

    def test_multi_target_command_builds_sequential_task_queue(self) -> None:
        context = {
            "user_command": "go room701 then return station",
            "world_state_summary": {
                "map": {"map_id": "test_current_main"},
                "robot": {"localized": True},
                "slam": {"health_status": "ok"},
                "topology": {
                    "available_nodes": [
                        {
                            "node_id": "station",
                            "name": "station",
                            "aliases": ["station"],
                            "tags": [],
                            "distance_from_robot_m": 0.05,
                        },
                        {
                            "node_id": "room701",
                            "name": "room701",
                            "aliases": ["room701"],
                            "tags": ["photo_required"],
                            "distance_from_robot_m": 5.0,
                        },
                    ]
                },
            },
        }

        task_queue = build_task_queue_from_context(context)
        self.assertIsNotNone(task_queue)
        validate_task_queue(task_queue)
        plan = task_queue_to_plan(task_queue, context)
        fixed = apply_context_policy_overrides(plan, context)
        intent = deterministic_intent_from_context(context)

        nav_targets = [
            step["arguments"]["target_node"]
            for step in fixed["steps"]
            if step["tool"] == "create_navigation_subgoal"
        ]
        self.assertEqual(fixed["mode"], "mapped_navigation")
        self.assertEqual(nav_targets, ["room701", "station"])
        self.assertIn("capture_keyframe", [step["tool"] for step in fixed["steps"]])
        self.assertEqual(intent["mode"], "mapped_navigation")
        validate_local_llm_plan(fixed)
        validate_execution_contract(fixed)
        validate_context_policy(fixed, context)


if __name__ == "__main__":
    unittest.main()
