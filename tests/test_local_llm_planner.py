import unittest

from edge_autonomy.local_llm_planner import (
    apply_context_policy_overrides,
    extract_json_object,
    validate_context_policy,
    validate_execution_contract,
    validate_local_llm_plan,
)


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


if __name__ == "__main__":
    unittest.main()
