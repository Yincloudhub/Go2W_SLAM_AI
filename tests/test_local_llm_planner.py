import unittest

from edge_autonomy.local_llm_planner import (
    apply_context_policy_overrides,
    build_lightweight_planner_context,
    build_lightweight_planner_prompt,
    build_task_queue_from_context,
    deterministic_intent_from_context,
    extract_json_object,
    plan_to_task_queue,
    task_queue_to_plan,
    validate_context_policy,
    validate_execution_contract,
    validate_local_llm_plan,
    run_local_llm_planner,
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
    def test_lightweight_context_preserves_unified_perception_context(self) -> None:
        perception_context = {"schema": "go2w_perception_context_v1", "context_id": "pc-1"}
        light = build_lightweight_planner_context(
            {
                "user_command": "观察前方",
                "world_state_summary": {},
                "perception_context": perception_context,
            }
        )

        self.assertIs(light["perception_context"], perception_context)

    def test_extract_prefers_plan_object(self) -> None:
        text = '{"copied_input": true}\n{"plan_id":"p1","mode":"safe_hold","confidence":0.8,"reason":"stop","steps":[{"step_id":"h","tool":"hold_position","arguments":{}}],"communication_policy":{"mode":"normal","send":["task_state"],"drop":[]},"requires_human_ack":false}'

        plan = extract_json_object(text)

        self.assertEqual(plan["plan_id"], "p1")
        self.assertEqual(plan["mode"], "safe_hold")

    def test_validate_plan_and_execution_contract(self) -> None:
        plan = make_plan()

        validate_local_llm_plan(plan)
        validate_execution_contract(plan)

    def test_single_target_plan_is_normalized_to_task_queue(self) -> None:
        context = {
            "world_state_summary": {
                "topology": {
                    "available_nodes": [
                        {
                            "node_id": "nie_guoli_office_front",
                            "name": "office",
                            "tags": ["live_verified"],
                        }
                    ]
                }
            }
        }

        task_queue = plan_to_task_queue(make_plan(), context)

        validate_task_queue(task_queue)
        self.assertEqual(task_queue["targets"], ["nie_guoli_office_front"])
        self.assertEqual(task_queue["steps"][0]["action"], "navigate")
        self.assertEqual(task_queue["steps"][1]["action"], "wait_until")

    def test_human_confirmation_plan_is_still_a_task_queue(self) -> None:
        plan = make_plan()
        plan["mode"] = "human_confirm"
        plan["reason"] = "target is ambiguous"
        plan["steps"] = [
            {
                "step_id": "ask_1",
                "tool": "request_human_confirm",
                "arguments": {"reason": "target is ambiguous"},
            }
        ]

        task_queue = plan_to_task_queue(plan, {})

        validate_task_queue(task_queue)
        self.assertEqual(task_queue["targets"], [])
        self.assertEqual(task_queue["steps"][0]["action"], "ask_confirm")

    def test_policy_override_cannot_reuse_stale_navigation_queue(self) -> None:
        existing_queue = plan_to_task_queue(make_plan(), {})
        plan = make_plan()
        plan["mode"] = "human_confirm"
        plan["reason"] = "low battery requires confirmation"
        plan["steps"] = [
            {
                "step_id": "ask_1",
                "tool": "request_human_confirm",
                "arguments": {"reason": "low battery requires confirmation"},
            }
        ]

        task_queue = plan_to_task_queue(plan, {}, existing_queue=existing_queue)

        validate_task_queue(task_queue)
        self.assertEqual(task_queue["targets"], [])
        self.assertEqual(task_queue["steps"][0]["action"], "ask_confirm")

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
                            "tags": ["live_verified"],
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
            "capability_contract": {"not_wired": [{"name": "relative_motion"}]},
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
                            "tags": ["live_verified"],
                            "distance_from_robot_m": 0.05,
                        },
                        {
                            "node_id": "room701",
                            "name": "room701",
                            "aliases": ["room701"],
                            "tags": ["photo_required", "live_verified"],
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
        self.assertEqual(light["capability_contract"]["not_wired"][0]["name"], "relative_motion")

    def test_lightweight_prompt_mentions_capability_contract(self) -> None:
        context = {
            "user_command": "forward 10 meters and capture",
            "capability_contract": {"not_wired": [{"name": "relative_motion"}]},
            "world_state_summary": {
                "robot": {"localized": True},
                "slam": {"health_status": "ok"},
                "topology": {"available_nodes": []},
            },
        }

        prompt = build_lightweight_planner_prompt(context)

        self.assertIn("capability_contract", prompt)
        self.assertIn("relative_motion", prompt)
        self.assertIn("not_wired", prompt)

    def test_relative_motion_is_preview_only_even_if_model_tries_navigation(self) -> None:
        context = {
            "user_command": "forward 10 meters and capture",
            "capability_contract": {"not_wired": [{"name": "relative_motion"}]},
            "world_state_summary": {
                "map": {"map_id": "test_current_main"},
                "robot": {"localized": True},
                "slam": {"health_status": "ok"},
                "topology": {
                    "available_nodes": [
                        {
                            "node_id": "room701",
                            "name": "room701",
                            "aliases": ["room701"],
                            "tags": ["live_verified"],
                            "distance_from_robot_m": 5.0,
                        }
                    ]
                },
            },
        }
        plan = make_plan()

        fixed = apply_context_policy_overrides(plan, context)

        self.assertEqual(fixed["mode"], "human_confirm")
        self.assertEqual([step["tool"] for step in fixed["steps"]], ["relative_motion_preview", "request_human_confirm"])
        self.assertEqual(fixed["steps"][0]["arguments"]["requested_distance_m"], 10.0)
        self.assertFalse(fixed["steps"][0]["arguments"]["real_execution"])
        validate_local_llm_plan(fixed)
        validate_execution_contract(fixed)
        validate_context_policy(fixed, context)

    def test_relative_motion_hybrid_path_does_not_call_llm_backend(self) -> None:
        class RaisingBackend:
            def generate(self, prompt: str, *, system_prompt: str, max_tokens: int, timeout_s: int) -> str:
                raise AssertionError("relative motion preview should be deterministic")

        context = {
            "user_command": "前进十米去拍照",
            "capability_contract": {"not_wired": [{"name": "relative_motion"}]},
            "world_state_summary": {
                "map": {"map_id": "test_current_main"},
                "robot": {"localized": False},
                "slam": {"health_status": "failed"},
                "topology": {"available_nodes": []},
            },
        }

        result = run_local_llm_planner(context, RaisingBackend(), prompt_mode="hybrid")

        self.assertEqual(result.elapsed_s, 0.0)
        self.assertEqual(result.plan["mode"], "human_confirm")
        self.assertEqual(result.plan["steps"][0]["tool"], "relative_motion_preview")
        self.assertEqual(result.plan["steps"][0]["arguments"]["requested_distance_m"], 10.0)
        self.assertIsNone(result.task_queue)

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
                            "tags": ["live_verified"],
                            "distance_from_robot_m": 0.05,
                        },
                        {
                            "node_id": "room701",
                            "name": "room701",
                            "aliases": ["room701"],
                            "tags": ["photo_required", "live_verified"],
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

    def test_all_prompt_modes_force_safe_hold_when_localization_is_not_ready(self) -> None:
        context = {
            "user_command": "go to office",
            "world_state_summary": {
                "map": {"map_id": "test_current_main"},
                "robot": {"localized": False},
                "slam": {"health_status": "failed"},
                "topology": {
                    "available_nodes": [
                        {
                            "node_id": "nie_guoli_office_front",
                            "name": "office",
                            "aliases": ["office"],
                            "tags": ["live_verified"],
                        }
                    ]
                },
            },
        }

        fixed = apply_context_policy_overrides(make_plan(), context)

        self.assertEqual(fixed["mode"], "safe_hold")
        self.assertEqual([step["tool"] for step in fixed["steps"]], ["hold_position"])
        self.assertTrue(fixed["requires_human_ack"])
        validate_context_policy(fixed, context)

    def test_mapless_scout_is_rejected_as_not_wired(self) -> None:
        context = {
            "user_command": "explore the unknown room",
            "world_state_summary": {
                "map": {"map_id": "test_current_main"},
                "robot": {"localized": True},
                "slam": {"health_status": "ok"},
                "topology": {"available_nodes": []},
            },
        }
        plan = make_plan()
        plan["mode"] = "mapless_scout"
        plan["steps"] = [
            {
                "step_id": "scout_1",
                "tool": "start_mapless_scout",
                "arguments": {"max_distance_m": 3.0},
            }
        ]

        fixed = apply_context_policy_overrides(plan, context)

        self.assertEqual(fixed["mode"], "human_confirm")
        self.assertEqual([step["tool"] for step in fixed["steps"]], ["request_human_confirm"])
        self.assertIn("not wired", fixed["reason"])
        validate_context_policy(fixed, context)

    def test_shared_alias_is_ambiguous_without_sequence_language(self) -> None:
        context = {
            "user_command": "go lab",
            "world_state_summary": {
                "map": {"map_id": "test_current_main"},
                "robot": {"localized": True},
                "slam": {"health_status": "ok"},
                "topology": {
                    "available_nodes": [
                        {"node_id": "lab_a", "name": "Lab A", "aliases": ["lab"], "tags": ["live_verified"]},
                        {"node_id": "lab_b", "name": "Lab B", "aliases": ["lab"], "tags": ["live_verified"]},
                    ]
                },
            },
        }

        light = build_lightweight_planner_context(context)
        fixed = apply_context_policy_overrides(make_plan(), context)

        self.assertFalse(light["multi_target"])
        self.assertTrue(light["ambiguous_target"])
        self.assertIsNone(build_task_queue_from_context(context))
        self.assertEqual(fixed["mode"], "human_confirm")
        self.assertNotIn("create_navigation_subgoal", [step["tool"] for step in fixed["steps"]])
        validate_context_policy(fixed, context)

    def test_unverified_target_is_blocked_before_gateway(self) -> None:
        context = {
            "user_command": "go office",
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
                            "tags": ["needs_calibration"],
                        }
                    ]
                },
            },
        }

        fixed = apply_context_policy_overrides(make_plan(), context)

        self.assertEqual(fixed["mode"], "human_confirm")
        self.assertIn("not verified", fixed["reason"])
        self.assertNotIn("create_navigation_subgoal", [step["tool"] for step in fixed["steps"]])
        validate_context_policy(fixed, context)


if __name__ == "__main__":
    unittest.main()
