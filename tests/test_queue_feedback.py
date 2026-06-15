import json
import queue
import tempfile
import unittest
from unittest.mock import patch
from pathlib import Path
from types import SimpleNamespace

from edge_autonomy.map_registry import MapRegistry
from scripts.run_robot_closed_loop import (
    PersistentNavigationSession,
    gateway_rejection_reason,
    generate_mobility_strategy_proposal,
    generate_llm_feedback_result,
    navigation_monitor_budget_s,
    navigation_motion_progressed,
    normalize_unitree_navigation_speed,
    operator_feedback_message,
    run_supervised_navigation_session,
    supervised_departure_decision,
    supervised_reposition_decision,
    wait_for_arrival,
)
from scripts.run_robot_closed_loop import execute_task_queue


REGISTRY_PATH = Path(__file__).resolve().parents[1] / "configs" / "maps" / "go2w_map_registry.example.json"


class FailingBackend:
    def generate(self, prompt: str, *, system_prompt: str, max_tokens: int, timeout_s: int) -> str:
        raise AssertionError("live LLM should not run for progress feedback by default")

class MobilityBackend:
    def generate(self, prompt: str, *, system_prompt: str, max_tokens: int, timeout_s: int) -> str:
        return json.dumps(
            {
                "action": "reposition",
                "direction": "left",
                "distance_m": 0.3,
                "confidence": 0.9,
                "reason": "left side creates turning room",
            }
        )

class EchoingMobilityBackend:
    def generate(self, prompt: str, *, system_prompt: str, max_tokens: int, timeout_s: int) -> str:
        return (
            prompt
            + "\n"
            + json.dumps(
                {
                    "action": "reposition",
                    "direction": "left",
                    "distance_m": 0.3,
                    "confidence": 0.9,
                    "reason": "left creates turning room",
                }
            )
        )


class QueueFeedbackTests(unittest.TestCase):
    def test_initial_turn_is_owned_by_native_navigation(self) -> None:
        state = {
            "world_state": {
                "current_pose": {"pose": {"x": 0.0, "y": 0.0, "yaw": 0.0}},
                "local_obstacle": {
                    "front_clearance_m": 2.0,
                    "left_clearance_m": 0.1,
                    "right_clearance_m": 0.1,
                    "rear_clearance_m": 0.1,
                    "supervised_release": {"active": True},
                },
            }
        }

        decision = supervised_departure_decision(
            state,
            {"target_pose": {"x": 2.0, "y": 1.0}},
        )

        self.assertFalse(decision["required"])
        self.assertFalse(decision["available"])
        self.assertTrue(decision["native_navigation_first"])
        self.assertEqual(decision["trigger"], "not_required")

    def test_aligned_target_does_not_request_departure(self) -> None:
        state = {
            "world_state": {
                "current_pose": {"pose": {"x": 0.0, "y": 0.0, "yaw": 0.0}},
                "local_obstacle": {
                    "front_clearance_m": 2.0,
                    "left_clearance_m": 1.0,
                    "right_clearance_m": 1.0,
                    "rear_clearance_m": 1.0,
                    "supervised_release": {"active": True},
                },
            }
        }

        decision = supervised_departure_decision(
            state,
            {"target_pose": {"x": 2.0, "y": 0.0}},
        )

        self.assertFalse(decision["required"])

    def test_side_rear_proximity_does_not_preempt_native_navigation(self) -> None:
        state = {
            "world_state": {
                "current_pose": {"pose": {"x": 0.0, "y": 0.0, "yaw": 0.0}},
                "local_obstacle": {
                    "front_clearance_m": 2.0,
                    "left_clearance_m": 1.0,
                    "right_clearance_m": 0.61,
                    "rear_clearance_m": 0.37,
                    "supervised_release": {"active": True},
                },
            }
        }

        decision = supervised_departure_decision(
            state,
            {"target_pose": {"x": 2.0, "y": 0.0}},
        )

        self.assertFalse(decision["required"])
        self.assertTrue(decision["native_navigation_first"])
        self.assertEqual(decision["trigger"], "not_required")

    def test_side_rear_advisory_does_not_depart_when_target_is_close(self) -> None:
        state = {
            "world_state": {
                "current_pose": {"pose": {"x": 0.0, "y": 0.0, "yaw": 0.0}},
                "local_obstacle": {
                    "front_clearance_m": 2.0,
                    "left_clearance_m": 1.0,
                    "right_clearance_m": 0.40,
                    "rear_clearance_m": 0.40,
                    "supervised_release": {"active": True},
                },
            }
        }

        decision = supervised_departure_decision(
            state,
            {"target_pose": {"x": 0.70, "y": 0.0}},
        )

        self.assertFalse(decision["required"])
        self.assertFalse(decision["available"])
        self.assertEqual(decision["trigger"], "not_required")

    def test_explicit_native_navigation_recovery_builds_bounded_candidate(self) -> None:
        state = {
            "world_state": {
                "current_pose": {"pose": {"x": 0.0, "y": 0.0, "yaw": 0.0}},
                "local_obstacle": {
                    "front_clearance_m": 0.9,
                    "left_clearance_m": 0.1,
                    "right_clearance_m": 0.1,
                    "rear_clearance_m": 1.0,
                    "supervised_release": {"active": True},
                },
            }
        }

        decision = supervised_reposition_decision(
            state,
            {
                "target_pose": {"x": 2.0, "y": 1.0},
                "recovery_requested": True,
            },
        )

        self.assertTrue(decision["required"])
        self.assertTrue(decision["available"])
        self.assertEqual(decision["direction"], "forward")
        self.assertEqual(decision["distance_m"], 0.4)

    def test_field_state_does_not_use_fixed_turn_clearance_gate(self) -> None:
        state = {
            "world_state": {
                "current_pose": {
                    "pose": {"x": -1.05, "y": -0.57, "yaw": 1.53}
                },
                "local_obstacle": {
                    "front_clearance_m": 0.787,
                    "left_clearance_m": 0.527,
                    "right_clearance_m": 0.011,
                    "rear_clearance_m": 0.030,
                    "supervised_release": {"active": True},
                },
            }
        }

        decision = supervised_reposition_decision(
            state,
            {"target_pose": {"x": 1.597, "y": 0.359}},
        )

        self.assertFalse(decision["required"])
        self.assertTrue(decision["native_navigation_first"])

    def test_live_mobility_strategy_selects_only_candidate_action(self) -> None:
        proposal = generate_mobility_strategy_proposal(
            {
                "required": True,
                "trigger": "native_navigation_recovery_requested",
                "bearing_error_rad": 1.0,
                "clearance_m": {
                    "front": 0.9,
                    "left": 2.0,
                    "right": 0.1,
                    "rear": 1.0,
                },
                "candidates": [
                    {
                        "direction": "left",
                        "distance_m": 0.5,
                        "clearance_m": 2.0,
                        "turning_relief_m": 0.25,
                        "goal_progress_m": 0.0,
                    }
                ],
            },
            {
                "target_node": "wp_a",
                "target_pose": {"x": 2.0, "y": 0.0},
            },
            SimpleNamespace(
                mobility_strategy_mode="live",
                local_command="unused",
                mobility_strategy_max_tokens=96,
                mobility_strategy_timeout_s=2,
            ),
            backend=MobilityBackend(),
        )

        self.assertEqual(proposal["source"], "local_llm")
        self.assertEqual(proposal["action"], "reposition")
        self.assertEqual(proposal["direction"], "left")

    def test_mobility_strategy_uses_last_json_after_echoed_prompt(self) -> None:
        analysis = {
            "required": True,
            "trigger": "native_navigation_recovery_requested",
            "bearing_error_rad": 1.0,
            "clearance_m": {"front": 1.0, "left": 2.0, "right": 0.1, "rear": 0.5},
            "candidates": [
                {
                    "direction": "left",
                    "distance_m": 0.5,
                    "clearance_m": 2.0,
                    "turning_relief_m": 0.25,
                    "goal_progress_m": 0.0,
                }
            ],
        }
        proposal = generate_mobility_strategy_proposal(
            analysis,
            {"target_node": "wp_a", "target_pose": {"x": 2.0, "y": 0.0}},
            SimpleNamespace(
                mobility_strategy_mode="live",
                local_command="unused",
                mobility_strategy_max_tokens=160,
                mobility_strategy_timeout_s=20,
            ),
            backend=EchoingMobilityBackend(),
        )

        self.assertEqual(proposal["source"], "local_llm")
        self.assertEqual(proposal["direction"], "left")

    def test_gateway_rejection_preserves_safety_reason(self) -> None:
        reason = gateway_rejection_reason(
            {
                "accepted": False,
                "reason": "safety_blocked",
                "safety": {"allow_navigation": False, "reason": "local_obstacle_not_fresh"},
            }
        )

        self.assertEqual(reason, "safety_blocked:local_obstacle_not_fresh")

    def test_persistent_session_matches_responses_by_request_id(self) -> None:
        class FakeStdin:
            def __init__(self):
                self.payload = ""

            def write(self, payload):
                self.payload += payload

            def flush(self):
                return None

        class FakeProcess:
            def __init__(self):
                self.stdin = FakeStdin()

            def poll(self):
                return None

        session = PersistentNavigationSession.__new__(PersistentNavigationSession)
        session.timeout_s = 1
        session.process = FakeProcess()
        session.messages = queue.Queue()
        session.session_token = "token"
        session.lease_timeout_ms = 2000
        session._request_sequence = 0
        session.active = False
        session.async_events = []
        session.messages.put(
            {
                "accepted": False,
                "action": "get_world_state",
                "request_id": "stale-request",
            }
        )
        session.messages.put(
            {
                "accepted": True,
                "action": "get_world_state",
                "request_id": "session-request-1",
            }
        )

        result = session.command({"action": "get_world_state"})

        self.assertTrue(result["accepted"])
        self.assertEqual(result["request_id"], "session-request-1")
        self.assertEqual(session.async_events[0]["request_id"], "stale-request")
        sent = json.loads(session.process.stdin.payload)
        self.assertEqual(sent["request_id"], "session-request-1")

    def test_supervised_navigation_reuses_persistent_session(self) -> None:
        commands = []

        class FakeSession:
            def __enter__(self):
                return self

            def __exit__(self, exc_type, exc, traceback):
                return None

            def command(self, command):
                commands.append(command)
                return {"accepted": True, "action": command["action"]}

        args = SimpleNamespace(
            gateway_client="gateway",
            network_interface="eth0",
            timeout_s=3,
            gateway_startup_wait_s=0.0,
        )
        command = {
            "action": "navigate_to_pose",
            "operator_ack": True,
            "target_node": "wp_a",
            "target_pose": {"name": "wp_a", "x": 1.0, "y": 2.0},
        }
        arrival = {"arrived": True, "paused": True}

        with patch(
            "scripts.run_robot_closed_loop.PersistentNavigationSession",
            return_value=FakeSession(),
        ):
            with patch(
                "scripts.run_robot_closed_loop.wait_for_arrival",
                return_value=arrival,
            ) as wait:
                result, observed_arrival = run_supervised_navigation_session(
                    command,
                    args,
                    target_name="A",
                )

        self.assertTrue(result["accepted"])
        self.assertEqual(observed_arrival, arrival)
        self.assertEqual(commands, [command])
        self.assertIs(wait.call_args.kwargs["session"].__class__, FakeSession)

    def test_navigation_monitor_budget_uses_distance_and_command_speed(self) -> None:
        args = SimpleNamespace(
            arrival_monitor_s=25.0,
            navigation_monitor_travel_factor=1.8,
            navigation_monitor_startup_margin_s=15.0,
        )
        command = {
            "target_pose": {
                "x": 1.0,
                "y": 2.0,
                "speed": 0.1,
            }
        }

        budget = navigation_monitor_budget_s(command, 2.85, args)

        self.assertAlmostEqual(budget, 66.3)

    def test_navigation_motion_progress_accepts_turn_in_place(self) -> None:
        self.assertTrue(
            navigation_motion_progressed(
                {"x": 0.0, "y": 0.0, "yaw": 0.0},
                {"x": 0.0, "y": 0.0, "yaw": 0.1},
                distance_threshold_m=0.03,
                yaw_threshold_rad=0.08,
            )
        )

    def test_mode_zero_navigation_speed_uses_unitree_minimum(self) -> None:
        self.assertEqual(normalize_unitree_navigation_speed(0.1, 0), 0.2)
        self.assertEqual(normalize_unitree_navigation_speed(0.3, 0), 0.3)
        self.assertEqual(normalize_unitree_navigation_speed(0.1, 1), 0.1)

    def test_navigation_stall_requests_pause(self) -> None:
        state = {
            "accepted": True,
            "world_state": {
                "safety": {"allow_navigation": True, "reason": "ok"},
                "slam_health": {
                    "status": "ok",
                    "slam_alive": True,
                    "localization_alive": True,
                },
                "localization": {
                    "status": "localized",
                    "confidence": 0.9,
                    "pose_age_ms": 100,
                },
                "current_pose": {
                    "pose": {"x": 0.0, "y": 0.0, "yaw": 0.0}
                },
                "navigation": {"state": "running"},
            },
        }

        class FakeSession:
            def heartbeat(self):
                return {"accepted": True}

            def command(self, command):
                if command["action"] == "pause_navigation":
                    return {"accepted": True}
                return state

        args = SimpleNamespace(
            timeout_s=1,
            gateway_error_limit=1,
            slam_poll_interval_s=0.1,
            arrival_monitor_interval_s=0.1,
            ui_refresh_interval_s=1.0,
            operator_feedback_interval_s=1.0,
            llm_feedback_interval_s=1.0,
            max_arrival_samples=10,
            max_feedback_events=10,
            max_llm_feedback_events=10,
            arrival_monitor_s=1.0,
            arrival_distance_m=0.25,
            arrival_confirm_samples=2,
            llm_feedback_mode="off",
            navigation_stall_s=0.12,
            navigation_progress_distance_m=0.03,
            navigation_progress_yaw_rad=0.08,
        )

        result = wait_for_arrival(
            {
                "action": "navigate_to_pose",
                "target_node": "wp_a",
                "target_pose": {"x": 1.0, "y": 0.0, "speed": 0.1},
            },
            args,
            target_name="A",
            session=FakeSession(),
        )

        self.assertEqual(result["reason"], "native_navigation_no_progress")
        self.assertTrue(result["paused"])

    def test_dry_run_queue_is_not_marked_completed(self) -> None:
        registry = MapRegistry.from_file(REGISTRY_PATH)
        task_queue = {
            "queue_id": "q-dry",
            "mode": "sequential",
            "status": "planned",
            "source": "semantic_topology",
            "targets": ["nie_guoli_office_front"],
            "steps": [
                {
                    "task_id": "task_1",
                    "action": "navigate",
                    "status": "pending",
                    "target_node": "nie_guoli_office_front",
                    "target_name": "office",
                }
            ],
            "communication_policy": {
                "mode": "normal",
                "send": ["task_state", "navigation_feedback", "world_state_summary"],
                "drop": [],
            },
        }
        args = SimpleNamespace(
            execute=False,
            skip_gateway_check=True,
            map_id="test_current_main",
            nav_mode=None,
            arrival_monitor_interval_s=1.0,
        )

        result = execute_task_queue(task_queue, registry=registry, args=args, nav_speed=None)

        self.assertTrue(result["dry_run"])
        self.assertFalse(result["executed"])
        self.assertFalse(result["completed"])
        self.assertIsNone(result["failed_step"])
        self.assertEqual(result["events"][0]["status"], "dry_run")

    def test_gateway_network_loss_blocks_execution_but_not_dry_run_preview(self) -> None:
        registry = MapRegistry.from_file(REGISTRY_PATH)
        task_queue = {
            "queue_id": "q-network-loss",
            "mode": "sequential",
            "status": "planned",
            "source": "semantic_topology",
            "targets": ["nie_guoli_office_front"],
            "steps": [
                {
                    "task_id": "task_1",
                    "action": "navigate",
                    "status": "pending",
                    "target_node": "nie_guoli_office_front",
                    "target_name": "office",
                }
            ],
            "communication_policy": {
                "mode": "normal",
                "send": ["task_state", "navigation_feedback", "world_state_summary"],
                "drop": [],
            },
        }

        for execute, expected_status in ((False, "dry_run"), (True, "blocked")):
            with self.subTest(execute=execute):
                args = SimpleNamespace(
                    execute=execute,
                    skip_gateway_check=False,
                    map_id="test_current_main",
                    nav_mode=None,
                    arrival_monitor_interval_s=1.0,
                    gateway_client="gateway",
                    network_interface="eth0",
                    timeout_s=1,
                    gateway_startup_wait_s=0.0,
                )
                with patch(
                    "scripts.run_robot_closed_loop.run_gateway_command",
                    side_effect=RuntimeError("connection refused"),
                ):
                    result = execute_task_queue(
                        task_queue,
                        registry=registry,
                        args=args,
                        nav_speed=None,
                    )

                self.assertEqual(result["events"][0]["status"], expected_status)
                if execute:
                    self.assertIn("gateway_preflight_error", result["blocked_reason"])
                    self.assertFalse(result["completed"])
                else:
                    self.assertEqual(result["blocked_reason"], "")

    def test_execute_queue_blocks_unverified_topology_target_before_gateway(self) -> None:
        registry_data = {
            "version": 1,
            "default_map_id": "site",
            "maps": [
                {
                    "map_id": "site",
                    "name": "site",
                    "status": "real",
                    "pcd_path": "/tmp/site.pcd",
                    "topology_path": "/tmp/site.json",
                    "mapping_origin_anchor_id": "mapping_origin",
                    "relocalization_anchors": [
                        {
                            "anchor_id": "mapping_origin",
                            "name": "mapping_origin",
                            "status": "verified_test",
                            "pose": {"x": 0.0, "y": 0.0, "yaw": 0.0},
                        }
                    ],
                    "topology_nodes": [
                        {
                            "node_id": "wp_a",
                            "name": "A",
                            "tags": ["needs_calibration"],
                            "pose": {"x": 1.0, "y": 2.0, "yaw": 0.0},
                        }
                    ],
                }
            ],
        }
        task_queue = {
            "queue_id": "q-block",
            "mode": "sequential",
            "status": "planned",
            "source": "semantic_topology",
            "targets": ["wp_a"],
            "communication_policy": {
                "mode": "normal",
                "send": ["task_state", "navigation_feedback", "world_state_summary"],
                "drop": [],
            },
            "steps": [
                {
                    "task_id": "task_1",
                    "action": "navigate",
                    "status": "pending",
                    "target_node": "wp_a",
                    "target_name": "A",
                }
            ],
        }
        args = SimpleNamespace(
            execute=True,
            skip_gateway_check=False,
            map_id="site",
            nav_mode=None,
            arrival_monitor_interval_s=1.0,
        )

        with tempfile.TemporaryDirectory() as tmp:
            registry_path = Path(tmp) / "registry.json"
            registry_path.write_text(json.dumps(registry_data), encoding="utf-8")
            registry = MapRegistry.from_file(registry_path)

            result = execute_task_queue(task_queue, registry=registry, args=args, nav_speed=None)

        self.assertTrue(result["executed"])
        self.assertFalse(result["completed"])
        self.assertEqual(result["failed_step"], "task_1")
        self.assertIn("needs_calibration", result["blocked_reason"])
        self.assertEqual(result["events"][0]["status"], "blocked")
        self.assertNotIn("preflight", result["events"][0])

    def test_operator_feedback_message_is_ui_and_llm_ready(self) -> None:
        message = operator_feedback_message(
            "progress",
            "正在前往701门口",
            target_node="room_701_door",
            target_name="701门口",
            distance_m=1.25,
        )

        self.assertEqual(message["channel"], "operator_display")
        self.assertTrue(message["llm_surface"])
        self.assertEqual(message["target_name"], "701门口")
        self.assertEqual(message["distance_to_target_m"], 1.25)

    def test_template_llm_feedback_result_is_display_ready(self) -> None:
        request = {
            "phase": "arrived",
            "target_node": "room_701_door",
            "target_name": "701门口",
            "distance_to_target_m": 0.18,
        }
        result = generate_llm_feedback_result(request, SimpleNamespace(llm_feedback_mode="template"))

        self.assertIsNotNone(result)
        self.assertEqual(result["source"], "template")
        self.assertIn("已到达701门口", result["text"])
        self.assertTrue(result["llm_surface"])

    def test_live_llm_progress_is_deferred_by_default(self) -> None:
        request = {
            "phase": "progress",
            "target_node": "room_701_door",
            "target_name": "701门口",
            "distance_to_target_m": 1.25,
        }
        args = SimpleNamespace(
            llm_feedback_mode="live",
            llm_feedback_live_progress=False,
            local_command="unused",
            llm_feedback_system="system",
            llm_feedback_max_tokens=16,
            llm_feedback_timeout_s=1,
        )

        result = generate_llm_feedback_result(request, args, backend=FailingBackend())

        self.assertEqual(result["source"], "template_progress_budget")
        self.assertTrue(result["live_deferred"])

    def test_live_llm_queued_feedback_is_deferred(self) -> None:
        request = {
            "phase": "queued",
            "target_node": "room_701_door",
            "target_name": "701门口",
        }
        args = SimpleNamespace(
            llm_feedback_mode="live",
            llm_feedback_live_progress=True,
            local_command="unused",
            llm_feedback_system="system",
            llm_feedback_max_tokens=16,
            llm_feedback_timeout_s=1,
        )

        result = generate_llm_feedback_result(request, args, backend=FailingBackend())

        self.assertEqual(result["source"], "template_queue_budget")
        self.assertTrue(result["live_deferred"])

    def test_runtime_safety_failure_requests_pause(self) -> None:
        state = {
            "world_state": {
                "safety": {"allow_navigation": False, "reason": "local_obstacle_not_fresh"},
                "slam_health": {"status": "ok", "slam_alive": True, "localization_alive": True},
                "localization": {"status": "localized", "confidence": 0.9, "pose_age_ms": 100},
                "current_pose": {"pose": {"x": 0.0, "y": 0.0, "yaw": 0.0}},
                "local_obstacle": {"source": "lidar_pointcloud", "stale": True, "age_ms": 100},
            }
        }
        args = SimpleNamespace(
            gateway_client="gateway",
            network_interface="eth0",
            timeout_s=1,
            gateway_startup_wait_s=0.0,
            gateway_error_limit=1,
            slam_poll_interval_s=0.1,
            arrival_monitor_interval_s=0.1,
            ui_refresh_interval_s=0.1,
            operator_feedback_interval_s=1.0,
            llm_feedback_interval_s=1.0,
            max_arrival_samples=10,
            max_feedback_events=10,
            max_llm_feedback_events=10,
            arrival_monitor_s=1.0,
            arrival_distance_m=0.25,
            arrival_confirm_samples=2,
            llm_feedback_mode="off",
        )

        with patch("scripts.run_robot_closed_loop.run_gateway_command", side_effect=[state, {"accepted": True}]) as gateway:
            result = wait_for_arrival(
                {"target_node": "wp_a", "target_pose": {"x": 1.0, "y": 0.0}},
                args,
                target_name="A",
            )

        self.assertFalse(result["arrived"])
        self.assertTrue(result["paused"])
        self.assertEqual(gateway.call_args_list[1].args[0], {"action": "pause_navigation"})


if __name__ == "__main__":
    unittest.main()
