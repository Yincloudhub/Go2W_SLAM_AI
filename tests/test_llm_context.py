from __future__ import annotations

import unittest
from pathlib import Path

from edge_autonomy.llm_context import build_planner_context, plan_to_slam_command, simulate_local_llm_plan
from edge_autonomy.map_registry import MapRegistry


REGISTRY_PATH = Path(__file__).resolve().parents[1] / "configs" / "maps" / "go2w_map_registry.example.json"


def make_snapshot(
    *,
    x: float = 3.26,
    y: float = -2.27,
    ok: bool = True,
    health_status: str | None = None,
    localization_status: str | None = None,
) -> dict:
    return {
        "timestamp_ms": 123,
        "expected_map_id": "test_current_main",
        "expected_map_path": "/home/unitree/test.pcd",
        "health_status": health_status or ("ok" if ok else "failed"),
        "localization_status": localization_status or ("localized_or_tracking" if ok else "relocation_odom_missing"),
        "processes": {"unitree_slam": ok, "xt16_driver": ok},
        "lidar_state": {"alive": ok, "cloud_frequency_hz": 15.0, "cloud_size": 56000, "error_state": 0},
        "live_pointcloud": {"alive": ok, "topic": "/unitree/slam_lidar/points", "width": 56000},
        "relocation_odom": {"alive": ok, "x": x, "y": y, "z": 0.0, "yaw": -1.5},
    }


class LlmContextTests(unittest.TestCase):
    def test_build_planner_context_nearest_node(self) -> None:
        registry = MapRegistry.from_file(REGISTRY_PATH)
        context = build_planner_context(make_snapshot(), registry, user_command="回到起点")

        self.assertEqual(context["world_state_summary"]["map"]["map_id"], "test_current_main")
        self.assertTrue(context["world_state_summary"]["robot"]["localized"])
        self.assertEqual(context["world_state_summary"]["robot"]["nearest_node"]["node_id"], "nie_guoli_office_front")
        self.assertIn("navigate_to_verified_node", context["world_state_summary"]["allowed_actions"])
        self.assertEqual(context["capability_contract"]["planning_style"], "capability_bounded_task_planning")
        self.assertIn("relative_motion", {item["name"] for item in context["capability_contract"]["not_wired"]})
        for edge in context["world_state_summary"]["topology"]["edges"]:
            self.assertFalse(edge["distance_verified"])
            self.assertIsNone(edge["expected_distance_m"])

    def test_capture_keyframe_capability_reflects_configured_command(self) -> None:
        registry = MapRegistry.from_file(REGISTRY_PATH)
        snapshot = make_snapshot()
        snapshot["capture_command_configured"] = True
        context = build_planner_context(snapshot, registry, user_command="photo")
        capture = next(
            item for item in context["capability_contract"]["conditional"] if item["name"] == "capture_keyframe"
        )

        self.assertTrue(capture["available"])
        self.assertEqual(capture["status"], "ready")

    def test_simulate_plan_return_to_start(self) -> None:
        registry = MapRegistry.from_file(REGISTRY_PATH)
        context = build_planner_context(make_snapshot(), registry, user_command="回到起点")
        plan = simulate_local_llm_plan(context, registry)
        command = plan_to_slam_command(plan, registry)

        self.assertEqual(plan["mode"], "mapped_navigation")
        self.assertEqual(plan["steps"][1]["arguments"]["target_node"], "701_entrance_hallway_mid")
        self.assertIsNotNone(command)
        assert command is not None
        self.assertEqual(command["target_node"], "701_entrance_hallway_mid")
        self.assertEqual(command["target_pose"]["speed"], 0.45)

    def test_simulate_plan_to_wp1(self) -> None:
        registry = MapRegistry.from_file(REGISTRY_PATH)
        context = build_planner_context(make_snapshot(x=1.15, y=-0.15), registry, user_command="去 wp_1")
        plan = simulate_local_llm_plan(context, registry)

        self.assertEqual(plan["mode"], "mapped_navigation")
        self.assertEqual(plan["steps"][1]["arguments"]["target_node"], "nie_guoli_office_front")

    def test_simulate_plan_accepts_gateway_localized_status(self) -> None:
        registry = MapRegistry.from_file(REGISTRY_PATH)
        context = build_planner_context(
            make_snapshot(x=1.15, y=-0.15, localization_status="localized"),
            registry,
            user_command="nie_guoli_office_front",
        )
        plan = simulate_local_llm_plan(context, registry)

        self.assertEqual(plan["mode"], "mapped_navigation")
        self.assertEqual(plan["steps"][1]["arguments"]["target_node"], "nie_guoli_office_front")

    def test_simulate_plan_holds_when_already_near_target(self) -> None:
        registry = MapRegistry.from_file(REGISTRY_PATH)
        context = build_planner_context(make_snapshot(x=1.15, y=-0.15), registry, user_command="回到起点")
        plan = simulate_local_llm_plan(context, registry)

        self.assertEqual(plan["mode"], "safe_hold")
        self.assertEqual(plan["steps"][0]["tool"], "hold_position")
        self.assertEqual(plan["steps"][0]["arguments"]["target_node"], "701_entrance_hallway_mid")

    def test_slam_command_allows_global_speed_and_mode_override(self) -> None:
        registry = MapRegistry.from_file(REGISTRY_PATH)
        plan = {
            "mode": "mapped_navigation",
            "steps": [
                {
                    "tool": "create_navigation_subgoal",
                    "arguments": {
                        "map_id": "test_current_main",
                        "target_node": "nie_guoli_office_front",
                        "speed_mps": 0.45,
                        "mode": 0,
                    },
                }
            ],
        }
        command = plan_to_slam_command(plan, registry, speed=0.2, mode=1)

        self.assertIsNotNone(command)
        assert command is not None
        self.assertEqual(command["target_pose"]["speed"], 0.2)
        self.assertEqual(command["target_pose"]["mode"], 1)

    def test_simulate_stop_plan(self) -> None:
        registry = MapRegistry.from_file(REGISTRY_PATH)
        context = build_planner_context(make_snapshot(), registry, user_command="停下别动")
        plan = simulate_local_llm_plan(context, registry)

        self.assertEqual(plan["mode"], "safe_hold")
        self.assertEqual(plan["steps"][0]["tool"], "hold_position")

    def test_simulate_plan_blocks_when_slam_unhealthy(self) -> None:
        registry = MapRegistry.from_file(REGISTRY_PATH)
        context = build_planner_context(make_snapshot(ok=False), registry, user_command="去 wp_1")
        plan = simulate_local_llm_plan(context, registry)

        self.assertEqual(plan["mode"], "safe_hold")
        self.assertTrue(plan["requires_human_ack"])
        self.assertIsNone(plan_to_slam_command(plan, registry))

    def test_simulate_relative_motion_requests_human_confirm(self) -> None:
        registry = MapRegistry.from_file(REGISTRY_PATH)
        context = build_planner_context(make_snapshot(), registry, user_command="前进十米去拍照")
        plan = simulate_local_llm_plan(context, registry)

        self.assertEqual(plan["mode"], "human_confirm")
        self.assertEqual(plan["steps"][0]["tool"], "relative_motion_preview")
        self.assertEqual(plan["steps"][0]["arguments"]["requested_distance_m"], 10.0)
        self.assertTrue(plan["steps"][0]["arguments"]["capture_requested"])
        self.assertFalse(plan["steps"][0]["arguments"]["real_execution"])
        self.assertEqual(plan["steps"][1]["tool"], "request_human_confirm")
        self.assertEqual(plan["steps"][1]["arguments"]["missing_capability"], "relative_motion")
        self.assertIsNone(plan_to_slam_command(plan, registry))


if __name__ == "__main__":
    unittest.main()
