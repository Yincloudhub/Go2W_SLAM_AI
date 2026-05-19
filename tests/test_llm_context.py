import unittest
from pathlib import Path

from edge_autonomy.llm_context import build_planner_context, plan_to_slam_command, simulate_local_llm_plan
from edge_autonomy.map_registry import MapRegistry


REGISTRY_PATH = Path(__file__).resolve().parents[1] / "configs" / "maps" / "go2w_map_registry.example.json"


def make_snapshot(*, x: float = 3.26, y: float = -2.27, ok: bool = True) -> dict:
    return {
        "timestamp_ms": 123,
        "expected_map_id": "test_current_main",
        "expected_map_path": "/home/unitree/test.pcd",
        "health_status": "ok" if ok else "failed",
        "localization_status": "localized_or_tracking" if ok else "relocation_odom_missing",
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

    def test_simulate_plan_holds_when_already_near_target(self) -> None:
        registry = MapRegistry.from_file(REGISTRY_PATH)
        context = build_planner_context(make_snapshot(x=1.15, y=-0.15), registry, user_command="回到起点")
        plan = simulate_local_llm_plan(context, registry)

        self.assertEqual(plan["mode"], "safe_hold")
        self.assertEqual(plan["steps"][0]["tool"], "hold_position")
        self.assertEqual(plan["steps"][0]["arguments"]["target_node"], "701_entrance_hallway_mid")

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


if __name__ == "__main__":
    unittest.main()
