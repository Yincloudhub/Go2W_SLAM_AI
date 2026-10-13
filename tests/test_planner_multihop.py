from __future__ import annotations

import unittest
from unittest.mock import patch

from edge_autonomy import local_llm_planner
from edge_autonomy.local_llm_planner import (
    build_lightweight_planner_context,
    build_lightweight_planner_prompt,
    deterministic_intent_from_context,
)


def map701_context() -> dict:
    return {
        "user_command": "去赵博办公室门口",
        "world_state_summary": {
            "robot": {
                "localized": True,
                "nearest_node": {"node_id": "wp_60497f"},
            },
            "slam": {"health_status": "ok"},
            "map": {"map_id": "go2w_real_site"},
            "topology": {
                "available_nodes": [
                    {
                        "node_id": "wp_60497f",
                        "name": "尹思园工位",
                        "distance_from_robot_m": 0.2,
                        "tags": ["live_verified"],
                        "aliases": ["尹思园"],
                    },
                    {
                        "node_id": "wp_b738c7",
                        "name": "701中心过道端点",
                        "distance_from_robot_m": 1.7,
                        "tags": ["live_verified"],
                        "aliases": [],
                    },
                    {
                        "node_id": "wp_bbcb4c",
                        "name": "赵博办公室门口",
                        "distance_from_robot_m": 5.8,
                        "tags": ["live_verified", "photo_required"],
                        "aliases": ["赵博"],
                    },
                ]
            },
        },
        "map_id": "go2w_real_site",
        "coarse_map": "··wp604··\n██wpb73··\n··wpbbc··",
        "coarse_map_nodes": {
            "wp_60497f": {"grid_x": 2, "grid_y": 0},
            "wp_b738c7": {"grid_x": 2, "grid_y": 1},
            "wp_bbcb4c": {"grid_x": 2, "grid_y": 2},
        },
        "navigation_hint": "Use waypoint_connectivity.adjacency for multi-hop routing.",
    }


def coarse_map() -> dict:
    return {
        "grid_string": "··wp604··\n██wpb73··\n··wpbbc··",
        "nodes": {
            "wp_60497f": {"grid_x": 2, "grid_y": 0},
            "wp_b738c7": {"grid_x": 2, "grid_y": 1},
            "wp_bbcb4c": {"grid_x": 2, "grid_y": 2},
        },
        "grid_size": 10,
    }


class PlannerMultihopTests(unittest.TestCase):
    def test_lightweight_context_keeps_map701_multihop_fields(self) -> None:
        with patch.object(local_llm_planner, "_COARSE_MAP", coarse_map()):
            context = build_lightweight_planner_context(map701_context())

        self.assertEqual(context["requested_target_guess"], "wp_bbcb4c")
        self.assertEqual(context["nearest_node"], "wp_60497f")
        self.assertTrue(context["coarse_map"])
        self.assertTrue(context["navigation_hint"])
        self.assertEqual(len(context["coarse_map_nodes"]), 3)

    def test_lightweight_prompt_contains_multihop_map_context(self) -> None:
        with patch.object(local_llm_planner, "_COARSE_MAP", coarse_map()):
            prompt = build_lightweight_planner_prompt(map701_context())

        self.assertIn("multi-hop", prompt.lower())
        self.assertIn("wp_bbcb4c", prompt)
        self.assertIn("coarse_map", prompt)

    def test_deterministic_intent_matches_map701_target_alias(self) -> None:
        intent = deterministic_intent_from_context(map701_context())

        self.assertIsNotNone(intent)
        self.assertEqual(intent["mode"], "mapped_navigation")
        self.assertEqual(intent["target_node"], "wp_bbcb4c")


if __name__ == "__main__":
    unittest.main()
