from __future__ import annotations

import importlib.util
import tempfile
import unittest
from contextlib import redirect_stdout
from io import StringIO
from pathlib import Path

from edge_autonomy.cloud_llm_planner import build_waypoint_connectivity
from scripts import waypoint_graph, waypoint_nav


REPO_ROOT = Path(__file__).resolve().parents[1]
REAL_REGISTRY = REPO_ROOT / "configs" / "maps" / "go2w_real_site_map_registry.json"
# Registry at HEAD: 12 waypoints, 11 edges (includes transition anchors +
# wp_832687 cluster). Direct edge wp_60497f <-> wp_e643bc exists.
EXPECTED_ROUTE = [
    "wp_e643bc",
    "wp_60497f",
]


def load_gateway_server():
    path = REPO_ROOT / "scripts" / "gateway_server.py"
    spec = importlib.util.spec_from_file_location("gateway_server_for_test", path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


class RegistryFirstRoutingTests(unittest.TestCase):
    def test_waypoint_graph_uses_declared_registry_edges(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            graph_path = Path(temp_dir) / "graph.json"
            graph = waypoint_graph.build_graph(
                map_id="go2w_real_site",
                registry_path=str(REAL_REGISTRY),
                graph_path=str(graph_path),
            )

        self.assertEqual(graph["edge_source"], "registry")
        self.assertEqual(len(graph["waypoints"]), 12)
        self.assertEqual(len(graph["edges"]), 11)
        self.assertEqual(
            waypoint_graph.bfs_route(graph, "wp_e643bc", "wp_60497f"),
            EXPECTED_ROUTE,
        )

    def test_waypoint_nav_dry_run_does_not_require_robot_processes(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            graph_path = Path(temp_dir) / "graph.json"
            output = StringIO()
            with redirect_stdout(output):
                result = waypoint_nav.main(
                    [
                        "wp_60497f",
                        "--dry-run",
                        "--from-node",
                        "wp_e643bc",
                        "--registry",
                        str(REAL_REGISTRY),
                        "--graph",
                        str(graph_path),
                    ]
                )

        self.assertEqual(result, 0)
        text = output.getvalue()
        self.assertIn("Dry-run complete", text)
        self.assertIn("wp_e643bc", text)
        self.assertIn("wp_60497f", text)

    def test_cloud_connectivity_prefers_registry_edges(self) -> None:
        connectivity = build_waypoint_connectivity(
            str(REAL_REGISTRY),
            map_id="go2w_real_site",
            force_refresh=True,
        )

        self.assertEqual(connectivity["edge_source"], "registry")
        self.assertEqual(connectivity["pcd_path"], "/home/unitree/maps/staging/map_701.pcd")
        self.assertIn("wp_2ae043", connectivity["adjacency"]["wp_e643bc"])
        self.assertIn("wp_60497f", connectivity["adjacency"]["wp_b738c7"])

    def test_gateway_server_resolves_map_path_from_registry(self) -> None:
        gateway_server = load_gateway_server()

        self.assertEqual(
            gateway_server.resolve_map_path(str(REAL_REGISTRY), "go2w_real_site", ""),
            "/home/unitree/maps/staging/map_701.pcd",
        )
        self.assertEqual(
            gateway_server.resolve_map_path(str(REAL_REGISTRY), "go2w_real_site", "/tmp/manual.pcd"),
            "/tmp/manual.pcd",
        )

    def test_planner_context_survives_unavailable_gateway(self) -> None:
        """Cloud LLM context must remain usable (map + connectivity) when the
        C++ Gateway binary is missing/rebooting — e.g. local dev machines or
        tunnel-only deployments without the robot runtime."""
        gateway_server = load_gateway_server()
        context = gateway_server.get_planner_context(
            user_command="test",
            map_id="go2w_real_site",
            registry_path=str(REAL_REGISTRY),
            map_path="",
        )

        self.assertNotIn("error", context)
        connectivity = context.get("waypoint_connectivity", {})
        self.assertEqual(len(connectivity.get("nodes", [])), 12)
        self.assertEqual(connectivity.get("edge_source"), "registry")
        self.assertIsInstance(context.get("coarse_map"), str)
        status = context.get("status")
        self.assertIsInstance(status, dict)
        # Gateway may be real (accepted=True) or unavailable (graceful fallback).
        self.assertTrue(
            status.get("reason") == "gateway_unavailable"
            or status.get("accepted") is True
        )


if __name__ == "__main__":
    unittest.main()
