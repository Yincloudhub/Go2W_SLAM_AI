import json
import tempfile
import unittest
from pathlib import Path

from edge_autonomy.chassis_controller import ChassisController, GatewayConfig, gateway_allows_navigation


class ChassisControllerTests(unittest.TestCase):
    def test_resolve_node_prefers_first_target_in_command_order(self) -> None:
        registry = {
            "version": 1,
            "default_map_id": "site",
            "maps": [
                {
                    "map_id": "site",
                    "pcd_path": "/tmp/site.pcd",
                    "topology_nodes": [
                        {"node_id": "station", "name": "station", "aliases": ["station"], "pose": {"x": 0, "y": 0}},
                        {"node_id": "room701", "name": "room701", "aliases": ["room701"], "pose": {"x": 1, "y": 0}},
                    ],
                }
            ],
        }
        with tempfile.TemporaryDirectory() as tmp:
            registry_path = Path(tmp) / "registry.json"
            registry_path.write_text(json.dumps(registry), encoding="utf-8")
            controller = ChassisController(
                registry_path=registry_path,
                map_id="site",
                gateway=GatewayConfig(client_path="/bin/false"),
            )

            result = controller.resolve_node("go room701 then return station")

        self.assertTrue(result["matched"])
        self.assertTrue(result["multi_target"])
        self.assertEqual(result["selected"]["node_id"], "room701")
        self.assertEqual([item["node_id"] for item in result["matches"]], ["room701", "station"])

    def test_preflight_trusts_gateway_safety_when_obstacle_advisory_is_manual(self) -> None:
        allowed, reason = gateway_allows_navigation(
            {
                "world_state": {
                    "safety": {"allow_navigation": True, "reason": "ok"},
                    "slam_health": {"status": "ok", "slam_alive": True, "localization_alive": True},
                    "localization": {"status": "localized", "confidence": 0.9, "pose_age_ms": 100},
                    "current_pose": {"pose": {"x": 0.0, "y": 0.0, "yaw": 0.0}},
                    "local_obstacle": {
                        "source": "manual_stub",
                        "stale": True,
                        "age_ms": -1,
                        "confidence": 0.0,
                        "front_confidence": 0.0,
                        "left_confidence": 0.0,
                        "right_confidence": 0.0,
                        "front_clearance_m": 6.0,
                        "left_clearance_m": 6.0,
                        "right_clearance_m": 6.0,
                    },
                }
            }
        )

        self.assertTrue(allowed, reason)
        self.assertIn("gateway allows navigation", reason)


if __name__ == "__main__":
    unittest.main()
