import json
import subprocess
import tempfile
import unittest
from pathlib import Path
from unittest.mock import MagicMock, patch

from edge_autonomy.chassis_controller import (
    ChassisController,
    GatewayConfig,
    gateway_allows_navigation,
    run_gateway_command,
)


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

    def test_preflight_rejects_manual_obstacle_stub(self) -> None:
        allowed, reason = gateway_allows_navigation(
            {
                "accepted": True,
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

        self.assertFalse(allowed)
        self.assertIn("not trusted", reason)

    def test_gateway_timeout_kills_child_and_fails_deterministically(self) -> None:
        process = MagicMock()
        process.communicate.side_effect = [
            subprocess.TimeoutExpired(cmd=["gateway"], timeout=1),
            ("", ""),
        ]
        with patch("edge_autonomy.chassis_controller.subprocess.Popen", return_value=process):
            with self.assertRaisesRegex(RuntimeError, "timed out after 1s"):
                run_gateway_command(
                    {"action": "get_world_state"},
                    GatewayConfig(client_path="gateway", timeout_s=1),
                )
        process.kill.assert_called_once_with()


if __name__ == "__main__":
    unittest.main()
