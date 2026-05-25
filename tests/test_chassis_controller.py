import json
import tempfile
import unittest
from pathlib import Path

from edge_autonomy.chassis_controller import ChassisController, GatewayConfig


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


if __name__ == "__main__":
    unittest.main()
