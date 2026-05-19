import json
import unittest
from pathlib import Path

from edge_autonomy.map_registry import MapRegistry, MapRegistryError, yaw_to_quaternion


REGISTRY_PATH = Path(__file__).resolve().parents[1] / "configs" / "maps" / "go2w_map_registry.example.json"


class MapRegistryTests(unittest.TestCase):
    def test_load_example_registry(self) -> None:
        registry = MapRegistry.from_file(REGISTRY_PATH)

        self.assertEqual(registry.default_map_id, "test_current_main")
        self.assertIn("test_current_main", registry.map_ids())
        self.assertIn("test513_candidate", registry.map_ids())

    def test_build_relocation_command(self) -> None:
        registry = MapRegistry.from_file(REGISTRY_PATH)
        command = registry.get_map("test_current_main").relocate_command("mapping_origin")

        self.assertEqual(command["action"], "relocate")
        self.assertEqual(command["map_path"], "/home/unitree/test.pcd")
        self.assertEqual(command["anchor_id"], "mapping_origin")
        self.assertEqual(command["initial_pose"]["x"], 0.0)
        self.assertEqual(command["initial_pose"]["q_w"], 1.0)

    def test_build_navigation_command_by_alias(self) -> None:
        registry = MapRegistry.from_file(REGISTRY_PATH)
        command = registry.get_map("test_current_main").navigate_to_node_command("wp_1")

        self.assertEqual(command["action"], "navigate_to_pose")
        self.assertEqual(command["target_node"], "nie_guoli_office_front")
        self.assertAlmostEqual(command["target_pose"]["x"], 3.258938789367676)
        self.assertEqual(command["target_pose"]["mode"], 0)
        self.assertEqual(command["target_pose"]["speed"], 0.45)

    def test_build_navigation_command_with_speed_override(self) -> None:
        registry = MapRegistry.from_file(REGISTRY_PATH)
        command = registry.get_map("test_current_main").navigate_to_node_command("wp_1", speed=0.35, mode=0)

        self.assertEqual(command["target_pose"]["speed"], 0.35)
        self.assertEqual(command["target_pose"]["mode"], 0)

    def test_export_unitree_topology_json(self) -> None:
        registry = MapRegistry.from_file(REGISTRY_PATH)
        topology = registry.get_map("test_current_main").to_unitree_topology_json()

        self.assertEqual(topology["version"], 1)
        self.assertEqual(topology["map_id"], "test_current_main")
        self.assertEqual(len(topology["waypoints"]), 2)
        json.dumps(topology, ensure_ascii=False)

    def test_unknown_anchor_raises_clear_error(self) -> None:
        registry = MapRegistry.from_file(REGISTRY_PATH)

        with self.assertRaises(MapRegistryError):
            registry.get_map("test_current_main").relocate_command("missing")

    def test_yaw_to_quaternion(self) -> None:
        q_x, q_y, q_z, q_w = yaw_to_quaternion(0.0)

        self.assertEqual(q_x, 0.0)
        self.assertEqual(q_y, 0.0)
        self.assertEqual(q_z, 0.0)
        self.assertEqual(q_w, 1.0)


if __name__ == "__main__":
    unittest.main()
