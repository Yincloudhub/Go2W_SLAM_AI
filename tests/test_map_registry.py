import json
import unittest
from pathlib import Path

from edge_autonomy.map_registry import MapProfile, MapRegistry, MapRegistryError, yaw_to_quaternion


REGISTRY_PATH = Path(__file__).resolve().parents[1] / "configs" / "maps" / "go2w_map_registry.example.json"
REAL_REGISTRY_PATH = Path(__file__).resolve().parents[1] / "configs" / "maps" / "go2w_real_site_map_registry.json"


class MapRegistryTests(unittest.TestCase):
    def test_real_map_requires_mapping_origin_anchor(self) -> None:
        with self.assertRaisesRegex(MapRegistryError, "mapping_origin_anchor_id"):
            MapProfile.from_dict(
                {
                    "map_id": "site",
                    "name": "site",
                    "status": "real",
                    "pcd_path": "/tmp/site.pcd",
                    "topology_path": "/tmp/site.json",
                }
            )

    def test_load_example_registry(self) -> None:
        registry = MapRegistry.from_file(REGISTRY_PATH)

        self.assertEqual(registry.default_map_id, "test_current_main")
        self.assertIn("test_current_main", registry.map_ids())
        self.assertIn("test513_candidate", registry.map_ids())

    def test_build_relocation_command(self) -> None:
        registry = MapRegistry.from_file(REGISTRY_PATH)
        command = registry.get_map("test_current_main").relocate_command("mapping_origin")

        self.assertEqual(command["action"], "relocate")
        self.assertTrue(command["operator_ack"])
        self.assertEqual(command["map_path"], "/home/unitree/test.pcd")
        self.assertEqual(command["anchor_id"], "mapping_origin")
        self.assertEqual(command["initial_pose"]["x"], 0.0)
        self.assertEqual(command["initial_pose"]["q_w"], 1.0)

    def test_build_navigation_command_by_alias(self) -> None:
        registry = MapRegistry.from_file(REGISTRY_PATH)
        command = registry.get_map("test_current_main").navigate_to_node_command("wp_1")

        self.assertEqual(command["action"], "navigate_to_pose")
        self.assertEqual(command["target_node"], "nie_guoli_office_front")
        self.assertEqual(command["map_path"], "/home/unitree/test.pcd")
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

    def test_real_registry_separates_relocation_anchor_history_from_topology(self) -> None:
        profile = MapRegistry.from_file(REAL_REGISTRY_PATH).get_map("go2w_real_site")

        self.assertEqual(
            [anchor.anchor_id for anchor in profile.relocalization_anchors],
            ["mapping_origin"],
        )
        self.assertEqual(profile.mapping_origin_anchor_id, "mapping_origin")
        self.assertIn(
            "initial_point",
            [anchor.anchor_id for anchor in profile.archived_relocalization_anchors],
        )
        self.assertEqual(profile.get_node("initial_point").node_id, "initial_point")
        with self.assertRaises(MapRegistryError):
            profile.relocate_command("initial_point")

    def test_unverified_active_anchor_cannot_emit_relocation_command(self) -> None:
        data = json.loads(REGISTRY_PATH.read_text(encoding="utf-8"))
        data["maps"][0]["relocalization_anchors"][0]["status"] = "candidate"
        profile = MapRegistry.from_dict(data).get_map("test_current_main")

        with self.assertRaisesRegex(MapRegistryError, "not verified"):
            profile.relocate_command("mapping_origin")

    def test_multiple_verified_relocalization_anchors_are_supported(self) -> None:
        data = json.loads(REGISTRY_PATH.read_text(encoding="utf-8"))
        profile_data = data["maps"][0]
        second = json.loads(json.dumps(profile_data["relocalization_anchors"][0]))
        second["anchor_id"] = "verified_secondary"
        second["name"] = "verified_secondary"
        second["status"] = "verified_field_test"
        second["pose"]["name"] = "verified_secondary"
        second["pose"]["x"] = 1.0
        profile_data["relocalization_anchors"].append(second)
        profile = MapRegistry.from_dict(data).get_map("test_current_main")

        command = profile.relocate_command("verified_secondary")

        self.assertEqual(command["anchor_id"], "verified_secondary")
        self.assertEqual(command["initial_pose"]["x"], 1.0)

    def test_yaw_to_quaternion(self) -> None:
        q_x, q_y, q_z, q_w = yaw_to_quaternion(0.0)

        self.assertEqual(q_x, 0.0)
        self.assertEqual(q_y, 0.0)
        self.assertEqual(q_z, 0.0)
        self.assertEqual(q_w, 1.0)

    def test_topology_edge_distance_is_untrusted_by_default(self) -> None:
        profile = MapProfile.from_dict(
            {
                "map_id": "simulation",
                "name": "simulation",
                "status": "simulation",
                "pcd_path": "/tmp/site.pcd",
                "topology_path": "/tmp/site.json",
                "topology_edges": [
                    {"from": "a", "to": "b", "expected_distance_m": 1.0}
                ],
            }
        )

        self.assertFalse(profile.topology_edges[0].distance_verified)

    def test_missing_pose_coordinates_are_rejected(self) -> None:
        data = json.loads(REGISTRY_PATH.read_text(encoding="utf-8"))
        del data["maps"][0]["topology_nodes"][0]["pose"]["x"]

        with self.assertRaises(MapRegistryError):
            MapRegistry.from_dict(data)

    def test_duplicate_topology_alias_is_rejected(self) -> None:
        data = json.loads(REGISTRY_PATH.read_text(encoding="utf-8"))
        data["maps"][0]["topology_nodes"][0]["aliases"].append("shared_lab")
        data["maps"][0]["topology_nodes"][1]["aliases"].append("shared_lab")

        with self.assertRaisesRegex(MapRegistryError, "ambiguous topology term"):
            MapRegistry.from_dict(data)


if __name__ == "__main__":
    unittest.main()
