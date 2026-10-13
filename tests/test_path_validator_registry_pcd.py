import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from edge_autonomy import path_validator


class PathValidatorRegistryPcdTests(unittest.TestCase):
    def test_build_adaptive_coarse_map_uses_registry_pcd_for_map_id(self):
        with tempfile.TemporaryDirectory() as td:
            registry = Path(td) / "registry.json"
            registry.write_text(
                json.dumps(
                    {
                        "maps": [
                            {
                                "map_id": "site",
                                "pcd_path": "/tmp/site_from_registry.pcd",
                                "topology_nodes": [
                                    {
                                        "node_id": "wp_a",
                                        "name": "wp_a",
                                        "pose": {"x": 0.0, "y": 0.0},
                                    }
                                ],
                            }
                        ]
                    }
                ),
                encoding="utf-8",
            )
            loaded_paths = []

            def fake_load_pcd(pcd_path):
                loaded_paths.append(pcd_path)
                return [(0.0, 0.0)]

            with patch.object(path_validator, "_load_pcd", fake_load_pcd):
                result = path_validator.build_adaptive_coarse_map(
                    registry_path=str(registry),
                    map_id="site",
                    grid_size=10,
                )

        self.assertEqual(result["pcd_path"], "/tmp/site_from_registry.pcd")
        self.assertEqual(loaded_paths, ["/tmp/site_from_registry.pcd"])


if __name__ == "__main__":
    unittest.main()
