from __future__ import annotations

import json
import tempfile
import unittest
from contextlib import redirect_stderr, redirect_stdout
from io import StringIO
from pathlib import Path

from scripts import check_go2w_runtime_layout


class RuntimeLayoutTests(unittest.TestCase):
    def make_active_repo(self, root: Path) -> Path:
        repo = root / "Go2W_SLAM_AI"
        registry = repo / "configs" / "maps" / "go2w_real_site_map_registry.json"
        registry.parent.mkdir(parents=True)
        registry.write_text(
            json.dumps(
                {
                    "default_map_id": "go2w_real_site",
                    "robot": {
                        "slam_gateway_client": (
                            "/home/unitree/Go2W_SLAM_AI/robot/"
                            "slam_gateway_refactor/build/slam_llm_command_client"
                        )
                    },
                    "maps": [
                        {
                            "map_id": "go2w_real_site",
                            "status": "real",
                            "pcd_path": "/home/unitree/test.pcd",
                            "topology_path": "/home/unitree/topology_points.json",
                            "mapping_origin_anchor_id": "mapping_origin",
                            "topology_nodes": [
                                {
                                    "node_id": "initial_point",
                                    "name": "initial_point",
                                    "pose": {"x": 0.0, "y": 0.0},
                                }
                            ],
                            "relocalization_anchors": [
                                {
                                    "anchor_id": "mapping_origin",
                                    "name": "mapping_origin",
                                    "status": "verified_test",
                                    "pose": {"x": 0.0, "y": 0.0},
                                }
                            ],
                            "archived_relocalization_anchors": [
                                {
                                    "anchor_id": "initial_point",
                                    "name": "initial_point",
                                    "status": "candidate_failed",
                                    "pose": {"x": 1.0, "y": 0.0},
                                }
                            ],
                        }
                    ],
                }
            ),
            encoding="utf-8",
        )
        (repo / "robot" / "slam_gateway_refactor").mkdir(parents=True)
        return repo

    def run_checker(self, *args: str) -> int:
        with redirect_stdout(StringIO()), redirect_stderr(StringIO()):
            return check_go2w_runtime_layout.main(list(args))

    def test_accepts_single_repo_layout(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            active_repo = self.make_active_repo(root)
            result = self.run_checker(
                "--active-repo",
                str(active_repo),
                "--legacy-repo",
                str(root / "missing-agent"),
                "--legacy-gateway",
                str(root / "missing-gateway"),
            )
            self.assertEqual(result, 0)

    def test_rejects_external_legacy_gateway(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            active_repo = self.make_active_repo(root)
            legacy_gateway = root / "slam_gateway_refactor"
            legacy_gateway.mkdir()
            result = self.run_checker(
                "--active-repo",
                str(active_repo),
                "--legacy-repo",
                str(root / "missing-agent"),
                "--legacy-gateway",
                str(legacy_gateway),
            )
            self.assertEqual(result, 2)

    def test_rejects_registry_pointing_to_legacy_gateway(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            active_repo = self.make_active_repo(root)
            registry = active_repo / "configs" / "maps" / "go2w_real_site_map_registry.json"
            data = json.loads(registry.read_text(encoding="utf-8"))
            data["robot"]["slam_gateway_client"] = (
                "/home/unitree/slam_gateway_refactor/build/slam_llm_command_client"
            )
            registry.write_text(json.dumps(data), encoding="utf-8")

            result = self.run_checker(
                "--active-repo",
                str(active_repo),
                "--legacy-repo",
                str(root / "missing-agent"),
                "--legacy-gateway",
                str(root / "missing-gateway"),
            )

            self.assertEqual(result, 2)

    def test_rejects_unverified_or_extra_active_relocalization_anchor(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            active_repo = self.make_active_repo(root)
            registry = active_repo / "configs" / "maps" / "go2w_real_site_map_registry.json"
            data = json.loads(registry.read_text(encoding="utf-8"))
            data["maps"][0]["relocalization_anchors"].append(
                {
                    "anchor_id": "initial_point",
                    "name": "initial_point",
                    "status": "candidate",
                    "pose": {"x": 1.0, "y": 0.0},
                }
            )
            registry.write_text(json.dumps(data), encoding="utf-8")

            result = self.run_checker(
                "--active-repo",
                str(active_repo),
                "--legacy-repo",
                str(root / "missing-agent"),
                "--legacy-gateway",
                str(root / "missing-gateway"),
            )

            self.assertEqual(result, 2)

    def test_accepts_multiple_verified_relocalization_anchors(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            active_repo = self.make_active_repo(root)
            registry = active_repo / "configs" / "maps" / "go2w_real_site_map_registry.json"
            data = json.loads(registry.read_text(encoding="utf-8"))
            data["maps"][0]["relocalization_anchors"].append(
                {
                    "anchor_id": "verified_station_anchor",
                    "name": "verified_station_anchor",
                    "status": "verified_field_test",
                    "pose": {"x": 1.0, "y": 0.0},
                }
            )
            registry.write_text(json.dumps(data), encoding="utf-8")

            result = self.run_checker(
                "--active-repo",
                str(active_repo),
                "--legacy-repo",
                str(root / "missing-agent"),
                "--legacy-gateway",
                str(root / "missing-gateway"),
            )

            self.assertEqual(result, 0)


if __name__ == "__main__":
    unittest.main()
