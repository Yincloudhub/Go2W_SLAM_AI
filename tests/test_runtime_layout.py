from __future__ import annotations

import json
import tempfile
import unittest
from contextlib import redirect_stderr, redirect_stdout
from io import StringIO
from pathlib import Path

from scripts import check_go2w_runtime_layout


class RuntimeLayoutTests(unittest.TestCase):
    def live_nodes(self) -> list[dict[str, object]]:
        return [
            {
                "node_id": node_id,
                "name": node_id,
                "pose": {"x": float(index), "y": 0.0},
            }
            for index, node_id in enumerate(
                [
                    "wp_60497f",
                    "wp_b738c7",
                    "wp_bbcb4c",
                    "wp_a684c6",
                    "wp_2ae043",
                    "wp_e643bc",
                    "transition_701_to_701_left",
                    "transition_701_to_terrace",
                ]
            )
        ]

    def live_edges(self) -> list[dict[str, object]]:
        return [
            {"from": "wp_60497f", "to": "wp_b738c7", "bidirectional": True},
            {"from": "wp_b738c7", "to": "wp_bbcb4c", "bidirectional": True},
            {"from": "wp_bbcb4c", "to": "wp_a684c6", "bidirectional": True},
            {"from": "wp_a684c6", "to": "wp_2ae043", "bidirectional": True},
            {"from": "wp_2ae043", "to": "wp_e643bc", "bidirectional": True},
            {"from": "wp_b738c7", "to": "transition_701_to_701_left", "bidirectional": True},
            {"from": "wp_e643bc", "to": "transition_701_to_terrace", "bidirectional": True},
        ]

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
                            "pcd_path": "/home/unitree/maps/staging/map_701.pcd",
                            "topology_path": "/home/unitree/maps/staging/map_701_topology.json",
                            "mapping_origin_anchor_id": "mapping_origin",
                            "topology_nodes": self.live_nodes(),
                            "topology_edges": self.live_edges(),
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
                                },
                                {
                                    "anchor_id": "yin_siyuan_station",
                                    "name": "yin_siyuan_station",
                                    "status": "verified_legacy",
                                    "pose": {"x": 2.0, "y": 0.0},
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

    def make_v2_symlink_repo(self, root: Path) -> Path:
        repo = self.make_active_repo(root)
        registry_dir = repo / "configs" / "maps"
        real_site = registry_dir / "go2w_real_site_map_registry.json"
        multi = registry_dir / "go2w_multi_map_registry_v2.json"
        data = json.loads(real_site.read_text(encoding="utf-8"))
        wrapper = data["maps"][0]
        wrapper["mapping_origin_anchor_id"] = "mapping_origin_701"
        wrapper["relocalization_anchors"] = [
            {
                "anchor_id": "mapping_origin_701",
                "name": "mapping_origin_701",
                "status": "verified",
                "pose": {"x": 0.0, "y": 0.0},
            },
            {
                "anchor_id": "transition_701_to_701_left",
                "name": "transition_701_to_701_left",
                "status": "verified",
                "pose": {"x": 1.0, "y": 0.0},
            },
        ]
        wrapper.pop("archived_relocalization_anchors", None)
        map_701 = {**wrapper, "map_id": "map_701", "relocalization_anchors": wrapper["relocalization_anchors"][:1]}
        data["maps"] = [wrapper, map_701]
        multi.write_text(json.dumps(data), encoding="utf-8")
        real_site.unlink()
        try:
            real_site.symlink_to(multi.name)
        except OSError:
            real_site.write_text(multi.read_text(encoding="utf-8"), encoding="utf-8")
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

    def test_accepts_v2_registry_symlink_wrapper_without_archived_history(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            active_repo = self.make_v2_symlink_repo(root)
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
