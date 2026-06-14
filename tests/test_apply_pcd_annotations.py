import argparse
import importlib.util
import json
import tempfile
import unittest
from pathlib import Path


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "apply_pcd_annotations.py"
SPEC = importlib.util.spec_from_file_location("apply_pcd_annotations", SCRIPT)
assert SPEC and SPEC.loader
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def args(calibration_dir: Path, *, allow_overwrite_verified: bool = False) -> argparse.Namespace:
    return argparse.Namespace(
        map_id="go2w_real_site",
        default_speed=0.3,
        mode=0,
        calibration_dir=str(calibration_dir),
        allow_overwrite_verified=allow_overwrite_verified,
    )


def registry_with(node: dict) -> dict:
    return {
        "maps": [
            {
                "map_id": "go2w_real_site",
                "topology_nodes": [node],
            }
        ]
    }


class ApplyPcdAnnotationsTests(unittest.TestCase):
    def test_confirmed_calibration_artifact_blocks_annotation_overwrite(self) -> None:
        node = {
            "node_id": "yin_siyuan_station",
            "name": "尹思园工位",
            "tags": ["real_site"],
            "pose": {"x": 1.5974299907684326, "y": 0.3592859208583832},
        }
        annotation = {
            "name": "尹思园工位",
            "pose": {"x": 1.9271903991699215, "y": 0.45498428344726705},
        }
        with tempfile.TemporaryDirectory() as temp:
            calibration_dir = Path(temp)
            (calibration_dir / "yin_siyuan_calibration_20260522.json").write_text(
                json.dumps(
                    {
                        "node_id": "yin_siyuan_station",
                        "confirmed_pose": {"x": 1.5974299907684326, "y": 0.3592859208583832},
                    }
                ),
                encoding="utf-8",
            )

            result = MODULE.apply_annotations(
                registry_with(node),
                [annotation],
                args(calibration_dir),
            )

        self.assertEqual(result["updated"], [])
        self.assertEqual(result["skipped"][0]["node_id"], "yin_siyuan_station")
        self.assertAlmostEqual(node["pose"]["x"], 1.5974299907684326)
        self.assertAlmostEqual(node["pose"]["y"], 0.3592859208583832)

    def test_unverified_annotation_can_update_existing_node(self) -> None:
        node = {
            "node_id": "yin_siyuan_station",
            "name": "尹思园工位",
            "tags": ["needs_calibration"],
            "pose": {"x": 0.0, "y": 0.0},
        }
        annotation = {
            "name": "尹思园工位",
            "pose": {"x": 1.9, "y": 0.45},
        }
        with tempfile.TemporaryDirectory() as temp:
            result = MODULE.apply_annotations(
                registry_with(node),
                [annotation],
                args(Path(temp)),
            )

        self.assertEqual(result["skipped"], [])
        self.assertEqual(result["updated"][0]["node_id"], "yin_siyuan_station")
        self.assertAlmostEqual(node["pose"]["x"], 1.9)

    def test_verified_node_requires_explicit_override(self) -> None:
        node = {
            "node_id": "yin_siyuan_station",
            "name": "尹思园工位",
            "tags": ["live_verified"],
            "pose": {"x": 1.5, "y": 0.35},
        }
        annotation = {
            "name": "尹思园工位",
            "pose": {"x": 1.9, "y": 0.45},
        }
        with tempfile.TemporaryDirectory() as temp:
            calibration_dir = Path(temp)
            blocked = MODULE.apply_annotations(
                registry_with(node),
                [annotation],
                args(calibration_dir),
            )
            allowed = MODULE.apply_annotations(
                registry_with(node),
                [annotation],
                args(calibration_dir, allow_overwrite_verified=True),
            )

        self.assertEqual(blocked["updated"], [])
        self.assertEqual(allowed["skipped"], [])
        self.assertAlmostEqual(node["pose"]["x"], 1.9)


if __name__ == "__main__":
    unittest.main()
