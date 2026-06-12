from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path

from scripts.finalize_xt16_calibration import (
    REQUIRED_SCENES,
    build_verified_record,
    validate_artifacts,
)


def record() -> dict:
    return {
        "sensor": "XT16",
        "sensor_serial": "SERIAL",
        "status": "pending_field_measurement",
        "parameters": {"range_m": 6.0},
        "evidence": {"acceptance_max_abs_error_m": 0.15},
    }


def artifact(scene: str, *, accepted: bool = True, dirty: bool = False) -> dict:
    return {
        "type": "go2w_xt16_calibration_scene",
        "scene": scene,
        "git": {"head": "abc123", "dirty": dirty},
        "calibration_record": {
            "sensor_serial": "SERIAL",
            "parameters": {"range_m": 6.0},
        },
        "map_identity": {
            "map_id": "go2w_real_site",
            "registry_sha256": "b" * 64,
            "pcd_sha256": "c" * 64,
        },
        "assessment": {
            "accepted": accepted,
            "thresholds": {"required_samples": 2},
            "comparisons": {
                scene if scene != "baseline" else "front": {
                    "absolute_error_m": 0.05,
                }
            },
        },
        "samples": [{"timestamp_ms": 1}, {"timestamp_ms": 2}],
    }


class FinalizeXt16CalibrationTests(unittest.TestCase):
    def write_artifacts(self, root: Path, *, rejected_scene: str = "") -> list[Path]:
        paths: list[Path] = []
        for scene in sorted(REQUIRED_SCENES):
            path = root / f"{scene}.json"
            path.write_text(
                json.dumps(artifact(scene, accepted=scene != rejected_scene)),
                encoding="utf-8",
            )
            paths.append(path)
        return paths

    def test_five_accepted_scenes_build_complete_evidence(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            paths = self.write_artifacts(Path(temp))
            reasons, evidence = validate_artifacts(paths, record())

        self.assertEqual(reasons, [])
        self.assertEqual(evidence["completed_stationary_measured_scenes"], 5)
        self.assertEqual(len(evidence["artifact_sha256"]), 5)
        self.assertEqual(evidence["max_abs_error_m"], 0.05)
        self.assertEqual(evidence["map_id"], "go2w_real_site")
        self.assertEqual(evidence["registry_sha256"], "b" * 64)
        self.assertEqual(evidence["pcd_sha256"], "c" * 64)

    def test_different_map_hashes_block_finalization(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            paths = self.write_artifacts(Path(temp))
            changed = json.loads(paths[0].read_text(encoding="utf-8"))
            changed["map_identity"]["pcd_sha256"] = "d" * 64
            paths[0].write_text(json.dumps(changed), encoding="utf-8")
            reasons, _ = validate_artifacts(paths, record())

        self.assertIn("scene_pcd_hashes_differ", reasons)

    def test_rejected_scene_blocks_finalization(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            paths = self.write_artifacts(Path(temp), rejected_scene="right")
            reasons, _ = validate_artifacts(paths, record())

        self.assertIn("scene_not_accepted:right", reasons)

    def test_verified_record_preserves_parameters_and_adds_evidence(self) -> None:
        updated = build_verified_record(
            record(),
            calibration_id="xt16-field-20260612",
            verified_by="operator",
            evidence={
                "completed_stationary_measured_scenes": 5,
                "artifact_paths": ["a", "b", "c", "d", "e"],
            },
            verified_at="2026-06-12T16:00:00+0800",
        )

        self.assertEqual(updated["status"], "verified")
        self.assertEqual(updated["parameters"], {"range_m": 6.0})
        self.assertEqual(updated["verified_by"], "operator")
        self.assertEqual(updated["evidence"]["completed_stationary_measured_scenes"], 5)


if __name__ == "__main__":
    unittest.main()
