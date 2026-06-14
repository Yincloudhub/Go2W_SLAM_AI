from __future__ import annotations

import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
SCRIPT = REPO_ROOT / "scripts" / "summarize_xt16_view_capture.py"


class SummarizeXt16ViewCaptureTests(unittest.TestCase):
    def test_writes_repeatable_scene_summary(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            capture = root / "capture.jsonl"
            output = root / "summary.json"
            frames = [
                {
                    "timestamp_ms": 1000 + index,
                    "sequence": index + 1,
                    "topic": "/unitree/slam_lidar/points",
                    "frame_id": "rslidar",
                    "raw_points": 100,
                    "plot_points": [
                        [0.0, -0.4, 0.1, "body_height"],
                        [0.0, 0.1, 0.1, "footprint_rejected"],
                    ],
                    "geometry": {
                        "front_clearance_m": 2.0,
                        "left_clearance_m": 1.0,
                        "right_clearance_m": 0.1,
                        "rear_clearance_m": 1.5,
                        "points_excluded_footprint": 10,
                        "footprint_m": {
                            "half_width": 0.3,
                            "lateral_filter_margin": 0.0,
                        },
                    },
                }
                for index in range(2)
            ]
            capture.write_text(
                "\n".join(json.dumps(frame) for frame in frames) + "\n",
                encoding="utf-8",
            )

            result = subprocess.run(
                [
                    sys.executable,
                    str(SCRIPT),
                    str(capture),
                    "--scene-id",
                    "test-scene",
                    "--output",
                    str(output),
                ],
                cwd=REPO_ROOT,
                text=True,
                capture_output=True,
                check=False,
            )

            self.assertEqual(result.returncode, 0, result.stderr)
            summary = json.loads(output.read_text(encoding="utf-8"))
            self.assertEqual(summary["frames"], 2)
            self.assertEqual(summary["frame_id"], "rslidar")
            self.assertEqual(summary["clearance_m"]["right"]["median"], 0.1)
            self.assertEqual(summary["geometry"]["lateral_filter_margin"], 0.0)
            self.assertEqual(len(summary["capture_sha256"]), 64)


if __name__ == "__main__":
    unittest.main()
