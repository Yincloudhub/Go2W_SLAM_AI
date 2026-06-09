import importlib.util
import json
import sys
import tempfile
import unittest
from pathlib import Path


def load_module():
    path = Path(__file__).resolve().parents[1] / "scripts" / "realsense_depth_summary.py"
    spec = importlib.util.spec_from_file_location("realsense_depth_summary", path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


depth_summary = load_module()


class RealsenseDepthSummaryTests(unittest.TestCase):
    def test_percentile_interpolates(self):
        self.assertAlmostEqual(depth_summary.percentile([1, 2, 3, 4], 50), 2.5)
        self.assertEqual(depth_summary.percentile([], 10), None)

    def test_build_depth_summary_uses_middle_band_rois(self):
        depth = [[2000 for _ in range(6)] for _ in range(6)]
        for y in range(2, 4):
            depth[y][0] = 1200
            depth[y][1] = 1300
            depth[y][2] = 500
            depth[y][3] = 700
            depth[y][4] = 1600
            depth[y][5] = 1700

        summary = depth_summary.build_depth_summary(depth, timestamp_ms=123, q=10)

        self.assertEqual(summary["timestamp_ms"], 123)
        self.assertLess(summary["front_clearance_m"], 0.8)
        self.assertGreater(summary["left_clearance_m"], 1.1)
        self.assertGreater(summary["right_clearance_m"], 1.5)
        self.assertEqual(summary["coverage"], "forward_fov")
        self.assertEqual(summary["roi_semantics"]["left"], "left_third_of_forward_fov")
        self.assertEqual(summary["roi_semantics"]["rear"], "not_observed")
        self.assertEqual(summary["summary"]["shape"], [6, 6])
        self.assertAlmostEqual(summary["center_distance_m"], 0.7)
        self.assertGreater(summary["roi_confidence"]["left"], 0.0)

    def test_invalid_depth_reduces_confidence_and_ignores_roi(self):
        depth = [[0 for _ in range(6)] for _ in range(6)]
        summary = depth_summary.build_depth_summary(depth, timestamp_ms=123)

        self.assertEqual(summary["confidence"], 0.0)
        self.assertIsNone(summary["front_clearance_m"])
        self.assertIsNone(summary["center_distance_m"])
        self.assertEqual(summary["roi_confidence"]["front"], 0.0)

    def test_numpy_input_matches_list_input(self):
        try:
            import numpy as np
        except ImportError:
            self.skipTest("numpy is optional on the local test host")
        depth = [[2000 for _ in range(6)] for _ in range(6)]
        depth[2][2] = 500
        depth[3][3] = 700

        list_summary = depth_summary.build_depth_summary(depth, timestamp_ms=123, q=10)
        numpy_summary = depth_summary.build_depth_summary(np.asarray(depth), timestamp_ms=123, q=10)

        self.assertAlmostEqual(numpy_summary["front_clearance_m"], list_summary["front_clearance_m"])
        self.assertEqual(numpy_summary["roi_confidence"], list_summary["roi_confidence"])
        self.assertAlmostEqual(numpy_summary["confidence"], list_summary["confidence"])

    def test_write_summary_replaces_output_atomically(self):
        with tempfile.TemporaryDirectory() as tmp:
            output = Path(tmp) / "stereo_depth_summary.json"
            text = depth_summary.write_summary({"source": "stereo_depth"}, output)

            self.assertEqual(json.loads(text)["source"], "stereo_depth")
            self.assertEqual(json.loads(output.read_text(encoding="utf-8"))["source"], "stereo_depth")
            self.assertFalse((Path(tmp) / "stereo_depth_summary.json.tmp").exists())


if __name__ == "__main__":
    unittest.main()
