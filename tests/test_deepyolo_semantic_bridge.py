import importlib.util
import json
import sys
import tempfile
import unittest
from pathlib import Path


def load_module():
    path = Path(__file__).resolve().parents[1] / "scripts" / "deepyolo_semantic_bridge.py"
    spec = importlib.util.spec_from_file_location("deepyolo_semantic_bridge", path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


bridge = load_module()


class DeepYoloSemanticBridgeTests(unittest.TestCase):
    def test_build_summary_bounds_objects_and_recommends_pause(self):
        packet = {
            "timestamp_ms": 1000,
            "session_id": "run_a",
            "frame_id": 7,
            "object_count": 2,
            "high_risk_count": 1,
            "has_high_risk": True,
            "dominant_class": "person",
            "main_region": "center",
            "scene_state": "alert",
            "depth_valid": True,
            "center_depth_m": 1.2,
            "objects": [
                {
                    "track_id": 1,
                    "class_name": "chair",
                    "confidence": 0.9,
                    "risk_level": "medium",
                    "region": "left",
                    "distance_m": 1.1,
                },
                {
                    "track_id": 2,
                    "class_name": "person",
                    "confidence": 0.7,
                    "risk_level": "high",
                    "region": "center",
                    "distance_m": 1.4,
                    "depth_valid": True,
                },
            ],
        }

        summary = bridge.build_semantic_summary(packet, source_path=Path("semantic.jsonl"), max_objects=1, bridge_timestamp_ms=1500)

        self.assertTrue(summary["available"])
        self.assertEqual(summary["recommended_action"], "hold_or_pause")
        self.assertEqual(summary["age_ms"], 500)
        self.assertEqual(len(summary["objects"]), 1)
        self.assertEqual(summary["objects"][0]["class_name"], "person")

    def test_read_last_json_line_ignores_partial_trailing_line(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "semantic_stream_a.jsonl"
            path.write_text('{"frame_id":1}\nnot-json\n{"frame_id":2}', encoding="utf-8")

            self.assertEqual(bridge.read_last_json_line(path)["frame_id"], 2)

    def test_emit_once_writes_unavailable_when_missing(self):
        with tempfile.TemporaryDirectory() as tmp:
            args = type(
                "Args",
                (),
                {
                    "input_jsonl": None,
                    "input_dir": Path(tmp) / "missing",
                    "max_objects": 8,
                    "stale_ms": 3000,
                    "center_stop_m": 1.5,
                    "near_watch_m": 2.5,
                },
            )()

            summary = bridge.emit_once(args)

            self.assertFalse(summary["available"])
            self.assertEqual(summary["reason"], "no_deepyolo_jsonl")

    def test_write_summary_replaces_output_atomically(self):
        with tempfile.TemporaryDirectory() as tmp:
            output = Path(tmp) / "vision_semantic_summary.json"
            bridge.write_summary({"source": "deepyolo_realsense"}, output)

            self.assertEqual(json.loads(output.read_text(encoding="utf-8"))["source"], "deepyolo_realsense")
            self.assertFalse((Path(tmp) / "vision_semantic_summary.json.tmp").exists())


if __name__ == "__main__":
    unittest.main()
