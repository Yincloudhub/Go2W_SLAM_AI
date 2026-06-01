import importlib.util
import json
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from jsonschema import validate


def load_module():
    path = Path(__file__).resolve().parents[1] / "scripts" / "deepyolo_semantic_bridge.py"
    spec = importlib.util.spec_from_file_location("deepyolo_semantic_bridge", path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


bridge = load_module()
REPO_ROOT = Path(__file__).resolve().parents[1]
FIXTURES = REPO_ROOT / "tests" / "fixtures" / "deepyolo"
SUMMARY_SCHEMA = json.loads((REPO_ROOT / "schemas" / "deepyolo_semantic_summary.schema.json").read_text(encoding="utf-8"))


def load_fixture(name: str):
    return bridge.read_last_json_line(FIXTURES / name)


class DeepYoloSemanticBridgeTests(unittest.TestCase):
    def test_read_last_json_line_expands_scan_window_for_large_packet(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "semantic.jsonl"
            packet = {"frame_id": 7, "padding": "x" * 70_000}
            path.write_text(json.dumps(packet) + "\n", encoding="utf-8")

            loaded = bridge.read_last_json_line(path)

            self.assertEqual(loaded["frame_id"], 7)

    def assert_summary_valid(self, summary):
        validate(instance=summary, schema=SUMMARY_SCHEMA)

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
        self.assertEqual(summary["schema_version"], 1)
        self.assertEqual(summary["source_status"], "fresh")
        self.assertEqual(summary["recommended_action"], "hold_or_pause")
        self.assertEqual(summary["effective_action"], "hold_or_pause")
        self.assertEqual(summary["age_ms"], 500)
        self.assertEqual(summary["packet_age_ms"], 500)
        self.assertEqual(summary["source_file_age_ms"], 500)
        self.assertEqual(len(summary["objects"]), 1)
        self.assertEqual(summary["objects"][0]["class_name"], "person")
        self.assert_summary_valid(summary)

    def test_event_only_idle_keeps_context_but_ignores_action(self):
        packet = load_fixture("event_only_stable.jsonl")

        summary = bridge.build_semantic_summary(
            packet,
            source_path=None,
            bridge_timestamp_ms=7000,
            source_file_mtime_ms=1000,
            stale_ms=3000,
            source_stale_ms=10000,
        )

        self.assertEqual(summary["source_status"], "event_only_idle")
        self.assertTrue(summary["stale"])
        self.assertEqual(summary["source_file_age_ms"], 6000)
        self.assertEqual(summary["recommended_action"], "hold_or_pause")
        self.assertEqual(summary["effective_action"], "ignored")
        self.assertEqual(summary["effective_action_reason"], "event_only_idle")
        self.assert_summary_valid(summary)

    def test_truly_stale_source_is_distinct_from_event_only_idle(self):
        packet = load_fixture("event_only_stable.jsonl")

        summary = bridge.build_semantic_summary(
            packet,
            source_path=None,
            bridge_timestamp_ms=12000,
            source_file_mtime_ms=1000,
            stale_ms=3000,
            source_stale_ms=10000,
        )

        self.assertEqual(summary["source_status"], "stale")
        self.assertTrue(summary["stale"])
        self.assertEqual(summary["effective_action"], "ignored")
        self.assert_summary_valid(summary)

    def test_clock_skew_ignores_otherwise_actionable_packet(self):
        packet = load_fixture("fresh_high_risk.jsonl")

        summary = bridge.build_semantic_summary(
            packet,
            source_path=None,
            bridge_timestamp_ms=15100,
            source_file_mtime_ms=15000,
            clock_skew_limit_ms=1000,
        )

        self.assertEqual(summary["clock_skew_ms"], -5000)
        self.assertEqual(summary["source_status"], "clock_skew")
        self.assertTrue(summary["stale"])
        self.assertEqual(summary["recommended_action"], "hold_or_pause")
        self.assertEqual(summary["effective_action"], "ignored")
        self.assert_summary_valid(summary)

    def test_depth_insufficient_ignores_non_normal_action(self):
        packet = load_fixture("depth_insufficient.jsonl")

        summary = bridge.build_semantic_summary(
            packet,
            source_path=None,
            bridge_timestamp_ms=10100,
            source_file_mtime_ms=10000,
        )

        self.assertEqual(summary["source_status"], "depth_insufficient")
        self.assertFalse(summary["action_depth_sufficient"])
        self.assertEqual(summary["recommended_action"], "slow_and_watch")
        self.assertEqual(summary["effective_action"], "ignored")
        self.assert_summary_valid(summary)

    def test_unrelated_depth_does_not_validate_high_risk_action(self):
        packet = {
            "timestamp_ms": 10000,
            "depth_valid": True,
            "objects": [
                {
                    "class_name": "person",
                    "risk_level": "high",
                    "region": "center",
                    "distance_m": None,
                    "depth_valid": False,
                },
                {
                    "class_name": "chair",
                    "risk_level": "medium",
                    "region": "left",
                    "distance_m": 1.0,
                    "depth_valid": True,
                },
            ],
        }

        summary = bridge.build_semantic_summary(
            packet,
            source_path=None,
            bridge_timestamp_ms=10100,
            source_file_mtime_ms=10000,
        )

        self.assertEqual(summary["recommended_action"], "slow_and_watch")
        self.assertEqual(summary["source_status"], "depth_insufficient")
        self.assertEqual(summary["effective_action"], "ignored")
        self.assert_summary_valid(summary)

    def test_read_last_json_line_ignores_partial_trailing_line(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "semantic_stream_a.jsonl"
            path.write_text('{"frame_id":1}\nnot-json\n{"frame_id":2}', encoding="utf-8")

            self.assertEqual(bridge.read_last_json_line(path)["frame_id"], 2)

    def test_emit_once_uses_source_file_mtime_separately_from_packet_timestamp(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "semantic_stream_a.jsonl"
            path.write_text('{"timestamp_ms":4500,"depth_valid":true,"objects":[]}\n', encoding="utf-8")
            path.touch()
            mtime_ms = bridge.file_mtime_ms(path)
            args = type(
                "Args",
                (),
                {
                    "input_jsonl": path,
                    "input_dir": Path(tmp),
                    "max_objects": 8,
                    "stale_ms": 3000,
                    "source_stale_ms": 10000,
                    "clock_skew_limit_ms": 1000,
                    "center_stop_m": 1.5,
                    "near_watch_m": 2.5,
                },
            )()

            with patch.object(bridge, "now_ms", return_value=mtime_ms + 500):
                summary = bridge.emit_once(args)

            self.assertEqual(summary["packet_timestamp_ms"], 4500)
            self.assertEqual(summary["source_file_mtime_ms"], mtime_ms)
            self.assertEqual(summary["source_file_age_ms"], 500)
            self.assertNotEqual(summary["packet_age_ms"], summary["source_file_age_ms"])
            self.assert_summary_valid(summary)

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
            self.assertEqual(summary["source_status"], "unavailable")
            self.assertEqual(summary["effective_action"], "ignored")
            self.assert_summary_valid(summary)

    def test_write_summary_replaces_output_atomically(self):
        with tempfile.TemporaryDirectory() as tmp:
            output = Path(tmp) / "vision_semantic_summary.json"
            bridge.write_summary({"source": "deepyolo_realsense"}, output)

            self.assertEqual(json.loads(output.read_text(encoding="utf-8"))["source"], "deepyolo_realsense")
            self.assertFalse((Path(tmp) / "vision_semantic_summary.json.tmp").exists())


if __name__ == "__main__":
    unittest.main()
