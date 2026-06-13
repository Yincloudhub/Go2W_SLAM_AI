import importlib.util
import json
import sys
import tempfile
import unittest
from pathlib import Path

from jsonschema import Draft202012Validator


REPO_ROOT = Path(__file__).resolve().parents[1]


def load_module():
    scripts = REPO_ROOT / "scripts"
    sys.path.insert(0, str(scripts))
    path = scripts / "d435_perception_summary.py"
    spec = importlib.util.spec_from_file_location("d435_perception_summary", path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


d435 = load_module()
SCHEMA = json.loads((REPO_ROOT / "schemas" / "d435_perception_summary.schema.json").read_text(encoding="utf-8"))
YOLO_SCHEMA = json.loads((REPO_ROOT / "schemas" / "deepyolo_semantic_summary.schema.json").read_text(encoding="utf-8"))


def depth_packet(timestamp_ms=10_000, sequence=42):
    return {
        "source": "d435_capture_owner",
        "captured_at_ms": timestamp_ms,
        "timestamp_ms": timestamp_ms,
        "frame_sequence": sequence,
        "sensor_timestamp_ms": 1234.5,
        "sensor_timestamp_domain": "hardware_clock",
        "frame_id": "camera_color_optical_frame",
        "front_clearance_m": 0.8,
        "left_clearance_m": 1.2,
        "right_clearance_m": 1.3,
        "center_distance_m": 0.9,
        "confidence": 0.7,
        "roi_confidence": {"front": 0.8, "left": 0.8, "right": 0.8, "center_window": 0.8},
        "stale": False,
    }


def yolo_packet(timestamp_ms=10_000, sequence=42):
    return {
        "session_id": "test",
        "timestamp_ms": timestamp_ms,
        "captured_at_ms": timestamp_ms,
        "frame_sequence": sequence,
        "sensor_timestamp_ms": 1234.5,
        "object_count": 0,
        "high_risk_count": 0,
        "has_high_risk": False,
        "depth_valid": True,
        "objects": [],
    }


def owner(running=True):
    return {
        "pid": 12,
        "running": running,
        "expected_process": "capture",
        "process_start_ticks": 100 if running else None,
        "boot_id": "boot" if running else None,
        "status": "owned" if running else "offline",
    }


class D435PerceptionSummaryTests(unittest.TestCase):
    def test_each_output_preserves_source_frame_metadata_and_schema(self):
        main, depth, yolo = d435.build_summaries(
            depth_packet(),
            yolo_packet(),
            owner=owner(),
            generated_at_ms=10_100,
            generation_id="g1",
        )

        Draft202012Validator(SCHEMA).validate(main)
        Draft202012Validator(YOLO_SCHEMA).validate(yolo)
        self.assertEqual(main["frame_sequence"], 42)
        self.assertEqual(main["depth"]["frame_sequence"], main["yolo"]["frame_sequence"])
        self.assertEqual(main["depth"]["sensor_timestamp_ms"], main["yolo"]["sensor_timestamp_ms"])
        self.assertEqual(main["yolo"]["capture_frame_lag"], 0)
        self.assertEqual(depth["source"], "stereo_depth")
        self.assertEqual(yolo["source"], "deepyolo_realsense")

    def test_async_frame_offset_is_signed(self):
        main, _, _ = d435.build_summaries(
            depth_packet(sequence=50),
            yolo_packet(sequence=45),
            owner=owner(),
            generated_at_ms=10_100,
        )
        self.assertEqual(main["status"], "fresh")
        self.assertEqual(main["yolo"]["capture_frame_lag"], 5)

        main, _, _ = d435.build_summaries(
            depth_packet(sequence=45),
            yolo_packet(sequence=50),
            owner=owner(),
            generated_at_ms=10_100,
        )
        self.assertEqual(main["status"], "fresh")
        self.assertEqual(main["yolo"]["capture_frame_lag"], -5)

    def test_malformed_depth_is_invalid_but_outputs_remain_schema_valid(self):
        malformed = depth_packet()
        malformed["sensor_timestamp_ms"] = "not-a-number"
        malformed["sensor_timestamp_domain"] = 42
        malformed["front_clearance_m"] = "near"
        malformed["frame_id"] = ""

        main, depth, yolo = d435.build_summaries(
            malformed,
            yolo_packet(),
            owner=owner(),
            generated_at_ms=10_100,
        )

        Draft202012Validator(SCHEMA).validate(main)
        Draft202012Validator(YOLO_SCHEMA).validate(yolo)
        self.assertEqual(main["status"], "invalid")
        self.assertTrue(depth["stale"])
        self.assertIsNone(depth["sensor_timestamp_ms"])
        self.assertIsNone(depth["sensor_timestamp_domain"])
        self.assertIsNone(depth["front_clearance_m"])
        self.assertEqual(depth["frame_id"], "camera_color_optical_frame")

    def test_stale_depth_invalidates_main_even_when_file_exists(self):
        main, depth, _ = d435.build_summaries(
            depth_packet(timestamp_ms=5_000),
            yolo_packet(timestamp_ms=9_900),
            owner=owner(),
            generated_at_ms=10_100,
            depth_stale_ms=500,
        )

        self.assertEqual(main["status"], "stale")
        self.assertTrue(main["stale"])
        self.assertTrue(depth["stale"])
        self.assertIn("d435_depth", main["degraded_capabilities"])

    def test_owner_mismatch_marks_depth_offline(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            (root / "9").mkdir()
            (root / "9" / "cmdline").write_bytes(b"python\0unrelated.py\0")
            pid_file = root / "capture.pid"
            pid_file.write_text("9\n", encoding="ascii")

            owner = d435.capture_owner_status(
                pid_file,
                expected_fragment="yolo_test_realsense_headless",
                proc_root=root,
            )
            main, _, _ = d435.build_summaries(
                depth_packet(),
                None,
                owner=owner,
                generated_at_ms=10_100,
            )

        self.assertFalse(owner["running"])
        self.assertEqual(main["status"], "offline")

    def test_missing_yolo_does_not_block_fresh_depth(self):
        main, depth, yolo = d435.build_summaries(
            depth_packet(),
            None,
            owner=owner(),
            generated_at_ms=10_100,
        )

        self.assertEqual(main["status"], "fresh")
        self.assertFalse(depth["stale"])
        self.assertFalse(yolo["available"])
        self.assertIn("d435_yolo", main["degraded_capabilities"])
        self.assertNotIn("d435_depth", main["degraded_capabilities"])

    def test_future_depth_timestamp_is_invalid(self):
        main, _, _ = d435.build_summaries(
            depth_packet(timestamp_ms=10_200),
            None,
            owner=owner(),
            generated_at_ms=10_100,
        )

        self.assertEqual(main["status"], "invalid")

    def test_frame_sequence_rollback_is_invalid_within_owner_session(self):
        main, depth, _ = d435.build_summaries(
            depth_packet(sequence=41),
            None,
            owner=owner(),
            generated_at_ms=10_100,
            previous_frame_sequence=42,
        )

        self.assertEqual(main["status"], "invalid")
        self.assertTrue(depth["stale"])

    def test_outputs_are_individually_atomic_and_share_generation(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            main, depth, yolo = d435.build_summaries(
                depth_packet(),
                yolo_packet(),
                owner=owner(),
                generated_at_ms=10_100,
                generation_id="same-generation",
            )
            paths = [root / "d435.json", root / "depth.json", root / "yolo.json"]
            d435.write_outputs(
                main,
                depth,
                yolo,
                output=paths[0],
                depth_output=paths[1],
                yolo_output=paths[2],
            )

            values = [json.loads(path.read_text(encoding="utf-8")) for path in paths]
            self.assertEqual({value["generation_id"] for value in values}, {"same-generation"})
            self.assertEqual(list(root.glob("*.tmp")), [])


if __name__ == "__main__":
    unittest.main()
