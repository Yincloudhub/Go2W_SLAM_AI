import json
import tempfile
import unittest
from pathlib import Path

from jsonschema import Draft202012Validator
from referencing import Registry, Resource

from edge_autonomy.perception_context import (
    SensorSequenceTracker,
    build_perception_context,
    load_d435_envelopes,
    load_ti_nx_envelope,
    load_xt16_geometry_envelope,
    reserved_motion_envelope,
)


REPO_ROOT = Path(__file__).resolve().parents[1]
SENSOR_SCHEMA = json.loads((REPO_ROOT / "schemas" / "sensor_envelope_v1.schema.json").read_text(encoding="utf-8"))
CONTEXT_SCHEMA = json.loads((REPO_ROOT / "schemas" / "perception_context_v1.schema.json").read_text(encoding="utf-8"))
REGISTRY = Registry().with_resource(SENSOR_SCHEMA["$id"], Resource.from_contents(SENSOR_SCHEMA))
SENSOR_VALIDATOR = Draft202012Validator(SENSOR_SCHEMA)
CONTEXT_VALIDATOR = Draft202012Validator(CONTEXT_SCHEMA, registry=REGISTRY)


def write_json(root: Path, name: str, value: dict) -> Path:
    path = root / name
    path.write_text(json.dumps(value), encoding="utf-8")
    return path


def xt16_summary(timestamp_ms: int = 10_000) -> dict:
    return {
        "type": "local_obstacle_summary",
        "schema_version": 2,
        "source": "lidar_pointcloud",
        "timestamp_ms": timestamp_ms,
        "sequence": 7,
        "frame_id": "base_link",
        "parameters": {"calibrated": True, "calibration_id": "xt16-test-v1"},
        "front_clearance_m": 2.0,
        "left_clearance_m": 1.5,
        "right_clearance_m": 1.7,
        "rear_clearance_m": 1.2,
        "confidence": 0.8,
        "latency_ms": 20,
        "stale": False,
        "stale_reasons": [],
        "roi_confidence": {"front": 0.8, "left": 0.8, "right": 0.8, "rear": 0.8},
    }


def d435_summary(timestamp_ms: int = 10_000) -> dict:
    depth = {
        "source_status": "fresh",
        "captured_at_ms": timestamp_ms,
        "timestamp_ms": timestamp_ms,
        "frame_sequence": 42,
        "frame_id": "camera_color_optical_frame",
        "confidence": 0.8,
        "stale": False,
        "front_clearance_m": 1.8,
        "left_clearance_m": 1.4,
        "right_clearance_m": 1.5,
        "center_distance_m": 1.7,
        "roi_confidence": {"front": 0.8},
        "generation_id": "generation-test",
    }
    yolo = {
        "source_status": "fresh",
        "captured_at_ms": timestamp_ms,
        "frame_sequence": 42,
        "objects": [{"class_name": "person", "confidence": 0.9}],
        "generation_id": "generation-test",
    }
    return {
        "schema_version": 1,
        "schema": "go2w_d435_perception_summary_v1",
        "source": "d435_perception",
        "producer": "d435_perception_sidecar",
        "generation_id": "generation-test",
        "confidence": 0.8,
        "owner": {
            "pid": 123,
            "running": True,
            "status": "owned",
            "expected_process": "deepyolo_headless",
            "process_start_ticks": 456,
            "boot_id": "boot-test",
        },
        "capture": {"frame_id": "camera_color_optical_frame"},
        "depth": depth,
        "yolo": yolo,
    }


def ti_nx_summary(timestamp_ms: int = 10_000) -> dict:
    return {
        "schema_version": 1,
        "node_id": "nx_ti_radar_01",
        "sensor_type": "ti_millimeter_wave_radar",
        "source": "nx_ti_radar",
        "timestamp_ms": timestamp_ms,
        "sequence": 9,
        "clock_domain": "unix_epoch_ms",
        "confidence": 0.85,
        "latency_ms": 40,
        "stale": False,
        "health": {"status": "ok"},
        "policy": {"mode": "semantic_only", "calibrated": False, "safety_candidate": False},
        "observations": [{"track_id": "track-1", "range_m": 2.4}],
        "events": [{"event_type": "moving_person"}],
        "summary": {"object_count": 1},
    }


class PerceptionContextTests(unittest.TestCase):
    def assert_sensor_valid(self, envelope: dict) -> None:
        errors = sorted(SENSOR_VALIDATOR.iter_errors(envelope), key=lambda item: list(item.path))
        self.assertEqual(errors, [], [error.message for error in errors])

    def assert_context_valid(self, context: dict) -> None:
        errors = sorted(CONTEXT_VALIDATOR.iter_errors(context), key=lambda item: list(item.path))
        self.assertEqual(errors, [], [error.message for error in errors])

    def test_xt16_requires_live_producer_and_verified_calibration(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            artifact = write_json(root, "xt16.json", xt16_summary())
            pid_file = root / "producer.pid"
            pid_file.write_text("123", encoding="ascii")
            envelope = load_xt16_geometry_envelope(
                artifact,
                received_ms=10_100,
                pid_file=pid_file,
                process_probe=lambda pid, expected: pid == 123 and expected == "xt16_lidar_geometry_summary.py",
            )
            self.assertEqual(envelope["status"], "fresh")
            self.assertEqual(envelope["sequence"], 7)
            self.assert_sensor_valid(envelope)

            offline = load_xt16_geometry_envelope(
                artifact,
                received_ms=10_100,
                pid_file=pid_file,
                process_probe=lambda pid, expected: False,
            )
            self.assertEqual(offline["status"], "offline")
            self.assert_sensor_valid(offline)

            uncalibrated_data = xt16_summary()
            uncalibrated_data["parameters"] = {"calibrated": False, "calibration_id": None}
            uncalibrated_artifact = write_json(root, "xt16-uncalibrated.json", uncalibrated_data)
            uncalibrated = load_xt16_geometry_envelope(
                uncalibrated_artifact,
                received_ms=10_100,
                pid_file=pid_file,
                process_probe=lambda pid, expected: True,
            )
            self.assertEqual(uncalibrated["status"], "uncalibrated")
            self.assert_sensor_valid(uncalibrated)

    def test_d435_depth_and_yolo_share_owner_but_keep_independent_freshness(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            artifact = write_json(Path(temp), "d435.json", d435_summary())
            depth, yolo = load_d435_envelopes(
                artifact,
                received_ms=10_500,
                process_probe=lambda pid, expected: pid == 123 and expected == "deepyolo_headless",
            )
            self.assertEqual(depth["status"], "fresh")
            self.assertEqual(yolo["status"], "fresh")
            self.assertEqual(depth["sequence"], yolo["sequence"])
            self.assert_sensor_valid(depth)
            self.assert_sensor_valid(yolo)

            stale_depth, fresh_yolo = load_d435_envelopes(
                artifact,
                received_ms=11_500,
                depth_stale_ms=1000,
                yolo_stale_ms=3000,
                process_probe=lambda pid, expected: True,
            )
            self.assertEqual(stale_depth["status"], "stale")
            self.assertEqual(fresh_yolo["status"], "fresh")

            offline = load_d435_envelopes(
                artifact,
                received_ms=10_500,
                process_probe=lambda pid, expected: False,
            )
            self.assertEqual([item["status"] for item in offline], ["offline", "offline"])

    def test_missing_or_malformed_artifact_fails_closed(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            missing = load_ti_nx_envelope(
                root / "missing.json",
                received_ms=10_100,
                producer_instance_id="bridge-session-1",
            )
            self.assertEqual(missing["status"], "offline")
            self.assert_sensor_valid(missing)

            malformed = root / "malformed.json"
            malformed.write_text("{", encoding="utf-8")
            invalid = load_ti_nx_envelope(
                malformed,
                received_ms=10_100,
                producer_instance_id="bridge-session-1",
            )
            self.assertEqual(invalid["status"], "invalid")
            self.assert_sensor_valid(invalid)

    def test_ti_nx_artifact_is_not_fresh_without_bridge_evidence(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            artifact = write_json(Path(temp), "edge.json", ti_nx_summary())
            unverified = load_ti_nx_envelope(artifact, received_ms=10_100)
            self.assertEqual(unverified["status"], "offline")
            self.assertIn("producer_online_unverified", unverified["status_reasons"])

            fresh = load_ti_nx_envelope(
                artifact,
                received_ms=10_100,
                producer_instance_id="bridge-session-1",
            )
            self.assertEqual(fresh["status"], "fresh")
            self.assertEqual(fresh["sequence"], 9)
            self.assert_sensor_valid(fresh)

    def test_context_aggregates_only_fresh_semantics_and_reserves_motion_sources(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            d435_path = write_json(root, "d435.json", d435_summary())
            ti_path = write_json(root, "edge.json", ti_nx_summary())
            sources = load_d435_envelopes(
                d435_path,
                received_ms=10_100,
                process_probe=lambda pid, expected: True,
            )
            sources.append(
                load_ti_nx_envelope(
                    ti_path,
                    received_ms=10_100,
                    producer_instance_id="bridge-session-1",
                )
            )
            sources.append(reserved_motion_envelope("xt16_imu_motion", "imu_motion", received_ms=10_100))
            sources.append(reserved_motion_envelope("unitree_odometry_motion", "odometry_motion", received_ms=10_100))
            context = build_perception_context(sources, generated_at_ms=10_100, context_id="pc-test")

            self.assertEqual(context["policy"]["motion_authority"], "slam_gateway")
            self.assertFalse(context["policy"]["llm_direct_motion"])
            self.assertFalse(context["policy"]["raw_sensor_streams_allowed"])
            self.assertEqual(len(context["visual_objects"]), 1)
            self.assertEqual(len(context["radar_tracks"]), 1)
            self.assertIsNone(context["local_geometry"]["primary"])
            self.assertEqual(
                [item["source_id"] for item in context["local_geometry"]["forward_supplements"]],
                ["d435_depth"],
            )
            self.assertIn("xt16_imu_motion", context["degraded_capabilities"])
            self.assert_context_valid(context)

    def test_context_rechecks_age_and_does_not_promote_d435_to_primary_geometry(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            artifact = write_json(Path(temp), "d435.json", d435_summary())
            sources = load_d435_envelopes(
                artifact,
                received_ms=10_100,
                process_probe=lambda pid, expected: True,
            )
            context = build_perception_context(sources, generated_at_ms=11_500, context_id="pc-aged")
            self.assertEqual([source["status"] for source in context["sources"]], ["stale", "fresh"])
            self.assertIsNone(context["local_geometry"]["primary"])
            self.assertEqual(context["local_geometry"]["forward_supplements"], [])
            self.assert_context_valid(context)

    def test_d435_unavailable_yolo_is_offline_not_invalid(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            summary = d435_summary()
            summary["yolo"] = {
                "source_status": "unavailable",
                "objects": [],
                "generation_id": "generation-test",
            }
            artifact = write_json(Path(temp), "d435.json", summary)
            _, yolo = load_d435_envelopes(
                artifact,
                received_ms=10_100,
                process_probe=lambda pid, expected: True,
            )
            self.assertEqual(yolo["status"], "offline")
            self.assert_sensor_valid(yolo)

    def test_attribution_cannot_be_overridden_by_payload(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            artifact = write_json(Path(temp), "d435.json", d435_summary())
            depth, _ = load_d435_envelopes(
                artifact,
                received_ms=10_100,
                process_probe=lambda pid, expected: True,
            )
            depth["payload"]["source_id"] = "spoofed"
            depth["payload"]["timestamp_ms"] = 1
            context = build_perception_context([depth], generated_at_ms=10_100)
            supplement = context["local_geometry"]["forward_supplements"][0]
            self.assertEqual(supplement["source_id"], "d435_depth")
            self.assertEqual(supplement["timestamp_ms"], 10_000)

    def test_context_rejects_incomplete_or_excess_sources(self) -> None:
        with self.assertRaisesRegex(ValueError, "missing fields"):
            build_perception_context([{"source_id": "bad"}], generated_at_ms=10_100)
        source = reserved_motion_envelope("motion-0", "imu_motion", received_ms=10_100)
        sources = [{**source, "source_id": f"motion-{index}"} for index in range(33)]
        with self.assertRaisesRegex(ValueError, "at most 32"):
            build_perception_context(sources, generated_at_ms=10_100)

    def test_context_rejects_duplicate_source_ids(self) -> None:
        source = reserved_motion_envelope("motion", "imu_motion", received_ms=10_100)
        with self.assertRaisesRegex(ValueError, "unique"):
            build_perception_context([source, source], generated_at_ms=10_100)

    def test_context_rejects_sequence_rollback_within_generation(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            artifact = Path(temp) / "d435.json"
            tracker = SensorSequenceTracker()
            write_json(artifact.parent, artifact.name, d435_summary())
            first = load_d435_envelopes(
                artifact,
                received_ms=10_100,
                process_probe=lambda pid, expected: True,
            )
            build_perception_context(first, generated_at_ms=10_100, sequence_tracker=tracker)

            rolled_back = d435_summary()
            rolled_back["depth"]["frame_sequence"] = 41
            rolled_back["yolo"]["frame_sequence"] = 41
            write_json(artifact.parent, artifact.name, rolled_back)
            second = load_d435_envelopes(
                artifact,
                received_ms=10_100,
                process_probe=lambda pid, expected: True,
            )
            context = build_perception_context(second, generated_at_ms=10_100, sequence_tracker=tracker)
            self.assertEqual([source["status"] for source in context["sources"]], ["invalid", "invalid"])
            self.assertEqual(context["degraded_capabilities"], ["d435_depth", "d435_yolo"])
            self.assert_context_valid(context)

    def test_schema_rejects_bool_confidence_and_negative_sequence(self) -> None:
        envelope = reserved_motion_envelope("motion", "imu_motion", received_ms=10_100)
        envelope["confidence"] = True
        envelope["sequence"] = -1
        self.assertGreaterEqual(len(list(SENSOR_VALIDATOR.iter_errors(envelope))), 2)


if __name__ == "__main__":
    unittest.main()
