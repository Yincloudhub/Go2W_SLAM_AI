from __future__ import annotations

import unittest

from scripts.capture_xt16_calibration_scene import (
    assess_scene,
    build_statistics,
    compact_sample,
    selected_map,
    validate_measurements,
)


def producer_summary(
    timestamp_ms: int,
    *,
    front: float = 1.0,
    left: float = 1.1,
    right: float = 1.2,
    rear: float = 1.3,
    stale_reasons: list[str] | None = None,
) -> dict:
    return {
        "timestamp_ms": timestamp_ms,
        "source": "lidar_pointcloud",
        "frame_id": "rslidar",
        "front_clearance_m": front,
        "left_clearance_m": left,
        "right_clearance_m": right,
        "rear_clearance_m": rear,
        "body_clearance_m": {
            "front": front,
            "left": left,
            "right": right,
            "rear": rear,
        },
        "low_hazard_clearance_m": {
            "front": None,
            "left": None,
            "right": None,
            "rear": None,
        },
        "roi_confidence": {direction: 1.0 for direction in ("front", "left", "right", "rear")},
        "stale": True,
        "stale_reasons": stale_reasons or ["uncalibrated_xt16_geometry"],
        "latency_ms": 100.0,
        "summary": {
            "processing_latency_ms": 150.0,
            "points_total": 50000,
            "points_excluded_footprint": 4000,
        },
    }


class Xt16CalibrationSceneTests(unittest.TestCase):
    def test_selected_map_uses_requested_map_id(self) -> None:
        registry = {"maps": [{"map_id": "a"}, {"map_id": "b", "pcd_path": "/tmp/b.pcd"}]}

        self.assertEqual(selected_map(registry, "b")["pcd_path"], "/tmp/b.pcd")
        self.assertIsNone(selected_map(registry, "missing"))

    def samples(self, count: int = 5) -> list[dict]:
        return [
            compact_sample(
                producer_summary(1000 + index, front=1.0 + index * 0.001),
                observed_ms=1100 + index,
            )
            for index in range(count)
        ]

    def test_compact_sample_uses_effective_sensor_age(self) -> None:
        sample = compact_sample(producer_summary(1000), observed_ms=1150)

        self.assertEqual(sample["receipt_age_ms"], 150)
        self.assertEqual(sample["sensor_latency_ms"], 100.0)
        self.assertEqual(sample["effective_age_ms"], 250.0)

    def test_baseline_requires_all_four_measurements(self) -> None:
        reasons = validate_measurements(
            "baseline",
            {"front": 1.0, "left": 1.0, "right": 1.0, "rear": None},
        )

        self.assertEqual(reasons, ["missing_or_invalid_measured_rear_m"])

    def test_direction_scene_accepts_stable_matching_measurement(self) -> None:
        samples = self.samples()
        statistics = build_statistics(samples)

        result = assess_scene(
            "front",
            samples,
            statistics,
            {"front": 1.0, "left": None, "right": None, "rear": None},
            required_samples=5,
            max_abs_error_m=0.15,
            max_span_m=0.08,
            max_effective_age_ms=1000,
        )

        self.assertTrue(result["accepted"], result["reasons"])
        self.assertLess(result["comparisons"]["front"]["absolute_error_m"], 0.01)

    def test_pending_or_other_stale_reason_rejects_scene(self) -> None:
        samples = self.samples()
        samples[0]["stale_reasons"].append("pending_low_hazard:right")
        statistics = build_statistics(samples)

        result = assess_scene(
            "right",
            samples,
            statistics,
            {"front": None, "left": None, "right": 1.2, "rear": None},
            required_samples=5,
            max_abs_error_m=0.15,
            max_span_m=0.08,
            max_effective_age_ms=1000,
        )

        self.assertFalse(result["accepted"])
        self.assertTrue(
            any(reason.startswith("disallowed_stale_reasons") for reason in result["reasons"])
        )

    def test_large_error_and_unstable_span_reject_scene(self) -> None:
        samples = self.samples()
        samples[-1]["clearance_m"]["front"] = 1.7
        statistics = build_statistics(samples)

        result = assess_scene(
            "front",
            samples,
            statistics,
            {"front": 0.5, "left": None, "right": None, "rear": None},
            required_samples=5,
            max_abs_error_m=0.15,
            max_span_m=0.08,
            max_effective_age_ms=1000,
        )

        self.assertFalse(result["accepted"])
        self.assertIn("front_absolute_error_exceeded", result["reasons"])
        self.assertIn("front_stability_span_exceeded", result["reasons"])


if __name__ == "__main__":
    unittest.main()
