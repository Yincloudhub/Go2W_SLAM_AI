#!/usr/bin/env python3
from __future__ import annotations

import argparse
import hashlib
import json
import math
import subprocess
import time
from pathlib import Path
from typing import Any, Callable, Iterable


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SUMMARY = REPO_ROOT / "artifacts" / "lidar_geometry_summary.json"
DEFAULT_CALIBRATION = REPO_ROOT / "configs" / "perception" / "xt16_geometry_calibration.json"
DEFAULT_REGISTRY = REPO_ROOT / "configs" / "maps" / "go2w_real_site_map_registry.json"
DEFAULT_MAP_ID = "go2w_real_site"
DIRECTIONS = ("front", "left", "right", "rear")
SCENE_DIRECTIONS = {
    "baseline": DIRECTIONS,
    "front": ("front",),
    "left": ("left",),
    "right": ("right",),
    "rear": ("rear",),
}
ALLOWED_PENDING_STALE_REASON = "uncalibrated_xt16_geometry"


def finite_number(value: Any) -> float | None:
    if isinstance(value, bool):
        return None
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def load_json(path: Path) -> dict[str, Any] | None:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError, json.JSONDecodeError):
        return None
    return value if isinstance(value, dict) else None


def sha256_file(path: Path) -> str | None:
    try:
        digest = hashlib.sha256()
        with path.open("rb") as stream:
            for chunk in iter(lambda: stream.read(1024 * 1024), b""):
                digest.update(chunk)
        return digest.hexdigest()
    except OSError:
        return None


def selected_map(registry: dict[str, Any] | None, map_id: str) -> dict[str, Any] | None:
    if not isinstance(registry, dict):
        return None
    for item in registry.get("maps", []):
        if isinstance(item, dict) and item.get("map_id") == map_id:
            return item
    return None


def run_command(command: list[str], *, timeout_s: int = 5) -> dict[str, Any]:
    try:
        completed = subprocess.run(
            command,
            cwd=REPO_ROOT,
            text=True,
            encoding="utf-8",
            errors="replace",
            capture_output=True,
            timeout=timeout_s,
            check=False,
        )
    except (OSError, subprocess.SubprocessError) as exc:
        return {"returncode": None, "stdout": "", "stderr": str(exc)}
    return {
        "returncode": completed.returncode,
        "stdout": completed.stdout[-12000:],
        "stderr": completed.stderr[-4000:],
    }


def percentile(values: Iterable[float], q: float) -> float | None:
    ordered = sorted(float(value) for value in values if math.isfinite(float(value)))
    if not ordered:
        return None
    if len(ordered) == 1:
        return ordered[0]
    position = max(0.0, min(1.0, q)) * (len(ordered) - 1)
    lower = int(position)
    upper = min(lower + 1, len(ordered) - 1)
    fraction = position - lower
    return ordered[lower] * (1.0 - fraction) + ordered[upper] * fraction


def compact_sample(summary: dict[str, Any], *, observed_ms: int) -> dict[str, Any]:
    producer = summary.get("summary") if isinstance(summary.get("summary"), dict) else {}
    timestamp_ms = int(summary.get("timestamp_ms") or 0)
    sensor_latency_ms = finite_number(summary.get("latency_ms"))
    receipt_age_ms = max(0, observed_ms - timestamp_ms) if timestamp_ms > 0 else None
    effective_age_ms = (
        receipt_age_ms + max(0.0, sensor_latency_ms or 0.0)
        if receipt_age_ms is not None
        else None
    )
    return {
        "timestamp_ms": timestamp_ms,
        "observed_ms": observed_ms,
        "source": summary.get("source"),
        "frame_id": summary.get("frame_id"),
        "clearance_m": {
            direction: finite_number(summary.get(f"{direction}_clearance_m"))
            for direction in DIRECTIONS
        },
        "body_clearance_m": {
            direction: finite_number(
                summary.get("body_clearance_m", {}).get(direction)
                if isinstance(summary.get("body_clearance_m"), dict)
                else None
            )
            for direction in DIRECTIONS
        },
        "low_hazard_clearance_m": {
            direction: finite_number(
                summary.get("low_hazard_clearance_m", {}).get(direction)
                if isinstance(summary.get("low_hazard_clearance_m"), dict)
                else None
            )
            for direction in DIRECTIONS
        },
        "roi_confidence": summary.get("roi_confidence", {}),
        "body_roi_confidence": summary.get("body_roi_confidence", {}),
        "low_hazard_roi_confidence": summary.get("low_hazard_roi_confidence", {}),
        "stale": bool(summary.get("stale", True)),
        "stale_reasons": list(summary.get("stale_reasons", []))
        if isinstance(summary.get("stale_reasons"), list)
        else [],
        "pending_body_directions": list(summary.get("pending_body_directions", []))
        if isinstance(summary.get("pending_body_directions"), list)
        else [],
        "pending_low_hazard_directions": list(summary.get("pending_low_hazard_directions", []))
        if isinstance(summary.get("pending_low_hazard_directions"), list)
        else [],
        "blocked_directions": list(summary.get("blocked_directions", []))
        if isinstance(summary.get("blocked_directions"), list)
        else [],
        "recommended_action": summary.get("recommended_action"),
        "sensor_latency_ms": sensor_latency_ms,
        "receipt_age_ms": receipt_age_ms,
        "effective_age_ms": effective_age_ms,
        "processing_latency_ms": finite_number(producer.get("processing_latency_ms")),
        "points_total": int(producer.get("points_total") or 0),
        "points_excluded_footprint": int(producer.get("points_excluded_footprint") or 0),
        "roi_counts": producer.get("roi_counts", {}),
        "body_cluster_support": producer.get("body_cluster_support", {}),
        "low_hazard_cluster_support": producer.get("low_hazard_cluster_support", {}),
        "footprint_m": producer.get("footprint_m", {}),
        "axes": producer.get("axes", {}),
    }


def collect_samples(
    summary_path: Path,
    *,
    sample_count: int,
    timeout_s: float,
    poll_interval_s: float = 0.05,
    now_ms_fn: Callable[[], int] | None = None,
    monotonic_fn: Callable[[], float] | None = None,
    sleep_fn: Callable[[float], None] | None = None,
) -> list[dict[str, Any]]:
    now_ms_fn = now_ms_fn or (lambda: int(time.time() * 1000))
    monotonic_fn = monotonic_fn or time.monotonic
    sleep_fn = sleep_fn or time.sleep
    deadline = monotonic_fn() + max(0.1, timeout_s)
    samples: list[dict[str, Any]] = []
    seen_timestamps: set[int] = set()
    while len(samples) < sample_count and monotonic_fn() < deadline:
        summary = load_json(summary_path)
        timestamp_ms = int(summary.get("timestamp_ms") or 0) if isinstance(summary, dict) else 0
        if timestamp_ms > 0 and timestamp_ms not in seen_timestamps and isinstance(summary, dict):
            seen_timestamps.add(timestamp_ms)
            samples.append(compact_sample(summary, observed_ms=now_ms_fn()))
        if len(samples) < sample_count:
            sleep_fn(max(0.01, poll_interval_s))
    return samples


def value_stats(values: Iterable[float | None]) -> dict[str, Any]:
    clean = [float(value) for value in values if finite_number(value) is not None]
    p05 = percentile(clean, 0.05)
    median = percentile(clean, 0.50)
    p95 = percentile(clean, 0.95)
    return {
        "count": len(clean),
        "min": min(clean) if clean else None,
        "p05": p05,
        "median": median,
        "p95": p95,
        "max": max(clean) if clean else None,
        "p95_p05_span": p95 - p05 if p05 is not None and p95 is not None else None,
    }


def build_statistics(samples: list[dict[str, Any]]) -> dict[str, Any]:
    channels = {
        "aggregate": "clearance_m",
        "body": "body_clearance_m",
        "low_hazard": "low_hazard_clearance_m",
    }
    return {
        "samples": len(samples),
        "channels": {
            channel: {
                direction: value_stats(
                    sample.get(key, {}).get(direction)
                    for sample in samples
                    if isinstance(sample.get(key), dict)
                )
                for direction in DIRECTIONS
            }
            for channel, key in channels.items()
        },
        "sensor_latency_ms": value_stats(sample.get("sensor_latency_ms") for sample in samples),
        "processing_latency_ms": value_stats(
            sample.get("processing_latency_ms") for sample in samples
        ),
        "effective_age_ms": value_stats(sample.get("effective_age_ms") for sample in samples),
        "points_total": value_stats(sample.get("points_total") for sample in samples),
        "points_excluded_footprint": value_stats(
            sample.get("points_excluded_footprint") for sample in samples
        ),
    }


def validate_measurements(scene: str, measurements: dict[str, float | None]) -> list[str]:
    reasons: list[str] = []
    for direction in SCENE_DIRECTIONS[scene]:
        value = finite_number(measurements.get(direction))
        if value is None or value < 0.0 or value > 6.0:
            reasons.append(f"missing_or_invalid_measured_{direction}_m")
    return reasons


def assess_scene(
    scene: str,
    samples: list[dict[str, Any]],
    statistics: dict[str, Any],
    measurements: dict[str, float | None],
    *,
    required_samples: int,
    max_abs_error_m: float,
    max_span_m: float,
    max_effective_age_ms: float,
) -> dict[str, Any]:
    reasons = validate_measurements(scene, measurements)
    if len(samples) < required_samples:
        reasons.append(f"insufficient_unique_samples:{len(samples)}/{required_samples}")

    for sample in samples:
        if sample.get("source") != "lidar_pointcloud":
            reasons.append("unexpected_sensor_source")
            break
    disallowed_stale_reasons = sorted(
        {
            str(reason)
            for sample in samples
            for reason in sample.get("stale_reasons", [])
            if str(reason) != ALLOWED_PENDING_STALE_REASON
        }
    )
    if disallowed_stale_reasons:
        reasons.append("disallowed_stale_reasons:" + ",".join(disallowed_stale_reasons))

    max_age = statistics.get("effective_age_ms", {}).get("max")
    if finite_number(max_age) is None or float(max_age) > max_effective_age_ms:
        reasons.append("effective_sensor_age_exceeded")

    comparisons: dict[str, Any] = {}
    aggregate = statistics.get("channels", {}).get("aggregate", {})
    for direction in SCENE_DIRECTIONS[scene]:
        measured = finite_number(measurements.get(direction))
        direction_stats = aggregate.get(direction, {}) if isinstance(aggregate, dict) else {}
        median = finite_number(direction_stats.get("median"))
        span = finite_number(direction_stats.get("p95_p05_span"))
        error = median - measured if median is not None and measured is not None else None
        comparisons[direction] = {
            "measured_m": measured,
            "xt16_median_m": median,
            "signed_error_m": error,
            "absolute_error_m": abs(error) if error is not None else None,
            "p95_p05_span_m": span,
        }
        if median is None:
            reasons.append(f"missing_xt16_{direction}_clearance")
        elif measured is not None and abs(median - measured) > max_abs_error_m:
            reasons.append(f"{direction}_absolute_error_exceeded")
        if span is None or span > max_span_m:
            reasons.append(f"{direction}_stability_span_exceeded")

    return {
        "accepted": not reasons,
        "reasons": sorted(set(reasons)),
        "thresholds": {
            "required_samples": required_samples,
            "max_abs_error_m": max_abs_error_m,
            "max_p95_p05_span_m": max_span_m,
            "max_effective_age_ms": max_effective_age_ms,
        },
        "comparisons": comparisons,
    }


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Capture one stationary XT16 calibration scene. No motion command is sent."
    )
    parser.add_argument("--scene", choices=tuple(SCENE_DIRECTIONS), required=True)
    parser.add_argument("--test-id", default="")
    parser.add_argument("--operator", required=True)
    parser.add_argument("--operator-note", default="")
    parser.add_argument("--summary", default=str(DEFAULT_SUMMARY))
    parser.add_argument("--calibration", default=str(DEFAULT_CALIBRATION))
    parser.add_argument("--registry", default=str(DEFAULT_REGISTRY))
    parser.add_argument("--map-id", default=DEFAULT_MAP_ID)
    parser.add_argument("--samples", type=int, default=25)
    parser.add_argument("--timeout-s", type=float, default=20.0)
    parser.add_argument("--max-abs-error-m", type=float, default=0.15)
    parser.add_argument("--max-span-m", type=float, default=0.08)
    parser.add_argument("--max-effective-age-ms", type=float, default=1000.0)
    for direction in DIRECTIONS:
        parser.add_argument(f"--measured-{direction}-m", type=float)
    parser.add_argument("--output", default="")
    parser.add_argument("--json", action="store_true")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    timestamp_ms = int(time.time() * 1000)
    summary_path = Path(args.summary)
    calibration_path = Path(args.calibration)
    registry_path = Path(args.registry)
    calibration = load_json(calibration_path)
    registry = load_json(registry_path)
    map_profile = selected_map(registry, args.map_id)
    pcd_path = Path(str(map_profile.get("pcd_path"))) if isinstance(map_profile, dict) and map_profile.get("pcd_path") else None
    measurements = {
        direction: getattr(args, f"measured_{direction}_m")
        for direction in DIRECTIONS
    }
    samples = collect_samples(
        summary_path,
        sample_count=max(1, args.samples),
        timeout_s=max(0.1, args.timeout_s),
    )
    statistics = build_statistics(samples)
    assessment = assess_scene(
        args.scene,
        samples,
        statistics,
        measurements,
        required_samples=max(1, args.samples),
        max_abs_error_m=max(0.0, args.max_abs_error_m),
        max_span_m=max(0.0, args.max_span_m),
        max_effective_age_ms=max(1.0, args.max_effective_age_ms),
    )
    git_head = run_command(["git", "rev-parse", "HEAD"])
    git_status = run_command(["git", "status", "--short"])
    test_id = args.test_id or f"xt16_{args.scene}"
    artifact = {
        "schema_version": 1,
        "type": "go2w_xt16_calibration_scene",
        "timestamp_ms": timestamp_ms,
        "test_id": test_id,
        "scene": args.scene,
        "operator": args.operator,
        "operator_note": args.operator_note,
        "capture_mode": "read_only",
        "motion_commands_sent": False,
        "git": {
            "head": git_head["stdout"].strip(),
            "dirty": bool(git_status["stdout"].strip()),
            "status": git_status["stdout"].splitlines(),
        },
        "calibration_record": {
            "path": str(calibration_path),
            "sha256": sha256_file(calibration_path),
            "sensor": calibration.get("sensor") if isinstance(calibration, dict) else None,
            "sensor_serial": (
                calibration.get("sensor_serial") if isinstance(calibration, dict) else None
            ),
            "status": calibration.get("status") if isinstance(calibration, dict) else None,
            "parameters": calibration.get("parameters") if isinstance(calibration, dict) else None,
        },
        "map_identity": {
            "map_id": args.map_id,
            "registry_path": str(registry_path),
            "registry_sha256": sha256_file(registry_path),
            "pcd_path": str(pcd_path) if pcd_path is not None else None,
            "pcd_sha256": sha256_file(pcd_path) if pcd_path is not None else None,
        },
        "summary_path": str(summary_path),
        "physical_measurements_m": measurements,
        "statistics": statistics,
        "assessment": assessment,
        "samples": samples,
    }
    output = (
        Path(args.output)
        if args.output
        else REPO_ROOT
        / "artifacts"
        / "xt16_calibration"
        / f"{timestamp}_{test_id}.json"
    )
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(artifact, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    result = {
        "artifact": str(output),
        "scene": args.scene,
        "accepted": assessment["accepted"],
        "reasons": assessment["reasons"],
        "comparisons": assessment["comparisons"],
        "samples": len(samples),
        "motion_commands_sent": False,
    }
    if args.json:
        print(json.dumps(result, ensure_ascii=False, indent=2))
    else:
        print(json.dumps(result, ensure_ascii=False))
    return 0 if assessment["accepted"] else 4


if __name__ == "__main__":
    raise SystemExit(main())
