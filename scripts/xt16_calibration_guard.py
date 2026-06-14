#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
from typing import Any, Mapping


PARAMETER_ENV = {
    "range_m": "GO2W_XT16_GEOMETRY_RANGE_M",
    "percentile": "GO2W_XT16_GEOMETRY_PERCENTILE",
    "min_points_per_roi": "GO2W_XT16_GEOMETRY_MIN_POINTS_PER_ROI",
    "front_half_width_m": "GO2W_XT16_GEOMETRY_FRONT_HALF_WIDTH_M",
    "side_forward_m": "GO2W_XT16_GEOMETRY_SIDE_FORWARD_M",
    "rear_half_width_m": "GO2W_XT16_GEOMETRY_REAR_HALF_WIDTH_M",
    "footprint_front_m": "GO2W_XT16_GEOMETRY_FOOTPRINT_FRONT_M",
    "footprint_rear_m": "GO2W_XT16_GEOMETRY_FOOTPRINT_REAR_M",
    "footprint_half_width_m": "GO2W_XT16_GEOMETRY_FOOTPRINT_HALF_WIDTH_M",
    "footprint_filter_margin_m": "GO2W_XT16_GEOMETRY_FOOTPRINT_FILTER_MARGIN_M",
    "footprint_lateral_filter_margin_m": "GO2W_XT16_GEOMETRY_FOOTPRINT_LATERAL_FILTER_MARGIN_M",
    "min_z_m": "GO2W_XT16_GEOMETRY_MIN_Z_M",
    "body_min_z_m": "GO2W_XT16_GEOMETRY_BODY_MIN_Z_M",
    "max_z_m": "GO2W_XT16_GEOMETRY_MAX_Z_M",
    "clearance_cluster_gap_m": "GO2W_XT16_GEOMETRY_CLUSTER_GAP_M",
    "support_bin_m": "GO2W_XT16_GEOMETRY_SUPPORT_BIN_M",
    "min_spatial_bins": "GO2W_XT16_GEOMETRY_MIN_SPATIAL_BINS",
    "pending_min_points": "GO2W_XT16_GEOMETRY_PENDING_MIN_POINTS",
    "min_cloud_points_for_no_return": "GO2W_XT16_GEOMETRY_MIN_CLOUD_POINTS",
    "no_return_confidence": "GO2W_XT16_GEOMETRY_NO_RETURN_CONFIDENCE",
    "forward_axis": "GO2W_XT16_GEOMETRY_FORWARD_AXIS",
    "lateral_axis": "GO2W_XT16_GEOMETRY_LATERAL_AXIS",
    "vertical_axis": "GO2W_XT16_GEOMETRY_VERTICAL_AXIS",
    "forward_sign": "GO2W_XT16_GEOMETRY_FORWARD_SIGN",
    "lateral_sign": "GO2W_XT16_GEOMETRY_LATERAL_SIGN",
    "vertical_sign": "GO2W_XT16_GEOMETRY_VERTICAL_SIGN",
}


def _same_value(expected: Any, actual: str) -> bool:
    if isinstance(expected, bool):
        values = {"1", "true", "yes"} if expected else {"0", "false", "no"}
        return actual.strip().lower() in values
    if isinstance(expected, (int, float)):
        try:
            number = float(actual)
        except ValueError:
            return False
        return math.isfinite(number) and math.isclose(number, float(expected), rel_tol=0.0, abs_tol=1e-9)
    return str(expected) == actual


def validate_record(record: Mapping[str, Any], environment: Mapping[str, str]) -> tuple[bool, str, str]:
    if record.get("schema_version") != 1:
        return False, "unsupported calibration schema", ""
    if str(record.get("status") or "") != "verified":
        return False, f"calibration status is {record.get('status') or 'missing'}, not verified", ""
    calibration_id = str(record.get("calibration_id") or "").strip()
    if not calibration_id:
        return False, "verified calibration record has no calibration_id", ""
    if str(record.get("sensor") or "").strip() != "XT16":
        return False, "verified calibration record is not for XT16", ""
    if not str(record.get("sensor_serial") or "").strip():
        return False, "verified calibration record has no sensor_serial", ""
    if not str(record.get("verified_at") or "").strip():
        return False, "verified calibration record has no verified_at", ""
    if not str(record.get("verified_by") or "").strip():
        return False, "verified calibration record has no verified_by", ""
    evidence = record.get("evidence")
    if not isinstance(evidence, Mapping):
        return False, "verified calibration evidence is missing", ""
    try:
        required_scenes = int(evidence.get("required_stationary_measured_scenes") or 0)
        completed_scenes = int(evidence.get("completed_stationary_measured_scenes") or 0)
        max_abs_error_m = float(evidence.get("max_abs_error_m"))
        acceptance_max_abs_error_m = float(evidence.get("acceptance_max_abs_error_m"))
    except (TypeError, ValueError):
        return False, "verified calibration evidence metrics are invalid", ""
    if required_scenes < 3 or completed_scenes < required_scenes:
        return False, "verified calibration evidence has incomplete measured scenes", ""
    artifact_paths = evidence.get("artifact_paths")
    if (
        not isinstance(artifact_paths, list)
        or len(artifact_paths) < required_scenes
        or any(not str(path).strip() for path in artifact_paths)
    ):
        return False, "verified calibration evidence artifacts are incomplete", ""
    artifact_sha256 = evidence.get("artifact_sha256")
    if (
        not isinstance(artifact_sha256, Mapping)
        or len(artifact_sha256) < required_scenes
        or any(
            not str(path).strip() or len(str(digest).strip()) != 64
            for path, digest in artifact_sha256.items()
        )
    ):
        return False, "verified calibration evidence hashes are incomplete", ""
    if (
        not math.isfinite(max_abs_error_m)
        or not math.isfinite(acceptance_max_abs_error_m)
        or acceptance_max_abs_error_m <= 0
        or max_abs_error_m < 0
        or max_abs_error_m > acceptance_max_abs_error_m
    ):
        return False, "verified calibration evidence exceeds accepted error", ""
    parameters = record.get("parameters")
    if not isinstance(parameters, Mapping):
        return False, "calibration parameters are missing", ""
    for key, env_name in PARAMETER_ENV.items():
        if key not in parameters:
            return False, f"calibration parameter is missing: {key}", ""
        actual = environment.get(env_name)
        if actual is None:
            return False, f"runtime parameter is missing: {env_name}", ""
        if not _same_value(parameters[key], actual):
            return False, f"runtime parameter mismatch: {key}", ""
    return True, "verified calibration record matches runtime geometry", calibration_id


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Validate an XT16 calibration record against runtime geometry.")
    parser.add_argument("--record", required=True)
    parser.add_argument("--json", action="store_true")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        record = json.loads(Path(args.record).read_text(encoding="utf-8"))
        if not isinstance(record, dict):
            raise ValueError("calibration record must be an object")
        ok, reason, calibration_id = validate_record(record, os.environ)
    except Exception as exc:
        ok, reason, calibration_id = False, f"calibration record error: {exc}", ""
    if args.json:
        print(json.dumps({"ok": ok, "reason": reason, "calibration_id": calibration_id}, ensure_ascii=False))
    elif ok:
        print(calibration_id)
    else:
        print(reason)
    return 0 if ok else 4


if __name__ == "__main__":
    raise SystemExit(main())
