#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
from typing import Any, Mapping

try:
    from .xt16_calibration_guard import PARAMETER_ENV, _same_value
except ImportError:
    from xt16_calibration_guard import PARAMETER_ENV, _same_value


NATIVE_NAVIGATION_SPEED_MPS = 0.2


def validate_record(
    record: Mapping[str, Any],
    environment: Mapping[str, str],
) -> tuple[bool, str, str, float]:
    if record.get("schema_version") != 1:
        return False, "unsupported supervised release schema", "", 0.0
    if str(record.get("status") or "") != "engineering_validated":
        return False, "supervised release is not engineering_validated", "", 0.0
    if str(record.get("sensor") or "").strip() != "XT16":
        return False, "supervised release is not for XT16", "", 0.0
    if not str(record.get("sensor_serial") or "").strip():
        return False, "supervised release has no sensor_serial", "", 0.0
    release_id = str(record.get("release_id") or "").strip()
    if not release_id:
        return False, "supervised release has no release_id", "", 0.0
    if not str(record.get("issued_at") or "").strip() or not str(record.get("issued_by") or "").strip():
        return False, "supervised release issuer metadata is incomplete", "", 0.0
    if record.get("formal_calibration") is not False:
        return False, "supervised release must not claim formal calibration", "", 0.0
    if record.get("requires_operator_presence") is not True:
        return False, "supervised release must require operator presence", "", 0.0
    if record.get("requires_emergency_stop") is not True:
        return False, "supervised release must require an emergency stop", "", 0.0
    if record.get("navigation_mode") not in (0, 1):
        return False, "supervised release must require Unitree navigation mode 0 or 1", "", 0.0
    if (
        str(record.get("motion_policy") or "")
        != "unitree_mode0_native_navigation_with_bounded_recovery"
    ):
        return False, "supervised release motion policy is invalid", "", 0.0
    try:
        max_speed_mps = float(record.get("max_speed_mps"))
    except (TypeError, ValueError):
        return False, "supervised release max speed is invalid", "", 0.0
    if (
        not math.isfinite(max_speed_mps)
        or not math.isclose(
            max_speed_mps,
            NATIVE_NAVIGATION_SPEED_MPS,
            rel_tol=0.0,
            abs_tol=1e-9,
        )
    ):
        return False, "supervised release native navigation speed must be 0.2 m/s", "", 0.0

    evidence = record.get("evidence")
    if not isinstance(evidence, list) or len(evidence) < 4:
        return False, "supervised release evidence is incomplete", "", 0.0
    for item in evidence:
        if not isinstance(item, Mapping):
            return False, "supervised release evidence entry is invalid", "", 0.0
        path = str(item.get("path") or "").strip()
        digest = str(item.get("sha256") or "").strip()
        purpose = str(item.get("purpose") or "").strip()
        if not path or len(digest) != 64 or not purpose:
            return False, "supervised release evidence metadata is incomplete", "", 0.0

    hard_stops = record.get("hard_stop_m")
    if not isinstance(hard_stops, Mapping):
        return False, "supervised release hard stops are missing", "", 0.0
    expected_hard_stops = {"front_departure": 0.8}
    for key, expected in expected_hard_stops.items():
        try:
            actual = float(hard_stops.get(key))
        except (TypeError, ValueError):
            return False, f"supervised release hard stop is invalid: {key}", "", 0.0
        if not math.isclose(actual, expected, rel_tol=0.0, abs_tol=1e-9):
            return False, f"supervised release changes hard stop: {key}", "", 0.0
    advisory = record.get("advisory_clearance_m")
    if not isinstance(advisory, Mapping):
        return False, "supervised release advisory clearances are missing", "", 0.0
    for key, expected in {"side": 0.2, "rear": 0.3}.items():
        try:
            actual = float(advisory.get(key))
        except (TypeError, ValueError):
            return False, f"supervised release advisory clearance is invalid: {key}", "", 0.0
        if not math.isclose(actual, expected, rel_tol=0.0, abs_tol=1e-9):
            return False, f"supervised release changes advisory clearance: {key}", "", 0.0

    parameters = record.get("parameters")
    if not isinstance(parameters, Mapping):
        return False, "supervised release parameters are missing", "", 0.0
    for key, env_name in PARAMETER_ENV.items():
        if key not in parameters:
            return False, f"supervised release parameter is missing: {key}", "", 0.0
        actual = environment.get(env_name)
        if actual is None:
            return False, f"runtime parameter is missing: {env_name}", "", 0.0
        if not _same_value(parameters[key], actual):
            return False, f"runtime parameter mismatch: {key}", "", 0.0
    return True, "supervised engineering release matches runtime geometry", release_id, max_speed_mps


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Validate an XT16 supervised engineering release.")
    parser.add_argument("--record", required=True)
    parser.add_argument("--json", action="store_true")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        record = json.loads(Path(args.record).read_text(encoding="utf-8"))
        if not isinstance(record, dict):
            raise ValueError("supervised release record must be an object")
        ok, reason, release_id, max_speed_mps = validate_record(record, os.environ)
    except Exception as exc:
        ok, reason, release_id, max_speed_mps = (
            False,
            f"supervised release record error: {exc}",
            "",
            0.0,
        )
    if args.json:
        print(
            json.dumps(
                {
                    "ok": ok,
                    "reason": reason,
                    "release_id": release_id,
                    "max_speed_mps": max_speed_mps,
                },
                ensure_ascii=False,
            )
        )
    elif ok:
        print(f"{release_id}|{max_speed_mps:g}")
    else:
        print(reason)
    return 0 if ok else 4


if __name__ == "__main__":
    raise SystemExit(main())
