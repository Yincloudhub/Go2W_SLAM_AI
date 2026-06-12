#!/usr/bin/env python3
from __future__ import annotations

import argparse
import hashlib
import json
import math
import time
from pathlib import Path
from typing import Any


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_RECORD = REPO_ROOT / "configs" / "perception" / "xt16_geometry_calibration.json"
REQUIRED_SCENES = {"baseline", "front", "left", "right", "rear"}


def load_json(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"{path} must contain a JSON object")
    return value


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def finite_number(value: Any) -> float | None:
    if isinstance(value, bool):
        return None
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def canonical_json(value: Any) -> str:
    return json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(",", ":"))


def validate_artifacts(
    artifact_paths: list[Path],
    record: dict[str, Any],
) -> tuple[list[str], dict[str, Any]]:
    reasons: list[str] = []
    artifacts: list[tuple[Path, dict[str, Any]]] = []
    for path in artifact_paths:
        try:
            artifact = load_json(path)
        except (OSError, ValueError, json.JSONDecodeError) as exc:
            reasons.append(f"artifact_unreadable:{path}:{exc}")
            continue
        artifacts.append((path, artifact))

    scenes: dict[str, tuple[Path, dict[str, Any]]] = {}
    max_abs_error_m = 0.0
    reference_parameters = canonical_json(record.get("parameters"))
    expected_serial = str(record.get("sensor_serial") or "").strip()
    git_heads: set[str] = set()
    for path, artifact in artifacts:
        if artifact.get("type") != "go2w_xt16_calibration_scene":
            reasons.append(f"wrong_artifact_type:{path}")
            continue
        scene = str(artifact.get("scene") or "").strip()
        if scene not in REQUIRED_SCENES:
            reasons.append(f"unexpected_scene:{path}:{scene or 'missing'}")
            continue
        if scene in scenes:
            reasons.append(f"duplicate_scene:{scene}")
            continue
        scenes[scene] = (path, artifact)

        assessment = artifact.get("assessment")
        if not isinstance(assessment, dict) or assessment.get("accepted") is not True:
            reasons.append(f"scene_not_accepted:{scene}")
        samples = artifact.get("samples")
        thresholds = assessment.get("thresholds", {}) if isinstance(assessment, dict) else {}
        required_samples = int(thresholds.get("required_samples") or 25)
        if not isinstance(samples, list) or len(samples) < required_samples:
            reasons.append(f"scene_samples_incomplete:{scene}")

        git = artifact.get("git") if isinstance(artifact.get("git"), dict) else {}
        if git.get("dirty") is not False:
            reasons.append(f"scene_git_dirty:{scene}")
        git_head = str(git.get("head") or "").strip()
        if not git_head:
            reasons.append(f"scene_git_head_missing:{scene}")
        else:
            git_heads.add(git_head)

        calibration = (
            artifact.get("calibration_record")
            if isinstance(artifact.get("calibration_record"), dict)
            else {}
        )
        serial = str(calibration.get("sensor_serial") or "").strip()
        if not expected_serial or serial != expected_serial:
            reasons.append(f"sensor_serial_mismatch:{scene}")
        if canonical_json(calibration.get("parameters")) != reference_parameters:
            reasons.append(f"calibration_parameters_mismatch:{scene}")

        comparisons = assessment.get("comparisons", {}) if isinstance(assessment, dict) else {}
        if isinstance(comparisons, dict):
            for comparison in comparisons.values():
                if not isinstance(comparison, dict):
                    continue
                error = finite_number(comparison.get("absolute_error_m"))
                if error is not None:
                    max_abs_error_m = max(max_abs_error_m, error)

    missing_scenes = sorted(REQUIRED_SCENES - set(scenes))
    if missing_scenes:
        reasons.append("missing_scenes:" + ",".join(missing_scenes))
    if len(git_heads) > 1:
        reasons.append("scene_git_heads_differ")

    ordered_paths = [
        str(scenes[scene][0])
        for scene in ("baseline", "front", "left", "right", "rear")
        if scene in scenes
    ]
    hashes = {
        str(path): sha256_file(path)
        for path, _ in artifacts
        if path.exists()
    }
    evidence = {
        "required_stationary_measured_scenes": 5,
        "completed_stationary_measured_scenes": len(scenes),
        "artifact_paths": ordered_paths,
        "artifact_sha256": hashes,
        "max_abs_error_m": round(max_abs_error_m, 6),
        "git_head": next(iter(git_heads)) if len(git_heads) == 1 else None,
    }
    return sorted(set(reasons)), evidence


def build_verified_record(
    record: dict[str, Any],
    *,
    calibration_id: str,
    verified_by: str,
    evidence: dict[str, Any],
    verified_at: str,
) -> dict[str, Any]:
    updated = json.loads(json.dumps(record))
    updated["calibration_id"] = calibration_id
    updated["status"] = "verified"
    updated["verified_at"] = verified_at
    updated["verified_by"] = verified_by
    current_evidence = (
        updated.get("evidence") if isinstance(updated.get("evidence"), dict) else {}
    )
    current_evidence.update(evidence)
    current_evidence["notes"] = (
        "Verified from baseline, front, left, right, and rear stationary measured scenes."
    )
    updated["evidence"] = current_evidence
    return updated


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Validate five XT16 scene artifacts and optionally promote the calibration record."
    )
    parser.add_argument("artifacts", nargs="+")
    parser.add_argument("--record", default=str(DEFAULT_RECORD))
    parser.add_argument("--calibration-id", required=True)
    parser.add_argument("--verified-by", required=True)
    parser.add_argument("--apply", action="store_true")
    parser.add_argument("--output", default="")
    parser.add_argument("--json", action="store_true")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    artifact_paths = [Path(value).resolve() for value in args.artifacts]
    if len(artifact_paths) != 5:
        result = {
            "accepted": False,
            "applied": False,
            "reasons": [f"exactly_five_artifacts_required:{len(artifact_paths)}"],
        }
        print(json.dumps(result, ensure_ascii=False, indent=2 if args.json else None))
        return 4

    record_path = Path(args.record)
    try:
        record = load_json(record_path)
    except (OSError, ValueError, json.JSONDecodeError) as exc:
        result = {
            "accepted": False,
            "applied": False,
            "reasons": [f"calibration_record_unreadable:{exc}"],
        }
        print(json.dumps(result, ensure_ascii=False, indent=2 if args.json else None))
        return 4

    reasons, evidence = validate_artifacts(artifact_paths, record)
    calibration_id = args.calibration_id.strip()
    verified_by = args.verified_by.strip()
    if not calibration_id:
        reasons.append("calibration_id_is_empty")
    if not verified_by:
        reasons.append("verified_by_is_empty")
    acceptance_limit = finite_number(
        record.get("evidence", {}).get("acceptance_max_abs_error_m")
        if isinstance(record.get("evidence"), dict)
        else None
    )
    if acceptance_limit is None or acceptance_limit <= 0:
        reasons.append("record_acceptance_error_limit_invalid")
    elif evidence["max_abs_error_m"] > acceptance_limit:
        reasons.append("aggregate_max_abs_error_exceeded")

    accepted = not reasons
    applied = False
    output_path: Path | None = None
    verified_record = None
    if accepted:
        verified_record = build_verified_record(
            record,
            calibration_id=calibration_id,
            verified_by=verified_by,
            evidence=evidence,
            verified_at=time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        )
        if args.output:
            output_path = Path(args.output)
        elif args.apply:
            output_path = record_path
        else:
            output_path = (
                REPO_ROOT
                / "artifacts"
                / "xt16_calibration"
                / f"{calibration_id}_verified_record.json"
            )
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text(
            json.dumps(verified_record, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        applied = bool(args.apply and output_path.resolve() == record_path.resolve())

    result = {
        "accepted": accepted,
        "applied": applied,
        "reasons": sorted(set(reasons)),
        "record": str(record_path),
        "output": str(output_path) if output_path is not None else None,
        "calibration_id": calibration_id or None,
        "evidence": evidence,
    }
    print(json.dumps(result, ensure_ascii=False, indent=2 if args.json else None))
    return 0 if accepted else 4


if __name__ == "__main__":
    raise SystemExit(main())
