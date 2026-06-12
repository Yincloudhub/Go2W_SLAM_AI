#!/usr/bin/env python3
from __future__ import annotations

import argparse
import hashlib
import json
import subprocess
import sys
import time
from pathlib import Path
from typing import Any


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_REGISTRY = REPO_ROOT / "configs" / "maps" / "go2w_real_site_map_registry.json"
DEFAULT_CALIBRATION = REPO_ROOT / "configs" / "perception" / "xt16_geometry_calibration.json"
DEFAULT_LIDAR_SUMMARY = REPO_ROOT / "artifacts" / "lidar_geometry_summary.json"
DEFAULT_DEPTH_SUMMARY = REPO_ROOT / "artifacts" / "stereo_depth_summary.json"


def run_command(command: list[str], *, timeout_s: int = 20) -> dict[str, Any]:
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
        return {
            "returncode": completed.returncode,
            "stdout": completed.stdout[-12000:],
            "stderr": completed.stderr[-4000:],
        }
    except (OSError, subprocess.SubprocessError) as exc:
        return {"returncode": None, "stdout": "", "stderr": str(exc)}


def load_json(path: Path) -> dict[str, Any] | None:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError, json.JSONDecodeError):
        return None
    return value if isinstance(value, dict) else None


def extract_last_json(text: str) -> dict[str, Any] | None:
    decoder = json.JSONDecoder()
    values: list[dict[str, Any]] = []
    for index, char in enumerate(text):
        if char != "{":
            continue
        try:
            value, _ = decoder.raw_decode(text[index:])
        except json.JSONDecodeError:
            continue
        if isinstance(value, dict):
            values.append(value)
    return values[-1] if values else None


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


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Capture a read-only GO2W field-acceptance artifact. No motion command is sent."
    )
    parser.add_argument("--test-id", required=True)
    parser.add_argument("--scene", default="")
    parser.add_argument("--operator-note", default="")
    parser.add_argument("--map-id", default="go2w_real_site")
    parser.add_argument("--registry", default=str(DEFAULT_REGISTRY))
    parser.add_argument("--calibration", default=str(DEFAULT_CALIBRATION))
    parser.add_argument("--lidar-summary", default=str(DEFAULT_LIDAR_SUMMARY))
    parser.add_argument("--depth-summary", default=str(DEFAULT_DEPTH_SUMMARY))
    parser.add_argument(
        "--verify-anchor",
        default="",
        help="Optionally capture consecutive read-only localization samples for this active anchor.",
    )
    parser.add_argument("--measured-front-m", type=float)
    parser.add_argument("--measured-left-m", type=float)
    parser.add_argument("--measured-right-m", type=float)
    parser.add_argument("--measured-rear-m", type=float)
    parser.add_argument("--output", default="")
    parser.add_argument("--json", action="store_true")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    timestamp_ms = int(time.time() * 1000)
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    registry_path = Path(args.registry)
    registry = load_json(registry_path)
    profile = selected_map(registry, args.map_id)
    calibration_path = Path(args.calibration)
    calibration = load_json(calibration_path)
    status_result = run_command(
        [
            sys.executable,
            str(REPO_ROOT / "scripts" / "go2w_supervised_acceptance.py"),
            "--stage",
            "status",
            "--map-id",
            args.map_id,
            "--registry",
            str(registry_path),
            "--json",
        ],
        timeout_s=25,
    )
    verification_result = None
    if args.verify_anchor:
        verification_result = run_command(
            [
                sys.executable,
                str(REPO_ROOT / "scripts" / "go2w_supervised_acceptance.py"),
                "--stage",
                "verify-localization",
                "--map-id",
                args.map_id,
                "--registry",
                str(registry_path),
                "--anchor",
                args.verify_anchor,
                "--json",
            ],
            timeout_s=30,
        )
    git_head = run_command(["git", "rev-parse", "HEAD"], timeout_s=5)
    git_status = run_command(["git", "status", "--short"], timeout_s=5)
    processes = run_command(
        [
            "ps",
            "-eo",
            "pid=,stat=,etime=,%cpu=,%mem=,comm=,args=",
        ],
        timeout_s=5,
    )
    pcd_path = Path(str(profile.get("pcd_path"))) if isinstance(profile, dict) else Path("")
    measurements = {
        "front_m": args.measured_front_m,
        "left_m": args.measured_left_m,
        "right_m": args.measured_right_m,
        "rear_m": args.measured_rear_m,
    }
    artifact = {
        "schema_version": 1,
        "type": "go2w_field_acceptance_snapshot",
        "timestamp_ms": timestamp_ms,
        "test_id": args.test_id,
        "scene": args.scene,
        "operator_note": args.operator_note,
        "capture_mode": "read_only",
        "motion_commands_sent": False,
        "git": {
            "head": git_head["stdout"].strip(),
            "dirty": bool(git_status["stdout"].strip()),
            "status": git_status["stdout"].splitlines(),
        },
        "map": {
            "map_id": args.map_id,
            "registry_path": str(registry_path),
            "registry_sha256": sha256_file(registry_path),
            "pcd_path": str(profile.get("pcd_path") or "") if isinstance(profile, dict) else "",
            "pcd_sha256": sha256_file(pcd_path),
            "mapping_origin_anchor_id": (
                profile.get("mapping_origin_anchor_id") if isinstance(profile, dict) else None
            ),
            "active_relocalization_anchor_ids": [
                item.get("anchor_id")
                for item in profile.get("relocalization_anchors", [])
                if isinstance(item, dict)
            ]
            if isinstance(profile, dict)
            else [],
        },
        "xt16_calibration": {
            "path": str(calibration_path),
            "sha256": sha256_file(calibration_path),
            "calibration_id": (
                calibration.get("calibration_id") if isinstance(calibration, dict) else None
            ),
            "status": calibration.get("status") if isinstance(calibration, dict) else None,
        },
        "physical_measurements": measurements,
        "runtime_status": extract_last_json(status_result["stdout"]),
        "runtime_status_command": status_result,
        "localization_verification": (
            extract_last_json(verification_result["stdout"])
            if isinstance(verification_result, dict)
            else None
        ),
        "localization_verification_command": verification_result,
        "lidar_summary": load_json(Path(args.lidar_summary)),
        "depth_summary": load_json(Path(args.depth_summary)),
        "process_snapshot": processes["stdout"].splitlines(),
    }
    output = (
        Path(args.output)
        if args.output
        else REPO_ROOT
        / "artifacts"
        / "field_acceptance"
        / f"{timestamp}_{args.test_id}.json"
    )
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(artifact, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    if args.json:
        print(json.dumps(artifact, ensure_ascii=False, indent=2))
    else:
        print(output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
