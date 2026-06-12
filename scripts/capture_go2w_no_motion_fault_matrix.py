#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import subprocess
import time
from pathlib import Path
from typing import Any


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_PANEL = REPO_ROOT / "cpp" / "build" / "go2w_operator_panel"
DEFAULT_ACCEPT = REPO_ROOT / "scripts" / "go2w_accept.sh"
DEFAULT_SUPERVISED = REPO_ROOT / "scripts" / "go2w_supervised_acceptance.py"


def run_case(case_id: str, command: list[str], *, input_text: str = "", timeout_s: int = 30) -> dict[str, Any]:
    started = time.time()
    completed = subprocess.run(
        command,
        cwd=REPO_ROOT,
        input=input_text or None,
        text=True,
        encoding="utf-8",
        errors="replace",
        capture_output=True,
        timeout=timeout_s,
        check=False,
    )
    return {
        "case_id": case_id,
        "command": command,
        "returncode": completed.returncode,
        "elapsed_s": round(time.time() - started, 3),
        "stdout": completed.stdout[-16000:],
        "stderr": completed.stderr[-8000:],
    }


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Capture fail-closed GO2W checks without sending chassis motion commands."
    )
    parser.add_argument("--target", default="initial_point")
    parser.add_argument("--panel-bin", default=str(DEFAULT_PANEL))
    parser.add_argument("--gateway-client", default="")
    parser.add_argument("--output", default="")
    parser.add_argument("--json", action="store_true")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    panel = Path(args.panel_bin)
    if not panel.is_file():
        raise SystemExit(f"operator panel binary not found: {panel}")

    panel_args = [
        str(panel),
        "--repo-root",
        str(REPO_ROOT),
        "--registry",
        str(REPO_ROOT / "configs" / "maps" / "go2w_real_site_map_registry.json"),
        "--map-id",
        "go2w_real_site",
    ]
    if args.gateway_client:
        panel_args.extend(["--gateway-client", args.gateway_client])

    cases = [
        run_case(
            "relocate_missing_confirmation",
            ["bash", str(DEFAULT_ACCEPT), "relocate", "mapping_origin"],
        ),
        run_case(
            "relocate_unknown_anchor",
            [
                "python3",
                str(DEFAULT_SUPERVISED),
                "--stage",
                "relocate",
                "--anchor",
                "missing_anchor",
                "--confirm-relocation",
                "missing_anchor",
                "--json",
            ],
        ),
        run_case(
            "navigation_preflight_fail_closed",
            ["bash", str(DEFAULT_ACCEPT), "check", args.target],
        ),
        run_case(
            "relative_motion_not_wired",
            panel_args,
            input_text="forward 1 meter\n/quit\n",
        ),
        run_case(
            "unknown_target_clarification",
            panel_args,
            input_text="去老板办公室门口\n/quit\n",
        ),
    ]

    failures: list[str] = []
    missing_confirmation = next(case for case in cases if case["case_id"] == "relocate_missing_confirmation")
    if missing_confirmation["returncode"] == 0:
        failures.append("relocate_missing_confirmation unexpectedly succeeded")
    relative_motion = next(case for case in cases if case["case_id"] == "relative_motion_not_wired")
    relative_text = relative_motion["stdout"] + relative_motion["stderr"]
    if "relative_motion is not wired for real execution" not in relative_text:
        failures.append("relative_motion capability guard was not observed")
    unknown_target = next(case for case in cases if case["case_id"] == "unknown_target_clarification")
    unknown_text = unknown_target["stdout"] + unknown_target["stderr"]
    if "no command sent" not in unknown_text and "未下发运动" not in unknown_text:
        failures.append("unknown target did not visibly remain non-motion")
    preflight = next(case for case in cases if case["case_id"] == "navigation_preflight_fail_closed")
    if preflight["returncode"] == 0:
        failures.append("navigation preflight unexpectedly reported ready")

    stamp = time.strftime("%Y%m%d_%H%M%S")
    artifact = {
        "schema_version": 1,
        "type": "go2w_no_motion_fault_matrix",
        "timestamp_ms": int(time.time() * 1000),
        "capture_mode": "read_only_and_dry_run",
        "motion_commands_sent": False,
        "execute_enabled": False,
        "passed": not failures,
        "failures": failures,
        "cases": cases,
    }
    output = (
        Path(args.output)
        if args.output
        else REPO_ROOT / "artifacts" / "field_acceptance" / f"{stamp}_no_motion_fault_matrix.json"
    )
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(artifact, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    if args.json:
        print(json.dumps(artifact, ensure_ascii=False, indent=2))
    else:
        print(output)
    return 0 if artifact["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
