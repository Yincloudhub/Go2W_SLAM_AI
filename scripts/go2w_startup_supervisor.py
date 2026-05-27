from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any


REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = REPO_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

try:
    sys.stdout.reconfigure(encoding="utf-8")
    sys.stderr.reconfigure(encoding="utf-8")
except (AttributeError, ValueError):
    pass


DEFAULT_GATEWAY_CLIENT = "/home/unitree/slam_gateway_refactor/build/slam_llm_command_client"
DEFAULT_START_SLAM_SCRIPT = REPO_ROOT / "scripts" / "start_go2w_slam_stack.sh"


@dataclass(frozen=True)
class StartupStep:
    name: str
    command: list[str]
    required: bool = True
    timeout_s: int = 30
    starts_motion: bool = False


def build_startup_plan(
    *,
    start_slam_script: str,
    gateway_client: str,
    network_interface: str,
    include_slam_stack: bool = True,
    include_gateway_probe: bool = True,
) -> list[StartupStep]:
    steps: list[StartupStep] = []
    if include_slam_stack:
        steps.append(
            StartupStep(
                name="slam_stack",
                command=["bash", start_slam_script],
                required=True,
                timeout_s=45,
                starts_motion=False,
            )
        )
    if include_gateway_probe:
        steps.append(
            StartupStep(
                name="gateway_world_state_probe",
                command=[gateway_client, network_interface],
                required=False,
                timeout_s=15,
                starts_motion=False,
            )
        )
    return steps


def run_step(step: StartupStep, *, dry_run: bool) -> dict[str, Any]:
    record: dict[str, Any] = {
        "name": step.name,
        "command": step.command,
        "required": step.required,
        "timeout_s": step.timeout_s,
        "starts_motion": step.starts_motion,
        "dry_run": dry_run,
    }
    if dry_run:
        record.update({"returncode": None, "stdout": "", "stderr": "", "ok": True})
        return record

    payload = ""
    if step.name == "gateway_world_state_probe":
        payload = json.dumps({"action": "get_world_state"}, separators=(",", ":")) + "\n"

    start = time.time()
    try:
        completed = subprocess.run(
            step.command,
            input=payload,
            text=True,
            encoding="utf-8",
            errors="replace",
            capture_output=True,
            timeout=step.timeout_s,
        )
        record.update(
            {
                "returncode": completed.returncode,
                "stdout": completed.stdout[-4000:],
                "stderr": completed.stderr[-4000:],
                "elapsed_s": round(time.time() - start, 3),
                "ok": completed.returncode == 0 or not step.required,
            }
        )
    except subprocess.TimeoutExpired as exc:
        record.update(
            {
                "returncode": None,
                "stdout": (exc.stdout or "")[-4000:] if isinstance(exc.stdout, str) else "",
                "stderr": (exc.stderr or "")[-4000:] if isinstance(exc.stderr, str) else "",
                "elapsed_s": round(time.time() - start, 3),
                "ok": not step.required,
                "timed_out": True,
            }
        )
    return record


def startup_summary(records: list[dict[str, Any]]) -> dict[str, Any]:
    failed_required = [item for item in records if item.get("required") and not item.get("ok")]
    gateway = next((item for item in records if item.get("name") == "gateway_world_state_probe"), None)
    return {
        "schema_version": 1,
        "timestamp_ms": int(time.time() * 1000),
        "ok": not failed_required,
        "failed_required": [item["name"] for item in failed_required],
        "motion_commands_sent": any(item.get("starts_motion") for item in records if not item.get("dry_run")),
        "gateway_probe_ok": bool(gateway and gateway.get("returncode") == 0),
        "records": records,
    }


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Start/check GO2W SLAM runtime components without sending motion commands.")
    parser.add_argument("--run", action="store_true", help="Execute startup steps. Default is dry-run.")
    parser.add_argument("--json", action="store_true", help="Print machine-readable JSON summary.")
    parser.add_argument("--no-slam-stack", action="store_true", help="Skip LiDAR/SLAM startup script.")
    parser.add_argument("--no-gateway-probe", action="store_true", help="Skip get_world_state gateway probe.")
    parser.add_argument("--start-slam-script", default=str(DEFAULT_START_SLAM_SCRIPT))
    parser.add_argument("--gateway-client", default=os.environ.get("GO2W_GATEWAY_CLIENT", DEFAULT_GATEWAY_CLIENT))
    parser.add_argument("--interface", default=os.environ.get("GO2W_NETWORK_INTERFACE", "eth0"))
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    steps = build_startup_plan(
        start_slam_script=args.start_slam_script,
        gateway_client=args.gateway_client,
        network_interface=args.interface,
        include_slam_stack=not args.no_slam_stack,
        include_gateway_probe=not args.no_gateway_probe,
    )
    records = [run_step(step, dry_run=not args.run) for step in steps]
    summary = startup_summary(records)
    if args.json:
        print(json.dumps(summary, ensure_ascii=False, indent=2))
    else:
        mode = "RUN" if args.run else "DRY-RUN"
        print(f"GO2W startup supervisor [{mode}] ok={summary['ok']} gateway_probe_ok={summary['gateway_probe_ok']}")
        for item in records:
            rc = item.get("returncode")
            print(f"- {item['name']}: ok={item.get('ok')} rc={rc} required={item.get('required')}")
            if item.get("stderr"):
                print(str(item["stderr"]).strip())
    return 0 if summary["ok"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
