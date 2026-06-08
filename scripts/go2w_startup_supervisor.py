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

from edge_autonomy.runtime_readiness import (  # noqa: E402
    assess_runtime_readiness,
    gateway_response_from_output,
    load_json_summary,
)

DEFAULT_GATEWAY_CLIENT = str(REPO_ROOT / "robot" / "slam_gateway_refactor" / "build" / "slam_llm_command_client")
DEFAULT_START_SLAM_SCRIPT = REPO_ROOT / "scripts" / "start_go2w_slam_stack.sh"
DEFAULT_LIDAR_SUMMARY = REPO_ROOT / "artifacts" / "lidar_geometry_summary.json"
DEFAULT_LLM_MODEL = Path("/home/unitree/models/Qwen_Qwen3-4B-Instruct-2507-Q4_K_M.gguf")
DEFAULT_LLM_ASK_SCRIPT = Path("/home/unitree/llm_runtime/scripts/ask_qwen.sh")


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
        gateway_response = (
            gateway_response_from_output(completed.stdout)
            if step.name == "gateway_world_state_probe"
            else None
        )
        record.update(
            {
                "returncode": completed.returncode,
                "stdout": completed.stdout[-4000:],
                "stderr": completed.stderr[-4000:],
                "elapsed_s": round(time.time() - start, 3),
                "ok": completed.returncode == 0,
            }
        )
        if gateway_response is not None:
            record["response"] = gateway_response
    except subprocess.TimeoutExpired as exc:
        record.update(
            {
                "returncode": None,
                "stdout": (exc.stdout or "")[-4000:] if isinstance(exc.stdout, str) else "",
                "stderr": (exc.stderr or "")[-4000:] if isinstance(exc.stderr, str) else "",
                "elapsed_s": round(time.time() - start, 3),
                "ok": False,
                "timed_out": True,
            }
        )
    except (OSError, subprocess.SubprocessError) as exc:
        record.update(
            {
                "returncode": None,
                "stdout": "",
                "stderr": str(exc),
                "elapsed_s": round(time.time() - start, 3),
                "ok": False,
            }
        )
    return record


def startup_summary(
    records: list[dict[str, Any]],
    *,
    lidar_summary: dict[str, Any] | None = None,
    llm_configured: bool = False,
) -> dict[str, Any]:
    failed_required = [item for item in records if item.get("required") and not item.get("ok")]
    gateway = next((item for item in records if item.get("name") == "gateway_world_state_probe"), None)
    startup_ok = not failed_required
    gateway_response = gateway.get("response") if isinstance(gateway, dict) else None
    if not isinstance(gateway_response, dict) and gateway:
        gateway_response = gateway_response_from_output(str(gateway.get("stdout") or ""))
    readiness = assess_runtime_readiness(
        gateway_response,
        startup_ok=startup_ok,
        lidar_summary=lidar_summary,
        llm_configured=llm_configured,
    )
    return {
        "schema_version": 2,
        "timestamp_ms": int(time.time() * 1000),
        "ok": startup_ok,
        "startup_ok": startup_ok,
        "failed_required": [item["name"] for item in failed_required],
        "motion_commands_sent": any(item.get("starts_motion") for item in records if not item.get("dry_run")),
        "gateway_probe_ok": bool(gateway and gateway.get("returncode") == 0),
        "readiness": readiness,
        "gateway_response": gateway_response,
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
    parser.add_argument("--lidar-summary", default=os.environ.get("GO2W_LIDAR_GEOMETRY_SUMMARY_PATH", str(DEFAULT_LIDAR_SUMMARY)))
    parser.add_argument("--llm-model", default=os.environ.get("GO2W_LLM_MODEL", str(DEFAULT_LLM_MODEL)))
    parser.add_argument("--llm-ask-script", default=os.environ.get("GO2W_LLM_ASK_SCRIPT", str(DEFAULT_LLM_ASK_SCRIPT)))
    parser.add_argument("--require-navigation-ready", action="store_true")
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
    lidar_summary = load_json_summary(args.lidar_summary)
    summary = startup_summary(
        records,
        lidar_summary=lidar_summary,
        llm_configured=bool(os.environ.get("GO2W_LLM_HTTP_URL"))
        or (Path(args.llm_model).is_file() and Path(args.llm_ask_script).is_file()),
    )
    if args.json:
        print(json.dumps(summary, ensure_ascii=False, indent=2))
    else:
        mode = "RUN" if args.run else "DRY-RUN"
        readiness = summary["readiness"]
        print(
            f"GO2W startup supervisor [{mode}] startup_ok={summary['startup_ok']} "
            f"gateway_ready={readiness['gateway_ready']} relocalization_ready={readiness['relocalization_ready']} "
            f"navigation_ready={readiness['navigation_ready']}"
        )
        for item in records:
            rc = item.get("returncode")
            print(f"- {item['name']}: ok={item.get('ok')} rc={rc} required={item.get('required')}")
            if item.get("stderr"):
                print(str(item["stderr"]).strip())
        print(f"- localization: ready={readiness['localization_ready']} reason={readiness['localization_reason']}")
        print(f"- perception: ready={readiness['perception_ready']} reason={readiness['perception_reason']}")
        print(f"- navigation: ready={readiness['navigation_ready']} reason={readiness['navigation_reason']}")
        print(f"- next_action: {readiness['next_action']}")
    if not summary["ok"]:
        return 2
    if args.require_navigation_ready and not summary["readiness"]["acceptance_ok"]:
        return 3
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
