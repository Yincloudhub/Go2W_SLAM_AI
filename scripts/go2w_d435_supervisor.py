#!/usr/bin/env python3
"""Continuously recover the unified D435 owner after confirmed health loss."""

from __future__ import annotations

import argparse
import json
import signal
import subprocess
import sys
import time
from pathlib import Path
from typing import Any


REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = REPO_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from edge_autonomy.service_supervision import ConsecutiveFailureRecovery  # noqa: E402


RUNNING = True


def stop_running(_signum: int, _frame: Any) -> None:
    global RUNNING
    RUNNING = False


def run_manager(manager: Path, action: str, *, timeout_s: float) -> dict[str, Any]:
    started = time.monotonic()
    try:
        completed = subprocess.run(
            ["bash", str(manager), action],
            text=True,
            encoding="utf-8",
            errors="replace",
            capture_output=True,
            timeout=timeout_s,
        )
        return {
            "ok": completed.returncode == 0,
            "returncode": completed.returncode,
            "elapsed_ms": int((time.monotonic() - started) * 1000),
            "stdout": completed.stdout[-2000:],
            "stderr": completed.stderr[-2000:],
        }
    except (OSError, subprocess.SubprocessError) as exc:
        return {
            "ok": False,
            "returncode": None,
            "elapsed_ms": int((time.monotonic() - started) * 1000),
            "stdout": "",
            "stderr": str(exc),
        }


def diagnose_summary(path: Path, *, now_ms: int) -> dict[str, Any]:
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except FileNotFoundError:
        return {"available": False, "reason": "summary_missing", "path": str(path)}
    except (OSError, json.JSONDecodeError) as exc:
        return {
            "available": False,
            "reason": "summary_unreadable",
            "path": str(path),
            "error": str(exc),
        }
    if not isinstance(data, dict):
        return {"available": False, "reason": "summary_not_object", "path": str(path)}

    timestamp_ms = data.get("timestamp_ms")
    owner = data.get("owner") if isinstance(data.get("owner"), dict) else {}
    capture = data.get("capture") if isinstance(data.get("capture"), dict) else {}
    return {
        "available": True,
        "path": str(path),
        "status": data.get("status"),
        "stale": data.get("stale"),
        "age_ms": (
            max(0, now_ms - timestamp_ms)
            if isinstance(timestamp_ms, int) and not isinstance(timestamp_ms, bool)
            else None
        ),
        "frame_sequence": data.get("frame_sequence"),
        "owner": {
            "pid": owner.get("pid"),
            "running": owner.get("running"),
            "status": owner.get("status"),
            "process_start_ticks": owner.get("process_start_ticks"),
        },
        "capture": {
            "status": capture.get("status"),
            "frame_sequence": capture.get("frame_sequence"),
        },
    }


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--manager",
        default=str(REPO_ROOT / "scripts" / "go2w_d435_perception_sidecar.sh"),
    )
    parser.add_argument(
        "--summary-path",
        default=str(REPO_ROOT / "artifacts" / "d435_perception_summary.json"),
    )
    parser.add_argument("--interval-s", type=float, default=2.0)
    parser.add_argument("--failure-threshold", type=int, default=3)
    parser.add_argument("--cooldown-s", type=float, default=30.0)
    parser.add_argument("--command-timeout-s", type=float, default=30.0)
    parser.add_argument("--max-cycles", type=int, default=0)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    manager = Path(args.manager).resolve()
    summary_path = Path(args.summary_path).resolve()
    if not manager.is_file():
        raise SystemExit(f"D435 manager not found: {manager}")

    interval_s = max(0.2, float(args.interval_s))
    command_timeout_s = max(1.0, float(args.command_timeout_s))
    recovery = ConsecutiveFailureRecovery(
        failure_threshold=args.failure_threshold,
        cooldown_ms=max(0, int(float(args.cooldown_s) * 1000)),
    )
    signal.signal(signal.SIGTERM, stop_running)
    signal.signal(signal.SIGINT, stop_running)

    cycle = 0
    last_report_key: tuple[Any, ...] | None = None
    while RUNNING:
        cycle += 1
        timestamp_ms = int(time.time() * 1000)
        health = run_manager(manager, "health", timeout_s=command_timeout_s)
        decision = recovery.observe(healthy=health["ok"], now_ms=timestamp_ms)
        event: dict[str, Any] = {
            "schema_version": 1,
            "schema": "go2w_service_supervision_event_v1",
            "timestamp_ms": timestamp_ms,
            "service": "d435_perception",
            "cycle": cycle,
            "health": health,
            "diagnosis": diagnose_summary(summary_path, now_ms=timestamp_ms),
            "decision": {
                "action": decision.action,
                "consecutive_failures": decision.consecutive_failures,
                "retry_after_ms": decision.retry_after_ms,
            },
        }
        if decision.action == "recover":
            event["recovery"] = run_manager(
                manager,
                "restart-if-stale",
                timeout_s=command_timeout_s,
            )
        diagnosis = event["diagnosis"]
        report_key = (
            health["ok"],
            decision.action,
            diagnosis.get("status"),
            diagnosis.get("stale"),
            diagnosis.get("owner", {}).get("pid"),
            diagnosis.get("owner", {}).get("running"),
        )
        if report_key != last_report_key or decision.action == "recover" or cycle % 30 == 0:
            print(json.dumps(event, ensure_ascii=False, separators=(",", ":")), flush=True)
            last_report_key = report_key

        if args.max_cycles > 0 and cycle >= args.max_cycles:
            break
        deadline = time.monotonic() + interval_s
        while RUNNING and time.monotonic() < deadline:
            time.sleep(min(0.2, max(0.0, deadline - time.monotonic())))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
