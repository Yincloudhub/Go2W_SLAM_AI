from __future__ import annotations

import argparse
import base64
import json
import subprocess
import sys
import time
from pathlib import Path
from typing import Any


REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from scripts.run_robot_closed_loop import gateway_allows_navigation, run_gateway_command  # noqa: E402


DEFAULT_GATEWAY_CLIENT = "/home/unitree/slam_gateway_refactor/build/slam_llm_command_client"
DEFAULT_START_SLAM = "/home/unitree/go2w_slam/go2w_edge_autonomy/scripts/start_go2w_slam_stack.sh"
DEFAULT_MAP_PATH = "/home/unitree/test.pcd"
DEFAULT_REGISTRY = REPO_ROOT / "configs" / "maps" / "go2w_real_site_map_registry.json"
DEFAULT_MAP_ID = "go2w_real_site"


def decode_command(args: argparse.Namespace) -> str:
    if args.command_b64:
        return base64.b64decode(args.command_b64).decode("utf-8")
    return args.command or ""


def print_json(value: Any, *, pretty: bool) -> None:
    if pretty:
        print(json.dumps(value, ensure_ascii=False, indent=2))
    else:
        print(json.dumps(value, ensure_ascii=False, separators=(",", ":")))


def run_start_slam(script: str) -> dict[str, Any]:
    completed = subprocess.run(["bash", script], text=True, capture_output=True, timeout=30)
    return {
        "returncode": completed.returncode,
        "stdout": completed.stdout,
        "stderr": completed.stderr,
    }


def get_world_state(args: argparse.Namespace) -> dict[str, Any]:
    return run_gateway_command(
        {"action": "get_world_state"},
        client_path=args.gateway_client,
        network_interface=args.network_interface,
        timeout_s=args.timeout_s,
        startup_wait_s=args.gateway_startup_wait_s,
    )


def relocate(args: argparse.Namespace) -> dict[str, Any]:
    init_pose = {
        "x": args.init_x,
        "y": args.init_y,
        "z": 0.0,
        "q_x": 0.0,
        "q_y": 0.0,
        "q_z": 0.0,
        "q_w": 1.0,
    }
    return run_gateway_command(
        {"action": "relocate", "map_path": args.map_path, "init_pose": init_pose},
        client_path=args.gateway_client,
        network_interface=args.network_interface,
        timeout_s=args.timeout_s,
        startup_wait_s=args.gateway_startup_wait_s,
    )


def run_closed_loop(args: argparse.Namespace, command: str) -> dict[str, Any]:
    argv = [
        sys.executable,
        str(REPO_ROOT / "scripts" / "run_robot_closed_loop.py"),
        "--command",
        command,
        "--registry",
        str(args.registry),
        "--map-id",
        args.map_id,
        "--map-path",
        args.map_path,
        "--prompt-mode",
        args.prompt_mode,
        "--gateway-client",
        args.gateway_client,
        "--network-interface",
        args.network_interface,
        "--gateway-startup-wait-s",
        str(args.gateway_startup_wait_s),
        "--timeout-s",
        str(args.timeout_s),
    ]
    if args.no_live_snapshot:
        argv.append("--no-live-snapshot")
    if args.execute:
        argv.append("--execute")
    completed = subprocess.run(argv, text=True, capture_output=True, timeout=args.closed_loop_timeout_s)
    try:
        payload = json.loads(completed.stdout)
    except json.JSONDecodeError:
        payload = {"raw_stdout": completed.stdout}
    return {
        "returncode": completed.returncode,
        "result": payload,
        "stderr": completed.stderr,
    }


def monitor(args: argparse.Namespace) -> list[dict[str, Any]]:
    samples = []
    deadline = time.time() + args.monitor_s
    while time.time() < deadline:
        try:
            state = get_world_state(args)
            world = state.get("world_state", {})
            samples.append(
                {
                    "timestamp_ms": world.get("timestamp_ms"),
                    "pose": world.get("current_pose", {}).get("pose"),
                    "localization": world.get("localization"),
                    "slam_health": world.get("slam_health"),
                    "safety": world.get("safety"),
                    "navigation": world.get("navigation"),
                }
            )
        except Exception as exc:  # pragma: no cover - operational guard
            samples.append({"error": str(exc)})
        time.sleep(args.monitor_interval_s)
    return samples


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Unified GO2W local LLM agent entrypoint.")
    parser.add_argument("--command", default="", help="Natural-language command. Prefer --command-b64 over SSH if encoding is unstable.")
    parser.add_argument("--command-b64", default="", help="UTF-8 base64 encoded natural-language command.")
    parser.add_argument("--execute", action="store_true", help="Actually execute the generated navigation command.")
    parser.add_argument("--dry-run", action="store_true", help="Plan and safety-check only. This is the default when --execute is absent.")
    parser.add_argument("--start-slam", action="store_true", help="Start xt16_driver and unitree_slam before other steps.")
    parser.add_argument("--relocate", action="store_true", help="Start relocation before planning/execution.")
    parser.add_argument("--status", action="store_true", help="Print gateway world_state.")
    parser.add_argument("--monitor-s", type=float, default=0.0, help="Monitor world_state for N seconds after command.")
    parser.add_argument("--monitor-interval-s", type=float, default=2.0)
    parser.add_argument("--prompt-mode", choices=["hybrid", "intent", "light", "full"], default="hybrid")
    parser.add_argument("--registry", default=str(DEFAULT_REGISTRY))
    parser.add_argument("--map-id", default=DEFAULT_MAP_ID)
    parser.add_argument("--no-live-snapshot", action="store_true", help="Use mock snapshot for planner; gateway safety is still live.")
    parser.add_argument("--gateway-client", default=DEFAULT_GATEWAY_CLIENT)
    parser.add_argument("--network-interface", default="eth0")
    parser.add_argument("--gateway-startup-wait-s", type=float, default=4.0)
    parser.add_argument("--timeout-s", type=int, default=30)
    parser.add_argument("--closed-loop-timeout-s", type=int, default=120)
    parser.add_argument("--start-slam-script", default=DEFAULT_START_SLAM)
    parser.add_argument("--map-path", default=DEFAULT_MAP_PATH)
    parser.add_argument("--init-x", type=float, default=0.0)
    parser.add_argument("--init-y", type=float, default=0.0)
    parser.add_argument("--pretty", action="store_true")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    command = decode_command(args)
    output: dict[str, Any] = {
        "command": command,
        "execute": bool(args.execute),
        "steps": [],
    }

    if args.start_slam:
        output["steps"].append({"step": "start_slam", "result": run_start_slam(args.start_slam_script)})
        time.sleep(2)

    if args.relocate:
        output["steps"].append({"step": "relocate", "result": relocate(args)})
        time.sleep(args.gateway_startup_wait_s)

    if args.status:
        state = get_world_state(args)
        allowed, reason = gateway_allows_navigation(state)
        output["steps"].append({"step": "status", "allowed": allowed, "reason": reason, "result": state})

    if command:
        output["steps"].append({"step": "closed_loop", "result": run_closed_loop(args, command)})

    if args.monitor_s > 0:
        output["steps"].append({"step": "monitor", "samples": monitor(args)})

    print_json(output, pretty=args.pretty)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
