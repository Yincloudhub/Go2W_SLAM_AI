from __future__ import annotations

import argparse
import getpass
import json
import os
import sys
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = REPO_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from edge_autonomy.llm_context import build_planner_context, plan_to_slam_command, simulate_local_llm_plan  # noqa: E402
from edge_autonomy.local_llm_planner import (  # noqa: E402
    DEFAULT_SYSTEM_PROMPT,
    LocalCommandBackend,
    SshAskQwenBackend,
    run_local_llm_planner,
)
from edge_autonomy.map_registry import MapRegistry  # noqa: E402
from edge_autonomy.runtime_state import build_runtime_snapshot  # noqa: E402
from scripts.slam_runtime_snapshot import parse_sections, run_remote_snapshot  # noqa: E402


DEFAULT_REGISTRY = REPO_ROOT / "configs" / "maps" / "go2w_map_registry.example.json"
DEFAULT_NX_MODEL = "/home/ysy/models/Qwen_Qwen3-4B-Instruct-2507-Q4_K_M.gguf"


def collect_snapshot(args: argparse.Namespace) -> dict:
    if args.snapshot_json:
        return json.loads(Path(args.snapshot_json).read_text(encoding="utf-8"))
    if args.no_live_snapshot:
        return {
            "timestamp_ms": 0,
            "expected_map_id": args.map_id,
            "expected_map_path": args.map_path,
            "health_status": "ok",
            "localization_status": "localized_or_tracking",
            "processes": {"unitree_slam": True},
            "lidar_state": {"alive": True, "cloud_frequency_hz": 15.0, "cloud_size": 56000, "error_state": 0},
            "live_pointcloud": {"alive": True, "topic": "/unitree/slam_lidar/points", "width": 56000},
            "relocation_odom": {"alive": True, "x": args.mock_x, "y": args.mock_y, "z": 0.0, "yaw": args.mock_yaw},
        }

    password = args.robot_password or getpass.getpass(f"{args.robot_username}@{args.robot_host} password: ")
    raw = run_remote_snapshot(args.robot_host, args.robot_username, password, timeout_s=args.timeout_s)
    snapshot = build_runtime_snapshot(
        parse_sections(raw),
        host=args.robot_host,
        expected_map_id=args.map_id,
        expected_map_path=args.map_path,
    )
    return snapshot.to_dict()


def make_backend(args: argparse.Namespace):
    if args.backend == "mock":
        return None
    if args.backend == "ssh":
        password = args.nx_password or os.environ.get("NX_SSH_PASSWORD") or getpass.getpass(f"{args.nx_username}@{args.nx_host} password: ")
        return SshAskQwenBackend(
            host=args.nx_host,
            username=args.nx_username,
            password=password,
            ask_script=args.ask_script,
            model_path=args.model_path,
        )
    return LocalCommandBackend(args.local_command)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Run GO2W local LLM planner and validate LocalLlmPlan output.")
    parser.add_argument("--backend", choices=["ssh", "command", "mock"], default="ssh")
    parser.add_argument("--command", required=True, help="User command, for example: 去国篱师兄门口看看")
    parser.add_argument("--registry", default=str(DEFAULT_REGISTRY))
    parser.add_argument("--map-id", default="test_current_main")
    parser.add_argument("--map-path", default="/home/unitree/test.pcd")
    parser.add_argument("--snapshot-json", help="Use existing runtime snapshot JSON instead of live robot collection.")
    parser.add_argument("--no-live-snapshot", action="store_true", help="Use a synthetic localized snapshot for local dry tests.")
    parser.add_argument("--mock-x", type=float, default=1.154)
    parser.add_argument("--mock-y", type=float, default=-0.147)
    parser.add_argument("--mock-yaw", type=float, default=-0.03)

    parser.add_argument("--robot-host", default="192.168.123.18")
    parser.add_argument("--robot-username", default="unitree")
    parser.add_argument("--robot-password", default=os.environ.get("GO2W_SSH_PASSWORD", ""))

    parser.add_argument("--nx-host", default="192.168.33.30")
    parser.add_argument("--nx-username", default="ysy")
    parser.add_argument("--nx-password", default=os.environ.get("NX_SSH_PASSWORD", ""))
    parser.add_argument("--ask-script", default="/home/ysy/ask_qwen.sh")
    parser.add_argument("--model-path", default=DEFAULT_NX_MODEL)

    parser.add_argument(
        "--local-command",
        default="MODEL_PATH=/home/ysy/models/Qwen_Qwen3-4B-Instruct-2507-Q4_K_M.gguf /home/ysy/ask_qwen.sh --max-tokens {max_tokens} --system {system} {prompt}",
        help="Command template for --backend command. Available fields: {prompt}, {system}, {max_tokens}.",
    )
    parser.add_argument("--system", default=DEFAULT_SYSTEM_PROMPT)
    parser.add_argument("--max-tokens", type=int, default=768)
    parser.add_argument("--timeout-s", type=int, default=240)
    parser.add_argument("--pretty", action="store_true")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    registry = MapRegistry.from_file(args.registry)
    snapshot = collect_snapshot(args)
    planner_context = build_planner_context(snapshot, registry, user_command=args.command, map_id=args.map_id)

    if args.backend == "mock":
        plan = simulate_local_llm_plan(planner_context, registry)
        raw_answer = json.dumps(plan, ensure_ascii=False)
        elapsed_s = 0.0
    else:
        backend = make_backend(args)
        result = run_local_llm_planner(
            planner_context,
            backend,
            system_prompt=args.system,
            max_tokens=args.max_tokens,
            timeout_s=args.timeout_s,
        )
        plan = result.plan
        raw_answer = result.raw_answer
        elapsed_s = result.elapsed_s

    output = {
        "planner_input": planner_context,
        "local_llm_plan": plan,
        "slam_command": plan_to_slam_command(plan, registry),
        "llm_elapsed_s": elapsed_s,
        "raw_answer": raw_answer,
    }
    if args.pretty:
        print(json.dumps(output, ensure_ascii=False, indent=2))
    else:
        print(json.dumps(output, ensure_ascii=False, separators=(",", ":")))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
