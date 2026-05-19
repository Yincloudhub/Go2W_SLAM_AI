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
from edge_autonomy.map_registry import MapRegistry  # noqa: E402
from edge_autonomy.runtime_state import build_runtime_snapshot  # noqa: E402
from scripts.slam_runtime_snapshot import parse_sections, run_remote_snapshot  # noqa: E402


DEFAULT_REGISTRY = REPO_ROOT / "configs" / "maps" / "go2w_map_registry.example.json"


def collect_snapshot(args: argparse.Namespace) -> dict:
    if args.snapshot_json:
        return json.loads(Path(args.snapshot_json).read_text(encoding="utf-8"))

    password = args.password or getpass.getpass(f"{args.username}@{args.host} password: ")
    raw = run_remote_snapshot(args.host, args.username, password, timeout_s=args.timeout_s)
    snapshot = build_runtime_snapshot(
        parse_sections(raw),
        host=args.host,
        expected_map_id=args.map_id,
        expected_map_path=args.map_path,
    )
    return snapshot.to_dict()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Build GO2W planner input and simulate a local LLM plan.")
    parser.add_argument("--registry", default=str(DEFAULT_REGISTRY))
    parser.add_argument("--command", required=True, help="User command to simulate, for example: 回到起点 or 去 wp_1")
    parser.add_argument("--map-id", default="test_current_main")
    parser.add_argument("--map-path", default="/home/unitree/test.pcd")
    parser.add_argument("--snapshot-json", help="Use an existing snapshot JSON instead of collecting live state.")
    parser.add_argument("--host", default="192.168.123.18")
    parser.add_argument("--username", default="unitree")
    parser.add_argument("--password", default=os.environ.get("GO2W_SSH_PASSWORD", ""))
    parser.add_argument("--timeout-s", type=int, default=40)
    parser.add_argument("--pretty", action="store_true")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    registry = MapRegistry.from_file(args.registry)
    snapshot = collect_snapshot(args)
    context = build_planner_context(snapshot, registry, user_command=args.command, map_id=args.map_id)
    plan = simulate_local_llm_plan(context, registry)
    slam_command = plan_to_slam_command(plan, registry)
    output = {
        "planner_input": context,
        "mock_llm_plan": plan,
        "slam_command": slam_command,
    }
    if args.pretty:
        print(json.dumps(output, ensure_ascii=False, indent=2))
    else:
        print(json.dumps(output, ensure_ascii=False, separators=(",", ":")))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
