from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

# Scripts are run directly on the robot, so add the project root to sys.path
# and import the local package without requiring installation.
from edge_autonomy.llm_context import build_planner_context, plan_to_slam_command, simulate_local_llm_plan  # noqa: E402
from edge_autonomy.map_registry import MapRegistry  # noqa: E402
from edge_autonomy.runtime_state import build_runtime_snapshot  # noqa: E402
from edge_autonomy.slam_gateway_executor import LocalSlamGatewayExecutor  # noqa: E402
from scripts.slam_runtime_snapshot import parse_sections, run_local_snapshot  # noqa: E402


DEFAULT_REGISTRY = REPO_ROOT / "configs" / "maps" / "go2w_map_registry.example.json"
DEFAULT_GATEWAY = REPO_ROOT / "bin" / "slam_llm_command_client"


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Run the local GO2W edge planner and send its command to the C++ SLAM gateway.")
    parser.add_argument("--command", required=True, help="Natural language command, for example: 去 wp_1")
    parser.add_argument("--registry", default=str(DEFAULT_REGISTRY))
    parser.add_argument("--map-id", default="test_current_main")
    parser.add_argument("--map-path", default="/home/unitree/test.pcd")
    parser.add_argument("--gateway", default=str(DEFAULT_GATEWAY))
    parser.add_argument("--network-interface", default="eth0")
    parser.add_argument("--snapshot-json", help="Use an existing snapshot JSON instead of live SSH/ROS snapshot.")
    parser.add_argument("--dry-run", action="store_true", help="Only print planner output and slam command.")
    parser.add_argument("--pretty", action="store_true")
    return parser


def load_snapshot(args: argparse.Namespace) -> dict:
    if args.snapshot_json:
        return json.loads(Path(args.snapshot_json).read_text(encoding="utf-8"))

    # Local deployment path: collect ROS/process state from this GO2W instead
    # of SSHing into another host.
    raw = run_local_snapshot(timeout_s=30)
    return build_runtime_snapshot(
        parse_sections(raw),
        host="localhost",
        expected_map_id=args.map_id,
        expected_map_path=args.map_path,
    ).to_dict()


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    registry = MapRegistry.from_file(args.registry)
    snapshot = load_snapshot(args)

    # Current MVP uses a deterministic local planner as a stand-in for the
    # future LLM call. The output shape is intentionally kept as planner JSON,
    # then translated into the narrower SLAM gateway command below.
    context = build_planner_context(snapshot, registry, user_command=args.command, map_id=args.map_id)
    plan = simulate_local_llm_plan(context, registry)
    slam_command = plan_to_slam_command(plan, registry)
    output = {"planner_input": context, "mock_llm_plan": plan, "slam_command": slam_command}

    if args.dry_run or slam_command is None:
        print(json.dumps(output, ensure_ascii=False, indent=2 if args.pretty else None))
        return 0 if slam_command is not None else 2

    with LocalSlamGatewayExecutor(args.gateway, network_interface=args.network_interface) as executor:
        output["slam_gateway_response"] = executor.send_command(slam_command)

    print(json.dumps(output, ensure_ascii=False, indent=2 if args.pretty else None))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
