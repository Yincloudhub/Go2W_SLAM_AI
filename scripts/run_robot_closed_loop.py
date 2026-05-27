from __future__ import annotations

import argparse
import getpass
import json
import os
import subprocess
import sys
import time
from pathlib import Path
from typing import Any


REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = REPO_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))


try:
    sys.stdout.reconfigure(encoding="utf-8")
    sys.stderr.reconfigure(encoding="utf-8")
except (AttributeError, ValueError):
    pass

from edge_autonomy.llm_context import build_planner_context, plan_to_slam_command  # noqa: E402
from edge_autonomy.local_llm_planner import (  # noqa: E402
    DEFAULT_SYSTEM_PROMPT,
    LocalCommandBackend,
    build_lightweight_planner_context,
    run_local_llm_planner,
)
from edge_autonomy.map_registry import MapRegistry  # noqa: E402
from edge_autonomy.runtime_state import build_runtime_snapshot  # noqa: E402
from edge_autonomy.task_queue import task_step_id, validate_task_queue  # noqa: E402
from scripts.slam_runtime_snapshot import parse_sections, run_remote_snapshot  # noqa: E402


DEFAULT_REGISTRY = REPO_ROOT / "configs" / "maps" / "go2w_floorplan_v4_map_registry.json"
SIMULATION_MAP_STATUSES = {"simulation", "simulated", "demo", "synthetic"}
DEFAULT_MODEL = "/home/unitree/models/Qwen_Qwen3-4B-Instruct-2507-Q4_K_M.gguf"
DEFAULT_ASK_SCRIPT = "/home/unitree/llm_runtime/scripts/ask_qwen.sh"
DEFAULT_LOCAL_COMMAND = (
    f"MODEL_PATH={DEFAULT_MODEL} {DEFAULT_ASK_SCRIPT} "
    "--ctx 4096 --max-tokens {max_tokens} --system {system} {prompt}"
)


def run_gateway_command(
    command: dict[str, Any],
    *,
    client_path: str,
    network_interface: str,
    timeout_s: int,
    startup_wait_s: float = 0.0,
) -> dict[str, Any]:
    import subprocess

    payload = json.dumps(command, ensure_ascii=False, separators=(",", ":")) + "\n"
    process = subprocess.Popen(
        [client_path, network_interface],
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    if startup_wait_s > 0:
        time.sleep(startup_wait_s)
    stdout, stderr = process.communicate(payload, timeout=timeout_s)
    if process.returncode != 0:
        raise RuntimeError(stderr.strip() or f"gateway command failed with exit {process.returncode}")

    decoder = json.JSONDecoder()
    objects: list[dict[str, Any]] = []
    for index, char in enumerate(stdout):
        if char != "{":
            continue
        try:
            value, _ = decoder.raw_decode(stdout[index:])
        except json.JSONDecodeError:
            continue
        if isinstance(value, dict):
            objects.append(value)
    if not objects:
        raise RuntimeError(f"gateway did not return JSON: {stdout[-1000:]}")
    for value in objects:
        if {"accepted", "action", "world_state"}.issubset(value.keys()):
            return value
    for value in objects:
        if "accepted" in value:
            return value
    return objects[-1]


def gateway_allows_navigation(world_state_result: dict[str, Any]) -> tuple[bool, str]:
    world_state = world_state_result.get("world_state", {})
    if not isinstance(world_state, dict):
        return False, "missing world_state"
    safety = world_state.get("safety", {})
    if isinstance(safety, dict):
        if safety.get("allow_navigation") is not True:
            return False, f"safety disallows navigation: {safety.get('reason', 'unknown')}"
    slam_health = world_state.get("slam_health", {})
    if isinstance(slam_health, dict) and slam_health.get("status") not in (None, "ok"):
        return False, f"slam health is {slam_health.get('status')}"
    localization = world_state.get("localization", {})
    if isinstance(localization, dict) and localization.get("status") not in (None, "localized_or_tracking", "tracking", "localized"):
        return False, f"localization is {localization.get('status')}"
    return True, "gateway allows navigation"


def registry_allows_execution(registry: MapRegistry, map_id: str) -> tuple[bool, str]:
    profile = registry.get_map(map_id)
    if profile.status.lower() in SIMULATION_MAP_STATUSES:
        return False, f"map '{profile.map_id}' is marked as {profile.status}; real execution is blocked"
    if not profile.topology_nodes:
        return False, f"map '{profile.map_id}' has no topology nodes"
    return True, "registry allows execution"


def _first_nav_target(plan: dict[str, Any]) -> str | None:
    for step in plan.get("steps", []):
        if not isinstance(step, dict) or step.get("tool") != "create_navigation_subgoal":
            continue
        args = step.get("arguments", {})
        if isinstance(args, dict) and isinstance(args.get("target_node"), str):
            return args["target_node"]
    return None


def _compact_pose(value: dict[str, Any] | None) -> dict[str, Any] | None:
    if not isinstance(value, dict):
        return None
    out: dict[str, Any] = {}
    for key in ("x", "y", "z", "yaw", "q_x", "q_y", "q_z", "q_w", "speed", "mode"):
        if key in value:
            out[key] = value.get(key)
    return out


def build_semantic_trace(
    *,
    command: str,
    planner_context: dict[str, Any],
    plan: dict[str, Any],
    slam_command: dict[str, Any] | None,
    gateway_state: dict[str, Any] | None,
    registry_allowed: bool,
    registry_reason: str,
    task_queue: dict[str, Any] | None = None,
    queue_execution: dict[str, Any] | None = None,
) -> dict[str, Any]:
    summary = planner_context.get("world_state_summary", {})
    robot = summary.get("robot", {}) if isinstance(summary, dict) else {}
    slam = summary.get("slam", {}) if isinstance(summary, dict) else {}
    topology = summary.get("topology", {}) if isinstance(summary, dict) else {}
    nodes = topology.get("available_nodes", []) if isinstance(topology, dict) else []
    light_context = build_lightweight_planner_context(planner_context)
    target_node = _first_nav_target(plan) or str(light_context.get("requested_target_guess") or "")
    target_info: dict[str, Any] | None = None
    if isinstance(nodes, list):
        for node in nodes:
            if isinstance(node, dict) and node.get("node_id") == target_node:
                tags = node.get("tags", [])
                tags_list = tags if isinstance(tags, list) else []
                target_info = {
                    "node_id": node.get("node_id"),
                    "name": node.get("name"),
                    "aliases": node.get("aliases", []),
                    "tags": tags_list,
                    "distance_from_robot_m": node.get("distance_from_robot_m"),
                    "needs_calibration": "needs_calibration" in tags_list,
                    "photo_required": "photo_required" in tags_list,
                    "pose": _compact_pose(node.get("pose") if isinstance(node.get("pose"), dict) else None),
                }
                break

    tools = [step.get("tool") for step in plan.get("steps", []) if isinstance(step, dict)]
    world = gateway_state.get("world_state", {}) if isinstance(gateway_state, dict) else {}
    return {
        "command": command,
        "semantic_source": "map_registry_topology",
        "requested_target_guess": light_context.get("requested_target_guess"),
        "multi_target": light_context.get("multi_target"),
        "matched_targets": light_context.get("matched_targets"),
        "candidate_count": len(light_context.get("candidates", [])) if isinstance(light_context.get("candidates"), list) else None,
        "slam": {
            "health_status": slam.get("health_status"),
            "localization_status": slam.get("localization_status"),
            "localized": robot.get("localized"),
            "nearest_node": robot.get("nearest_node"),
            "pose": _compact_pose(robot.get("pose") if isinstance(robot.get("pose"), dict) else None),
        },
        "target": target_info,
        "planner": {
            "mode": plan.get("mode"),
            "plan_id": plan.get("plan_id"),
            "reason": plan.get("reason"),
            "tools": tools,
        },
        "task_queue": {
            "queue_id": task_queue.get("queue_id"),
            "status": task_queue.get("status"),
            "targets": task_queue.get("targets"),
            "step_count": len(task_queue.get("steps", [])) if isinstance(task_queue.get("steps"), list) else 0,
            "user_reply": task_queue.get("user_reply"),
        }
        if isinstance(task_queue, dict)
        else None,
        "queue_execution": {
            "completed": queue_execution.get("completed"),
            "failed_step": queue_execution.get("failed_step"),
            "blocked_reason": queue_execution.get("blocked_reason"),
            "event_count": len(queue_execution.get("events", [])) if isinstance(queue_execution.get("events"), list) else 0,
        }
        if isinstance(queue_execution, dict)
        else None,
        "policy_gates": {
            "registry_allowed": registry_allowed,
            "registry_reason": registry_reason,
            "gateway_checked": gateway_state is not None,
            "gateway_safety": (world.get("safety") if isinstance(world, dict) else None),
            "gateway_localization": (world.get("localization") if isinstance(world, dict) else None),
            "gateway_slam_health": (world.get("slam_health") if isinstance(world, dict) else None),
        },
        "slam_command": {
            "action": slam_command.get("action"),
            "map_id": slam_command.get("map_id"),
            "target_node": slam_command.get("target_node"),
            "target_pose": _compact_pose(slam_command.get("target_pose") if isinstance(slam_command.get("target_pose"), dict) else None),
        }
        if isinstance(slam_command, dict)
        else None,
    }


def distance_to_pose(world_state_result: dict[str, Any], target_pose: dict[str, Any]) -> float | None:
    world = world_state_result.get("world_state", {})
    pose = world.get("current_pose", {}).get("pose", {}) if isinstance(world, dict) else {}
    if not isinstance(pose, dict):
        return None
    try:
        return ((float(pose["x"]) - float(target_pose["x"])) ** 2 + (float(pose["y"]) - float(target_pose["y"])) ** 2) ** 0.5
    except (KeyError, TypeError, ValueError):
        return None


def wait_for_arrival(command: dict[str, Any], args: argparse.Namespace) -> dict[str, Any]:
    target_pose = command.get("target_pose")
    if not isinstance(target_pose, dict):
        return {"arrived": False, "paused": False, "reason": "missing target_pose", "samples": []}

    samples = []
    entered_count = 0
    deadline = time.time() + args.arrival_monitor_s
    while time.time() < deadline:
        state = run_gateway_command(
            {"action": "get_world_state"},
            client_path=args.gateway_client,
            network_interface=args.network_interface,
            timeout_s=args.timeout_s,
            startup_wait_s=args.gateway_startup_wait_s,
        )
        distance_m = distance_to_pose(state, target_pose)
        world = state.get("world_state", {}) if isinstance(state, dict) else {}
        sample = {
            "timestamp_ms": world.get("timestamp_ms") if isinstance(world, dict) else None,
            "distance_to_target_m": distance_m,
            "navigation": world.get("navigation") if isinstance(world, dict) else None,
            "localization": world.get("localization") if isinstance(world, dict) else None,
            "safety": world.get("safety") if isinstance(world, dict) else None,
        }
        samples.append(sample)
        if distance_m is not None and distance_m <= args.arrival_distance_m:
            entered_count += 1
            if entered_count >= args.arrival_confirm_samples:
                pause_result = run_gateway_command(
                    {"action": "pause_navigation"},
                    client_path=args.gateway_client,
                    network_interface=args.network_interface,
                    timeout_s=args.timeout_s,
                    startup_wait_s=args.gateway_startup_wait_s,
                )
                return {
                    "arrived": True,
                    "paused": bool(pause_result.get("accepted")),
                    "threshold_m": args.arrival_distance_m,
                    "samples": samples,
                    "pause_result": pause_result,
                }
        else:
            entered_count = 0
        time.sleep(args.arrival_monitor_interval_s)

    return {
        "arrived": False,
        "paused": False,
        "threshold_m": args.arrival_distance_m,
        "samples": samples,
        "reason": "arrival threshold not reached before timeout",
    }


def run_capture_keyframe(task: dict[str, Any], args: argparse.Namespace) -> dict[str, Any]:
    target_node = str(task.get("target_node") or "")
    if not args.capture_command:
        return {
            "captured": False,
            "target_node": target_node,
            "reason": "capture command not configured; recorded semantic keyframe event only",
        }
    env = os.environ.copy()
    env["GO2W_TARGET_NODE"] = target_node
    completed = subprocess.run(
        ["bash", "-lc", args.capture_command],
        text=True,
        encoding="utf-8",
        errors="replace",
        capture_output=True,
        timeout=args.timeout_s,
        env=env,
    )
    return {
        "captured": completed.returncode == 0,
        "target_node": target_node,
        "returncode": completed.returncode,
        "stdout": completed.stdout,
        "stderr": completed.stderr,
    }


def execute_task_queue(
    task_queue: dict[str, Any],
    *,
    registry: MapRegistry,
    args: argparse.Namespace,
    nav_speed: float | None,
) -> dict[str, Any]:
    validate_task_queue(task_queue)
    events: list[dict[str, Any]] = []
    blocked_reason = ""
    failed_step = None
    profile = registry.get_map(args.map_id)

    for task in task_queue.get("steps", []):
        if not isinstance(task, dict):
            continue
        action = str(task.get("action") or "")
        task_id = task_step_id(task)
        if action == "report":
            events.append({"task_id": task_id, "action": action, "status": "ok", "message": task.get("message")})
            continue
        if action == "capture_keyframe":
            result = run_capture_keyframe(task, args) if args.execute else {"captured": False, "reason": "dry run; capture not executed", "target_node": task.get("target_node")}
            events.append({"task_id": task_id, "action": action, "status": "ok", "result": result})
            continue
        if action != "navigate":
            events.append({"task_id": task_id, "action": action, "status": "skipped", "reason": "unsupported task action"})
            continue

        target_node = str(task.get("target_node") or "")
        try:
            slam_command = profile.navigate_to_node_command(target_node, speed=nav_speed, mode=args.nav_mode)
        except Exception as exc:
            blocked_reason = f"failed to build navigation command for {target_node}: {exc}"
            failed_step = task_id
            events.append({"task_id": task_id, "action": action, "status": "failed", "target_node": target_node, "blocked_reason": blocked_reason})
            break

        preflight_state = None
        gateway_allowed = True
        gateway_reason = "gateway check skipped"
        if not args.skip_gateway_check:
            preflight_state = run_gateway_command(
                {"action": "get_world_state"},
                client_path=args.gateway_client,
                network_interface=args.network_interface,
                timeout_s=args.timeout_s,
                startup_wait_s=args.gateway_startup_wait_s,
            )
            gateway_allowed, gateway_reason = gateway_allows_navigation(preflight_state)
        if not gateway_allowed:
            blocked_reason = gateway_reason
            failed_step = task_id
            events.append(
                {
                    "task_id": task_id,
                    "action": action,
                    "status": "blocked",
                    "target_node": target_node,
                    "blocked_reason": blocked_reason,
                    "preflight": preflight_state,
                }
            )
            break

        if not args.execute:
            events.append(
                {
                    "task_id": task_id,
                    "action": action,
                    "status": "dry_run",
                    "target_node": target_node,
                    "slam_command": slam_command,
                    "preflight": preflight_state,
                }
            )
            continue

        result = run_gateway_command(
            slam_command,
            client_path=args.gateway_client,
            network_interface=args.network_interface,
            timeout_s=args.timeout_s,
            startup_wait_s=args.gateway_startup_wait_s,
        )
        accepted = bool(result.get("accepted", False))
        arrival = wait_for_arrival(slam_command, args) if accepted else {"arrived": False, "reason": "gateway did not accept navigation"}
        status = "ok" if accepted and arrival.get("arrived") else "failed"
        events.append(
            {
                "task_id": task_id,
                "action": action,
                "status": status,
                "target_node": target_node,
                "slam_command": slam_command,
                "send_result": result,
                "arrival": arrival,
            }
        )
        if status != "ok":
            blocked_reason = str(arrival.get("reason") or "navigation failed")
            failed_step = task_id
            break

    return {
        "queue_id": task_queue.get("queue_id"),
        "executed": bool(args.execute),
        "completed": failed_step is None,
        "failed_step": failed_step,
        "blocked_reason": blocked_reason,
        "events": events,
        "user_reply": task_queue.get("user_reply"),
    }


def build_snapshot(args: argparse.Namespace) -> dict[str, Any]:
    if args.no_live_snapshot:
        return {
            "timestamp_ms": int(time.time() * 1000),
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


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Run the robot LLM planner with gateway safety gating.")
    parser.add_argument("--command", required=True)
    parser.add_argument("--registry", default=str(DEFAULT_REGISTRY))
    parser.add_argument("--map-id", default="floorplan_demo_v4")
    parser.add_argument("--map-path", default="/home/unitree/maps/floorplan_demo_v4.pcd")
    parser.add_argument("--prompt-mode", choices=["hybrid", "intent", "light", "full"], default="hybrid")
    parser.add_argument("--local-command", default=DEFAULT_LOCAL_COMMAND)
    parser.add_argument("--system", default=DEFAULT_SYSTEM_PROMPT)
    parser.add_argument("--max-tokens", type=int, default=128)
    parser.add_argument("--timeout-s", type=int, default=30)
    parser.add_argument("--robot-host", default="192.168.3.17")
    parser.add_argument("--robot-username", default="unitree")
    parser.add_argument("--robot-password", default=os.environ.get("GO2W_SSH_PASSWORD", ""))
    parser.add_argument("--no-live-snapshot", action="store_true")
    parser.add_argument("--mock-x", type=float, default=1.154)
    parser.add_argument("--mock-y", type=float, default=-0.147)
    parser.add_argument("--mock-yaw", type=float, default=-0.03)
    parser.add_argument("--gateway-client", default="/home/unitree/slam_gateway_refactor/build/slam_llm_command_client")
    parser.add_argument("--network-interface", default="eth0")
    parser.add_argument("--gateway-startup-wait-s", type=float, default=4.0)
    parser.add_argument("--skip-gateway-check", action="store_true")
    parser.add_argument("--nav-speed-mps", type=float, default=0.0, help="Override navigation speed for the generated slam command. 0 keeps registry/plan speed.")
    parser.add_argument("--nav-mode", type=int, default=None, help="Override Unitree navigation mode for the generated slam command.")
    parser.add_argument("--arrival-distance-m", type=float, default=0.25)
    parser.add_argument("--arrival-confirm-samples", type=int, default=2)
    parser.add_argument("--arrival-monitor-s", type=float, default=25.0)
    parser.add_argument("--arrival-monitor-interval-s", type=float, default=1.0)
    parser.add_argument("--capture-command", default="", help="Optional bash command for capture_keyframe; GO2W_TARGET_NODE is set.")
    parser.add_argument("--execute", action="store_true", help="Actually send the navigation command after safety gates pass.")
    parser.add_argument("--pretty", action="store_true")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    registry = MapRegistry.from_file(args.registry)
    registry_allowed, registry_reason = registry_allows_execution(registry, args.map_id)
    if args.execute and not registry_allowed:
        output = {
            "command": args.command,
            "dry_run": False,
            "planner": None,
            "gateway": {"checked": False, "allowed": False, "reason": "not checked; registry blocked execution"},
            "execution": {"executed": False, "blocked_reason": registry_reason, "result": None},
        }
        if args.pretty:
            print(json.dumps(output, ensure_ascii=False, indent=2))
        else:
            print(json.dumps(output, ensure_ascii=False, separators=(",", ":")))
        return 2
    snapshot = build_snapshot(args)
    planner_context = build_planner_context(snapshot, registry, user_command=args.command, map_id=args.map_id)

    result = run_local_llm_planner(
        planner_context,
        LocalCommandBackend(args.local_command),
        system_prompt=args.system,
        max_tokens=args.max_tokens,
        timeout_s=args.timeout_s,
        prompt_mode=args.prompt_mode,
    )
    nav_speed = args.nav_speed_mps if args.nav_speed_mps > 0 else None
    slam_command = plan_to_slam_command(result.plan, registry, speed=nav_speed, mode=args.nav_mode)
    task_queue = result.task_queue
    queue_execution = None

    gateway_state = None
    gateway_allowed = False
    gateway_reason = "gateway check skipped"
    if not args.skip_gateway_check:
        gateway_state = run_gateway_command(
            {"action": "get_world_state"},
            client_path=args.gateway_client,
            network_interface=args.network_interface,
            timeout_s=args.timeout_s,
            startup_wait_s=args.gateway_startup_wait_s,
        )
        gateway_allowed, gateway_reason = gateway_allows_navigation(gateway_state)

    executed = False
    execution_result = None
    blocked_reason = ""
    if isinstance(task_queue, dict) and result.plan.get("mode") == "mapped_navigation":
        queue_execution = execute_task_queue(task_queue, registry=registry, args=args, nav_speed=nav_speed)
        executed = bool(args.execute and queue_execution.get("completed"))
        blocked_reason = str(queue_execution.get("blocked_reason") or ("dry run; pass --execute to send queued commands" if not args.execute else ""))
        execution_result = queue_execution
    elif slam_command is None:
        blocked_reason = "planner did not produce a slam command"
    elif not args.execute:
        blocked_reason = "dry run; pass --execute to send command"
    elif not gateway_allowed and not args.skip_gateway_check:
        blocked_reason = gateway_reason
    else:
        execution_result = run_gateway_command(
            slam_command,
            client_path=args.gateway_client,
            network_interface=args.network_interface,
            timeout_s=args.timeout_s,
            startup_wait_s=args.gateway_startup_wait_s,
        )
        executed = bool(execution_result.get("accepted", False))

    output = {
        "command": args.command,
        "dry_run": not args.execute,
        "planner": {
            "llm_elapsed_s": result.elapsed_s,
            "plan": result.plan,
            "slam_command": slam_command,
            "task_queue": task_queue,
            "user_reply": result.user_reply,
            "weak_link_payload": result.weak_link_payload,
        },
        "semantic_trace": build_semantic_trace(
            command=args.command,
            planner_context=planner_context,
            plan=result.plan,
            slam_command=slam_command,
            gateway_state=gateway_state,
            registry_allowed=registry_allowed,
            registry_reason=registry_reason,
            task_queue=task_queue,
            queue_execution=queue_execution,
        ),
        "queue_execution": queue_execution,
        "gateway": {
            "checked": not args.skip_gateway_check,
            "allowed": gateway_allowed,
            "reason": gateway_reason,
            "world_state": gateway_state,
        },
        "registry_gate": {
            "allowed": registry_allowed,
            "reason": registry_reason,
            "registry": args.registry,
            "map_id": args.map_id,
        },
        "execution": {
            "executed": executed,
            "blocked_reason": blocked_reason,
            "result": execution_result,
        },
    }
    if args.pretty:
        print(json.dumps(output, ensure_ascii=False, indent=2))
    else:
        print(json.dumps(output, ensure_ascii=False, separators=(",", ":")))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
