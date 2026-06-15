from __future__ import annotations

import argparse
import copy
import getpass
import json
import math
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
from edge_autonomy.chassis_controller import PersistentGatewaySession  # noqa: E402
from edge_autonomy.communication_policy import (  # noqa: E402
    AppendOnlyJournal,
    CommunicationPolicyExecutor,
)
from edge_autonomy.gateway_safety import gateway_allows_navigation  # noqa: E402
from edge_autonomy.local_llm_planner import (  # noqa: E402
    DEFAULT_SYSTEM_PROMPT,
    LocalCommandBackend,
    build_lightweight_planner_context,
    plan_to_task_queue,
    run_local_llm_planner,
)
from edge_autonomy.map_registry import MapProfile, MapRegistry  # noqa: E402
from edge_autonomy.mission_decision import (  # noqa: E402
    build_mission_decision,
    build_recovery_decision,
)
from edge_autonomy.operator_display import build_operator_display_state  # noqa: E402
from edge_autonomy.perception_context import load_perception_context_file  # noqa: E402
from edge_autonomy.runtime_state import build_runtime_snapshot  # noqa: E402
from edge_autonomy.runtime_log import build_runtime_log_record  # noqa: E402
from edge_autonomy.task_queue import task_step_id, validate_task_queue  # noqa: E402
from edge_autonomy.world_state_v1 import build_world_state_v1  # noqa: E402
from scripts.slam_runtime_snapshot import parse_sections, run_remote_snapshot  # noqa: E402


PersistentNavigationSession = PersistentGatewaySession


DEFAULT_REGISTRY = REPO_ROOT / "configs" / "maps" / "go2w_real_site_map_registry.json"
DEFAULT_MAP_ID = "go2w_real_site"
DEFAULT_MAP_PATH = "/home/unitree/test.pcd"
SIMULATION_MAP_STATUSES = {"simulation", "simulated", "demo", "synthetic"}
DEFAULT_MODEL = "/home/unitree/models/Qwen_Qwen3-4B-Instruct-2507-Q4_K_M.gguf"
DEFAULT_ASK_SCRIPT = "/home/unitree/llm_runtime/scripts/ask_qwen.sh"
DEFAULT_COMMUNICATION_JOURNAL = (
    REPO_ROOT / "artifacts" / "communication" / "communication_journal_v1.jsonl"
)
UNITREE_GO2_NAV_MIN_SPEED_MPS = 0.2
BLOCKING_NAVIGATION_TARGET_TAGS = frozenset(
    {
        "disabled",
        "ui_disabled",
        "deleted",
        "needs_calibration",
        "needs_standing_verification",
        "requires_standing_verification",
    }
)
DEFAULT_LOCAL_COMMAND = (
    f"MODEL_PATH={DEFAULT_MODEL} {DEFAULT_ASK_SCRIPT} "
    "--ctx 4096 --max-tokens {max_tokens} --system {system} {prompt}"
)
FEEDBACK_SYSTEM_PROMPT = (
    "你是 Unitree GO2W 机器狗的操作面板反馈助手。"
    "只输出一句自然中文，面向现场操作者，说明当前去哪、是否到达、是否需要人工介入。"
    "不要输出 JSON、Markdown、解释或多余前后缀。"
)
RECOVERY_STRATEGY_SYSTEM_PROMPT = (
    "You select one bounded straight recovery translation for a Unitree GO2W robot "
    "after native navigation has stalled. "
    "Return exactly one JSON object with keys action, direction, distance_m, "
    "confidence, reason. action must be reposition or hold. "
    "For reposition, choose only a listed candidate direction and choose "
    "distance_m between 0.20 and that candidate's distance_m. "
    "Keep reason to 12 words or fewer. "
    "Never output velocity, yaw rate, raw API ids, shell commands, or extra keys."
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


def gateway_rejection_reason(result: dict[str, Any]) -> str:
    top_reason = str(result.get("reason") or "gateway_rejected_navigation")
    safety = result.get("safety")
    if not isinstance(safety, dict):
        world = result.get("world_state")
        safety = world.get("safety") if isinstance(world, dict) and isinstance(world.get("safety"), dict) else {}
    safety_reason = str(safety.get("reason") or "") if isinstance(safety, dict) else ""
    if safety_reason and safety_reason != top_reason:
        return f"{top_reason}:{safety_reason}"
    return top_reason


def registry_allows_execution(registry: MapRegistry, map_id: str) -> tuple[bool, str]:
    profile = registry.get_map(map_id)
    if profile.status.lower() in SIMULATION_MAP_STATUSES:
        return False, f"map '{profile.map_id}' is marked as {profile.status}; real execution is blocked"
    if not profile.topology_nodes:
        return False, f"map '{profile.map_id}' has no topology nodes"
    return True, "registry allows execution"


def topology_target_allows_navigation(profile: MapProfile, node_id_or_alias: str) -> tuple[bool, str]:
    node = profile.get_node(node_id_or_alias)
    blocking_tags = sorted(set(node.tags) & BLOCKING_NAVIGATION_TARGET_TAGS)
    if blocking_tags:
        return False, f"target '{node.node_id}' is not cleared for real navigation: {', '.join(blocking_tags)}"
    if not math.isfinite(node.pose.x) or not math.isfinite(node.pose.y):
        return False, f"target '{node.node_id}' pose x/y is invalid"
    return True, "topology target allows navigation"


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
    mission_decision: dict[str, Any] | None = None,
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
                    "needs_standing_verification": "needs_standing_verification" in tags_list,
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
        "mission_decision": mission_decision,
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


def navigation_monitor_budget_s(
    command: dict[str, Any],
    distance_m: float | None,
    args: argparse.Namespace,
) -> float:
    minimum_s = max(1.0, float(getattr(args, "arrival_monitor_s", 25.0)))
    if distance_m is None or not math.isfinite(distance_m) or distance_m < 0.0:
        return minimum_s
    target_pose = command.get("target_pose", {})
    speed_candidates = (
        target_pose.get("speed") if isinstance(target_pose, dict) else None,
        command.get("speed_mps"),
        getattr(args, "nav_speed_mps", None),
    )
    speed_mps = next(
        (
            float(value)
            for value in speed_candidates
            if isinstance(value, (int, float))
            and math.isfinite(float(value))
            and float(value) > 0.0
        ),
        None,
    )
    if speed_mps is None:
        return minimum_s
    travel_factor = max(
        1.0,
        float(getattr(args, "navigation_monitor_travel_factor", 1.8)),
    )
    startup_margin_s = max(
        0.0,
        float(getattr(args, "navigation_monitor_startup_margin_s", 15.0)),
    )
    return max(minimum_s, distance_m / speed_mps * travel_factor + startup_margin_s)


def navigation_target_progress_deadline(
    deadline: float,
    *,
    now: float,
    distance_progress: bool,
    command: dict[str, Any],
    distance_m: float | None,
    args: argparse.Namespace,
) -> float:
    if not distance_progress or distance_m is None:
        return deadline
    return max(
        deadline,
        now + navigation_monitor_budget_s(command, distance_m, args),
    )


def normalize_unitree_navigation_speed(
    speed_mps: float | None,
    mode: int | None,
) -> float | None:
    if speed_mps is None:
        return None
    if mode == 0 and speed_mps > 0.0:
        return max(speed_mps, UNITREE_GO2_NAV_MIN_SPEED_MPS)
    return speed_mps


def navigation_motion_progressed(
    previous_pose: dict[str, Any] | None,
    current_pose: dict[str, Any] | None,
    *,
    distance_threshold_m: float,
    yaw_threshold_rad: float,
) -> bool:
    if not isinstance(previous_pose, dict) or not isinstance(current_pose, dict):
        return False
    try:
        dx = float(current_pose["x"]) - float(previous_pose["x"])
        dy = float(current_pose["y"]) - float(previous_pose["y"])
        previous_yaw = float(previous_pose["yaw"])
        current_yaw = float(current_pose["yaw"])
    except (KeyError, TypeError, ValueError):
        return False
    yaw_delta = math.atan2(
        math.sin(current_yaw - previous_yaw),
        math.cos(current_yaw - previous_yaw),
    )
    return (
        math.hypot(dx, dy) >= max(0.0, distance_threshold_m)
        or abs(yaw_delta) >= max(0.0, yaw_threshold_rad)
    )


def operator_feedback_message(
    phase: str,
    text: str,
    *,
    target_node: str = "",
    target_name: str = "",
    severity: str = "info",
    distance_m: float | None = None,
) -> dict[str, Any]:
    message = {
        "phase": phase,
        "severity": severity,
        "channel": "operator_display",
        "llm_surface": True,
        "text": text,
        "target_node": target_node,
        "target_name": target_name or target_node,
    }
    if distance_m is not None:
        message["distance_to_target_m"] = distance_m
    return message


def build_llm_feedback_request(
    phase: str,
    *,
    target_node: str,
    target_name: str,
    distance_m: float | None = None,
    reason: str = "",
) -> dict[str, Any]:
    return {
        "phase": phase,
        "target_node": target_node,
        "target_name": target_name,
        "distance_to_target_m": distance_m,
        "reason": reason,
        "instruction": "用一句自然中文向操作者说明当前正在前往哪里、剩余大致距离、是否到达或是否需要人工干预。",
    }


def template_llm_feedback_text(request: dict[str, Any]) -> str:
    phase = str(request.get("phase") or "progress")
    target_name = str(request.get("target_name") or request.get("target_node") or "目标点")
    reason = str(request.get("reason") or "")
    distance = request.get("distance_to_target_m")
    if phase == "queued":
        return f"已将{target_name}加入任务队列，等待执行。"
    if phase == "arrived":
        return f"已到达{target_name}，导航已暂停。"
    if phase == "blocked":
        suffix = f"：{reason}" if reason else "，请人工确认。"
        return f"前往{target_name}已被安全门阻断{suffix}"
    if phase == "timeout":
        return f"还未确认到达{target_name}，已停止后续队列，请人工确认。"
    try:
        return f"正在前往{target_name}，距离约{float(distance):.2f}米。"
    except (TypeError, ValueError):
        return f"正在前往{target_name}。"


def clean_llm_feedback_text(raw_text: str, fallback: str) -> str:
    text = raw_text.strip().strip("`").strip()
    if not text:
        return fallback
    if text.startswith('"') and text.endswith('"') and len(text) >= 2:
        text = text[1:-1].strip()
    first_line = next((line.strip() for line in text.splitlines() if line.strip()), "")
    text = first_line or text
    return text[:120] if len(text) > 120 else text


def append_limited(items: list[dict[str, Any]], item: dict[str, Any], max_items: int) -> int:
    if max_items <= 0:
        return 1
    dropped = 0
    while len(items) >= max_items:
        del items[0]
        dropped += 1
    items.append(item)
    return dropped


def generate_llm_feedback_result(
    request: dict[str, Any],
    args: argparse.Namespace,
    *,
    backend: LocalCommandBackend | None = None,
) -> dict[str, Any] | None:
    mode = str(getattr(args, "llm_feedback_mode", "template"))
    if mode == "off":
        return None

    fallback = template_llm_feedback_text(request)
    base = {
        "phase": request.get("phase"),
        "target_node": request.get("target_node"),
        "target_name": request.get("target_name"),
        "distance_to_target_m": request.get("distance_to_target_m"),
        "llm_surface": True,
    }
    if mode != "live":
        return {**base, "source": "template", "text": fallback}
    if request.get("phase") == "queued":
        return {**base, "source": "template_queue_budget", "text": fallback, "live_deferred": True}
    if request.get("phase") == "progress" and not bool(getattr(args, "llm_feedback_live_progress", False)):
        return {**base, "source": "template_progress_budget", "text": fallback, "live_deferred": True}

    prompt = (
        "请根据这条机器人运行反馈请求，输出一句给现场操作者看的中文短句。\n"
        f"{json.dumps(request, ensure_ascii=False, separators=(',', ':'))}"
    )
    try:
        llm_backend = backend or LocalCommandBackend(args.local_command)
        raw_answer = llm_backend.generate(
            prompt,
            system_prompt=getattr(args, "llm_feedback_system", FEEDBACK_SYSTEM_PROMPT),
            max_tokens=int(getattr(args, "llm_feedback_max_tokens", 48)),
            timeout_s=int(getattr(args, "llm_feedback_timeout_s", 6)),
        )
        return {**base, "source": "local_llm", "text": clean_llm_feedback_text(raw_answer, fallback), "raw_answer": raw_answer}
    except Exception as exc:  # pragma: no cover - field robustness
        return {**base, "source": "template_fallback", "text": fallback, "error": str(exc)}


def append_llm_feedback(
    *,
    phase: str,
    target_node: str,
    target_name: str,
    distance_m: float | None,
    reason: str,
    args: argparse.Namespace,
    requests: list[dict[str, Any]],
    results: list[dict[str, Any]],
    operator_feedback: list[dict[str, Any]],
    max_feedback_events: int = 120,
    max_llm_feedback_events: int = 40,
    dropped_counts: dict[str, int] | None = None,
) -> None:
    request = build_llm_feedback_request(
        phase,
        target_node=target_node,
        target_name=target_name,
        distance_m=distance_m,
        reason=reason,
    )
    result = generate_llm_feedback_result(request, args)
    if result is None:
        return
    dropped_request = append_limited(requests, request, max_llm_feedback_events)
    dropped_result = append_limited(results, result, max_llm_feedback_events)
    message = operator_feedback_message(
        "llm_feedback",
        str(result.get("text") or ""),
        target_node=target_node,
        target_name=target_name,
        severity="info" if phase not in {"blocked", "timeout"} else "warning",
        distance_m=distance_m,
    )
    message["source"] = result.get("source")
    message["llm_request_index"] = max(0, len(requests) - 1)
    dropped_feedback = append_limited(operator_feedback, message, max_feedback_events)
    if dropped_counts is not None:
        dropped_counts["llm_feedback"] = dropped_counts.get("llm_feedback", 0) + max(dropped_request, dropped_result)
        dropped_counts["operator_feedback"] = dropped_counts.get("operator_feedback", 0) + dropped_feedback


def wait_for_arrival(
    command: dict[str, Any],
    args: argparse.Namespace,
    *,
    target_name: str = "",
    session: PersistentNavigationSession | None = None,
) -> dict[str, Any]:
    target_pose = command.get("target_pose")
    if not isinstance(target_pose, dict):
        return {"arrived": False, "paused": False, "reason": "missing target_pose", "samples": []}

    target_node = str(command.get("target_node") or "")
    target_label = target_name or target_node
    samples = []
    operator_feedback: list[dict[str, Any]] = []
    llm_feedback_requests: list[dict[str, Any]] = []
    llm_feedback_results: list[dict[str, Any]] = []
    entered_count = 0
    consecutive_errors = 0
    max_errors = max(1, int(getattr(args, "gateway_error_limit", 3)))
    poll_interval = max(0.1, float(getattr(args, "slam_poll_interval_s", 0.0) or args.arrival_monitor_interval_s))
    ui_interval = max(0.1, float(getattr(args, "ui_refresh_interval_s", poll_interval)))
    feedback_interval = max(0.1, float(getattr(args, "operator_feedback_interval_s", 5.0)))
    llm_feedback_interval = max(0.1, float(getattr(args, "llm_feedback_interval_s", 8.0)))
    max_samples = max(1, int(getattr(args, "max_arrival_samples", 120)))
    max_feedback_events = max(1, int(getattr(args, "max_feedback_events", 120)))
    max_llm_feedback_events = max(1, int(getattr(args, "max_llm_feedback_events", 40)))
    dropped_counts = {"arrival_samples": 0, "operator_feedback": 0, "llm_feedback": 0}
    perf = {"poll_overruns": 0, "max_loop_elapsed_s": 0.0}
    last_ui = time.monotonic() - ui_interval
    last_feedback = time.monotonic() - feedback_interval
    last_llm_feedback = time.monotonic() - llm_feedback_interval
    monitor_started = time.monotonic()
    monitor_budget_s = max(1.0, float(args.arrival_monitor_s))
    deadline = monitor_started + monitor_budget_s
    monitor_budget_initialized = False
    navigation_stall_s = max(
        0.0,
        float(getattr(args, "navigation_stall_s", 20.0)),
    )
    progress_distance_m = max(
        0.0,
        float(getattr(args, "navigation_progress_distance_m", 0.08)),
    )
    progress_yaw_rad = max(
        0.0,
        float(getattr(args, "navigation_progress_yaw_rad", 0.12)),
    )
    last_motion_progress = monitor_started
    progress_pose: dict[str, Any] | None = None
    best_distance_m: float | None = None

    def request_pause(reason: str) -> dict[str, Any]:
        try:
            if session is not None:
                result = session.command({"action": "pause_navigation"})
            else:
                result = run_gateway_command(
                    {"action": "pause_navigation"},
                    client_path=args.gateway_client,
                    network_interface=args.network_interface,
                    timeout_s=args.timeout_s,
                    startup_wait_s=args.gateway_startup_wait_s,
                )
            return {
                "requested": True,
                "accepted": result.get("accepted") is True,
                "reason": reason,
                "result": result,
            }
        except Exception as exc:  # pragma: no cover - field robustness
            return {
                "requested": True,
                "accepted": False,
                "reason": reason,
                "error": str(exc),
            }

    def record_performance(loop_start: float) -> float:
        elapsed = time.monotonic() - loop_start
        perf["max_loop_elapsed_s"] = max(float(perf["max_loop_elapsed_s"]), elapsed)
        if elapsed > poll_interval:
            perf["poll_overruns"] = int(perf["poll_overruns"]) + 1
            return 0.0
        return poll_interval - elapsed

    def performance_summary() -> dict[str, Any]:
        return {
            "poll_interval_s": poll_interval,
            "ui_interval_s": ui_interval,
            "operator_feedback_interval_s": feedback_interval,
            "llm_feedback_interval_s": llm_feedback_interval,
            "poll_overruns": perf["poll_overruns"],
            "max_loop_elapsed_s": round(float(perf["max_loop_elapsed_s"]), 4),
            "max_arrival_samples": max_samples,
            "max_feedback_events": max_feedback_events,
            "max_llm_feedback_events": max_llm_feedback_events,
            "dropped_counts": dropped_counts,
            "monitor_budget_s": round(monitor_budget_s, 3),
            "navigation_stall_s": navigation_stall_s,
        }

    while time.monotonic() < deadline:
        loop_start = time.monotonic()
        try:
            if session is not None:
                heartbeat = session.heartbeat()
                if heartbeat.get("accepted") is not True:
                    dropped_counts["arrival_samples"] += append_limited(
                        samples,
                        {
                            "heartbeat_rejection": heartbeat,
                            "consecutive_errors": consecutive_errors + 1,
                        },
                        max_samples,
                    )
                    raise RuntimeError(
                        f"navigation heartbeat rejected: {heartbeat.get('reason')}"
                    )
                state = session.command({"action": "get_world_state"})
            else:
                state = run_gateway_command(
                    {"action": "get_world_state"},
                    client_path=args.gateway_client,
                    network_interface=args.network_interface,
                    timeout_s=args.timeout_s,
                    startup_wait_s=args.gateway_startup_wait_s,
                )
            consecutive_errors = 0
        except Exception as exc:
            consecutive_errors += 1
            dropped_counts["arrival_samples"] += append_limited(samples, {"error": str(exc), "consecutive_errors": consecutive_errors}, max_samples)
            if consecutive_errors >= max_errors:
                reason = f"gateway feedback failed repeatedly: {exc}"
                pause = request_pause(reason)
                dropped_counts["operator_feedback"] += append_limited(
                    operator_feedback,
                    operator_feedback_message(
                        "blocked",
                        "连续读取 SLAM 状态失败，停止等待并请求人工确认。",
                        target_node=target_node,
                        target_name=target_label,
                        severity="error",
                    ),
                    max_feedback_events,
                )
                append_llm_feedback(
                    phase="blocked",
                    target_node=target_node,
                    target_name=target_label,
                    distance_m=None,
                    reason=reason,
                    args=args,
                    requests=llm_feedback_requests,
                    results=llm_feedback_results,
                    operator_feedback=operator_feedback,
                    max_feedback_events=max_feedback_events,
                    max_llm_feedback_events=max_llm_feedback_events,
                    dropped_counts=dropped_counts,
                )
                return {
                    "arrived": False,
                    "paused": pause["accepted"],
                    "threshold_m": args.arrival_distance_m,
                    "samples": samples,
                    "operator_feedback": operator_feedback,
                    "llm_feedback_requests": llm_feedback_requests,
                    "llm_feedback_results": llm_feedback_results,
                    "performance": performance_summary(),
                    "reason": reason,
                    "pause": pause,
                }
            sleep_s = record_performance(loop_start)
            if sleep_s > 0:
                time.sleep(sleep_s)
            continue

        distance_m = distance_to_pose(state, target_pose)
        world = state.get("world_state", {}) if isinstance(state, dict) else {}
        if not monitor_budget_initialized and distance_m is not None:
            monitor_budget_s = navigation_monitor_budget_s(command, distance_m, args)
            deadline = max(deadline, monitor_started + monitor_budget_s)
            monitor_budget_initialized = True
        allowed, reason = gateway_allows_navigation(state)
        if not allowed:
            pause = request_pause(reason)
            dropped_counts["operator_feedback"] += append_limited(
                operator_feedback,
                operator_feedback_message(
                    "blocked",
                    f"运行中安全门阻断，已停止等待：{reason}",
                    target_node=target_node,
                    target_name=target_label,
                    severity="error",
                    distance_m=distance_m,
                ),
                max_feedback_events,
            )
            append_llm_feedback(
                phase="blocked",
                target_node=target_node,
                target_name=target_label,
                distance_m=distance_m,
                reason=reason,
                args=args,
                requests=llm_feedback_requests,
                results=llm_feedback_results,
                operator_feedback=operator_feedback,
                max_feedback_events=max_feedback_events,
                max_llm_feedback_events=max_llm_feedback_events,
                dropped_counts=dropped_counts,
            )
            return {
                "arrived": False,
                "paused": pause["accepted"],
                "threshold_m": args.arrival_distance_m,
                "samples": samples,
                "operator_feedback": operator_feedback,
                "llm_feedback_requests": llm_feedback_requests,
                "llm_feedback_results": llm_feedback_results,
                "runtime_safety": {"allowed": False, "reason": reason},
                "performance": performance_summary(),
                "reason": reason,
                "pause": pause,
            }

        sample = {
            "timestamp_ms": world.get("timestamp_ms") if isinstance(world, dict) else None,
            "distance_to_target_m": distance_m,
            "current_pose": (
                world.get("current_pose", {}).get("pose")
                if isinstance(world, dict)
                and isinstance(world.get("current_pose"), dict)
                else None
            ),
            "navigation": world.get("navigation") if isinstance(world, dict) else None,
            "localization": world.get("localization") if isinstance(world, dict) else None,
            "safety": world.get("safety") if isinstance(world, dict) else None,
        }
        dropped_counts["arrival_samples"] += append_limited(samples, sample, max_samples)
        navigation = (
            world.get("navigation", {})
            if isinstance(world, dict)
            else {}
        )
        if (
            isinstance(navigation, dict)
            and navigation.get("state") in {"failed", "cancelled", "timeout"}
        ):
            failure_reason = str(
                navigation.get("failure_reason")
                or f"navigation state is {navigation.get('state')}"
            )
            pause = request_pause(failure_reason)
            return {
                "arrived": False,
                "paused": pause["accepted"],
                "threshold_m": args.arrival_distance_m,
                "samples": samples,
                "operator_feedback": operator_feedback,
                "llm_feedback_requests": llm_feedback_requests,
                "llm_feedback_results": llm_feedback_results,
                "performance": performance_summary(),
                "reason": failure_reason,
                "pause": pause,
            }
        now = time.monotonic()
        current_pose = (
            world.get("current_pose", {}).get("pose", {})
            if isinstance(world, dict)
            and isinstance(world.get("current_pose"), dict)
            else {}
        )
        initial_progress_sample = best_distance_m is None
        distance_progress = (
            distance_m is not None
            and best_distance_m is not None
            and best_distance_m - distance_m >= progress_distance_m
        )
        pose_progress = navigation_motion_progressed(
            progress_pose,
            current_pose,
            distance_threshold_m=progress_distance_m,
            yaw_threshold_rad=progress_yaw_rad,
        )
        if initial_progress_sample or distance_progress or pose_progress:
            progress_pose = dict(current_pose) if isinstance(current_pose, dict) else None
            if distance_m is not None:
                best_distance_m = (
                    distance_m
                    if best_distance_m is None
                    else min(best_distance_m, distance_m)
                )
            last_motion_progress = now
        deadline = navigation_target_progress_deadline(
            deadline,
            now=now,
            distance_progress=distance_progress,
            command=command,
            distance_m=distance_m,
            args=args,
        )
        if (
            command.get("action") == "navigate_to_pose"
            and navigation_stall_s > 0.0
            and now - last_motion_progress >= navigation_stall_s
        ):
            stall_reason = "native_navigation_no_progress"
            pause = request_pause(stall_reason)
            return {
                "arrived": False,
                "paused": pause["accepted"],
                "threshold_m": args.arrival_distance_m,
                "samples": samples,
                "operator_feedback": operator_feedback,
                "llm_feedback_requests": llm_feedback_requests,
                "llm_feedback_results": llm_feedback_results,
                "performance": performance_summary(),
                "reason": stall_reason,
                "pause": pause,
                "stall_evidence": {
                    "last_motion_progress_age_s": round(now - last_motion_progress, 3),
                    "best_distance_m": best_distance_m,
                    "current_distance_m": distance_m,
                },
            }
        if now - last_ui >= ui_interval:
            sample["ui_refresh"] = True
            last_ui = now
        if now - last_feedback >= feedback_interval:
            text = f"正在前往{target_label}"
            if distance_m is not None:
                text += f"，距离约{distance_m:.2f}米"
            dropped_counts["operator_feedback"] += append_limited(
                operator_feedback,
                operator_feedback_message("progress", text, target_node=target_node, target_name=target_label, distance_m=distance_m),
                max_feedback_events,
            )
            last_feedback = now
        if now - last_llm_feedback >= llm_feedback_interval:
            append_llm_feedback(
                phase="progress",
                target_node=target_node,
                target_name=target_label,
                distance_m=distance_m,
                reason="",
                args=args,
                requests=llm_feedback_requests,
                results=llm_feedback_results,
                operator_feedback=operator_feedback,
                max_feedback_events=max_feedback_events,
                max_llm_feedback_events=max_llm_feedback_events,
                dropped_counts=dropped_counts,
            )
            last_llm_feedback = now
        if distance_m is not None and distance_m <= args.arrival_distance_m:
            entered_count += 1
            if entered_count >= args.arrival_confirm_samples:
                pause = request_pause("arrival threshold confirmed")
                pause_result = pause.get("result")
                dropped_counts["operator_feedback"] += append_limited(
                    operator_feedback,
                    operator_feedback_message(
                        "arrived",
                        f"已到达{target_label}，导航已暂停。",
                        target_node=target_node,
                        target_name=target_label,
                        severity="ok",
                        distance_m=distance_m,
                    ),
                    max_feedback_events,
                )
                append_llm_feedback(
                    phase="arrived",
                    target_node=target_node,
                    target_name=target_label,
                    distance_m=distance_m,
                    reason="arrived and paused",
                    args=args,
                    requests=llm_feedback_requests,
                    results=llm_feedback_results,
                    operator_feedback=operator_feedback,
                    max_feedback_events=max_feedback_events,
                    max_llm_feedback_events=max_llm_feedback_events,
                    dropped_counts=dropped_counts,
                )
                return {
                    "arrived": True,
                    "paused": pause["accepted"],
                    "threshold_m": args.arrival_distance_m,
                    "samples": samples,
                    "operator_feedback": operator_feedback,
                    "llm_feedback_requests": llm_feedback_requests,
                    "llm_feedback_results": llm_feedback_results,
                    "pause_result": pause_result,
                    "pause": pause,
                    "performance": performance_summary(),
                    "reason": "" if pause["accepted"] else "arrival reached but pause was not accepted",
                }
        else:
            entered_count = 0
        sleep_s = record_performance(loop_start)
        if sleep_s > 0:
            time.sleep(sleep_s)

    timeout_reason = "arrival threshold not reached before timeout"
    pause = request_pause(timeout_reason)
    dropped_counts["operator_feedback"] += append_limited(
        operator_feedback,
        operator_feedback_message(
            "timeout",
            f"未在限定时间内确认到达{target_label}，停止后续队列。",
            target_node=target_node,
            target_name=target_label,
            severity="warning",
        ),
        max_feedback_events,
    )
    append_llm_feedback(
        phase="timeout",
        target_node=target_node,
        target_name=target_label,
        distance_m=None,
        reason=timeout_reason,
        args=args,
        requests=llm_feedback_requests,
        results=llm_feedback_results,
        operator_feedback=operator_feedback,
        max_feedback_events=max_feedback_events,
        max_llm_feedback_events=max_llm_feedback_events,
        dropped_counts=dropped_counts,
    )
    return {
        "arrived": False,
        "paused": pause["accepted"],
        "threshold_m": args.arrival_distance_m,
        "samples": samples,
        "operator_feedback": operator_feedback,
        "llm_feedback_requests": llm_feedback_requests,
        "llm_feedback_results": llm_feedback_results,
        "performance": performance_summary(),
        "reason": timeout_reason,
        "pause": pause,
    }


def run_supervised_navigation_session(
    command: dict[str, Any],
    args: argparse.Namespace,
    *,
    target_name: str,
) -> tuple[dict[str, Any], dict[str, Any] | None]:
    try:
        with PersistentNavigationSession(
            client_path=args.gateway_client,
            network_interface=args.network_interface,
            timeout_s=args.timeout_s,
            startup_wait_s=args.gateway_startup_wait_s,
        ) as session:
            result = session.command(command)
            if result.get("accepted") is not True:
                return result, None
            monitor_command = command
            if command.get("action") == "supervised_reposition":
                target_pose = result.get("reposition_target_pose")
                if isinstance(target_pose, dict):
                    monitor_command = {
                        **command,
                        "target_node": str(
                            target_pose.get("name")
                            or "__supervised_reposition__"
                        ),
                        "target_pose": target_pose,
                    }
            arrival = wait_for_arrival(
                monitor_command,
                args,
                target_name=target_name,
                session=session,
            )
            return result, arrival
    except Exception as exc:
        return (
            {
                "accepted": False,
                "action": command.get("action"),
                "reason": f"persistent navigation session failed: {exc}",
            },
            None,
        )


def build_recovery_translation_analysis(
    preflight_state: dict[str, Any] | None,
    slam_command: dict[str, Any],
) -> dict[str, Any]:
    world = (
        preflight_state.get("world_state", {})
        if isinstance(preflight_state, dict)
        else {}
    )
    obstacle = world.get("local_obstacle", {})
    pose = world.get("current_pose", {}).get("pose", {})
    target = slam_command.get("target_pose", {})
    supervised = obstacle.get("supervised_release", {})
    if not isinstance(supervised, dict) or supervised.get("active") is not True:
        return {
            "required": False,
            "available": False,
            "reason": "supervised release is not active",
        }
    try:
        dx = float(target["x"]) - float(pose["x"])
        dy = float(target["y"]) - float(pose["y"])
        yaw = float(pose["yaw"])
        bearing_error = math.atan2(dy, dx) - yaw
        bearing_error = math.atan2(math.sin(bearing_error), math.cos(bearing_error))
        front = float(obstacle["front_clearance_m"])
        left = float(obstacle["left_clearance_m"])
        right = float(obstacle["right_clearance_m"])
        rear = float(obstacle["rear_clearance_m"])
    except (KeyError, TypeError, ValueError):
        return {
            "required": False,
            "available": False,
            "reason": "mobility geometry is incomplete",
        }
    target_distance_m = math.hypot(dx, dy)
    if target_distance_m <= 0.75:
        return {
            "required": False,
            "available": False,
            "reason": "target is too close to justify a recovery translation",
            "trigger": "native_navigation_no_progress",
            "target_distance_m": target_distance_m,
            "native_navigation_first": True,
            "candidates": [],
        }

    clearances = {
        "forward": front,
        "backward": rear,
        "left": left,
        "right": right,
    }
    offsets = {
        "forward": (math.cos(yaw), math.sin(yaw)),
        "backward": (-math.cos(yaw), -math.sin(yaw)),
        "left": (-math.sin(yaw), math.cos(yaw)),
        "right": (math.sin(yaw), -math.cos(yaw)),
    }
    candidates = []
    translation_reserve_m = {
        "forward": 0.50,
        "backward": 0.30,
        "left": 0.35,
        "right": 0.35,
    }
    SELF_OCCLUSION_RADIUS_M = 0.15  # robot body radius; closer = self-occlusion
    for direction in ("forward", "left", "right", "backward"):
        clearance = clearances[direction]
        # Treat self-occlusion (XT16 seeing robot body) as no valid measurement
        if clearance < SELF_OCCLUSION_RADIUS_M:
            clearance = 2.0
        distance_m = min(
            0.50,
            max(0.0, clearance - translation_reserve_m[direction]),
        )
        if distance_m < 0.20:
            continue
        offset_x, offset_y = offsets[direction]
        candidate_distance_to_target = math.hypot(
            dx - offset_x * distance_m,
            dy - offset_y * distance_m,
        )
        goal_progress_m = target_distance_m - candidate_distance_to_target
        # Balanced scoring: clearance + progress + avoid oscillation
        clearance_bonus = min(clearance, 2.0) * 0.35
        progress_bonus = max(0.0, goal_progress_m / max(target_distance_m, 0.1)) * 2.0 * 0.35
        # Penalize returning to the direction we just came from
        novelty = 0.3 if direction not in (_RECOVERY_HISTORY or []) else 0.0
        score = clearance_bonus + progress_bonus + novelty
        candidates.append(
            {
                "direction": direction,
                "clearance_m": clearance,
                "distance_m": distance_m,
                "goal_progress_m": goal_progress_m,
                "score": score,
            }
        )
    candidates.sort(key=lambda item: item["score"], reverse=True)
    selected = candidates[0] if candidates else None
    available = selected is not None

    # Rotation fallback: if all four directions blocked, try turning
    if not available:
        rotation_candidates = []
        for yaw_offset_deg, label, check_dir in [(45, 'right', 'right'), (-45, 'left', 'left'), (90, 'back', 'rear')]:
            yaw_offset = math.radians(yaw_offset_deg)
            check_clearance = clearances.get(check_dir, 0)
            if check_clearance < SELF_OCCLUSION_RADIUS_M:
                check_clearance = 2.0  # self-occlusion, actually open
            if check_clearance >= 0.35:
                rotation_candidates.append({
                    'direction': 'rotate_' + label,
                    'yaw_offset_deg': yaw_offset_deg,
                    'side_clearance_m': check_clearance,
                    'reason': 'translation blocked; rotate ' + label + ' to find exit',
                })
        if rotation_candidates:
            best = rotation_candidates[0]
            selected = {
                'direction': best['direction'],
                'clearance_m': 0,
                'distance_m': 0,
                'goal_progress_m': 0,
                'score': 0.5,
                'rotation': True,
                'yaw_offset_deg': best['yaw_offset_deg'],
            }
            available = True
            reason = (
                'rotation recovery ' + best['direction'] + ' after translation blocked; '
                'side clearance ' + str(round(best['side_clearance_m'], 2)) + 'm'
            )
    # End rotation fallback
    if available and not selected.get("rotation"):
        reason = (
            f"bounded {selected['direction']} translation is available after "
            "native planner stall"
        )
    else:
        reason = "no direction has enough clearance for bounded reposition"
    return {
        "required": True,
        "available": available,
        "reason": reason,
        "trigger": "native_navigation_no_progress",
        "bearing_error_rad": bearing_error,
        "target_distance_m": target_distance_m,
        "native_navigation_first": True,
        "clearance_m": {
            "front": front,
            "left": left,
            "right": right,
            "rear": rear,
        },
        "direction": selected["direction"] if selected else None,
        "distance_m": selected["distance_m"] if selected else 0.0,
        "speed_mps": 0.20,
        "candidates": candidates,
    }


def _extract_json_object(text: str) -> dict[str, Any]:
    decoder = json.JSONDecoder()
    objects: list[dict[str, Any]] = []
    for index, char in enumerate(text):
        if char != "{":
            continue
        try:
            value, _ = decoder.raw_decode(text[index:])
        except json.JSONDecodeError:
            continue
        if isinstance(value, dict):
            objects.append(value)
    if objects:
        return objects[-1]
    raise ValueError("recovery strategy returned no JSON object")


def generate_recovery_strategy_proposal(
    recovery_analysis: dict[str, Any],
    slam_command: dict[str, Any],
    args: argparse.Namespace,
    *,
    backend: LocalCommandBackend | None = None,
) -> dict[str, Any]:
    candidates = [
        {
            "direction": item.get("direction"),
            "max_distance_m": item.get("distance_m"),
            "clearance_m": item.get("clearance_m"),
            "goal_progress_m": item.get("goal_progress_m"),
        }
        for item in recovery_analysis.get("candidates", [])
        if isinstance(item, dict)
    ]

    def deterministic(source: str, error: str = "") -> dict[str, Any]:
        if candidates:
            selected = candidates[0]
            proposal = {
                "action": "reposition",
                "direction": selected["direction"],
                "distance_m": selected["max_distance_m"],
                "confidence": 0.5,
                "reason": "deterministic fallback selected highest ranked candidate",
                "source": source,
            }
        else:
            proposal = {
                "action": "hold",
                "direction": "",
                "distance_m": 0.0,
                "confidence": 1.0,
                "reason": "no bounded reposition candidate is available",
                "source": source,
            }
        if error:
            proposal["fallback_error"] = error
        return proposal

    mode = str(getattr(args, "recovery_strategy_mode", "live"))
    if mode == "deterministic":
        return deterministic("deterministic_configured")

    request = {
        "target_node": slam_command.get("target_node"),
        "trigger": recovery_analysis.get("trigger"),
        "bearing_error_rad": recovery_analysis.get("bearing_error_rad"),
        "current_clearance_m": recovery_analysis.get("clearance_m"),
        "candidates": candidates,
    }
    prompt = (
        "Select one bounded straight recovery translation or hold. "
        "Return JSON only.\n"
        + json.dumps(request, ensure_ascii=False, separators=(",", ":"))
    )
    try:
        llm_backend = backend or LocalCommandBackend(args.local_command)
        raw_answer = llm_backend.generate(
            prompt,
            system_prompt=RECOVERY_STRATEGY_SYSTEM_PROMPT,
            max_tokens=int(getattr(args, "recovery_strategy_max_tokens", 160)),
            timeout_s=int(getattr(args, "recovery_strategy_timeout_s", 20)),
        )
        proposal = _extract_json_object(raw_answer)
        required_keys = {
            "action",
            "direction",
            "distance_m",
            "confidence",
            "reason",
        }
        if set(proposal) != required_keys:
            raise ValueError("recovery strategy keys do not match contract")
        if proposal.get("action") not in {"reposition", "hold"}:
            raise ValueError("recovery strategy action is invalid")
        proposal["source"] = "local_llm"
        proposal["raw_answer"] = raw_answer
        return proposal
    except Exception as exc:
        return deterministic("deterministic_fallback", str(exc))


def run_post_navigation_recovery(
    preflight_state: dict[str, Any],
    slam_command: dict[str, Any],
    args: argparse.Namespace,
    *,
    target_name: str,
) -> dict[str, Any]:
    recovery_analysis = build_recovery_translation_analysis(
        preflight_state,
        slam_command,
    )
    strategy = generate_recovery_strategy_proposal(
        recovery_analysis,
        slam_command,
        args,
    )
    recovery_decision = build_recovery_decision(
        recovery_analysis,
        strategy,
    )
    result: dict[str, Any] = {
        "ok": False,
        "recovery_analysis": recovery_analysis,
        "recovery_decision": recovery_decision,
    }
    if (
        recovery_decision.get("accepted") is not True
        or recovery_decision.get("decision") != "execute_reposition"
    ):
        result["reason"] = str(
            recovery_decision.get("reason")
            or recovery_analysis.get("reason")
            or "post-navigation recovery was not authorized"
        )
        return result

    reposition_command = dict(recovery_decision["authorized_command"])
    reposition_args = copy.copy(args)
    reposition_args.arrival_distance_m = min(
        0.10,
        max(
            0.05,
            float(reposition_command["distance_m"]) * 0.20,
        ),
    )
    reposition_args.arrival_monitor_s = max(
        25.0,
        float(reposition_command["distance_m"])
        / float(reposition_command["speed_mps"])
        + 10.0,
    )
    send_result, arrival = run_supervised_navigation_session(
        reposition_command,
        reposition_args,
        target_name=(
            "supervised "
            f"{reposition_command['direction']} reposition"
        ),
    )
    result["send_result"] = send_result
    result["arrival"] = arrival
    reposition_ok = bool(
        send_result.get("accepted")
        and isinstance(arrival, dict)
        and arrival.get("arrived")
        and arrival.get("paused")
    )
    if not reposition_ok:
        result["reason"] = str(
            (arrival or {}).get("reason")
            or gateway_rejection_reason(send_result)
        )
        return result

    result["ok"] = True
    result["reason"] = ""
    return result


def run_capture_keyframe(task: dict[str, Any], args: argparse.Namespace) -> dict[str, Any]:
    target_node = str(task.get("target_node") or "")
    target_name = str(task.get("target_name") or target_node)
    if not args.capture_command:
        return {
            "captured": False,
            "target_node": target_node,
            "reason": "capture command not configured; recorded semantic keyframe event only",
        }
    env = os.environ.copy()
    env["GO2W_TARGET_NODE"] = target_node
    env["GO2W_TARGET_NAME"] = target_name
    env.setdefault("GO2W_RUN_ID", time.strftime("run_%Y%m%d_%H%M%S"))
    completed = subprocess.run(
        ["bash", "-lc", args.capture_command],
        text=True,
        encoding="utf-8",
        errors="replace",
        capture_output=True,
        timeout=args.timeout_s,
        env=env,
    )
    result = {
        "captured": completed.returncode == 0,
        "target_node": target_node,
        "target_name": target_name,
        "returncode": completed.returncode,
        "stdout": completed.stdout,
        "stderr": completed.stderr,
    }
    for line in reversed(completed.stdout.splitlines()):
        try:
            payload = json.loads(line)
        except json.JSONDecodeError:
            continue
        if isinstance(payload, dict):
            result["payload"] = payload
            if "captured" in payload:
                result["captured"] = bool(payload.get("captured"))
            for key in ("image_path", "sidecar_path", "run_id", "timestamp_ms", "source", "reason", "image_bytes"):
                if key in payload:
                    result[key] = payload[key]
            break
    if completed.returncode != 0 and not result.get("reason"):
        result["reason"] = "capture command failed"
    return result


def execute_task_queue(
    task_queue: dict[str, Any],
    *,
    registry: MapRegistry,
    args: argparse.Namespace,
    nav_speed: float | None,
    event_sink: Any = None,
) -> dict[str, Any]:
    validate_task_queue(task_queue)
    events: list[dict[str, Any]] = []
    blocked_reason = ""
    failed_step = None
    profile = registry.get_map(args.map_id)
    feedback_policy = {
        "slam_poll_interval_s": getattr(args, "slam_poll_interval_s", args.arrival_monitor_interval_s),
        "ui_refresh_interval_s": getattr(args, "ui_refresh_interval_s", args.arrival_monitor_interval_s),
        "operator_feedback_interval_s": getattr(args, "operator_feedback_interval_s", 5.0),
        "llm_feedback_interval_s": getattr(args, "llm_feedback_interval_s", 8.0),
        "llm_feedback_mode": getattr(args, "llm_feedback_mode", "template"),
        "llm_feedback_live_progress": getattr(args, "llm_feedback_live_progress", False),
        "max_arrival_samples": getattr(args, "max_arrival_samples", 120),
        "max_feedback_events": getattr(args, "max_feedback_events", 120),
        "max_llm_feedback_events": getattr(args, "max_llm_feedback_events", 40),
        "max_consecutive_gateway_errors": getattr(args, "gateway_error_limit", 3),
        "runtime_safety_check": True,
    }

    def append_queue_event(event: dict[str, Any]) -> None:
        events.append(event)
        if event_sink is not None:
            event_sink(
                {
                    "queue_id": task_queue.get("queue_id"),
                    "status": "running",
                    "event": event,
                }
            )

    for task in task_queue.get("steps", []):
        if not isinstance(task, dict):
            continue
        action = str(task.get("action") or "")
        task_id = task_step_id(task)
        target_name = str(task.get("target_name") or task.get("target_node") or "")
        if action == "report":
            append_queue_event(
                {
                    "task_id": task_id,
                    "action": action,
                    "status": "ok",
                    "message": task.get("message"),
                    "operator_feedback": [
                        operator_feedback_message("report", str(task.get("message") or ""), severity="info")
                    ],
                }
            )
            continue
        if action == "capture_keyframe":
            result = run_capture_keyframe(task, args) if args.execute else {"captured": False, "reason": "dry run; capture not executed", "target_node": task.get("target_node")}
            capture_failed = bool(args.execute and args.capture_command and not result.get("captured"))
            append_queue_event(
                {
                    "task_id": task_id,
                    "action": action,
                    "status": "failed" if capture_failed else "ok",
                    "result": result,
                    "operator_feedback": [
                        operator_feedback_message(
                            "capture",
                            f"记录{target_name}的关键帧事件。",
                            target_node=str(task.get("target_node") or ""),
                            target_name=target_name,
                        )
                    ],
                }
            )
            if capture_failed:
                blocked_reason = str(result.get("reason") or "capture_keyframe failed")
                failed_step = task_id
                break
            continue
        if action != "navigate":
            append_queue_event({"task_id": task_id, "action": action, "status": "skipped", "reason": "unsupported task action"})
            continue

        target_node = str(task.get("target_node") or "")
        if args.execute:
            try:
                topology_allowed, topology_reason = topology_target_allows_navigation(profile, target_node)
            except Exception as exc:
                topology_allowed = False
                topology_reason = f"failed to validate topology target {target_node}: {exc}"
            if not topology_allowed:
                blocked_reason = topology_reason
                failed_step = task_id
                blocked_feedback = [
                    operator_feedback_message(
                        "blocked",
                        f"Topology gate blocked real navigation: {blocked_reason}",
                        target_node=target_node,
                        target_name=target_name,
                        severity="error",
                    )
                ]
                blocked_llm_requests: list[dict[str, Any]] = []
                blocked_llm_results: list[dict[str, Any]] = []
                append_llm_feedback(
                    phase="blocked",
                    target_node=target_node,
                    target_name=target_name,
                    distance_m=None,
                    reason=blocked_reason,
                    args=args,
                    requests=blocked_llm_requests,
                    results=blocked_llm_results,
                    operator_feedback=blocked_feedback,
                )
                append_queue_event(
                    {
                        "task_id": task_id,
                        "action": action,
                        "status": "blocked",
                        "target_node": target_node,
                        "blocked_reason": blocked_reason,
                        "operator_feedback": blocked_feedback,
                        "llm_feedback_requests": blocked_llm_requests,
                        "llm_feedback_results": blocked_llm_results,
                    }
                )
                break
        try:
            slam_command = profile.navigate_to_node_command(target_node, speed=nav_speed, mode=args.nav_mode)
        except Exception as exc:
            blocked_reason = f"failed to build navigation command for {target_node}: {exc}"
            failed_step = task_id
            append_queue_event({"task_id": task_id, "action": action, "status": "failed", "target_node": target_node, "blocked_reason": blocked_reason})
            break

        preflight_state = None
        gateway_allowed = True
        gateway_reason = "gateway check skipped"
        if not args.skip_gateway_check:
            try:
                preflight_state = run_gateway_command(
                    {"action": "get_world_state"},
                    client_path=args.gateway_client,
                    network_interface=args.network_interface,
                    timeout_s=args.timeout_s,
                    startup_wait_s=args.gateway_startup_wait_s,
                )
            except Exception as exc:
                preflight_state = {
                    "accepted": False,
                    "reason": "gateway_preflight_error",
                    "error": str(exc),
                }
            gateway_allowed, gateway_reason = gateway_allows_navigation(preflight_state)
        if args.execute and not gateway_allowed:
            blocked_reason = gateway_reason
            failed_step = task_id
            blocked_feedback = [
                operator_feedback_message(
                    "blocked",
                    f"安全检查未通过：{blocked_reason}",
                    target_node=target_node,
                    target_name=target_name,
                    severity="error",
                )
            ]
            blocked_llm_requests: list[dict[str, Any]] = []
            blocked_llm_results: list[dict[str, Any]] = []
            append_llm_feedback(
                phase="blocked",
                target_node=target_node,
                target_name=target_name,
                distance_m=None,
                reason=blocked_reason,
                args=args,
                requests=blocked_llm_requests,
                results=blocked_llm_results,
                operator_feedback=blocked_feedback,
            )
            append_queue_event(
                {
                    "task_id": task_id,
                    "action": action,
                    "status": "blocked",
                    "target_node": target_node,
                    "blocked_reason": blocked_reason,
                    "preflight": preflight_state,
                    "operator_feedback": blocked_feedback,
                    "llm_feedback_requests": blocked_llm_requests,
                    "llm_feedback_results": blocked_llm_results,
                }
            )
            break

        if not args.execute:
            queued_feedback = [
                operator_feedback_message(
                    "queued",
                    f"干跑：将前往{target_name}，不会下发运动。",
                    target_node=target_node,
                    target_name=target_name,
                )
            ]
            queued_llm_requests: list[dict[str, Any]] = []
            queued_llm_results: list[dict[str, Any]] = []
            append_llm_feedback(
                phase="queued",
                target_node=target_node,
                target_name=target_name,
                distance_m=None,
                reason="dry run",
                args=args,
                requests=queued_llm_requests,
                results=queued_llm_results,
                operator_feedback=queued_feedback,
            )
            append_queue_event(
                {
                    "task_id": task_id,
                    "action": action,
                    "status": "dry_run",
                    "target_node": target_node,
                    "slam_command": slam_command,
                    "preflight": preflight_state,
                    "operator_feedback": queued_feedback,
                    "llm_feedback_requests": queued_llm_requests,
                    "llm_feedback_results": queued_llm_results,
                }
            )
            continue

        departing_feedback = operator_feedback_message(
            "departing",
            f"正在前往{target_name}。",
            target_node=target_node,
            target_name=target_name,
        )
        guarded_slam_command = {
            **slam_command,
            "operator_ack": True,
        }
        result, session_arrival = run_supervised_navigation_session(
            guarded_slam_command,
            args,
            target_name=target_name,
        )
        accepted = bool(result.get("accepted", False))
        if accepted:
            arrival = session_arrival or {
                "arrived": False,
                "paused": True,
                "reason": "persistent navigation session returned no arrival result",
            }
        else:
            rejected_reason = gateway_rejection_reason(result)
            rejected_feedback = [
                operator_feedback_message(
                    "blocked",
                    "gateway 未接受导航命令，已停止等待。",
                    target_node=target_node,
                    target_name=target_name,
                    severity="error",
                )
            ]
            rejected_llm_requests: list[dict[str, Any]] = []
            rejected_llm_results: list[dict[str, Any]] = []
            append_llm_feedback(
                phase="blocked",
                target_node=target_node,
                target_name=target_name,
                distance_m=None,
                reason=rejected_reason,
                args=args,
                requests=rejected_llm_requests,
                results=rejected_llm_results,
                operator_feedback=rejected_feedback,
            )
            arrival = {
                "arrived": False,
                "reason": rejected_reason,
                "operator_feedback": rejected_feedback,
                "llm_feedback_requests": rejected_llm_requests,
                "llm_feedback_results": rejected_llm_results,
            }
        if (
            accepted
            and not arrival.get("arrived")
            and arrival.get("reason") == "native_navigation_no_progress"
        ):
            append_queue_event(
                {
                    "task_id": f"{task_id}_native_attempt_1",
                    "action": action,
                    "status": "recoverable_failure",
                    "target_node": target_node,
                    "slam_command": guarded_slam_command,
                    "send_result": result,
                    "arrival": arrival,
                }
            )
            try:
                recovery_state = run_gateway_command(
                    {"action": "get_world_state"},
                    client_path=args.gateway_client,
                    network_interface=args.network_interface,
                    timeout_s=args.timeout_s,
                    startup_wait_s=args.gateway_startup_wait_s,
                )
                recovery = run_post_navigation_recovery(
                    recovery_state,
                    slam_command,
                    args,
                    target_name=target_name,
                )
            except Exception as exc:
                recovery = {
                    "ok": False,
                    "reason": f"post-navigation recovery failed: {exc}",
                }
            append_queue_event(
                {
                    "task_id": f"{task_id}_recovery_1",
                    "action": "supervised_reposition",
                    "status": "ok" if recovery.get("ok") else "failed",
                    "target_node": target_node,
                    **recovery,
                }
            )
            if recovery.get("ok"):
                result, session_arrival = run_supervised_navigation_session(
                    guarded_slam_command,
                    args,
                    target_name=target_name,
                )
                accepted = bool(result.get("accepted", False))
                if accepted:
                    arrival = session_arrival or {
                        "arrived": False,
                        "paused": True,
                        "reason": (
                            "persistent navigation session returned no "
                            "arrival result after recovery"
                        ),
                    }
                else:
                    arrival = {
                        "arrived": False,
                        "paused": True,
                        "reason": gateway_rejection_reason(result),
                    }
            else:
                arrival = {
                    "arrived": False,
                    "paused": True,
                    "reason": str(
                        recovery.get("reason")
                        or "post-navigation recovery failed"
                    ),
                }
        status = "ok" if accepted and arrival.get("arrived") and arrival.get("paused") else "failed"
        append_queue_event(
            {
                "task_id": task_id,
                "action": action,
                "status": status,
                "target_node": target_node,
                "slam_command": guarded_slam_command,
                "send_result": result,
                "arrival": arrival,
                "operator_feedback": [departing_feedback] + list(arrival.get("operator_feedback") or []),
                "llm_feedback_requests": list(arrival.get("llm_feedback_requests") or []),
                "llm_feedback_results": list(arrival.get("llm_feedback_results") or []),
            }
        )
        if status != "ok":
            blocked_reason = str(arrival.get("reason") or "navigation failed")
            failed_step = task_id
            break

    return {
        "queue_id": task_queue.get("queue_id"),
        "executed": bool(args.execute),
        "dry_run": not bool(args.execute),
        "completed": failed_step is None and bool(args.execute),
        "failed_step": failed_step,
        "blocked_reason": blocked_reason,
        "feedback_policy": feedback_policy,
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
    parser.add_argument("--map-id", default=DEFAULT_MAP_ID)
    parser.add_argument("--map-path", default=DEFAULT_MAP_PATH)
    parser.add_argument(
        "--perception-context-path",
        default=str(REPO_ROOT / "artifacts" / "perception_context_v1.json"),
    )
    parser.add_argument("--prompt-mode", choices=["hybrid", "intent", "light", "full"], default="hybrid")
    parser.add_argument("--local-command", default=DEFAULT_LOCAL_COMMAND)
    parser.add_argument("--system", default=DEFAULT_SYSTEM_PROMPT)
    parser.add_argument("--max-tokens", type=int, default=128)
    parser.add_argument("--timeout-s", type=int, default=30)
    parser.add_argument("--robot-host", default=os.environ.get("GO2W_SSH_HOST", "192.168.3.17"))
    parser.add_argument("--robot-username", default="unitree")
    parser.add_argument("--robot-password", default=os.environ.get("GO2W_SSH_PASSWORD", ""))
    parser.add_argument("--no-live-snapshot", action="store_true")
    parser.add_argument("--mock-x", type=float, default=1.154)
    parser.add_argument("--mock-y", type=float, default=-0.147)
    parser.add_argument("--mock-yaw", type=float, default=-0.03)
    parser.add_argument(
        "--gateway-client",
        default=str(REPO_ROOT / "robot" / "slam_gateway_refactor" / "build" / "slam_llm_command_client"),
    )
    parser.add_argument("--network-interface", default=os.environ.get("GO2W_NETWORK_INTERFACE", "eth0"))
    parser.add_argument("--gateway-startup-wait-s", type=float, default=4.0)
    parser.add_argument("--skip-gateway-check", action="store_true")
    parser.add_argument("--nav-speed-mps", type=float, default=0.0, help="Override navigation speed. Unitree GO2 mode 0 is normalized to its 0.2 m/s minimum; 0 keeps registry speed.")
    parser.add_argument("--nav-mode", type=int, default=None, help="Override Unitree navigation mode for the generated slam command.")
    parser.add_argument("--arrival-distance-m", type=float, default=0.25)
    parser.add_argument("--arrival-confirm-samples", type=int, default=2)
    parser.add_argument("--arrival-monitor-s", type=float, default=25.0)
    parser.add_argument("--navigation-monitor-travel-factor", type=float, default=1.8)
    parser.add_argument("--navigation-monitor-startup-margin-s", type=float, default=15.0)
    parser.add_argument("--navigation-stall-s", type=float, default=20.0)
    parser.add_argument("--navigation-progress-distance-m", type=float, default=0.08)
    parser.add_argument("--navigation-progress-yaw-rad", type=float, default=0.12)
    parser.add_argument("--arrival-monitor-interval-s", type=float, default=1.0)
    parser.add_argument("--slam-poll-interval-s", type=float, default=1.0)
    parser.add_argument("--ui-refresh-interval-s", type=float, default=1.0)
    parser.add_argument("--operator-feedback-interval-s", type=float, default=5.0)
    parser.add_argument("--llm-feedback-interval-s", type=float, default=8.0)
    parser.add_argument("--llm-feedback-mode", choices=["off", "template", "live"], default="template")
    parser.add_argument("--llm-feedback-live-progress", action="store_true", help="Allow live LLM calls for in-flight progress feedback. Default keeps the hot loop template-only.")
    parser.add_argument("--llm-feedback-max-tokens", type=int, default=48)
    parser.add_argument("--llm-feedback-timeout-s", type=int, default=6)
    parser.add_argument("--llm-feedback-system", default=FEEDBACK_SYSTEM_PROMPT)
    parser.add_argument("--max-arrival-samples", type=int, default=120)
    parser.add_argument("--max-feedback-events", type=int, default=120)
    parser.add_argument("--max-llm-feedback-events", type=int, default=40)
    parser.add_argument("--gateway-error-limit", type=int, default=3)
    parser.add_argument(
        "--recovery-strategy-mode",
        choices=["live", "deterministic"],
        default="live",
    )
    parser.add_argument("--recovery-strategy-max-tokens", type=int, default=160)
    parser.add_argument("--recovery-strategy-timeout-s", type=int, default=20)
    parser.add_argument("--capture-command", default=os.environ.get("GO2W_CAPTURE_COMMAND", ""), help="Optional bash command for capture_keyframe; GO2W_TARGET_NODE is set.")
    parser.add_argument(
        "--communication-journal",
        default=os.environ.get(
            "GO2W_COMMUNICATION_JOURNAL",
            str(DEFAULT_COMMUNICATION_JOURNAL),
        ),
    )
    parser.add_argument(
        "--communication-source-id",
        default=os.environ.get("GO2W_COMMUNICATION_SOURCE_ID", "go2w_robot"),
    )
    parser.add_argument(
        "--link-state",
        choices=["normal", "weak", "disconnected", "recovered"],
        default=os.environ.get("GO2W_LINK_STATE", "normal"),
    )
    parser.add_argument("--ack-sequence", type=int, default=None)
    parser.add_argument("--replay-limit", type=int, default=100)
    parser.add_argument("--execute", action="store_true", help="Actually send the navigation command after safety gates pass.")
    parser.add_argument("--pretty", action="store_true")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if args.execute and args.skip_gateway_check:
        output = {
            "command": args.command,
            "dry_run": False,
            "planner": None,
            "gateway": {
                "checked": False,
                "allowed": False,
                "reason": "--skip-gateway-check is only allowed for dry-runs; remove it before --execute",
            },
            "execution": {
                "executed": False,
                "blocked_reason": "--skip-gateway-check is only allowed for dry-runs; remove it before --execute",
                "result": None,
            },
        }
        if args.pretty:
            print(json.dumps(output, ensure_ascii=False, indent=2))
        else:
            print(json.dumps(output, ensure_ascii=False, separators=(",", ":")))
        return 2
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
    snapshot["capture_command_configured"] = bool(args.capture_command)
    perception_context = load_perception_context_file(
        Path(args.perception_context_path),
        current_time_ms=int(time.time() * 1000),
    )
    planner_context = build_planner_context(
        snapshot,
        registry,
        user_command=args.command,
        map_id=args.map_id,
        perception_context=perception_context,
    )

    result = run_local_llm_planner(
        planner_context,
        LocalCommandBackend(args.local_command),
        system_prompt=args.system,
        max_tokens=args.max_tokens,
        timeout_s=args.timeout_s,
        prompt_mode=args.prompt_mode,
    )
    nav_speed = normalize_unitree_navigation_speed(
        args.nav_speed_mps if args.nav_speed_mps > 0 else None,
        args.nav_mode,
    )
    slam_command = plan_to_slam_command(result.plan, registry, speed=nav_speed, mode=args.nav_mode)
    task_queue = plan_to_task_queue(result.plan, planner_context, existing_queue=result.task_queue)
    queue_execution = None
    queue_id = str(task_queue.get("queue_id") or "")
    communication = CommunicationPolicyExecutor(
        AppendOnlyJournal(
            args.communication_journal,
            source_id=args.communication_source_id,
        ),
        communication_policy=task_queue["communication_policy"],
        link_state=args.link_state,
    )
    communication_recovery = communication.status()
    communication_ack = None
    if args.ack_sequence is not None:
        communication_ack = communication.acknowledge(args.ack_sequence)
    task_journal_event = communication.record_event(
        "task_state",
        {
            "status": str(task_queue.get("status") or "planned"),
            "task_queue": task_queue,
        },
        queue_id=queue_id,
    )

    gateway_state = None
    gateway_allowed = False
    gateway_reason = "gateway check skipped"
    if not args.skip_gateway_check:
        try:
            gateway_state = run_gateway_command(
                {"action": "get_world_state"},
                client_path=args.gateway_client,
                network_interface=args.network_interface,
                timeout_s=args.timeout_s,
                startup_wait_s=args.gateway_startup_wait_s,
            )
        except Exception as exc:
            gateway_state = {
                "accepted": False,
                "reason": "gateway_preflight_error",
                "error": str(exc),
            }
        gateway_allowed, gateway_reason = gateway_allows_navigation(gateway_state)

    topology_allowed = True
    topology_reason = "topology target allows navigation"
    topology_failures: list[str] = []
    for target_node in task_queue.get("targets", []):
        try:
            target_allowed, target_reason = topology_target_allows_navigation(
                registry.get_map(args.map_id),
                str(target_node),
            )
        except Exception as exc:
            target_allowed = False
            target_reason = f"failed to validate topology target {target_node}: {exc}"
        if not target_allowed:
            topology_failures.append(target_reason)
    if topology_failures:
        topology_allowed = False
        topology_reason = "; ".join(topology_failures)

    mission_decision = build_mission_decision(
        task_queue,
        execute_requested=args.execute,
        registry_allowed=registry_allowed,
        registry_reason=registry_reason,
        topology_allowed=topology_allowed,
        topology_reason=topology_reason,
        gateway_checked=not args.skip_gateway_check,
        gateway_allowed=gateway_allowed,
        gateway_reason=gateway_reason,
        gateway_state=gateway_state,
    )
    decision_journal_event = communication.record_event(
        "mission_decision",
        mission_decision,
        queue_id=queue_id,
    )

    executed = False
    execution_result = None
    blocked_reason = ""
    execution_claim = None
    communication_errors: list[str] = []

    def journal_queue_event(event: dict[str, Any]) -> None:
        try:
            communication.record_event(
                "navigation_feedback",
                event,
                queue_id=queue_id,
            )
        except Exception as exc:
            communication_errors.append(str(exc))

    if mission_decision["decision"] in {"execute_queue", "dry_run_queue"}:
        if args.execute:
            execution_claim = communication.reserve_execution(
                queue_id=queue_id,
                decision_id=str(mission_decision.get("decision_id") or ""),
            )
        if args.execute and not execution_claim.get("claimed"):
            blocked_reason = str(
                execution_claim.get("reason")
                or "queue execution is already claimed; automatic replay is blocked"
            )
        else:
            queue_execution = execute_task_queue(
                task_queue,
                registry=registry,
                args=args,
                nav_speed=nav_speed,
                event_sink=journal_queue_event,
            )
            executed = bool(args.execute and queue_execution.get("completed"))
            blocked_reason = str(queue_execution.get("blocked_reason") or ("dry run; pass --execute to send queued commands" if not args.execute else ""))
            execution_result = queue_execution
    else:
        blocked_reason = str(mission_decision.get("reason") or "MissionDecisionEngine blocked execution")

    if queue_execution and queue_execution.get("completed"):
        execution_status = "completed"
    elif not args.execute and mission_decision["decision"] == "dry_run_queue":
        execution_status = "dry_run"
    elif blocked_reason:
        execution_status = "blocked"
    else:
        execution_status = "not_started"
    execution_journal_event = communication.record_event(
        "execution_state",
        {
            "status": execution_status,
            "dry_run": not args.execute,
            "executed": executed,
            "blocked_reason": blocked_reason,
            "queue_execution": queue_execution,
        },
        queue_id=queue_id,
    )

    if queue_execution and queue_execution.get("completed"):
        task_phase = "completed"
    elif mission_decision["decision"] == "await_confirmation":
        task_phase = "human_confirm_required"
    elif blocked_reason:
        task_phase = "blocked" if args.execute else "planning"
    elif executed:
        task_phase = "executing_navigation"
    else:
        task_phase = "planning"
    world_state_v1 = build_world_state_v1(
        gateway_state or snapshot,
        planner_context=planner_context,
        task_phase=task_phase,
        last_execution_result=blocked_reason or ("executed" if executed else ""),
        network_level=args.link_state,
        motion_allowed=bool(mission_decision.get("motion_allowed")),
        perception_context=perception_context,
        timestamp_ms=int(time.time() * 1000),
    )
    operator_display = build_operator_display_state(
        world_state_v1,
        task_queue=task_queue,
        mission_decision=mission_decision,
        queue_execution=queue_execution,
        user_command=args.command,
    )
    runtime_log_record = build_runtime_log_record(
        world_state=world_state_v1,
        operator_display=operator_display,
        task_queue=task_queue,
        mission_decision=mission_decision,
        queue_execution=queue_execution,
        user_command=args.command,
        llm_result={"elapsed_s": result.elapsed_s, "plan": result.plan, "user_reply": result.user_reply},
    )

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
        "world_state_v1": world_state_v1,
        "operator_display": operator_display,
        "runtime_log_record": runtime_log_record,
        "mission_decision": mission_decision,
        "semantic_trace": build_semantic_trace(
            command=args.command,
            planner_context=planner_context,
            plan=result.plan,
            slam_command=slam_command,
            gateway_state=gateway_state,
            registry_allowed=registry_allowed,
            registry_reason=registry_reason,
            task_queue=task_queue,
            mission_decision=mission_decision,
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
        "topology_gate": {
            "allowed": topology_allowed,
            "reason": topology_reason,
        },
        "execution": {
            "executed": executed,
            "blocked_reason": blocked_reason,
            "result": execution_result,
        },
        "communication": {
            "recovered_before_run": communication_recovery,
            "ack": communication_ack,
            "task_event": {
                "event_id": task_journal_event["event_id"],
                "sequence": task_journal_event["sequence"],
            },
            "decision_event": {
                "event_id": decision_journal_event["event_id"],
                "sequence": decision_journal_event["sequence"],
            },
            "execution_claim": execution_claim,
            "execution_event": {
                "event_id": execution_journal_event["event_id"],
                "sequence": execution_journal_event["sequence"],
            },
            "status": communication.status(),
            "replay_batch": [
                {
                    "event_id": event["event_id"],
                    "sequence": event["sequence"],
                    "event_type": event["event_type"],
                    "execution_directive": event["execution_directive"],
                }
                for event in communication.replay_batch(limit=args.replay_limit)
            ],
            "errors": communication_errors,
        },
    }
    if args.pretty:
        print(json.dumps(output, ensure_ascii=False, indent=2))
    else:
        print(json.dumps(output, ensure_ascii=False, separators=(",", ":")))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
