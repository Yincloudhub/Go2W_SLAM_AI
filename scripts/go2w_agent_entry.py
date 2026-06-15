from __future__ import annotations

import argparse
import base64
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

from edge_autonomy.chassis_controller import (  # noqa: E402
    ChassisController,
    GatewayConfig,
    gateway_allows_navigation,
    pose_distance,
    run_gateway_command,
    yaw_error,
)
from edge_autonomy.execution_report import summarize_agent_output, write_execution_log  # noqa: E402


DEFAULT_GATEWAY_CLIENT = str(REPO_ROOT / "robot" / "slam_gateway_refactor" / "build" / "slam_llm_command_client")
DEFAULT_START_SLAM = str(REPO_ROOT / "scripts" / "start_go2w_slam_stack.sh")
DEFAULT_MAP_PATH = "/home/unitree/test.pcd"
DEFAULT_REGISTRY = REPO_ROOT / "configs" / "maps" / "go2w_real_site_map_registry.json"
DEFAULT_MAP_ID = "go2w_real_site"
DEFAULT_LOG_DIR = REPO_ROOT / "artifacts" / "robot_runs"


def decode_command(args: argparse.Namespace) -> str:
    if args.go_b64:
        return base64.b64decode(args.go_b64).decode("utf-8")
    if args.go:
        return args.go
    if args.command_b64:
        return base64.b64decode(args.command_b64).decode("utf-8")
    return args.command or ""


def decode_say(args: argparse.Namespace) -> str:
    if args.say_b64:
        return base64.b64decode(args.say_b64).decode("utf-8")
    return args.say or ""


def print_json(value: Any, *, pretty: bool) -> None:
    if pretty:
        print(json.dumps(value, ensure_ascii=False, indent=2))
    else:
        print(json.dumps(value, ensure_ascii=False, separators=(",", ":")))


def fmt_m(value: Any) -> str:
    try:
        return f"{float(value):.2f}m"
    except (TypeError, ValueError):
        return "未知"


def fmt_num(value: Any, digits: int = 2) -> str:
    try:
        return f"{float(value):.{digits}f}"
    except (TypeError, ValueError):
        return "未知"


def run_start_slam(script: str) -> dict[str, Any]:
    completed = subprocess.run(["bash", script], text=True, encoding="utf-8", errors="replace", capture_output=True, timeout=30)
    return {
        "returncode": completed.returncode,
        "stdout": completed.stdout,
        "stderr": completed.stderr,
    }


def run_bash(command: str, *, timeout_s: int = 30) -> dict[str, Any]:
    completed = subprocess.run(
        ["bash", "-lc", command],
        text=True,
        encoding="utf-8",
        errors="replace",
        capture_output=True,
        timeout=timeout_s,
    )
    return {
        "returncode": completed.returncode,
        "stdout": completed.stdout,
        "stderr": completed.stderr,
    }


def ros2_topic_sample(topic: str, *, timeout_s: int, lines: int = 80) -> dict[str, Any]:
    quoted_topic = topic.replace("'", "'\"'\"'")
    command = (
        "source /opt/ros/foxy/setup.bash >/dev/null 2>&1 || true; "
        f"timeout {int(timeout_s)} ros2 topic echo '{quoted_topic}' --qos-reliability reliable --full-length --no-arr 2>/tmp/go2w_topic_echo.err "
        f"| sed -n '1,{int(lines)}p'; "
        "cat /tmp/go2w_topic_echo.err"
    )
    result = run_bash(command, timeout_s=timeout_s + 5)
    result["alive"] = bool(result["stdout"].strip())
    return result


def run_ensure_slam(args: argparse.Namespace) -> dict[str, Any]:
    start = run_start_slam(args.start_slam_script)
    deadline = time.time() + args.ensure_slam_wait_s
    samples: list[dict[str, Any]] = []
    pointcloud_alive = False
    slam_info_alive = False
    while time.time() < deadline:
        pointcloud = ros2_topic_sample("/unitree/slam_lidar/points", timeout_s=args.ensure_slam_sample_timeout_s, lines=24)
        slam_info = ros2_topic_sample("/slam_info", timeout_s=args.ensure_slam_sample_timeout_s, lines=40)
        pointcloud_alive = bool(pointcloud.get("alive"))
        slam_info_alive = bool(slam_info.get("alive"))
        samples.append(
            {
                "pointcloud_alive": pointcloud_alive,
                "slam_info_alive": slam_info_alive,
                "pointcloud_excerpt": str(pointcloud.get("stdout", ""))[:600],
                "slam_info_excerpt": str(slam_info.get("stdout", ""))[:900],
            }
        )
        if pointcloud_alive and slam_info_alive:
            break
        time.sleep(args.ensure_slam_interval_s)

    try:
        preflight = make_chassis(args).preflight()
    except Exception as exc:  # pragma: no cover - field robustness
        preflight = {"allowed": False, "reason": f"preflight failed: {exc}", "result": None}
    return {
        "started": start.get("returncode") == 0,
        "start_result": start,
        "pointcloud_alive": pointcloud_alive,
        "slam_info_alive": slam_info_alive,
        "preflight_allowed": preflight.get("allowed"),
        "preflight_reason": preflight.get("reason"),
        "preflight": preflight,
        "samples": samples,
    }


def make_chassis(args: argparse.Namespace, *, startup_wait_s: float | None = None) -> ChassisController:
    wait_s = args.gateway_startup_wait_s if startup_wait_s is None else startup_wait_s
    return ChassisController(
        registry_path=args.registry,
        map_id=args.map_id,
        gateway=GatewayConfig(
            client_path=args.gateway_client,
            network_interface=args.network_interface,
            timeout_s=args.timeout_s,
            startup_wait_s=wait_s,
        ),
    )


def get_world_state(args: argparse.Namespace) -> dict[str, Any]:
    return make_chassis(args).world_state()


def load_registry_map(args: argparse.Namespace) -> dict[str, Any] | None:
    return make_chassis(args, startup_wait_s=0.0).registry_map()


def registry_map_path(args: argparse.Namespace) -> str:
    profile = load_registry_map(args)
    if isinstance(profile, dict) and profile.get("pcd_path"):
        return str(profile["pcd_path"])
    return args.map_path


def registry_node(args: argparse.Namespace, node_id: str) -> dict[str, Any] | None:
    return make_chassis(args, startup_wait_s=0.0).node(node_id)


def relocate_to_anchor(args: argparse.Namespace, anchor_id: str) -> dict[str, Any]:
    return make_chassis(args).relocate_to_anchor(anchor_id, map_path_fallback=args.map_path)


def pause_navigation(args: argparse.Namespace) -> dict[str, Any]:
    return make_chassis(args, startup_wait_s=0.0).pause()


def calibrate_node_from_current_pose(args: argparse.Namespace, node_id: str) -> dict[str, Any]:
    state = get_world_state(args)
    pose = state.get("world_state", {}).get("current_pose", {}).get("pose", {})
    if not isinstance(pose, dict):
        return {"updated": False, "reason": "missing current pose", "state": state}

    registry_path = Path(args.registry)
    registry = json.loads(registry_path.read_text(encoding="utf-8"))
    target_node: dict[str, Any] | None = None
    for item in registry.get("maps", []):
        if item.get("map_id") != args.map_id:
            continue
        for node in item.get("topology_nodes", []):
            if node.get("node_id") == node_id:
                target_node = node
                break
        break
    if target_node is None:
        return {"updated": False, "reason": f"node_id {node_id!r} not found", "state": state}

    calibrated_pose = {
        "x": float(pose["x"]),
        "y": float(pose["y"]),
        "z": float(pose.get("z", 0.0)),
        "q_x": float(pose.get("q_x", 0.0)),
        "q_y": float(pose.get("q_y", 0.0)),
        "q_z": float(pose.get("q_z", 0.0)),
        "q_w": float(pose.get("q_w", 1.0)),
        "speed": float(target_node.get("pose", {}).get("speed", 0.3)),
        "mode": int(target_node.get("pose", {}).get("mode", 0)),
    }
    old_pose = target_node.get("pose")
    target_node["pose"] = calibrated_pose
    tags = target_node.get("tags", [])
    if isinstance(tags, list):
        target_node["tags"] = [tag for tag in tags if tag != "needs_calibration"]
        if "live_calibrated" not in target_node["tags"]:
            target_node["tags"].append("live_calibrated")
    target_node["calibration"] = {
        "confirmed_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "source": "gateway_world_state.current_pose",
        "method": "cli_calibrate_node_current_pose",
        "previous_pose": old_pose,
    }
    target_node["description"] = f"{target_node.get('description', '')} calibrated from current live pose.".strip()
    registry_path.write_text(json.dumps(registry, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return {
        "updated": True,
        "node_id": node_id,
        "old_pose": old_pose,
        "new_pose": calibrated_pose,
        "registry": str(registry_path),
    }


def pose_distance_to_target(world_state_result: dict[str, Any], target_pose: dict[str, Any]) -> float | None:
    world = world_state_result.get("world_state", {})
    pose = world.get("current_pose", {}).get("pose", {}) if isinstance(world, dict) else {}
    if not isinstance(pose, dict):
        return None
    return pose_distance(pose, target_pose)


def yaw_error_to_target(world_state_result: dict[str, Any], target_pose: dict[str, Any]) -> float | None:
    world = world_state_result.get("world_state", {})
    pose = world.get("current_pose", {}).get("pose", {}) if isinstance(world, dict) else {}
    if not isinstance(pose, dict):
        return None
    return yaw_error(pose, target_pose)


def nearest_registry_node(args: argparse.Namespace, world_state_result: dict[str, Any]) -> dict[str, Any] | None:
    world = world_state_result.get("world_state", {})
    pose = world.get("current_pose", {}).get("pose", {}) if isinstance(world, dict) else {}
    if not isinstance(pose, dict):
        return None
    chassis = make_chassis(args, startup_wait_s=0.0)
    nearest: dict[str, Any] | None = None
    nearest_distance: float | None = None
    for node in chassis.nodes():
        node_pose = node.get("pose")
        if not isinstance(node_pose, dict):
            continue
        distance = pose_distance(pose, node_pose)
        if distance is None:
            continue
        if nearest_distance is None or distance < nearest_distance:
            nearest = node
            nearest_distance = distance
    if nearest is None:
        return None
    return {
        "node_id": nearest.get("node_id"),
        "name": nearest.get("name"),
        "distance_m": nearest_distance,
        "needs_calibration": "needs_calibration" in nearest.get("tags", []) if isinstance(nearest.get("tags"), list) else False,
    }


def semantic_world_snapshot(args: argparse.Namespace, world_state_result: dict[str, Any]) -> dict[str, Any]:
    world = world_state_result.get("world_state", {})
    if not isinstance(world, dict):
        return {"available": False, "reason": "missing world_state"}
    pose = world.get("current_pose", {}).get("pose", {})
    localization = world.get("localization", {})
    slam_health = world.get("slam_health", {})
    safety = world.get("safety", {})
    navigation = world.get("navigation", {})
    obstacle = world.get("local_obstacle", {})
    return {
        "available": True,
        "timestamp_ms": world.get("timestamp_ms"),
        "pose": {
            "x": pose.get("x"),
            "y": pose.get("y"),
            "yaw": pose.get("yaw"),
        }
        if isinstance(pose, dict)
        else None,
        "nearest_node": nearest_registry_node(args, world_state_result),
        "localization": {
            "status": localization.get("status"),
            "confidence": localization.get("confidence"),
            "pose_age_ms": localization.get("pose_age_ms"),
        }
        if isinstance(localization, dict)
        else None,
        "slam_health": {
            "status": slam_health.get("status"),
            "lidar_alive": slam_health.get("lidar_alive"),
            "localization_alive": slam_health.get("localization_alive"),
            "odom_alive": slam_health.get("odom_alive"),
        }
        if isinstance(slam_health, dict)
        else None,
        "safety": {
            "allow_navigation": safety.get("allow_navigation"),
            "reason": safety.get("reason"),
            "recommended_mode": safety.get("recommended_mode"),
        }
        if isinstance(safety, dict)
        else None,
        "navigation": {
            "state": navigation.get("state"),
            "target_node": navigation.get("target_node"),
            "distance_to_goal_m": navigation.get("distance_to_goal_m"),
            "failure_reason": navigation.get("failure_reason"),
            "is_arrived": navigation.get("is_arrived"),
        }
        if isinstance(navigation, dict)
        else None,
        "local_obstacle": {
            "front_clearance_m": obstacle.get("front_clearance_m"),
            "left_clearance_m": obstacle.get("left_clearance_m"),
            "right_clearance_m": obstacle.get("right_clearance_m"),
            "recommended_action": obstacle.get("recommended_action"),
        }
        if isinstance(obstacle, dict)
        else None,
    }


def format_semantic_world(snapshot: dict[str, Any]) -> str:
    if not snapshot.get("available"):
        return f"世界状态不可用：{snapshot.get('reason', 'unknown')}"
    pose = snapshot.get("pose") if isinstance(snapshot.get("pose"), dict) else {}
    nearest = snapshot.get("nearest_node") if isinstance(snapshot.get("nearest_node"), dict) else {}
    loc = snapshot.get("localization") if isinstance(snapshot.get("localization"), dict) else {}
    health = snapshot.get("slam_health") if isinstance(snapshot.get("slam_health"), dict) else {}
    safety = snapshot.get("safety") if isinstance(snapshot.get("safety"), dict) else {}
    nav = snapshot.get("navigation") if isinstance(snapshot.get("navigation"), dict) else {}
    obstacle = snapshot.get("local_obstacle") if isinstance(snapshot.get("local_obstacle"), dict) else {}
    allow_text = "允许导航" if safety.get("allow_navigation") is True else "禁止导航"
    target = nav.get("target_node") or "无"
    has_active_target = bool(nav.get("target_node")) and nav.get("state") not in (None, "", "idle")
    goal_distance_text = fmt_m(nav.get("distance_to_goal_m")) if has_active_target else "无"
    arrived_text = str(nav.get("is_arrived")) if has_active_target else "无"
    nearest_text = "未知"
    if nearest:
        nearest_text = f"{nearest.get('name') or nearest.get('node_id')}({fmt_m(nearest.get('distance_m'))})"
    return (
        f"定位:{loc.get('status', '未知')} conf={fmt_num(loc.get('confidence'))} | "
        f"SLAM:{health.get('status', '未知')} | "
        f"位置:x={fmt_num(pose.get('x'))}, y={fmt_num(pose.get('y'))}, yaw={fmt_num(pose.get('yaw'))} | "
        f"最近点:{nearest_text} | "
        f"导航:{nav.get('state', '未知')} 目标:{target} 距目标:{goal_distance_text} 到达:{arrived_text} | "
        f"前方净空:{fmt_m(obstacle.get('front_clearance_m'))} 建议:{obstacle.get('recommended_action', '未知')} | "
        f"安全:{allow_text}({safety.get('reason', '未知')})"
    )


def format_agent_summary(summary: dict[str, Any]) -> str:
    lines = []
    command = summary.get("command")
    if command:
        lines.append(f"用户指令：{command}")
    if summary.get("resolved_target") or summary.get("target_name"):
        lines.append(f"目标匹配：{summary.get('target_name') or summary.get('resolved_target')} ({summary.get('resolved_target') or summary.get('target_node')})")
    if summary.get("preflight_allowed") is not None:
        state = "通过" if summary.get("preflight_allowed") else "未通过"
        lines.append(f"安全检查：{state}，原因：{summary.get('preflight_reason') or '无'}")
    if summary.get("plan_mode") or summary.get("planner_route"):
        lines.append(f"规划路径：{summary.get('planner_route') or '未知'} / {summary.get('plan_mode') or '未知'}")
    if summary.get("task_queue_targets"):
        lines.append(f"任务队列：{summary.get('task_queue_targets')}，完成：{summary.get('task_queue_completed')}")
    if summary.get("execute"):
        result = "已执行" if summary.get("executed") else "未执行"
        lines.append(f"执行状态：{result}，到点：{summary.get('arrived')}，自动暂停：{summary.get('paused')}，最终距离：{fmt_m(summary.get('final_distance_m'))}")
    else:
        lines.append("执行状态：干跑/未下发运动")
    if summary.get("blocked_reason"):
        lines.append(f"阻塞原因：{summary.get('blocked_reason')}")
    if summary.get("operator_feedback_latest"):
        lines.append(f"现场反馈：{summary.get('operator_feedback_latest')}")
    if summary.get("llm_feedback_latest"):
        source = summary.get("llm_feedback_source") or "unknown"
        lines.append(f"LLM回馈：{summary.get('llm_feedback_latest')}（{source}）")
    if summary.get("runtime_poll_overruns") is not None:
        lines.append(
            f"实时性能：poll_overruns={summary.get('runtime_poll_overruns')}，"
            f"max_loop={fmt_num(summary.get('runtime_max_loop_elapsed_s'), 3)}s，"
            f"dropped={summary.get('runtime_dropped_counts') or {}}"
        )
    if summary.get("user_reply"):
        lines.append(f"对用户反馈：{summary.get('user_reply')}")
    logs = summary.get("logs")
    if isinstance(logs, dict):
        lines.append(f"日志：JSON={logs.get('json')} CSV={logs.get('csv')}")
    return "\n".join(lines) if lines else "没有可显示的中文摘要。"


def format_human_output(args: argparse.Namespace, output: dict[str, Any], logs: dict[str, str] | None = None) -> str:
    summary = summarize_agent_output(output)
    if logs:
        summary["logs"] = logs
    text = format_agent_summary(summary)
    latest_state: dict[str, Any] | None = None
    for step in output.get("steps", []):
        if not isinstance(step, dict):
            continue
        result = step.get("result")
        if isinstance(result, dict) and isinstance(result.get("world_state"), dict):
            latest_state = result
        if isinstance(result, dict) and isinstance(result.get("result"), dict) and isinstance(result["result"].get("world_state"), dict):
            latest_state = result["result"]
    if latest_state:
        text = f"{text}\n当前世界：{format_semantic_world(semantic_world_snapshot(args, latest_state))}"
    return text


def auto_pause_on_arrival(args: argparse.Namespace, slam_command: dict[str, Any] | None) -> dict[str, Any]:
    if not slam_command or slam_command.get("action") != "navigate_to_pose":
        return {"skipped": True, "reason": "no navigation command"}
    target_pose = slam_command.get("target_pose")
    if not isinstance(target_pose, dict):
        return {"skipped": True, "reason": "missing target_pose"}

    samples = []
    entered_count = 0
    deadline = time.time() + args.arrival_monitor_s
    while time.time() < deadline:
        state = get_world_state(args)
        distance_m = pose_distance_to_target(state, target_pose)
        yaw_error_rad = yaw_error_to_target(state, target_pose)
        sample = {
            "timestamp_ms": state.get("world_state", {}).get("timestamp_ms"),
            "distance_to_target_m": distance_m,
            "yaw_error_rad": yaw_error_rad,
            "pose": state.get("world_state", {}).get("current_pose", {}).get("pose"),
        }
        samples.append(sample)
        distance_ok = distance_m is not None and distance_m <= args.arrival_distance_m
        yaw_ok = True
        if args.require_arrival_yaw:
            yaw_ok = yaw_error_rad is None or yaw_error_rad <= args.arrival_yaw_rad
        if distance_ok and yaw_ok:
            entered_count += 1
            if entered_count >= args.arrival_confirm_samples:
                pause_result = pause_navigation(args)
                return {
                    "arrived": True,
                    "paused": bool(pause_result.get("accepted")),
                    "threshold_m": args.arrival_distance_m,
                    "yaw_threshold_rad": args.arrival_yaw_rad,
                    "require_yaw": bool(args.require_arrival_yaw),
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
        "yaw_threshold_rad": args.arrival_yaw_rad,
        "require_yaw": bool(args.require_arrival_yaw),
        "samples": samples,
        "reason": "arrival threshold not reached before timeout",
    }


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
        registry_map_path(args),
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
        "--communication-journal",
        args.communication_journal,
        "--communication-source-id",
        args.communication_source_id,
        "--link-state",
        args.link_state,
        "--replay-limit",
        str(args.replay_limit),
    ]
    if args.ack_sequence is not None:
        argv.extend(["--ack-sequence", str(args.ack_sequence)])
    if args.nav_speed_mps > 0:
        argv.extend(["--nav-speed-mps", str(args.nav_speed_mps)])
    if args.nav_mode is not None and args.nav_mode >= 0:
        argv.extend(["--nav-mode", str(args.nav_mode)])
    argv.extend(
        [
            "--arrival-distance-m",
            str(args.arrival_distance_m),
            "--arrival-confirm-samples",
            str(args.arrival_confirm_samples),
            "--arrival-monitor-s",
            str(args.arrival_monitor_s),
            "--navigation-monitor-travel-factor",
            str(args.navigation_monitor_travel_factor),
            "--navigation-monitor-startup-margin-s",
            str(args.navigation_monitor_startup_margin_s),
            "--navigation-stall-s",
            str(args.navigation_stall_s),
            "--navigation-progress-distance-m",
            str(args.navigation_progress_distance_m),
            "--navigation-progress-yaw-rad",
            str(args.navigation_progress_yaw_rad),
            "--arrival-monitor-interval-s",
            str(args.arrival_monitor_interval_s),
            "--slam-poll-interval-s",
            str(args.slam_poll_interval_s),
            "--ui-refresh-interval-s",
            str(args.ui_refresh_interval_s),
            "--operator-feedback-interval-s",
            str(args.operator_feedback_interval_s),
            "--llm-feedback-interval-s",
            str(args.llm_feedback_interval_s),
            "--llm-feedback-mode",
            args.llm_feedback_mode,
            "--llm-feedback-max-tokens",
            str(args.llm_feedback_max_tokens),
            "--llm-feedback-timeout-s",
            str(args.llm_feedback_timeout_s),
            "--max-arrival-samples",
            str(args.max_arrival_samples),
            "--max-feedback-events",
            str(args.max_feedback_events),
            "--max-llm-feedback-events",
            str(args.max_llm_feedback_events),
            "--gateway-error-limit",
            str(args.gateway_error_limit),
            "--max-reposition-steps",
            str(args.max_reposition_steps),
            "--reposition-settle-s",
            str(args.reposition_settle_s),
            "--mobility-strategy-mode",
            args.mobility_strategy_mode,
            "--mobility-strategy-max-tokens",
            str(args.mobility_strategy_max_tokens),
            "--mobility-strategy-timeout-s",
            str(args.mobility_strategy_timeout_s),
        ]
    )
    if args.llm_feedback_live_progress:
        argv.append("--llm-feedback-live-progress")
    if args.capture_command:
        argv.extend(["--capture-command", args.capture_command])
    if args.no_live_snapshot:
        argv.append("--no-live-snapshot")
        if not (args.skip_gateway_check and not args.execute):
            try:
                state = get_world_state(args)
                pose = state.get("world_state", {}).get("current_pose", {}).get("pose", {})
                argv.extend(["--mock-x", str(float(pose["x"])), "--mock-y", str(float(pose["y"])), "--mock-yaw", str(float(pose.get("yaw", 0.0)))])
            except Exception:
                pass
    if args.execute:
        argv.append("--execute")
    if args.skip_gateway_check and not args.execute:
        argv.append("--skip-gateway-check")
    completed = subprocess.run(argv, text=True, encoding="utf-8", errors="replace", capture_output=True, timeout=args.closed_loop_timeout_s)
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


def watch_world(args: argparse.Namespace) -> int:
    deadline = time.time() + args.monitor_s if args.monitor_s > 0 else None
    index = 0
    try:
        while deadline is None or time.time() < deadline:
            index += 1
            try:
                state = get_world_state(args)
                snapshot = semantic_world_snapshot(args, state)
                print(f"[{time.strftime('%H:%M:%S')}] {format_semantic_world(snapshot)}", flush=True)
            except Exception as exc:  # pragma: no cover - field robustness
                print(f"[{time.strftime('%H:%M:%S')}] 世界状态读取失败：{exc}", flush=True)
            time.sleep(max(0.2, args.monitor_interval_s))
    except KeyboardInterrupt:
        print("已停止实时世界状态监控。", flush=True)
    return 0


def speak(args: argparse.Namespace, text: str) -> dict[str, Any]:
    volume = max(0, min(100, int(args.voice_volume_percent)))
    volume_cmd = subprocess.run(
        ["pactl", "set-sink-volume", "@DEFAULT_SINK@", f"{volume}%"],
        text=True,
        encoding="utf-8",
        errors="replace",
        capture_output=True,
        timeout=5,
    )
    say_cmd = subprocess.run(
        ["spd-say", "-l", args.voice_language, "-r", str(args.voice_rate), "-p", str(args.voice_pitch), text],
        text=True,
        encoding="utf-8",
        errors="replace",
        capture_output=True,
        timeout=args.voice_timeout_s,
    )
    return {
        "text": text,
        "volume_percent": volume,
        "volume_returncode": volume_cmd.returncode,
        "volume_stderr": volume_cmd.stderr,
        "say_returncode": say_cmd.returncode,
        "say_stdout": say_cmd.stdout,
        "say_stderr": say_cmd.stderr,
    }


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Unified GO2W local LLM agent entrypoint.")
    parser.add_argument("--go", default="", help="One-shot field command: preflight, optional relocation, LLM planning, execute, and auto-pause.")
    parser.add_argument("--go-b64", default="", help="UTF-8 base64 encoded one-shot field command.")
    parser.add_argument("--fast", action="store_true", help="For --go, force deterministic/hybrid target matching before LLM. This is now the default route.")
    parser.add_argument("--force-llm", action="store_true", help="For --go, bypass deterministic target routing and force the real LLM light planner.")
    parser.add_argument("--list-nodes", action="store_true", help="List registry nodes without touching the robot gateway.")
    parser.add_argument("--resolve-target", default="", help="Resolve text to a registry node without touching the robot gateway.")
    parser.add_argument("--resolve-target-b64", default="", help="UTF-8 base64 encoded text to resolve to a registry node.")
    parser.add_argument("--preflight-only", action="store_true", help="Read gateway safety/localization state once and exit; no auto-start, no relocation, no navigation.")
    parser.add_argument("--ensure-slam", action="store_true", help="One-shot startup for LiDAR driver and unitree_slam, then wait for pointcloud/slam_info and run preflight.")
    parser.add_argument("--current-node", default="", help="Known navigation topology node for planner context; never used as a relocation pose.")
    parser.add_argument("--no-auto-start-slam", action="store_true", help="For --go, do not auto-start SLAM when health is not ready.")
    parser.add_argument("--no-auto-relocate", action="store_true", help="Deprecated compatibility flag; automatic relocation is always disabled.")
    parser.add_argument("--brief", action="store_true", help="Print a compact execution summary instead of the full trace.")
    parser.add_argument("--human", action="store_true", help="Print a Chinese operator summary instead of JSON.")
    parser.add_argument("--full-output", action="store_true", help="For --go, print the full trace instead of the default compact summary.")
    parser.add_argument("--log-dir", default="", help="Write full JSON and CSV run logs to this directory. --go defaults to artifacts/robot_runs.")
    parser.add_argument("--command", default="", help="Natural-language command. Prefer --command-b64 over SSH if encoding is unstable.")
    parser.add_argument("--command-b64", default="", help="UTF-8 base64 encoded natural-language command.")
    parser.add_argument("--say", default="", help="Speak a short sentence through the robot speaker.")
    parser.add_argument("--say-b64", default="", help="UTF-8 base64 encoded sentence to speak.")
    parser.add_argument("--voice-volume-percent", type=int, default=15)
    parser.add_argument("--voice-language", default="zh")
    parser.add_argument("--voice-rate", type=int, default=-30)
    parser.add_argument("--voice-pitch", type=int, default=-20)
    parser.add_argument("--voice-timeout-s", type=int, default=8)
    parser.add_argument("--execute", action="store_true", help="Actually execute the generated navigation command.")
    parser.add_argument("--dry-run", action="store_true", help="Plan and safety-check only. This is the default when --execute is absent.")
    parser.add_argument("--skip-gateway-check", action="store_true", help="For planner dry-runs, skip the closed-loop gateway check after planning.")
    parser.add_argument("--no-auto-pause", action="store_true", help="Do not pause navigation after reaching the target distance.")
    parser.add_argument("--nav-speed-mps", type=float, default=0.3, help="Global navigation speed override. Unitree GO2 mode 0 is normalized to its 0.2 m/s minimum.")
    parser.add_argument("--nav-mode", type=int, default=0, help="Global Unitree navigation mode override for field runs. Default 0 enables obstacle avoidance; use 1 only for explicit stop-mode diagnostics or -1 to keep registry mode.")
    parser.add_argument("--arrival-distance-m", type=float, default=0.25)
    parser.add_argument("--arrival-yaw-rad", type=float, default=0.18)
    parser.add_argument("--require-arrival-yaw", action="store_true", help="Require yaw threshold before auto-pause; default pauses by distance only.")
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
    parser.add_argument("--llm-feedback-live-progress", action="store_true", help="Allow live LLM calls for progress feedback inside the arrival loop.")
    parser.add_argument("--llm-feedback-max-tokens", type=int, default=48)
    parser.add_argument("--llm-feedback-timeout-s", type=int, default=6)
    parser.add_argument("--max-arrival-samples", type=int, default=120)
    parser.add_argument("--max-feedback-events", type=int, default=120)
    parser.add_argument("--max-llm-feedback-events", type=int, default=40)
    parser.add_argument("--gateway-error-limit", type=int, default=3)
    parser.add_argument("--max-reposition-steps", type=int, default=3)
    parser.add_argument("--reposition-settle-s", type=float, default=0.6)
    parser.add_argument(
        "--mobility-strategy-mode",
        choices=["live", "deterministic"],
        default="live",
    )
    parser.add_argument("--mobility-strategy-max-tokens", type=int, default=160)
    parser.add_argument("--mobility-strategy-timeout-s", type=int, default=20)
    parser.add_argument("--capture-command", default=os.environ.get("GO2W_CAPTURE_COMMAND", ""), help="Optional bash command for queued capture_keyframe steps.")
    parser.add_argument(
        "--communication-journal",
        default=os.environ.get(
            "GO2W_COMMUNICATION_JOURNAL",
            str(REPO_ROOT / "artifacts" / "communication" / "communication_journal_v1.jsonl"),
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
    parser.add_argument("--start-slam", action="store_true", help="Start xt16_driver and unitree_slam before other steps.")
    parser.add_argument("--relocate", action="store_true", help="Run explicit relocation against a verified registry anchor.")
    parser.add_argument("--relocation-anchor", default="")
    parser.add_argument("--confirm-relocation", default="")
    parser.add_argument("--status", action="store_true", help="Print gateway world_state.")
    parser.add_argument("--watch-world", action="store_true", help="Stream live Chinese semantic world-state lines. Use --monitor-s to stop after N seconds; default runs until Ctrl+C.")
    parser.add_argument("--pause", action="store_true", help="Pause current navigation task.")
    parser.add_argument("--calibrate-node", default="", help="Update this registry node pose from the current live robot pose.")
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
    parser.add_argument("--ensure-slam-wait-s", type=float, default=20.0)
    parser.add_argument("--ensure-slam-interval-s", type=float, default=2.0)
    parser.add_argument("--ensure-slam-sample-timeout-s", type=int, default=5)
    parser.add_argument("--map-path", default=DEFAULT_MAP_PATH)
    parser.add_argument("--pretty", action="store_true")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if args.execute and args.skip_gateway_check:
        print_json(
            {
                "error": "--skip-gateway-check is only allowed for dry-runs; remove it before --execute",
                "execute": True,
            },
            pretty=args.pretty,
        )
        return 2
    if args.watch_world:
        return watch_world(args)

    if args.go or args.go_b64:
        if args.dry_run:
            args.execute = False
        args.no_live_snapshot = True
        if args.force_llm:
            args.prompt_mode = "light"
        elif args.fast:
            args.prompt_mode = "hybrid"
        elif args.prompt_mode == "hybrid":
            args.prompt_mode = "hybrid"
        if args.nav_mode == 0 and 0.0 < args.nav_speed_mps < 0.2:
            args.nav_speed_mps = 0.2

    command = decode_command(args)
    say_text = decode_say(args)
    output: dict[str, Any] = {
        "command": command,
        "say": say_text,
        "execute": bool(args.execute),
        "nav_speed_mps": args.nav_speed_mps if args.nav_speed_mps > 0 else None,
        "nav_mode": args.nav_mode if args.nav_mode is not None and args.nav_mode >= 0 else None,
        "steps": [],
    }

    if args.list_nodes:
        output["steps"].append({"step": "list_nodes", "nodes": make_chassis(args, startup_wait_s=0.0).node_summary()})

    resolve_text = base64.b64decode(args.resolve_target_b64).decode("utf-8") if args.resolve_target_b64 else args.resolve_target
    if not resolve_text and (args.go or args.go_b64) and command:
        resolve_text = command
    if resolve_text:
        output["steps"].append({"step": "resolve_target", "text": resolve_text, "result": make_chassis(args, startup_wait_s=0.0).resolve_node(resolve_text)})

    if args.go or args.go_b64:
        output["steps"].append(
            {
                "step": "go_route",
                "prompt_mode": args.prompt_mode,
                "force_llm": bool(args.force_llm),
                "reason": "force real LLM" if args.force_llm else "auto route: deterministic target first, LLM fallback",
            }
        )

    if args.preflight_only:
        preflight = make_chassis(args).preflight()
        output["steps"].append({"step": "preflight_only", **preflight})

    if args.ensure_slam:
        output["steps"].append({"step": "ensure_slam", "result": run_ensure_slam(args)})

    if args.go or args.go_b64:
        preflight_checked = not (args.skip_gateway_check and not args.execute)
        if preflight_checked:
            try:
                state = get_world_state(args)
                allowed, reason = gateway_allows_navigation(state)
            except Exception as exc:  # pragma: no cover - field robustness
                state = None
                allowed, reason = False, f"preflight failed: {exc}"
            output["steps"].append({"step": "go_preflight", "checked": True, "allowed": allowed, "reason": reason, "result": state})
        else:
            state = None
            allowed, reason = True, "gateway preflight skipped for dry-run"
            output["steps"].append({"step": "go_preflight", "checked": False, "allowed": None, "reason": reason, "result": None})
        if not args.execute and preflight_checked and not allowed:
            output["steps"].append({"step": "go_dry_run_preflight_not_enforced", "reason": reason})
        if args.execute and not allowed and not args.no_auto_start_slam:
            ensure_result = run_ensure_slam(args)
            output["steps"].append({"step": "go_auto_ensure_slam", "result": ensure_result})
            allowed = bool(ensure_result.get("preflight_allowed"))
            reason = str(ensure_result.get("preflight_reason") or reason)
            state = ensure_result.get("preflight", {}).get("result") if isinstance(ensure_result.get("preflight"), dict) else None
            output["steps"].append({"step": "go_status_after_ensure_slam", "allowed": allowed, "reason": reason, "result": state})
        if args.execute and not allowed:
            output["steps"].append(
                {
                    "step": "go_auto_relocate_blocked",
                    "reason": (
                        "automatic relocation from current_node is disabled; "
                        "physically align the robot and run supervised anchor relocation"
                    ),
                }
            )
        if args.execute and not allowed:
            output["steps"].append({"step": "go_blocked", "reason": reason})
            if args.human:
                print(format_human_output(args, output))
            else:
                print_json(output, pretty=args.pretty)
            return 2

    if args.start_slam:
        output["steps"].append({"step": "start_slam", "result": run_start_slam(args.start_slam_script)})
        time.sleep(2)

    if args.relocate:
        if not args.relocation_anchor:
            output["steps"].append(
                {
                    "step": "relocate",
                    "result": {
                        "accepted": False,
                        "reason": (
                            "--relocation-anchor is required; select the active verified "
                            "anchor matching the robot's physical pose"
                        ),
                    },
                }
            )
            if args.human:
                print(format_human_output(args, output))
            else:
                print_json(output, pretty=args.pretty)
            return 2
        if args.confirm_relocation != args.relocation_anchor:
            output["steps"].append(
                {
                    "step": "relocate",
                    "result": {
                        "accepted": False,
                        "reason": (
                            "explicit anchor confirmation required: "
                            f"--confirm-relocation {args.relocation_anchor}"
                        ),
                    },
                }
            )
            if args.human:
                print(format_human_output(args, output))
            else:
                print_json(output, pretty=args.pretty)
            return 2
        else:
            relocation_result = relocate_to_anchor(args, args.relocation_anchor)
            output["steps"].append(
                {
                    "step": "relocate",
                    "anchor_id": args.relocation_anchor,
                    "result": relocation_result,
                }
            )
            if relocation_result.get("accepted") is not True:
                if args.human:
                    print(format_human_output(args, output))
                else:
                    print_json(output, pretty=args.pretty)
                return 3
            time.sleep(args.gateway_startup_wait_s)

    if args.status:
        state = get_world_state(args)
        allowed, reason = gateway_allows_navigation(state)
        output["steps"].append({"step": "status", "allowed": allowed, "reason": reason, "result": state})

    if args.pause:
        output["steps"].append({"step": "pause", "result": pause_navigation(args)})

    if args.calibrate_node:
        output["steps"].append({"step": "calibrate_node", "result": calibrate_node_from_current_pose(args, args.calibrate_node)})

    if command:
        closed_loop_result = run_closed_loop(args, command)
        output["steps"].append({"step": "closed_loop", "result": closed_loop_result})
        closed_loop_payload = closed_loop_result.get("result", {})
        slam_command = None
        if isinstance(closed_loop_payload, dict):
            slam_command = closed_loop_payload.get("planner", {}).get("slam_command") if isinstance(closed_loop_payload.get("planner"), dict) else None
        queue_execution = closed_loop_payload.get("queue_execution") if isinstance(closed_loop_payload, dict) else None
        if args.execute and not args.no_auto_pause and not queue_execution:
            output["steps"].append({"step": "auto_pause_on_arrival", "result": auto_pause_on_arrival(args, slam_command)})

    if args.monitor_s > 0:
        output["steps"].append({"step": "monitor", "samples": monitor(args)})

    if say_text:
        output["steps"].append({"step": "speak", "result": speak(args, say_text)})

    log_dir = args.log_dir
    if (args.go or args.go_b64) and not log_dir:
        log_dir = str(DEFAULT_LOG_DIR)
    logs = write_execution_log(output, log_dir) if log_dir else None
    if args.human:
        print(format_human_output(args, output, logs))
    elif args.brief or ((args.go or args.go_b64) and not args.full_output):
        summary = summarize_agent_output(output)
        if logs:
            summary["logs"] = logs
        print_json(summary, pretty=True)
    else:
        if logs:
            output["logs"] = logs
        print_json(output, pretty=args.pretty)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
