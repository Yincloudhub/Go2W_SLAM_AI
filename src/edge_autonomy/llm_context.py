from __future__ import annotations

import re
import time
from typing import Any

from .map_registry import MapProfile, MapRegistry, TopologyNode
from .perception_context import validate_perception_context


NAVIGABLE_LOCALIZATION_STATUSES = {"localized", "localized_or_tracking", "tracking", "degraded"}
BLOCKING_NAVIGATION_TAGS = {
    "disabled",
    "ui_disabled",
    "deleted",
    "needs_calibration",
    "needs_standing_verification",
    "requires_standing_verification",
}
RELATIVE_MOTION_TERMS = (
    "\u524d\u8fdb",
    "\u5411\u524d",
    "\u5f80\u524d",
    "\u76f4\u8d70",
    "\u540e\u9000",
    "\u5411\u540e",
    "\u5f80\u540e",
    "\u5de6\u79fb",
    "\u5411\u5de6\u79fb\u52a8",
    "\u53f3\u79fb",
    "\u5411\u53f3\u79fb\u52a8",
    "\u8f6c\u5411",
    "\u8f6c\u5f2f",
    "\u65cb\u8f6c",
    "\u6389\u5934",
    "forward",
    "ahead",
    "straight",
    "backward",
    "backwards",
    "reverse",
    "move left",
    "move right",
    "strafe",
    "turn",
    "rotate",
)
MOTION_AMOUNT_TERMS = (
    "\u7c73",
    "\u5398\u7c73",
    "\u5ea6",
    "\u4e00\u70b9",
    "\u4e00\u4e0b",
    "meter",
    "meters",
    "metre",
    "metres",
    "cm",
    "centimeter",
    "centimeters",
    "centimetre",
    "centimetres",
    "degree",
    "degrees",
    "a little",
)
CHINESE_DIGITS = {
    "\u96f6": 0,
    "\u4e00": 1,
    "\u4e8c": 2,
    "\u4e24": 2,
    "\u4e09": 3,
    "\u56db": 4,
    "\u4e94": 5,
    "\u516d": 6,
    "\u4e03": 7,
    "\u516b": 8,
    "\u4e5d": 9,
}
CAPTURE_REQUEST_TERMS = (
    "\u62cd\u7167",
    "\u62cd\u4e00\u5f20",
    "\u7167\u7247",
    "\u5173\u952e\u5e27",
    "photo",
    "capture",
    "keyframe",
)


def _distance_xy(a: dict[str, float], b: dict[str, float]) -> float:
    return ((a["x"] - b["x"]) ** 2 + (a["y"] - b["y"]) ** 2) ** 0.5


def _pose_dict_from_snapshot(snapshot: dict[str, Any]) -> dict[str, float] | None:
    odom = snapshot.get("relocation_odom", {})
    if not odom.get("alive"):
        return None
    if odom.get("x") is None or odom.get("y") is None:
        return None
    return {
        "x": float(odom.get("x", 0.0)),
        "y": float(odom.get("y", 0.0)),
        "z": float(odom.get("z", 0.0) or 0.0),
        "yaw": float(odom.get("yaw", 0.0) or 0.0),
    }


def _node_pose_xy(node: TopologyNode) -> dict[str, float]:
    return {"x": node.pose.x, "y": node.pose.y}


def nearest_topology_node(profile: MapProfile, pose: dict[str, float] | None) -> dict[str, Any] | None:
    if pose is None or not profile.topology_nodes:
        return None
    best_node = min(profile.topology_nodes, key=lambda node: _distance_xy(pose, _node_pose_xy(node)))
    return {
        "node_id": best_node.node_id,
        "name": best_node.name,
        "distance_m": _distance_xy(pose, _node_pose_xy(best_node)),
        "node_type": best_node.node_type,
    }


def command_requests_relative_motion(user_command: str) -> bool:
    command = user_command.lower()
    return any(term in command for term in RELATIVE_MOTION_TERMS) and any(
        term in command for term in MOTION_AMOUNT_TERMS
    )


def _parse_chinese_number(text: str) -> float | None:
    if not text:
        return None
    if text == "\u5341":
        return 10.0
    if "\u5341" in text:
        left, right = text.split("\u5341", 1)
        tens = CHINESE_DIGITS.get(left, 1 if left == "" else 0)
        ones = CHINESE_DIGITS.get(right, 0) if right else 0
        value = tens * 10 + ones
        return float(value) if value > 0 else None
    value = CHINESE_DIGITS.get(text)
    return float(value) if value is not None else None


def relative_motion_distance_m(user_command: str) -> float | None:
    command = user_command.lower()
    match = re.search(
        r"(\d+(?:\.\d+)?)\s*(?:cm|centimeter|centimeters|centimetre|centimetres)",
        command,
    )
    if match:
        return float(match.group(1)) / 100.0
    match = re.search(r"(\d+(?:\.\d+)?)\s*\u5398\u7c73", user_command)
    if match:
        return float(match.group(1)) / 100.0
    match = re.search(r"(\d+(?:\.\d+)?)\s*(?:m|meter|meters|metre|metres)", command)
    if match:
        return float(match.group(1))
    match = re.search(r"(\d+(?:\.\d+)?)\s*\u7c73", user_command)
    if match:
        return float(match.group(1))
    match = re.search(r"([\u96f6\u4e00\u4e8c\u4e24\u4e09\u56db\u4e94\u516d\u4e03\u516b\u4e5d\u5341]+)\s*\u7c73", user_command)
    if match:
        return _parse_chinese_number(match.group(1))
    return None


def relative_motion_direction(user_command: str) -> str:
    command = user_command.lower()
    direction_terms = (
        ("backward", ("\u540e\u9000", "\u5411\u540e", "\u5f80\u540e", "backward", "backwards", "reverse")),
        ("left", ("\u5de6\u79fb", "\u5411\u5de6\u79fb\u52a8", "move left", "strafe left")),
        ("right", ("\u53f3\u79fb", "\u5411\u53f3\u79fb\u52a8", "move right", "strafe right")),
        ("rotate", ("\u8f6c\u5411", "\u8f6c\u5f2f", "\u65cb\u8f6c", "\u6389\u5934", "turn", "rotate")),
    )
    for direction, terms in direction_terms:
        if any(term in command for term in terms):
            return direction
    return "forward"


def relative_motion_preview_from_command(user_command: str) -> dict[str, Any] | None:
    if not command_requests_relative_motion(user_command):
        return None
    command = user_command.lower()
    capture_requested = any(term.lower() in command for term in CAPTURE_REQUEST_TERMS)
    return {
        "capability": "relative_motion",
        "tool": "relative_motion_preview",
        "status": "dry_run_only",
        "real_execution": False,
        "requested_direction": relative_motion_direction(user_command),
        "requested_distance_m": relative_motion_distance_m(user_command),
        "capture_requested": capture_requested,
        "safety_requirements": [
            "odometry_or_visual_inertial_tracking",
            "fresh_local_obstacle_summary",
            "operator_confirmed_recovery_policy",
            "hard_stop_on_gateway_or_obstacle_reject",
        ],
        "blocked_reason": "relative_motion is not wired for real execution",
    }


def build_capability_contract(*, localized: bool, snapshot: dict[str, Any]) -> dict[str, Any]:
    capture_configured = bool(snapshot.get("capture_command_configured") or snapshot.get("camera_capture_configured"))
    return {
        "schema_version": 1,
        "planning_style": "capability_bounded_task_planning",
        "ready": [
            {
                "name": "hold_position",
                "tools": ["hold_position"],
                "authority": "operator_core",
            },
            {
                "name": "request_human_confirm",
                "tools": ["request_human_confirm"],
                "authority": "operator_core",
            },
        ],
        "conditional": [
            {
                "name": "mapped_topology_navigation",
                "tools": ["create_navigation_subgoal", "wait_until"],
                "available": localized,
                "requires": ["registered_topology_node", "fresh_localization", "SafetyGate_allow", "SLAM_Gateway_accept"],
                "fallback": "human_confirm_or_safe_hold",
            },
            {
                "name": "capture_keyframe",
                "tools": ["capture_keyframe"],
                "available": capture_configured,
                "status": "ready" if capture_configured else "semantic_event_only",
                "fallback": "record_semantic_keyframe_event",
            },
            {
                "name": "bounded_supervised_reposition",
                "tools": [],
                "available": localized,
                "status": "internal_llm_strategy_with_deterministic_guard",
                "requires": [
                    "operator_present",
                    "supervised_release",
                    "native_navigation_reported_no_progress",
                    "directional_escape_clearance",
                ],
                "semantic_inputs": [
                    "native_navigation_failure_or_stall",
                    "four_direction_clearance_candidates",
                    "target_progress",
                ],
                "limits": {
                    "directions": ["forward", "backward", "left", "right"],
                    "max_distance_m": 0.5,
                    "controller": "unitree_pose_navigation_mode_0",
                    "max_speed_mps": 0.2,
                },
                "fallback": "pause_and_request_human",
            },
        ],
        "not_wired": [
            {
                "name": "relative_motion",
                "examples": ["forward_10m_photo", "odom_only_drive"],
                "dry_run_tool": "relative_motion_preview",
                "status": "dry_run_only",
                "real_execution": False,
                "fallback": "human_confirm",
            },
            {
                "name": "mapless_scout",
                "examples": ["explore_unknown_area_without_registered_node"],
                "fallback": "human_confirm",
            },
            {
                "name": "raw_base_control",
                "examples": ["cmd_vel", "raw_Unitree_API"],
                "fallback": "reject",
            },
        ],
    }


def build_planner_context(
    snapshot: dict[str, Any],
    registry: MapRegistry,
    *,
    user_command: str,
    map_id: str | None = None,
    perception_context: dict[str, Any] | None = None,
) -> dict[str, Any]:
    profile = registry.get_map(map_id or snapshot.get("expected_map_id") or registry.default_map_id)
    pose = _pose_dict_from_snapshot(snapshot)
    nearest = nearest_topology_node(profile, pose)
    health_status = str(snapshot.get("health_status", "unknown"))
    localization_status = str(snapshot.get("localization_status", "unknown"))
    localized = health_status in {"ok", "degraded"} and localization_status in NAVIGABLE_LOCALIZATION_STATUSES and pose is not None

    available_nodes = []
    for node in profile.topology_nodes:
        node_pose = node.pose.to_unitree_json(name=node.node_id)
        tags = list(node.tags)
        available_nodes.append(
            {
                "node_id": node.node_id,
                "name": node.name,
                "aliases": list(node.aliases),
                "node_type": node.node_type,
                "tags": tags,
                "navigation_eligible": (
                    not bool(set(tags) & BLOCKING_NAVIGATION_TAGS)
                    and "live_verified" in tags
                ),
                "pose": {
                    "x": node_pose["x"],
                    "y": node_pose["y"],
                    "yaw": node_pose["yaw"],
                },
                "distance_from_robot_m": _distance_xy(pose, _node_pose_xy(node)) if pose else None,
                "default_speed_mps": node_pose["speed"],
                "mode": node_pose["mode"],
                "description": node.description,
            }
        )

    allowed_actions = ["hold_position", "request_human_confirm"]
    if localized:
        allowed_actions.extend(["navigate_to_verified_node", "pause_navigation"])
    capability_contract = build_capability_contract(localized=localized, snapshot=snapshot)
    relative_motion_request = relative_motion_preview_from_command(user_command)

    lidar = snapshot.get("lidar_state", {})
    pointcloud = snapshot.get("live_pointcloud", {})
    timestamp_ms = int(snapshot.get("timestamp_ms", int(time.time() * 1000)))
    validated_perception_context = validate_perception_context(
        perception_context,
        current_time_ms=int(time.time() * 1000),
    )
    return {
        "schema_version": 1,
        "timestamp_ms": timestamp_ms,
        "user_command": user_command,
        "perception_context": validated_perception_context,
        "world_state_summary": {
            "map": {
                "map_id": profile.map_id,
                "name": profile.name,
                "pcd_path": profile.pcd_path,
                "frame_id": profile.frame_id,
            },
            "robot": {
                "pose": pose,
                "nearest_node": nearest,
                "localized": localized,
            },
            "slam": {
                "health_status": health_status,
                "localization_status": localization_status,
                "processes": snapshot.get("processes", {}),
            },
            "lidar": {
                "alive": bool(lidar.get("alive") or pointcloud.get("alive")),
                "cloud_frequency_hz": lidar.get("cloud_frequency_hz"),
                "cloud_size": lidar.get("cloud_size") or pointcloud.get("width"),
                "error_state": lidar.get("error_state"),
                "pointcloud_topic": pointcloud.get("topic"),
                "pointcloud_width": pointcloud.get("width"),
            },
            "topology": {
                "available_nodes": available_nodes,
                "edges": [
                    {
                        "from": edge.from_node,
                        "to": edge.to_node,
                        "bidirectional": edge.bidirectional,
                        "expected_distance_m": (
                            edge.expected_distance_m if edge.distance_verified else None
                        ),
                        "distance_verified": edge.distance_verified,
                        "description": edge.description,
                    }
                    for edge in profile.topology_edges
                ],
            },
            "allowed_actions": allowed_actions,
            "relative_motion_request": relative_motion_request,
        },
        "capability_contract": capability_contract,
        "relative_motion_request": relative_motion_request,
        "planner_rules": [
            "Do not output raw Unitree API IDs.",
            "Use mapped_navigation only when SLAM health is navigable, localization is fresh, and a topology node is selected.",
            "Use create_navigation_subgoal with a topology node instead of raw coordinates when possible.",
            "Use /slam_info ctrl_info is_arrived or stateMachine FINISHED as the arrival condition.",
            "Do not request dense pointcloud or raw video for weak-bandwidth planning.",
            "relative_motion_preview is dry-run only and must never produce a SLAM or raw base-control command.",
            "The LLM may select one bounded supervised reposition candidate, but MissionDecisionEngine and Gateway must validate every step and retain final motion authority.",
        ],
    }


def _find_requested_node(profile: MapProfile, user_command: str) -> TopologyNode | None:
    command = user_command.lower()
    for node in profile.topology_nodes:
        candidates = [node.node_id, node.name, *node.aliases]
        if any(candidate and candidate.lower() in command for candidate in candidates):
            return node

    if any(token in user_command for token in ("起点", "回来", "回去", "返回", "wp_0")):
        for node in profile.topology_nodes:
            if "startup" in node.tags or "wp_0" in node.aliases or node.node_id.endswith("wp_0"):
                return node

    if any(token in user_command for token in ("巡视点", "候选", "wp_1", "前面", "过去")):
        for node in profile.topology_nodes:
            if "wp_1" in node.aliases or node.node_id.endswith("wp_1"):
                return node

    return None


def simulate_local_llm_plan(context: dict[str, Any], registry: MapRegistry) -> dict[str, Any]:
    summary = context["world_state_summary"]
    profile = registry.get_map(summary["map"]["map_id"])
    user_command = str(context.get("user_command", ""))
    health_status = summary["slam"]["health_status"]
    localization_status = summary["slam"]["localization_status"]
    localized = summary["robot"]["localized"]

    communication_policy = {
        "mode": "semantic_only",
        "send": ["task_state", "risk_events", "semantic_topology", "navigation_feedback", "world_state_summary"],
        "drop": ["raw_video", "dense_pointcloud", "full_log", "high_rate_images"],
        "reason": "weak-bandwidth loop uses structured state instead of raw sensor streams",
    }

    if any(token in user_command for token in ("停", "暂停", "别动", "原地")):
        return {
            "plan_id": f"mock_plan_{int(time.time() * 1000)}",
            "mode": "safe_hold",
            "confidence": 0.95,
            "reason": "user requested stop or hold position",
            "steps": [{"step_id": "hold_1", "tool": "hold_position", "arguments": {"reason": user_command}}],
            "communication_policy": communication_policy,
            "requires_human_ack": False,
        }

    if health_status not in {"ok", "degraded"} or localization_status not in NAVIGABLE_LOCALIZATION_STATUSES or not localized:
        return {
            "plan_id": f"mock_plan_{int(time.time() * 1000)}",
            "mode": "safe_hold",
            "confidence": 0.9,
            "reason": "SLAM or localization is not reliable enough for mapped navigation",
            "steps": [{"step_id": "hold_1", "tool": "hold_position", "arguments": {"health_status": health_status, "localization_status": localization_status}}],
            "communication_policy": communication_policy,
            "requires_human_ack": True,
        }

    if command_requests_relative_motion(user_command):
        preview = relative_motion_preview_from_command(user_command) or {
            "capability": "relative_motion",
            "status": "dry_run_only",
            "real_execution": False,
            "blocked_reason": "relative_motion is not wired for real execution",
        }
        return {
            "plan_id": f"mock_plan_{int(time.time() * 1000)}",
            "mode": "human_confirm",
            "confidence": 0.78,
            "reason": "relative_motion preview only; real execution is blocked",
            "steps": [
                {
                    "step_id": "preview_1",
                    "tool": "relative_motion_preview",
                    "arguments": preview,
                },
                {
                    "step_id": "ask_1",
                    "tool": "request_human_confirm",
                    "arguments": {
                        "message": "\u76f8\u5bf9\u8fd0\u52a8\u8fd8\u672a\u63a5\u5165\u771f\u5b9e\u6267\u884c\uff0c\u8bf7\u9009\u62e9\u5df2\u767b\u8bb0\u62d3\u6251\u70b9\u6216\u4ec5\u8fdb\u884c\u5e72\u8dd1\u3002",
                        "missing_capability": "relative_motion",
                        "available_actions": summary.get("allowed_actions", []),
                    },
                }
            ],
            "communication_policy": communication_policy,
            "requires_human_ack": True,
        }

    target_node = _find_requested_node(profile, user_command)
    if target_node is None:
        return {
            "plan_id": f"mock_plan_{int(time.time() * 1000)}",
            "mode": "human_confirm",
            "confidence": 0.45,
            "reason": "no known topology node was clearly requested",
            "steps": [
                {
                    "step_id": "ask_1",
                    "tool": "request_human_confirm",
                    "arguments": {
                        "message": "请选择一个已登记拓扑点",
                        "available_nodes": [node["node_id"] for node in summary["topology"]["available_nodes"]],
                    },
                }
            ],
            "communication_policy": communication_policy,
            "requires_human_ack": True,
        }

    robot_pose = summary["robot"].get("pose")
    if robot_pose is not None:
        distance_to_target = _distance_xy(robot_pose, _node_pose_xy(target_node))
        if distance_to_target <= 0.25:
            return {
                "plan_id": f"mock_plan_{int(time.time() * 1000)}",
                "mode": "safe_hold",
                "confidence": 0.9,
                "reason": f"robot is already near topology node {target_node.node_id}",
                "steps": [
                    {
                        "step_id": "hold_1",
                        "tool": "hold_position",
                        "arguments": {
                            "target_node": target_node.node_id,
                            "distance_to_target_m": distance_to_target,
                            "arrival_source": "/slam_info ctrl_info or odom proximity",
                        },
                    }
                ],
                "communication_policy": communication_policy,
                "requires_human_ack": False,
            }

    target_pose = target_node.pose.to_unitree_json(name=target_node.node_id)
    return {
        "plan_id": f"mock_plan_{int(time.time() * 1000)}",
        "mode": "mapped_navigation",
        "confidence": 0.82,
        "reason": f"matched user command to topology node {target_node.node_id}",
        "steps": [
            {
                "step_id": "comm_1",
                "tool": "set_communication_policy",
                "arguments": communication_policy,
            },
            {
                "step_id": "nav_1",
                "tool": "create_navigation_subgoal",
                "arguments": {
                    "map_id": profile.map_id,
                    "target_node": target_node.node_id,
                    "target_pose": {
                        "x": target_pose["x"],
                        "y": target_pose["y"],
                        "yaw": target_pose["yaw"],
                    },
                    "speed_mps": target_pose["speed"],
                    "mode": target_pose["mode"],
                    "safety_mode": "normal",
                },
            },
            {
                "step_id": "wait_1",
                "tool": "wait_until",
                "arguments": {
                    "source": "/slam_info",
                    "condition": "type == 'ctrl_info' and (data.is_arrived == true or data.stateMachine.state == 'FINISHED')",
                },
            },
        ],
        "communication_policy": communication_policy,
        "requires_human_ack": False,
    }


def plan_to_slam_command(plan: dict[str, Any], registry: MapRegistry, *, speed: float | None = None, mode: int | None = None) -> dict[str, Any] | None:
    if plan.get("mode") != "mapped_navigation":
        return None
    for step in plan.get("steps", []):
        if step.get("tool") == "create_navigation_subgoal":
            args = step.get("arguments", {})
            map_id = args.get("map_id")
            target_node = args.get("target_node")
            if not map_id or not target_node:
                return None
            return registry.get_map(str(map_id)).navigate_to_node_command(
                str(target_node),
                speed=speed if speed is not None else (float(args["speed_mps"]) if args.get("speed_mps") is not None else None),
                mode=mode if mode is not None else (int(args["mode"]) if args.get("mode") is not None else None),
            )
    return None
