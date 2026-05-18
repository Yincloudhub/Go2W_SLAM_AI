from __future__ import annotations

import time
from typing import Any

from .map_registry import MapProfile, MapRegistry, TopologyNode


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


def build_planner_context(
    snapshot: dict[str, Any],
    registry: MapRegistry,
    *,
    user_command: str,
    map_id: str | None = None,
) -> dict[str, Any]:
    """Build the compact state package that should be sent to an LLM planner."""
    profile = registry.get_map(map_id or snapshot.get("expected_map_id") or registry.default_map_id)
    pose = _pose_dict_from_snapshot(snapshot)
    nearest = nearest_topology_node(profile, pose)
    health_status = str(snapshot.get("health_status", "unknown"))
    localization_status = str(snapshot.get("localization_status", "unknown"))
    localized = health_status == "ok" and localization_status == "localized_or_tracking" and pose is not None

    available_nodes = []
    for node in profile.topology_nodes:
        node_pose = node.pose.to_unitree_json(name=node.node_id)
        available_nodes.append(
            {
                "node_id": node.node_id,
                "name": node.name,
                "aliases": list(node.aliases),
                "node_type": node.node_type,
                "tags": list(node.tags),
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

    lidar = snapshot.get("lidar_state", {})
    pointcloud = snapshot.get("live_pointcloud", {})
    return {
        "schema_version": 1,
        "timestamp_ms": int(snapshot.get("timestamp_ms", int(time.time() * 1000))),
        "user_command": user_command,
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
                        "expected_distance_m": edge.expected_distance_m,
                        "description": edge.description,
                    }
                    for edge in profile.topology_edges
                ],
            },
            "allowed_actions": allowed_actions,
        },
        "planner_rules": [
            "Do not output raw Unitree API IDs.",
            "Use mapped_navigation only when health_status is ok and localization_status is localized_or_tracking.",
            "Use create_navigation_subgoal with a topology node instead of raw coordinates when possible.",
            "Use /slam_info ctrl_info is_arrived or stateMachine FINISHED as the arrival condition.",
            "Do not request dense pointcloud or raw video for weak-bandwidth planning.",
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
    """Rule-based planner used until a real LLM call is wired in.

    Keep this function's output compatible with schemas/local_llm_plan.schema.json
    so it can be replaced by a model response without changing the executor.
    """
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

    if health_status != "ok" or localization_status != "localized_or_tracking" or not localized:
        return {
            "plan_id": f"mock_plan_{int(time.time() * 1000)}",
            "mode": "safe_hold",
            "confidence": 0.9,
            "reason": "SLAM or localization is not reliable enough for mapped navigation",
            "steps": [{"step_id": "hold_1", "tool": "hold_position", "arguments": {"health_status": health_status, "localization_status": localization_status}}],
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


def plan_to_slam_command(plan: dict[str, Any], registry: MapRegistry) -> dict[str, Any] | None:
    """Compile planner intent into the restricted JSON accepted by C++ gateway."""
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
                speed=float(args["speed_mps"]) if args.get("speed_mps") is not None else None,
                mode=int(args["mode"]) if args.get("mode") is not None else None,
            )
    return None
