from __future__ import annotations

import time
from typing import Any


LOCALIZED_STATUSES = {"localized", "localized_or_tracking", "tracking", "degraded"}
GOOD_SLAM_STATUSES = {"ok", "degraded"}
TASK_PHASES = {
    "idle",
    "planning",
    "waiting_safety_check",
    "executing_navigation",
    "arrived",
    "executing_after_arrival_action",
    "reporting",
    "completed",
    "failed",
    "blocked",
    "not_localized",
    "target_unknown",
    "human_confirm_required",
    "cancelled",
}

REFRESH_POLICY = {
    "sensor_ingest_hz": {"min": 10.0, "max": 20.0, "consumer": "internal_safety_only"},
    "world_state_hz": {"min": 2.0, "max": 5.0, "consumer": "safety_and_ui"},
    "ui_display_hz": {"min": 1.0, "max": 2.0, "consumer": "operator_panel"},
    "llm_feedback": {"mode": "event_driven", "min_interval_s": 6.0, "progress_interval_s": 8.0},
}


def now_ms() -> int:
    return int(time.time() * 1000)


def _path(value: Any, keys: list[str], default: Any = None) -> Any:
    current = value
    for key in keys:
        if not isinstance(current, dict) or key not in current:
            return default
        current = current[key]
    return current


def _as_bool(value: Any, default: bool = False) -> bool:
    if isinstance(value, bool):
        return value
    return default


def _as_float(value: Any) -> float | None:
    if isinstance(value, bool):
        return None
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def _first_not_none(*values: float | None) -> float | None:
    for value in values:
        if value is not None:
            return value
    return None


def _compact_pose(value: Any) -> dict[str, float] | None:
    if not isinstance(value, dict):
        return None
    pose: dict[str, float] = {}
    for key in ("x", "y", "z", "yaw"):
        number = _as_float(value.get(key))
        if number is not None:
            pose[key] = number
    return pose or None


def _planner_summary(planner_context: dict[str, Any] | None) -> dict[str, Any]:
    summary = _path(planner_context, ["world_state_summary"], {})
    return summary if isinstance(summary, dict) else {}


def _gateway_world(runtime_or_gateway: dict[str, Any]) -> dict[str, Any]:
    world = runtime_or_gateway.get("world_state")
    return world if isinstance(world, dict) else {}


def _candidate_nodes(planner_context: dict[str, Any] | None, *, limit: int = 12) -> list[dict[str, Any]]:
    summary = _planner_summary(planner_context)
    nodes = _path(summary, ["topology", "available_nodes"], [])
    if not isinstance(nodes, list):
        return []
    out: list[dict[str, Any]] = []
    for node in nodes[:limit]:
        if not isinstance(node, dict):
            continue
        out.append(
            {
                "node_id": node.get("node_id"),
                "name": node.get("name"),
                "distance_from_robot_m": node.get("distance_from_robot_m"),
                "node_type": node.get("node_type"),
                "tags": node.get("tags", []),
            }
        )
    return out


def _current_node(planner_context: dict[str, Any] | None) -> str | None:
    nearest = _path(_planner_summary(planner_context), ["robot", "nearest_node"], {})
    if isinstance(nearest, dict) and isinstance(nearest.get("node_id"), str):
        return nearest["node_id"]
    return None


def obstacle_status(front_clearance_m: float | None) -> str:
    if front_clearance_m is None:
        return "unknown"
    if front_clearance_m < 0.8:
        return "blocked"
    if front_clearance_m < 1.5:
        return "slow"
    return "clear"


def _normalize_perception_summary(value: dict[str, Any]) -> dict[str, Any]:
    return {
        "source": str(value.get("source") or "unknown"),
        "confidence": _as_float(value.get("confidence")),
        "stale": _as_bool(value.get("stale"), False),
        "latency_ms": _as_float(value.get("latency_ms")),
        "timestamp_ms": int(value.get("timestamp_ms") or now_ms()),
        "summary": value.get("summary", {}),
    }


def _available_tools(*, localized: bool, motion_allowed: bool, map_loaded: bool) -> list[str]:
    tools = ["safe_hold", "ask_human_confirm", "semantic_report", "cancel_task"]
    if not localized:
        tools.append("request_relocalization")
    if map_loaded:
        tools.extend(["capture_keyframe", "speak"])
    if localized and motion_allowed:
        tools.extend(["navigate", "patrol_route", "inspect_area", "return_to_base"])
    return tools


def _task_phase(value: str | None, *, localized: bool) -> str:
    if value in TASK_PHASES:
        return str(value)
    if not localized:
        return "not_localized"
    return "idle"


def build_world_state_v1(
    runtime_or_gateway: dict[str, Any],
    *,
    planner_context: dict[str, Any] | None = None,
    task_phase: str | None = None,
    last_execution_result: str = "",
    network_level: str = "normal",
    motion_allowed: bool | None = None,
    detected_objects: list[dict[str, Any]] | None = None,
    perception_summaries: list[dict[str, Any]] | None = None,
    timestamp_ms: int | None = None,
) -> dict[str, Any]:
    """Build the low-rate state contract consumed by UI, LLM, and policy code.

    This function accepts either the robot-side gateway response
    {"world_state": ...} or the read-only SSH runtime snapshot dict produced by
    scripts/slam_runtime_snapshot.py. It deliberately emits bounded semantic
    summaries instead of high-rate sensor data.
    """

    summary = _planner_summary(planner_context)
    gateway_world = _gateway_world(runtime_or_gateway)

    loc_status = str(
        _path(gateway_world, ["localization", "status"], "")
        or _path(summary, ["slam", "localization_status"], "")
        or runtime_or_gateway.get("localization_status", "")
    )
    slam_status = str(
        _path(gateway_world, ["slam_health", "status"], "")
        or _path(summary, ["slam", "health_status"], "")
        or runtime_or_gateway.get("health_status", "")
    )
    pose = (
        _compact_pose(_path(gateway_world, ["current_pose", "pose"]))
        or _compact_pose(_path(summary, ["robot", "pose"]))
        or _compact_pose(_path(runtime_or_gateway, ["relocation_odom"]))
    )
    localized = (
        loc_status in LOCALIZED_STATUSES
        and (slam_status in GOOD_SLAM_STATUSES or not slam_status)
        and pose is not None
    )
    map_id = (
        _path(summary, ["map", "map_id"])
        or runtime_or_gateway.get("expected_map_id")
        or _path(gateway_world, ["current_pose", "map_id"])
        or "unknown"
    )
    map_loaded = bool(map_id and map_id != "unknown")
    safety = _path(gateway_world, ["safety"], {})
    safety_allow = _as_bool(safety.get("allow_navigation") if isinstance(safety, dict) else None, localized)
    final_motion_allowed = safety_allow if motion_allowed is None else bool(motion_allowed)
    front_clearance = _first_not_none(
        _as_float(_path(gateway_world, ["local_obstacle", "front_clearance_m"])),
        _as_float(_path(summary, ["lidar", "front_clearance_m"])),
    )

    normalized_perception = [
        _normalize_perception_summary(item)
        for item in (perception_summaries or [])
        if isinstance(item, dict)
    ]
    if front_clearance is not None:
        normalized_perception.append(
            _normalize_perception_summary(
                {
                    "source": "local_obstacle",
                    "confidence": _path(gateway_world, ["local_obstacle", "confidence"], 0.5),
                    "stale": _path(gateway_world, ["local_obstacle", "stale"], False),
                    "latency_ms": _path(gateway_world, ["local_obstacle", "latency_ms"]),
                    "timestamp_ms": _path(gateway_world, ["local_obstacle", "timestamp_ms"], timestamp_ms or now_ms()),
                    "summary": {"front_clearance_m": front_clearance},
                }
            )
        )

    phase = _task_phase(task_phase, localized=localized)
    return {
        "schema_version": 1,
        "timestamp_ms": int(timestamp_ms or runtime_or_gateway.get("timestamp_ms") or _path(gateway_world, ["timestamp_ms"], now_ms())),
        "localized": localized,
        "map_loaded": map_loaded,
        "map_id": map_id,
        "current_pose": pose,
        "current_node": _current_node(planner_context),
        "candidate_nodes": _candidate_nodes(planner_context),
        "front_clearance_m": front_clearance,
        "obstacle_status": obstacle_status(front_clearance),
        "detected_objects": detected_objects or [],
        "network_level": network_level,
        "task_phase": phase,
        "last_execution_result": last_execution_result,
        "motion_allowed": bool(final_motion_allowed and localized),
        "available_tools": _available_tools(localized=localized, motion_allowed=bool(final_motion_allowed), map_loaded=map_loaded),
        "source_health": {
            "slam_status": slam_status or "unknown",
            "localization_status": loc_status or "unknown",
            "safety_reason": safety.get("reason") if isinstance(safety, dict) else "",
        },
        "perception_summaries": normalized_perception,
        "refresh_policy": REFRESH_POLICY,
    }
