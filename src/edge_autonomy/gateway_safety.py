from __future__ import annotations

import math
from typing import Any


LOCALIZED_STATUSES = {"localized", "localized_or_tracking", "tracking", "degraded"}
TRUSTED_OBSTACLE_SOURCES = {"stereo_depth", "lidar_pointcloud", "lidar_pointcloud+stereo_depth"}


def _finite_number(value: Any) -> float | None:
    if isinstance(value, bool):
        return None
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def _dict_at(value: dict[str, Any], key: str) -> dict[str, Any] | None:
    item = value.get(key)
    return item if isinstance(item, dict) else None


def gateway_allows_navigation(
    world_state_result: dict[str, Any],
    *,
    max_localization_pose_age_ms: float = 2000.0,
    max_obstacle_age_ms: float = 1000.0,
    min_roi_confidence: float = 0.15,
    min_clearance_m: float = 0.8,
) -> tuple[bool, str]:
    """Host preflight gate for sending real navigation commands to the gateway.

    The gateway/SLAM safety decision is the hard authority. Local obstacle
    summaries are advisory: they only block when a fresh trusted sensor reports
    a concrete close obstacle, and they do not have to be present for relocation
    or navigation preflight.
    """

    world_state = world_state_result.get("world_state", {})
    if not isinstance(world_state, dict):
        return False, "missing world_state"

    safety = _dict_at(world_state, "safety")
    if safety is None:
        return False, "missing safety decision"
    if safety.get("allow_navigation") is not True:
        return False, f"safety disallows navigation: {safety.get('reason', 'unknown')}"

    slam_health = _dict_at(world_state, "slam_health")
    if slam_health is not None:
        health_status = str(slam_health.get("status", ""))
        if health_status in {"failed", "lost", "not_started"}:
            return False, f"slam health is {health_status}"
        for key in ("slam_alive", "localization_alive"):
            if key in slam_health and slam_health.get(key) is not True:
                return False, f"{key} is not true"

    localization = _dict_at(world_state, "localization")
    if localization is None:
        return False, "missing localization"
    loc_status = str(localization.get("status", ""))
    if loc_status not in LOCALIZED_STATUSES:
        return False, f"localization is {loc_status or 'unknown'}"
    pose_age_ms = _finite_number(localization.get("pose_age_ms"))
    if pose_age_ms is None or pose_age_ms < 0 or pose_age_ms > max_localization_pose_age_ms:
        return False, "localization pose is stale or missing"
    confidence = _finite_number(localization.get("confidence"))
    if confidence is not None and confidence <= 0:
        return False, "localization confidence is too low"

    current_pose = _dict_at(world_state, "current_pose")
    pose = _dict_at(current_pose, "pose") if current_pose else None
    if pose is None:
        return False, "missing current pose"
    if _finite_number(pose.get("x")) is None or _finite_number(pose.get("y")) is None:
        return False, "current pose x/y is invalid"

    obstacle = _dict_at(world_state, "local_obstacle")
    if obstacle is not None and obstacle.get("source") in TRUSTED_OBSTACLE_SOURCES and obstacle.get("stale") is False:
        obstacle_age_ms = _finite_number(obstacle.get("age_ms"))
        if obstacle_age_ms is not None and 0 <= obstacle_age_ms <= max_obstacle_age_ms:
            obstacle_action = str(obstacle.get("recommended_action") or "")
            if obstacle_action in {"pause", "stop", "emergency_stop"}:
                return False, f"local_obstacle recommends {obstacle_action}"
            for direction in ("front", "left", "right"):
                roi_confidence = _finite_number(obstacle.get(f"{direction}_confidence"))
                clearance = _finite_number(obstacle.get(f"{direction}_clearance_m"))
                if (
                    roi_confidence is not None
                    and roi_confidence >= min_roi_confidence
                    and clearance is not None
                    and 0 <= clearance < min_clearance_m
                ):
                    return False, f"local_obstacle {direction} clearance is unsafe"

    mode = str(safety.get("recommended_mode") or "")
    if mode and mode != "normal":
        return True, f"gateway allows navigation in {mode} mode"
    return True, "gateway allows navigation"
