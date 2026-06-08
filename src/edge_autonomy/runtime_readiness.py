from __future__ import annotations

import json
import math
import time
from pathlib import Path
from typing import Any

from .gateway_safety import LOCALIZED_STATUSES, TRUSTED_OBSTACLE_SOURCES, gateway_allows_navigation


def extract_json_objects(text: str) -> list[dict[str, Any]]:
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
    return objects


def gateway_response_from_output(text: str) -> dict[str, Any] | None:
    objects = extract_json_objects(text)
    for value in reversed(objects):
        if isinstance(value.get("world_state"), dict):
            return value
    return objects[-1] if objects else None


def load_json_summary(path: str | Path, *, max_bytes: int = 256 * 1024) -> dict[str, Any] | None:
    summary_path = Path(path)
    if not summary_path.is_file() or summary_path.stat().st_size > max_bytes:
        return None
    try:
        value = json.loads(summary_path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError):
        return None
    return value if isinstance(value, dict) else None


def _finite_number(value: Any) -> float | None:
    if isinstance(value, bool):
        return None
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def _localization_readiness(world: dict[str, Any], max_pose_age_ms: float) -> tuple[bool, str]:
    localization = world.get("localization")
    if not isinstance(localization, dict):
        return False, "missing localization"
    status = str(localization.get("status") or "")
    if status not in LOCALIZED_STATUSES:
        return False, f"localization is {status or 'unknown'}"
    pose_age_ms = _finite_number(localization.get("pose_age_ms"))
    if pose_age_ms is None or pose_age_ms < 0 or pose_age_ms > max_pose_age_ms:
        return False, "localization pose is stale or missing"
    pose = world.get("current_pose", {}).get("pose") if isinstance(world.get("current_pose"), dict) else None
    if not isinstance(pose, dict) or _finite_number(pose.get("x")) is None or _finite_number(pose.get("y")) is None:
        return False, "current pose x/y is invalid"
    return True, "localization pose is fresh"


def _slam_readiness(world: dict[str, Any]) -> tuple[bool, str]:
    health = world.get("slam_health")
    if not isinstance(health, dict):
        return False, "missing slam health"
    status = str(health.get("status") or "")
    if status not in {"ok", "degraded"}:
        return False, f"slam health is {status or 'unknown'}"
    if health.get("slam_alive") is not True:
        return False, "slam_alive is not true"
    return True, f"slam health is {status}"


def _perception_readiness(
    world: dict[str, Any],
    lidar_summary: dict[str, Any] | None,
    *,
    max_age_ms: int,
) -> tuple[bool, str, dict[str, Any]]:
    obstacle = lidar_summary
    if not isinstance(obstacle, dict):
        candidate = world.get("local_obstacle")
        obstacle = candidate if isinstance(candidate, dict) else None
    if not isinstance(obstacle, dict):
        return False, "missing local obstacle summary", {}

    source = str(obstacle.get("source") or "")
    stale = bool(obstacle.get("stale", True))
    timestamp_ms = int(obstacle.get("timestamp_ms") or 0)
    age_ms = max(0, int(time.time() * 1000) - timestamp_ms) if timestamp_ms > 0 else obstacle.get("age_ms")
    producer_summary = obstacle.get("summary") if isinstance(obstacle.get("summary"), dict) else {}
    calibrated = producer_summary.get("calibrated")
    details = {
        "source": source,
        "stale": stale,
        "age_ms": age_ms,
        "calibrated": calibrated,
        "recommended_action": obstacle.get("recommended_action"),
        "blocked_directions": obstacle.get("blocked_directions", []),
        "front_clearance_m": obstacle.get("front_clearance_m"),
        "left_clearance_m": obstacle.get("left_clearance_m"),
        "right_clearance_m": obstacle.get("right_clearance_m"),
        "rear_clearance_m": obstacle.get("rear_clearance_m"),
    }
    if source not in TRUSTED_OBSTACLE_SOURCES:
        return False, f"local obstacle source is {source or 'missing'}", details
    if calibrated is False:
        return False, "XT16 geometry is not calibrated", details
    if stale:
        reasons = obstacle.get("stale_reasons")
        suffix = f": {reasons}" if reasons else ""
        return False, f"local obstacle summary is stale{suffix}", details
    if not isinstance(age_ms, (int, float)) or isinstance(age_ms, bool) or age_ms < 0 or age_ms > max_age_ms:
        return False, "local obstacle summary age is invalid or stale", details
    return True, "trusted local obstacle summary is fresh", details


def assess_runtime_readiness(
    world_state_result: dict[str, Any] | None,
    *,
    startup_ok: bool,
    lidar_summary: dict[str, Any] | None = None,
    llm_configured: bool = False,
    max_localization_pose_age_ms: float = 2000.0,
    max_perception_age_ms: int = 1500,
) -> dict[str, Any]:
    response = world_state_result if isinstance(world_state_result, dict) else {}
    world = response.get("world_state")
    gateway_ready = response.get("accepted") is True and isinstance(world, dict)
    if not isinstance(world, dict):
        world = {}

    slam_ready, slam_reason = _slam_readiness(world)
    localization_ready, localization_reason = _localization_readiness(world, max_localization_pose_age_ms)
    perception_ready, perception_reason, perception = _perception_readiness(
        world,
        lidar_summary,
        max_age_ms=max_perception_age_ms,
    )

    if gateway_ready:
        navigation_ready, navigation_reason = gateway_allows_navigation(
            response,
            max_localization_pose_age_ms=max_localization_pose_age_ms,
        )
    else:
        navigation_ready, navigation_reason = False, "gateway world state is unavailable"

    services_ready = bool(startup_ok and gateway_ready)
    execution_arming_ready = bool(services_ready and localization_ready)
    relocalization_ready = bool(services_ready)
    acceptance_ok = bool(navigation_ready and perception_ready)

    blockers: list[str] = []
    if not gateway_ready:
        blockers.append("gateway")
    if not slam_ready:
        blockers.append("slam")
    if not localization_ready:
        blockers.append("localization")
    if not perception_ready:
        blockers.append("perception")
    if not navigation_ready:
        blockers.append("navigation")

    if not services_ready:
        next_action = "repair startup or gateway connectivity"
    elif not localization_ready:
        next_action = "request relocalization from a verified anchor"
    elif not perception_ready:
        next_action = "calibrate or restore XT16 geometry before navigation"
    elif not navigation_ready:
        next_action = "clear the reported navigation safety blocker"
    else:
        next_action = "ready for supervised topology navigation"

    return {
        "services_ready": services_ready,
        "gateway_ready": gateway_ready,
        "slam_ready": slam_ready,
        "slam_reason": slam_reason,
        "localization_ready": localization_ready,
        "localization_reason": localization_reason,
        "relocalization_ready": relocalization_ready,
        "execution_arming_ready": execution_arming_ready,
        "perception_ready": perception_ready,
        "perception_reason": perception_reason,
        "perception": perception,
        "navigation_ready": navigation_ready,
        "navigation_reason": navigation_reason,
        "llm_ready": bool(llm_configured),
        "llm_required_for_navigation": False,
        "acceptance_ok": acceptance_ok,
        "blockers": blockers,
        "next_action": next_action,
    }
