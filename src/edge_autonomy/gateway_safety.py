from __future__ import annotations

from typing import Any


LOCALIZED_STATUSES = {"localized", "localized_or_tracking", "tracking"}
TRUSTED_OBSTACLE_SOURCES = {"lidar_pointcloud", "lidar_pointcloud+stereo_depth"}


def gateway_allows_navigation(
    world_state_result: dict[str, Any],
) -> tuple[bool, str]:
    """Return the robot gateway's authoritative navigation decision.

    Host code intentionally does not re-evaluate localization, perception, or
    clearance thresholds. Those checks belong to the robot-side gateway and its
    runtime watchdog; duplicating them here creates policy drift.
    """

    if world_state_result.get("accepted") is not True:
        return False, f"gateway response was not accepted: {world_state_result.get('reason', 'unknown')}"

    world_state = world_state_result.get("world_state", {})
    if not isinstance(world_state, dict):
        return False, "missing world_state"

    safety = world_state.get("safety")
    if not isinstance(safety, dict):
        return False, "missing safety decision"
    if safety.get("allow_navigation") is not True:
        return False, f"safety disallows navigation: {safety.get('reason', 'unknown')}"

    mode = str(safety.get("recommended_mode") or "")
    if mode and mode != "normal":
        return True, f"gateway allows navigation in {mode} mode"
    return True, str(safety.get("reason") or "gateway allows navigation")
