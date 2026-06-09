from __future__ import annotations

from dataclasses import asdict, dataclass
from typing import Any

from .slam_state import LocalObstacleSummary


@dataclass(frozen=True)
class DepthCameraSummary:
    """Compact depth-camera result for the closed-loop runtime.

    Raw stereo frames or dense depth clouds should be consumed by a separate
    perception process. The camera is forward-facing: left/right values are
    image sectors inside its forward field of view, not robot-side clearances.
    """

    timestamp_ms: int
    frame_id: str = "camera_depth_optical_frame"
    source: str = "stereo_depth"
    range_m: float = 6.0
    front_clearance_m: float | None = None
    left_clearance_m: float | None = None
    right_clearance_m: float | None = None
    rear_clearance_m: float | None = None
    confidence: float = 0.0
    stale: bool = False
    latency_ms: float | None = None

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


def _clearance_min(primary_value: float, secondary_value: float | None) -> float:
    if secondary_value is None or secondary_value < 0.0:
        return primary_value
    return min(primary_value, secondary_value)


def _recommended_action(front: float, left: float, right: float, base_action: str) -> str:
    if base_action == "pause" or front < 0.8:
        return "pause"
    if base_action == "go_slow" or front < 1.5 or left < 0.8 or right < 0.8:
        return "go_slow"
    return "normal"


def _blocked_directions(front: float, left: float, right: float, rear: float, base: list[str]) -> list[str]:
    blocked = set(base)
    if front < 0.8:
        blocked.add("front")
    if left < 0.6:
        blocked.add("left")
    if right < 0.6:
        blocked.add("right")
    if rear < 0.6:
        blocked.add("rear")
    return [direction for direction in ("front", "left", "right", "rear") if direction in blocked]


def depth_summary_is_usable(
    summary: DepthCameraSummary | None,
    *,
    now_ms: int | None = None,
    max_age_ms: int = 500,
    min_confidence: float = 0.5,
) -> bool:
    if summary is None:
        return False
    if summary.stale or summary.confidence < min_confidence:
        return False
    if summary.latency_ms is not None and summary.latency_ms > max_age_ms:
        return False
    if now_ms is not None and now_ms - summary.timestamp_ms > max_age_ms:
        return False
    return True


def fuse_local_obstacle_summary(
    lidar_summary: LocalObstacleSummary,
    depth_summary: DepthCameraSummary | None,
    *,
    now_ms: int | None = None,
    max_depth_age_ms: int = 500,
    min_depth_confidence: float = 0.5,
) -> LocalObstacleSummary:
    """Fuse optional stereo depth into the LiDAR obstacle summary conservatively.

    Forward-facing stereo may only reduce the front clearance. Robot-side and
    rear clearances remain owned by the 360-degree LiDAR geometry path.
    Invalid, old, or low-confidence depth is ignored.
    """

    if not depth_summary_is_usable(
        depth_summary,
        now_ms=now_ms,
        max_age_ms=max_depth_age_ms,
        min_confidence=min_depth_confidence,
    ):
        return lidar_summary

    assert depth_summary is not None
    front = _clearance_min(lidar_summary.front_clearance_m, depth_summary.front_clearance_m)
    left = lidar_summary.left_clearance_m
    right = lidar_summary.right_clearance_m
    rear = lidar_summary.rear_clearance_m
    blocked = _blocked_directions(front, left, right, rear, list(lidar_summary.blocked_directions))
    confidence = (
        depth_summary.confidence
        if lidar_summary.confidence is None
        else min(lidar_summary.confidence, depth_summary.confidence)
    )

    return LocalObstacleSummary(
        timestamp_ms=max(lidar_summary.timestamp_ms, depth_summary.timestamp_ms),
        frame_id=lidar_summary.frame_id,
        source=f"{lidar_summary.source}+{depth_summary.source}",
        range_m=min(lidar_summary.range_m, depth_summary.range_m),
        front_clearance_m=front,
        left_clearance_m=left,
        right_clearance_m=right,
        rear_clearance_m=rear,
        body_clearance_m=dict(lidar_summary.body_clearance_m),
        low_hazard_clearance_m=dict(lidar_summary.low_hazard_clearance_m),
        low_hazard_directions=list(lidar_summary.low_hazard_directions),
        blocked_directions=blocked,
        narrow_passage=lidar_summary.narrow_passage or (left < 0.8 and right < 0.8),
        recommended_action=_recommended_action(front, left, right, lidar_summary.recommended_action),
        confidence=confidence,
        stale=False,
        latency_ms=depth_summary.latency_ms,
    )
