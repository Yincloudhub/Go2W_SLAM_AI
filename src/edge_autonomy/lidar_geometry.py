from __future__ import annotations

from .slam_state import LocalObstacleSummary
from .slam_topics import now_ms


class ManualLidarGeometryPerception:
    """Phase-1 conservative LiDAR summary until real PointCloud2 processing is wired."""

    def __init__(self, range_m: float = 6.0) -> None:
        self.range_m = range_m
        self._summary = self.set_clearance(range_m, range_m, range_m, range_m)

    def set_clearance(
        self,
        front_clearance_m: float,
        left_clearance_m: float,
        right_clearance_m: float,
        rear_clearance_m: float,
        *,
        timestamp_ms: int | None = None,
    ) -> LocalObstacleSummary:
        blocked_directions: list[str] = []
        if front_clearance_m < 0.8:
            blocked_directions.append("front")
        if left_clearance_m < 0.6:
            blocked_directions.append("left")
        if right_clearance_m < 0.6:
            blocked_directions.append("right")
        if rear_clearance_m < 0.6:
            blocked_directions.append("rear")

        if front_clearance_m < 0.8:
            recommended_action = "pause"
        elif front_clearance_m < 1.5 or left_clearance_m < 0.8 or right_clearance_m < 0.8:
            recommended_action = "go_slow"
        else:
            recommended_action = "normal"

        self._summary = LocalObstacleSummary(
            timestamp_ms=timestamp_ms if timestamp_ms is not None else now_ms(),
            source="manual_stub",
            range_m=self.range_m,
            front_clearance_m=front_clearance_m,
            left_clearance_m=left_clearance_m,
            right_clearance_m=right_clearance_m,
            rear_clearance_m=rear_clearance_m,
            blocked_directions=blocked_directions,
            narrow_passage=left_clearance_m < 0.8 and right_clearance_m < 0.8,
            recommended_action=recommended_action,
            confidence=0.0,
            stale=True,
        )
        return self._summary

    def get_local_obstacle_summary(self) -> LocalObstacleSummary:
        return self._summary
