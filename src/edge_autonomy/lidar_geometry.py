from __future__ import annotations

from .obstacle_policy import blocked_directions, narrow_passage, recommended_action
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
        clearance = {
            "front": front_clearance_m,
            "left": left_clearance_m,
            "right": right_clearance_m,
            "rear": rear_clearance_m,
        }

        self._summary = LocalObstacleSummary(
            timestamp_ms=timestamp_ms if timestamp_ms is not None else now_ms(),
            source="manual_stub",
            range_m=self.range_m,
            front_clearance_m=front_clearance_m,
            left_clearance_m=left_clearance_m,
            right_clearance_m=right_clearance_m,
            rear_clearance_m=rear_clearance_m,
            blocked_directions=blocked_directions(clearance),
            narrow_passage=narrow_passage(clearance),
            recommended_action=recommended_action(clearance),
            confidence=0.0,
            stale=True,
        )
        return self._summary

    def get_local_obstacle_summary(self) -> LocalObstacleSummary:
        return self._summary
