from __future__ import annotations

from .slam_state import CurrentPose, LocalizationState, SlamHealth
from .slam_topics import now_ms


class SlamHealthMonitor:
    def __init__(self, *, map_id: str = "unknown") -> None:
        self._map_id = map_id
        self._last_pose: CurrentPose | None = None
        self._lost_since_ms: int | None = None

    def observe_pose(self, pose: CurrentPose) -> None:
        self._map_id = pose.map_id
        self._last_pose = pose
        self._lost_since_ms = None

    def get_localization_state(self, *, timestamp_ms: int | None = None) -> LocalizationState:
        ts = timestamp_ms if timestamp_ms is not None else now_ms()
        if self._last_pose is None:
            return LocalizationState(timestamp_ms=ts, map_id=self._map_id, status="not_started", confidence=0.0, pose_age_ms=-1)

        age_ms = ts - self._last_pose.timestamp_ms
        if age_ms <= 500:
            return LocalizationState(timestamp_ms=ts, map_id=self._map_id, status="localized", confidence=0.9, pose_age_ms=age_ms)
        if age_ms <= 2000:
            return LocalizationState(timestamp_ms=ts, map_id=self._map_id, status="degraded", confidence=0.5, pose_age_ms=age_ms)

        if self._lost_since_ms is None:
            self._lost_since_ms = self._last_pose.timestamp_ms + 2000
        return LocalizationState(
            timestamp_ms=ts,
            map_id=self._map_id,
            status="lost",
            confidence=0.0,
            pose_age_ms=age_ms,
            lost_duration_ms=max(0, ts - self._lost_since_ms),
        )

    def get_slam_health(self, *, timestamp_ms: int | None = None) -> SlamHealth:
        ts = timestamp_ms if timestamp_ms is not None else now_ms()
        if self._last_pose is None:
            return SlamHealth(timestamp_ms=ts, status="failed")

        age_ms = ts - self._last_pose.timestamp_ms
        slam_alive = age_ms < 5000
        localization_alive = age_ms < 2000
        if age_ms <= 500:
            status = "ok"
        elif age_ms <= 2000:
            status = "degraded"
        else:
            status = "failed"

        return SlamHealth(
            timestamp_ms=ts,
            slam_alive=slam_alive,
            lidar_alive=True,
            imu_alive=True,
            odom_alive=True,
            localization_alive=localization_alive,
            last_pose_age_ms=age_ms,
            status=status,
        )

