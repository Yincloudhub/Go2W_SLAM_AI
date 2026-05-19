from __future__ import annotations

from .slam_state import CurrentPose, LocalizationState, SlamHealth
from .slam_topics import now_ms


class SlamHealthMonitor:
    def __init__(
        self,
        *,
        map_id: str = "unknown",
        fresh_pose_ms: int = 500,
        degraded_pose_ms: int = 2000,
        failed_pose_ms: int = 5000,
        assume_sensor_alive_with_pose: bool = True,
    ) -> None:
        self.map_id = map_id
        self.fresh_pose_ms = fresh_pose_ms
        self.degraded_pose_ms = degraded_pose_ms
        self.failed_pose_ms = failed_pose_ms
        self.assume_sensor_alive_with_pose = assume_sensor_alive_with_pose
        self._last_pose_update_ms: int | None = None
        self._lost_since_ms: int | None = None

    def observe_pose(self, pose: CurrentPose) -> None:
        self.map_id = pose.map_id
        self._last_pose_update_ms = pose.timestamp_ms
        self._lost_since_ms = None

    def get_localization_state(self, *, timestamp_ms: int | None = None) -> LocalizationState:
        ts = timestamp_ms if timestamp_ms is not None else now_ms()
        if self._last_pose_update_ms is None:
            return LocalizationState(timestamp_ms=ts, map_id=self.map_id, status="not_started", confidence=0.0, pose_age_ms=None)

        pose_age_ms = max(0, ts - self._last_pose_update_ms)
        if pose_age_ms <= self.fresh_pose_ms:
            self._lost_since_ms = None
            return LocalizationState(timestamp_ms=ts, map_id=self.map_id, status="localized", confidence=0.9, pose_age_ms=pose_age_ms)

        if pose_age_ms <= self.degraded_pose_ms:
            self._lost_since_ms = None
            return LocalizationState(timestamp_ms=ts, map_id=self.map_id, status="degraded", confidence=0.5, pose_age_ms=pose_age_ms)

        if self._lost_since_ms is None:
            self._lost_since_ms = self._last_pose_update_ms + self.degraded_pose_ms
        return LocalizationState(
            timestamp_ms=ts,
            map_id=self.map_id,
            status="lost",
            confidence=0.0,
            pose_age_ms=pose_age_ms,
            lost_duration_ms=max(0, ts - self._lost_since_ms),
        )

    def get_slam_health(self, *, timestamp_ms: int | None = None) -> SlamHealth:
        ts = timestamp_ms if timestamp_ms is not None else now_ms()
        if self._last_pose_update_ms is None:
            return SlamHealth(
                timestamp_ms=ts,
                slam_alive=False,
                lidar_alive=False,
                imu_alive=False,
                odom_alive=False,
                localization_alive=False,
                last_pose_age_ms=None,
                status="failed",
            )

        pose_age_ms = max(0, ts - self._last_pose_update_ms)
        slam_alive = pose_age_ms < self.failed_pose_ms
        localization_alive = pose_age_ms <= self.degraded_pose_ms
        sensor_alive = self.assume_sensor_alive_with_pose and slam_alive

        if pose_age_ms <= self.fresh_pose_ms:
            status = "ok"
        elif pose_age_ms <= self.degraded_pose_ms:
            status = "degraded"
        else:
            status = "failed"

        return SlamHealth(
            timestamp_ms=ts,
            slam_alive=slam_alive,
            lidar_alive=sensor_alive,
            imu_alive=sensor_alive,
            odom_alive=sensor_alive,
            localization_alive=localization_alive,
            last_pose_age_ms=pose_age_ms,
            status=status,
        )
