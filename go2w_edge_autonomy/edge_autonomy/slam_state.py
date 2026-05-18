from __future__ import annotations

from dataclasses import asdict, dataclass, field
from typing import Any

from .models import Pose2D


@dataclass(frozen=True)
class CurrentPose:
    timestamp_ms: int
    map_id: str = "unknown"
    frame_id: str = "map"
    pose: Pose2D = field(default_factory=lambda: Pose2D(0.0, 0.0, 0.0))
    z: float = 0.0
    source: str = "rt/slam_info"

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True)
class LocalizationState:
    timestamp_ms: int
    map_id: str = "unknown"
    status: str = "not_started"
    confidence: float = 0.0
    pose_age_ms: int = -1
    lost_duration_ms: int = 0

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True)
class SlamHealth:
    timestamp_ms: int
    slam_alive: bool = False
    lidar_alive: bool = False
    imu_alive: bool = False
    odom_alive: bool = False
    localization_alive: bool = False
    last_pose_age_ms: int = -1
    status: str = "failed"

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True)
class NavigationTaskState:
    timestamp_ms: int
    task_id: str = ""
    target_node: str = ""
    target_pose: Pose2D = field(default_factory=lambda: Pose2D(0.0, 0.0, 0.0))
    state: str = "idle"
    distance_to_goal_m: float | None = None
    is_arrived: bool = False
    failure_reason: str = ""
    last_service_reply: str = ""
    last_status_code: int = 0

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True)
class LocalObstacleSummary:
    timestamp_ms: int
    frame_id: str = "base_link"
    range_m: float = 6.0
    front_clearance_m: float = 6.0
    left_clearance_m: float = 6.0
    right_clearance_m: float = 6.0
    rear_clearance_m: float = 6.0
    blocked_directions: tuple[str, ...] = ()
    narrow_passage: bool = False
    recommended_action: str = "normal"

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)

