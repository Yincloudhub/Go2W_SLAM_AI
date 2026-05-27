from __future__ import annotations

from dataclasses import asdict, dataclass, field
from typing import Any

from .models import Pose2D


@dataclass(frozen=True)
class CurrentPose:
    timestamp_ms: int
    map_id: str
    frame_id: str
    pose: Pose2D
    z: float = 0.0
    source: str = "rt/slam_info"

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True)
class LocalizationState:
    timestamp_ms: int
    map_id: str
    status: str
    confidence: float | None = None
    pose_age_ms: int | None = None
    lost_duration_ms: int = 0

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True)
class SlamHealth:
    timestamp_ms: int
    slam_alive: bool
    lidar_alive: bool
    imu_alive: bool
    odom_alive: bool
    localization_alive: bool
    last_pose_age_ms: int | None
    status: str

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True)
class NavigationTaskState:
    timestamp_ms: int
    state: str = "idle"
    task_id: str = ""
    target_node: str = ""
    target_pose: Pose2D = field(default_factory=lambda: Pose2D(0.0, 0.0, 0.0))
    distance_to_goal_m: float | None = None
    is_arrived: bool = False
    failure_reason: str = ""
    last_status_code: int | None = None
    last_service_reply: str = ""

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True)
class LocalObstacleSummary:
    timestamp_ms: int
    frame_id: str = "base_link"
    source: str = "lidar"
    range_m: float = 6.0
    front_clearance_m: float = 6.0
    left_clearance_m: float = 6.0
    right_clearance_m: float = 6.0
    rear_clearance_m: float = 6.0
    blocked_directions: list[str] = field(default_factory=list)
    narrow_passage: bool = False
    recommended_action: str = "normal"
    confidence: float | None = None
    stale: bool = False
    latency_ms: float | None = None

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)
