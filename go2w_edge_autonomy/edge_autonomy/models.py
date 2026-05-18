from __future__ import annotations

from dataclasses import asdict, dataclass, field
from typing import Any


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class MapReference:
    map_id: str
    frame_id: str = "map"
    version: str = "unknown"


@dataclass(frozen=True)
class SemanticObject:
    object_id: str
    category: str
    pose: Pose2D
    distance_m: float
    risk_level: str = "low"
    traversable: bool = True
    is_dynamic: bool = False
    velocity_mps: float = 0.0


@dataclass(frozen=True)
class RiskEvent:
    event_type: str
    severity: str
    description: str
    distance_m: float | None = None


@dataclass(frozen=True)
class RobotState:
    pose: Pose2D
    mode: str
    battery_percent: float
    localized: bool = True


@dataclass(frozen=True)
class NavigationGoal:
    goal_id: str
    target_pose: Pose2D
    max_linear_speed_mps: float = 0.8
    max_angular_speed_rps: float = 0.5
    safety_mode: str = "normal"


@dataclass
class WorldState:
    timestamp_ms: int
    frame_id: str
    robot: RobotState
    objects: list[SemanticObject] = field(default_factory=list)
    risk_events: list[RiskEvent] = field(default_factory=list)
    task_phase: str = "idle"
    map_reference: MapReference | None = None

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)
