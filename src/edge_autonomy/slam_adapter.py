from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass
from enum import Enum

from .models import MapReference, NavigationGoal, Pose2D, RobotState
from .slam_health_monitor import SlamHealthMonitor
from .slam_state import CurrentPose, LocalObstacleSummary, LocalizationState, NavigationTaskState, SlamHealth
from .slam_topics import now_ms, parse_slam_ctrl_info, parse_slam_info, parse_slam_key_info


class NavigationStatus(str, Enum):
    IDLE = "idle"
    NAVIGATING = "navigating"
    PAUSED = "paused"
    GOAL_REACHED = "goal_reached"
    BLOCKED = "blocked"
    FAILED = "failed"
    CANCELED = "canceled"


@dataclass(frozen=True)
class NavigationFeedback:
    status: NavigationStatus
    pose: Pose2D
    distance_to_goal_m: float | None = None
    message: str = ""


class SlamNavigationAdapter(ABC):
    @abstractmethod
    def get_robot_state(self) -> RobotState:
        raise NotImplementedError

    @abstractmethod
    def get_map_reference(self) -> MapReference:
        raise NotImplementedError

    @abstractmethod
    def get_localization_state(self) -> LocalizationState:
        raise NotImplementedError

    @abstractmethod
    def get_slam_health(self) -> SlamHealth:
        raise NotImplementedError

    @abstractmethod
    def get_navigation_task_state(self) -> NavigationTaskState:
        raise NotImplementedError

    @abstractmethod
    def get_local_obstacle_summary(self) -> LocalObstacleSummary:
        raise NotImplementedError

    @abstractmethod
    def submit_navigation_goal(self, goal: NavigationGoal) -> None:
        raise NotImplementedError

    @abstractmethod
    def pause_navigation(self) -> None:
        raise NotImplementedError

    @abstractmethod
    def resume_navigation(self) -> None:
        raise NotImplementedError

    @abstractmethod
    def cancel_navigation(self) -> None:
        raise NotImplementedError

    @abstractmethod
    def get_feedback(self) -> NavigationFeedback:
        raise NotImplementedError


class InMemorySlamAdapter(SlamNavigationAdapter):
    def __init__(self, initial_pose: Pose2D | None = None, *, map_id: str = "debug-map") -> None:
        pose = initial_pose or Pose2D(0.0, 0.0, 0.0)
        self._robot_state = RobotState(pose=pose, mode="standby", battery_percent=100.0, localized=True)
        self._map_reference = MapReference(map_id=map_id)
        self._feedback = NavigationFeedback(status=NavigationStatus.IDLE, pose=pose, distance_to_goal_m=None)
        self._last_goal: NavigationGoal | None = None
        self._health_monitor = SlamHealthMonitor(map_id=map_id)
        self._health_monitor.observe_pose(CurrentPose(timestamp_ms=now_ms(), map_id=map_id, frame_id="map", pose=pose, source="in_memory"))
        self._navigation_task_state = NavigationTaskState(timestamp_ms=now_ms())
        self._local_obstacle_summary = LocalObstacleSummary(timestamp_ms=now_ms())

    def get_robot_state(self) -> RobotState:
        localized = self.get_localization_state().status in {"localized", "degraded"}
        self._robot_state = RobotState(
            pose=self._robot_state.pose,
            mode=self._robot_state.mode,
            battery_percent=self._robot_state.battery_percent,
            localized=localized,
        )
        return self._robot_state

    def get_map_reference(self) -> MapReference:
        return self._map_reference

    def get_localization_state(self) -> LocalizationState:
        return self._health_monitor.get_localization_state()

    def get_slam_health(self) -> SlamHealth:
        return self._health_monitor.get_slam_health()

    def get_navigation_task_state(self) -> NavigationTaskState:
        return self._navigation_task_state

    def get_local_obstacle_summary(self) -> LocalObstacleSummary:
        return self._local_obstacle_summary

    def set_local_obstacle_summary(self, summary: LocalObstacleSummary) -> None:
        self._local_obstacle_summary = summary

    def submit_navigation_goal(self, goal: NavigationGoal) -> None:
        self._last_goal = goal
        distance_to_goal_m = self._distance_to(goal.target_pose)
        self._feedback = NavigationFeedback(
            status=NavigationStatus.NAVIGATING,
            pose=self._robot_state.pose,
            distance_to_goal_m=distance_to_goal_m,
            message=f"goal {goal.goal_id} accepted",
        )
        self._navigation_task_state = NavigationTaskState(
            timestamp_ms=now_ms(),
            task_id=goal.goal_id,
            target_node=goal.goal_id,
            target_pose=goal.target_pose,
            state="running",
            distance_to_goal_m=distance_to_goal_m,
            is_arrived=False,
        )

    def pause_navigation(self) -> None:
        self._feedback = NavigationFeedback(
            status=NavigationStatus.PAUSED,
            pose=self._robot_state.pose,
            distance_to_goal_m=self._feedback.distance_to_goal_m,
            message="navigation paused",
        )
        self._navigation_task_state = NavigationTaskState(
            timestamp_ms=now_ms(),
            task_id=self._navigation_task_state.task_id,
            target_node=self._navigation_task_state.target_node,
            target_pose=self._navigation_task_state.target_pose,
            state="paused",
            distance_to_goal_m=self._navigation_task_state.distance_to_goal_m,
            is_arrived=False,
        )

    def resume_navigation(self) -> None:
        status = NavigationStatus.NAVIGATING if self._last_goal else NavigationStatus.IDLE
        state = "running" if self._last_goal else "idle"
        self._feedback = NavigationFeedback(
            status=status,
            pose=self._robot_state.pose,
            distance_to_goal_m=self._feedback.distance_to_goal_m,
            message="navigation resumed" if self._last_goal else "no navigation goal to resume",
        )
        self._navigation_task_state = NavigationTaskState(
            timestamp_ms=now_ms(),
            task_id=self._navigation_task_state.task_id,
            target_node=self._navigation_task_state.target_node,
            target_pose=self._navigation_task_state.target_pose,
            state=state,
            distance_to_goal_m=self._navigation_task_state.distance_to_goal_m,
            is_arrived=False,
        )

    def cancel_navigation(self) -> None:
        self._feedback = NavigationFeedback(
            status=NavigationStatus.CANCELED,
            pose=self._robot_state.pose,
            distance_to_goal_m=self._feedback.distance_to_goal_m,
            message="navigation canceled",
        )
        self._navigation_task_state = NavigationTaskState(
            timestamp_ms=now_ms(),
            task_id=self._navigation_task_state.task_id,
            target_node=self._navigation_task_state.target_node,
            target_pose=self._navigation_task_state.target_pose,
            state="cancelled",
            distance_to_goal_m=self._navigation_task_state.distance_to_goal_m,
            is_arrived=False,
        )

    def get_feedback(self) -> NavigationFeedback:
        return self._feedback

    def _distance_to(self, target_pose: Pose2D) -> float:
        return ((target_pose.x - self._robot_state.pose.x) ** 2 + (target_pose.y - self._robot_state.pose.y) ** 2) ** 0.5

    def _apply_current_pose(self, pose: CurrentPose) -> None:
        self._map_reference = MapReference(map_id=pose.map_id, frame_id=pose.frame_id)
        self._robot_state = RobotState(
            pose=pose.pose,
            mode=self._robot_state.mode,
            battery_percent=self._robot_state.battery_percent,
            localized=True,
        )
        self._health_monitor.observe_pose(pose)
        if self._last_goal is not None:
            distance_to_goal_m = self._distance_to(self._last_goal.target_pose)
            self._feedback = NavigationFeedback(
                status=self._feedback.status,
                pose=pose.pose,
                distance_to_goal_m=distance_to_goal_m,
                message=self._feedback.message,
            )
            self._navigation_task_state = NavigationTaskState(
                timestamp_ms=pose.timestamp_ms,
                task_id=self._navigation_task_state.task_id,
                target_node=self._navigation_task_state.target_node,
                target_pose=self._navigation_task_state.target_pose,
                state=self._navigation_task_state.state,
                distance_to_goal_m=distance_to_goal_m,
                is_arrived=self._navigation_task_state.is_arrived,
                failure_reason=self._navigation_task_state.failure_reason,
                last_status_code=self._navigation_task_state.last_status_code,
                last_service_reply=self._navigation_task_state.last_service_reply,
            )


class ReplaySlamAdapter(InMemorySlamAdapter):
    """Offline adapter for replaying captured Unitree SLAM JSON samples."""

    def ingest_slam_info(self, raw: str, *, timestamp_ms: int | None = None, map_id: str | None = None) -> CurrentPose | None:
        pose = parse_slam_info(raw, timestamp_ms=timestamp_ms, map_id=map_id or self.get_map_reference().map_id)
        if pose is not None:
            self._apply_current_pose(pose)
        return pose

    def ingest_slam_key_info(self, raw: str, *, timestamp_ms: int | None = None) -> NavigationTaskState | None:
        state = parse_slam_key_info(raw, timestamp_ms=timestamp_ms, current_state=self._navigation_task_state)
        return self._apply_navigation_task_state(state)

    def ingest_slam_ctrl_info(self, raw: str, *, timestamp_ms: int | None = None) -> NavigationTaskState | None:
        state = parse_slam_ctrl_info(raw, timestamp_ms=timestamp_ms, current_state=self._navigation_task_state)
        return self._apply_navigation_task_state(state)

    def _apply_navigation_task_state(self, state: NavigationTaskState | None) -> NavigationTaskState | None:
        if state is None:
            return None

        self._navigation_task_state = state
        if state.state == "arrived":
            self._feedback = NavigationFeedback(
                status=NavigationStatus.GOAL_REACHED,
                pose=self._robot_state.pose,
                distance_to_goal_m=0.0,
                message=f"goal {state.target_node or state.task_id} reached",
            )
        elif state.state == "failed":
            self._feedback = NavigationFeedback(
                status=NavigationStatus.FAILED,
                pose=self._robot_state.pose,
                distance_to_goal_m=self._feedback.distance_to_goal_m,
                message=state.failure_reason,
            )
        return state
