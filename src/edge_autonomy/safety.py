from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

from .models import WorldState
from .slam_adapter import NavigationFeedback, NavigationStatus
from .slam_state import LocalObstacleSummary, LocalizationState, SlamHealth


class SupervisorAction(str, Enum):
    PASS_THROUGH = "pass_through"
    SLOW_DOWN = "slow_down"
    PAUSE = "pause"
    EMERGENCY_STOP = "emergency_stop"
    REQUEST_REPLAN = "request_replan"
    TAKEOVER = "takeover"


@dataclass(frozen=True)
class LinkQuality:
    bandwidth_kbps: float
    latency_ms: float
    packet_loss_ratio: float = 0.0


@dataclass(frozen=True)
class SafetyDecision:
    action: SupervisorAction
    reason: str
    speed_limit_scale: float = 1.0
    requires_human_ack: bool = False


class SafetySupervisor:
    def __init__(
        self,
        emergency_distance_m: float = 0.8,
        pause_distance_m: float = 1.5,
        weak_bandwidth_kbps: float = 200.0,
        weak_latency_ms: float = 500.0,
    ) -> None:
        self.emergency_distance_m = emergency_distance_m
        self.pause_distance_m = pause_distance_m
        self.weak_bandwidth_kbps = weak_bandwidth_kbps
        self.weak_latency_ms = weak_latency_ms

    def evaluate(
        self,
        world_state: WorldState,
        navigation_feedback: NavigationFeedback,
        link_quality: LinkQuality,
    ) -> SafetyDecision:
        if navigation_feedback.status == NavigationStatus.FAILED:
            return SafetyDecision(
                action=SupervisorAction.EMERGENCY_STOP,
                reason="navigation backend reported failure",
                speed_limit_scale=0.0,
                requires_human_ack=True,
            )

        for event in world_state.risk_events:
            if event.severity == "critical":
                if event.distance_m is None or event.distance_m <= self.emergency_distance_m:
                    return SafetyDecision(
                        action=SupervisorAction.EMERGENCY_STOP,
                        reason=f"critical risk: {event.description}",
                        speed_limit_scale=0.0,
                        requires_human_ack=True,
                    )

        for obj in world_state.objects:
            if not obj.traversable and obj.distance_m <= self.pause_distance_m:
                return SafetyDecision(
                    action=SupervisorAction.REQUEST_REPLAN,
                    reason=f"path blocked by {obj.category}",
                    speed_limit_scale=0.0,
                )

        for event in world_state.risk_events:
            if event.severity in {"high", "critical"}:
                if event.distance_m is not None and event.distance_m <= self.pause_distance_m:
                    return SafetyDecision(
                        action=SupervisorAction.PAUSE,
                        reason=f"nearby high risk: {event.description}",
                        speed_limit_scale=0.0,
                    )

        if (
            link_quality.bandwidth_kbps < self.weak_bandwidth_kbps
            or link_quality.latency_ms > self.weak_latency_ms
            or link_quality.packet_loss_ratio >= 0.2
        ):
            return SafetyDecision(
                action=SupervisorAction.SLOW_DOWN,
                reason="weak network detected, switching to conservative mode",
                speed_limit_scale=0.4,
            )

        if navigation_feedback.status == NavigationStatus.BLOCKED:
            return SafetyDecision(
                action=SupervisorAction.REQUEST_REPLAN,
                reason="navigation backend reported blocked state",
                speed_limit_scale=0.0,
            )

        return SafetyDecision(
            action=SupervisorAction.PASS_THROUGH,
            reason="world state is safe enough for continued execution",
            speed_limit_scale=1.0,
        )

    def evaluate_runtime(
        self,
        slam_health: SlamHealth,
        localization_state: LocalizationState,
        local_obstacle: LocalObstacleSummary,
    ) -> SafetyDecision:
        if slam_health.status == "failed" or not slam_health.slam_alive:
            return SafetyDecision(
                action=SupervisorAction.EMERGENCY_STOP,
                reason="slam health failed",
                speed_limit_scale=0.0,
                requires_human_ack=True,
            )

        if localization_state.status in {"lost", "not_started", "map_mismatch"}:
            return SafetyDecision(
                action=SupervisorAction.PAUSE,
                reason=f"localization is not valid: {localization_state.status}",
                speed_limit_scale=0.0,
                requires_human_ack=True,
            )

        if local_obstacle.front_clearance_m < self.emergency_distance_m:
            return SafetyDecision(
                action=SupervisorAction.EMERGENCY_STOP,
                reason="front obstacle too close",
                speed_limit_scale=0.0,
                requires_human_ack=True,
            )

        if local_obstacle.front_clearance_m < self.pause_distance_m or local_obstacle.recommended_action == "pause":
            return SafetyDecision(
                action=SupervisorAction.PAUSE,
                reason="front obstacle inside pause distance",
                speed_limit_scale=0.0,
            )

        if (
            slam_health.status == "degraded"
            or localization_state.status == "degraded"
            or local_obstacle.recommended_action == "go_slow"
        ):
            return SafetyDecision(
                action=SupervisorAction.SLOW_DOWN,
                reason="degraded localization or near obstacle",
                speed_limit_scale=0.4,
            )

        return SafetyDecision(
            action=SupervisorAction.PASS_THROUGH,
            reason="runtime state is safe enough for continued execution",
            speed_limit_scale=1.0,
        )
