from __future__ import annotations

from typing import Any


def _latest_llm_feedback(queue_execution: dict[str, Any] | None) -> dict[str, Any] | None:
    events = queue_execution.get("events", []) if isinstance(queue_execution, dict) else []
    if not isinstance(events, list):
        return None
    for event in reversed(events):
        if not isinstance(event, dict):
            continue
        feedback = event.get("llm_feedback_results", [])
        if isinstance(feedback, list) and feedback:
            latest = feedback[-1]
            return latest if isinstance(latest, dict) else None
    return None


def _latest_operator_feedback(queue_execution: dict[str, Any] | None) -> dict[str, Any] | None:
    events = queue_execution.get("events", []) if isinstance(queue_execution, dict) else []
    if not isinstance(events, list):
        return None
    for event in reversed(events):
        if not isinstance(event, dict):
            continue
        feedback = event.get("operator_feedback", [])
        if isinstance(feedback, list) and feedback:
            latest = feedback[-1]
            return latest if isinstance(latest, dict) else None
    return None


def _queue_targets(task_queue: dict[str, Any] | None) -> list[str]:
    targets = task_queue.get("targets", []) if isinstance(task_queue, dict) else []
    return [str(item) for item in targets if item] if isinstance(targets, list) else []


def build_operator_display_state(
    world_state: dict[str, Any],
    *,
    task_queue: dict[str, Any] | None = None,
    queue_execution: dict[str, Any] | None = None,
    user_command: str = "",
) -> dict[str, Any]:
    """Create the bounded display payload for the operator panel.

    The panel should render this at 1-2 Hz. It is intentionally separate from
    high-rate SLAM polling and from LLM planning so UI updates cannot block the
    robot execution loop.
    """

    llm_feedback = _latest_llm_feedback(queue_execution)
    operator_feedback = _latest_operator_feedback(queue_execution)
    targets = _queue_targets(task_queue)
    blocked_reason = queue_execution.get("blocked_reason", "") if isinstance(queue_execution, dict) else ""
    completed = bool(queue_execution.get("completed")) if isinstance(queue_execution, dict) else False
    task_phase = str(world_state.get("task_phase") or "idle")
    if completed:
        task_phase = "completed"
    elif blocked_reason:
        task_phase = "blocked"

    current_target = targets[0] if targets else str(world_state.get("current_node") or "")
    return {
        "schema_version": 1,
        "timestamp_ms": world_state.get("timestamp_ms"),
        "refresh_hz": 1.0,
        "screen": {
            "task_phase": task_phase,
            "user_command": user_command,
            "current_target": current_target,
            "targets": targets,
            "localized": bool(world_state.get("localized")),
            "map_loaded": bool(world_state.get("map_loaded")),
            "motion_allowed": bool(world_state.get("motion_allowed")),
            "obstacle_status": world_state.get("obstacle_status"),
            "front_clearance_m": world_state.get("front_clearance_m"),
            "network_level": world_state.get("network_level"),
            "safety_reason": world_state.get("source_health", {}).get("safety_reason", "")
            if isinstance(world_state.get("source_health"), dict)
            else "",
            "blocked_reason": blocked_reason,
            "llm_reply": llm_feedback.get("text", "") if isinstance(llm_feedback, dict) else "",
            "operator_reply": operator_feedback.get("text", "") if isinstance(operator_feedback, dict) else "",
        },
        "event_budget": {
            "ui_display_hz": world_state.get("refresh_policy", {}).get("ui_display_hz", {})
            if isinstance(world_state.get("refresh_policy"), dict)
            else {},
            "llm_feedback": world_state.get("refresh_policy", {}).get("llm_feedback", {})
            if isinstance(world_state.get("refresh_policy"), dict)
            else {},
        },
    }
