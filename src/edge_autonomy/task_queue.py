from __future__ import annotations

from typing import Any


QUEUE_MODES = {"sequential"}
QUEUE_STATUSES = {"planned", "running", "completed", "failed", "blocked"}
QUEUE_SOURCES = {"semantic_topology", "deterministic_cpp", "llm_fallback", "operator_panel", "scripted"}
QUEUE_ACTIONS = {
    "navigate",
    "wait_until",
    "capture_keyframe",
    "report",
    "speak",
    "ask_confirm",
    "hold_position",
    "set_communication_policy",
}
STEP_STATUSES = {"pending", "running", "ok", "failed", "blocked", "dry_run", "skipped"}
COMMUNICATION_MODES = {"normal", "semantic_only", "keyframe_low_rate", "hold_remote"}
COMMUNICATION_SEND = {
    "task_state",
    "mission_decision",
    "execution_state",
    "risk_events",
    "keyframe",
    "semantic_topology",
    "navigation_feedback",
    "world_state_summary",
}
COMMUNICATION_DROP = {"raw_video", "dense_pointcloud", "full_log", "high_rate_images"}


def _require(condition: bool, message: str) -> None:
    if not condition:
        raise ValueError(message)


def _non_empty_string(value: Any) -> bool:
    return isinstance(value, str) and bool(value)


def _validate_string_items(values: Any, allowed: set[str], label: str) -> None:
    _require(isinstance(values, list), f"{label} must be list")
    for item in values:
        _require(isinstance(item, str) and item in allowed, f"{label} contains invalid item: {item!r}")


def validate_communication_policy(policy: dict[str, Any]) -> None:
    _require(isinstance(policy, dict), "communication_policy must be object")
    _require(policy.get("mode") in COMMUNICATION_MODES, "communication_policy.mode is invalid")
    _validate_string_items(policy.get("send"), COMMUNICATION_SEND, "communication_policy.send")
    _validate_string_items(policy.get("drop"), COMMUNICATION_DROP, "communication_policy.drop")
    if "reason" in policy:
        _require(isinstance(policy["reason"], str), "communication_policy.reason must be string")


def task_step_id(step: dict[str, Any]) -> str:
    task_id = step.get("task_id")
    if _non_empty_string(task_id):
        return str(task_id)
    step_id = step.get("step_id")
    if _non_empty_string(step_id):
        return str(step_id)
    return ""


def validate_task_queue(task_queue: dict[str, Any]) -> None:
    _require(isinstance(task_queue, dict), "task_queue must be object")
    _require(_non_empty_string(task_queue.get("queue_id")), "queue_id must be non-empty string")
    _require(task_queue.get("mode") in QUEUE_MODES, "mode must be sequential")
    _require(task_queue.get("status") in QUEUE_STATUSES, "status is invalid")
    _require(task_queue.get("source") in QUEUE_SOURCES, "source is invalid")

    targets = task_queue.get("targets")
    _require(isinstance(targets, list), "targets must be list")
    for target in targets:
        _require(_non_empty_string(target), "targets contains invalid target")

    validate_communication_policy(task_queue.get("communication_policy", {}))

    steps = task_queue.get("steps")
    _require(isinstance(steps, list) and bool(steps), "steps must be non-empty list")
    _require(len(steps) <= 24, "steps exceeds max length 24")

    seen: set[str] = set()
    has_navigation = False
    for step in steps:
        _require(isinstance(step, dict), "step must be object")
        step_id = task_step_id(step)
        _require(bool(step_id), "step missing task_id")
        _require(step_id not in seen, f"duplicate task_id: {step_id}")
        seen.add(step_id)

        action = step.get("action")
        _require(action in QUEUE_ACTIONS, f"step action is invalid: {action!r}")
        if "status" in step:
            _require(step["status"] in STEP_STATUSES, "step status is invalid")

        if action == "navigate":
            has_navigation = True
            _require(_non_empty_string(step.get("target_node")), "navigate step requires target_node")
        elif action == "capture_keyframe":
            _require(_non_empty_string(step.get("target_node")), "capture_keyframe step requires target_node")
        elif action in {"report", "speak", "ask_confirm"}:
            _require(_non_empty_string(step.get("message")), f"{action} step requires message")

    if targets:
        _require(has_navigation, "targets present but no navigate step")
