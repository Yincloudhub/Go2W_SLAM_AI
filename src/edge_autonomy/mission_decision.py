from __future__ import annotations

import time
from typing import Any

from .task_queue import validate_task_queue


EXECUTION_CHAIN = [
    "task_queue",
    "mission_decision_engine",
    "slam_gateway",
    "unitree_sdk",
]

RECOVERY_ACTIONS = {"reposition", "hold"}
REPOSITION_DIRECTIONS = {"forward", "backward", "left", "right"}


def build_gateway_decision_record(
    gateway_state: dict[str, Any] | None,
    *,
    checked: bool,
    allowed: bool,
    reason: str,
) -> dict[str, Any]:
    state = gateway_state if isinstance(gateway_state, dict) else {}
    world = state.get("world_state") if isinstance(state.get("world_state"), dict) else {}
    safety = state.get("safety") if isinstance(state.get("safety"), dict) else {}
    if not safety and isinstance(world.get("safety"), dict):
        safety = world["safety"]

    sensor_age_ms: dict[str, Any] = {}
    localization = world.get("localization")
    if isinstance(localization, dict) and "pose_age_ms" in localization:
        sensor_age_ms["localization_pose"] = localization.get("pose_age_ms")
    obstacle = world.get("local_obstacle")
    if isinstance(obstacle, dict) and "age_ms" in obstacle:
        sensor_age_ms["local_obstacle"] = obstacle.get("age_ms")

    return {
        "authority": "slam_gateway",
        "checked": bool(checked),
        "accepted": state.get("accepted") if state else None,
        "decision": "allow" if checked and allowed else ("deny" if checked else "not_checked"),
        "reason": reason,
        "gateway_reason": state.get("reason"),
        "policy_version": safety.get("policy_version"),
        "recommended_mode": safety.get("recommended_mode"),
        "motion_direction": safety.get("motion_direction"),
        "sensor_age_ms": sensor_age_ms,
        "timestamp_ms": world.get("timestamp_ms"),
        "safety": safety or None,
    }


def build_mission_decision(
    task_queue: dict[str, Any] | None,
    *,
    execute_requested: bool,
    registry_allowed: bool,
    registry_reason: str,
    topology_allowed: bool,
    topology_reason: str,
    gateway_checked: bool,
    gateway_allowed: bool,
    gateway_reason: str,
    gateway_state: dict[str, Any] | None = None,
    timestamp_ms: int | None = None,
) -> dict[str, Any]:
    now_ms = int(timestamp_ms if timestamp_ms is not None else time.time() * 1000)
    queue_id = task_queue.get("queue_id") if isinstance(task_queue, dict) else None
    queue_valid = True
    queue_error = ""
    try:
        validate_task_queue(task_queue)  # type: ignore[arg-type]
    except (TypeError, ValueError) as exc:
        queue_valid = False
        queue_error = str(exc)

    actions = [
        str(step.get("action") or "")
        for step in (task_queue.get("steps", []) if isinstance(task_queue, dict) else [])
        if isinstance(step, dict)
    ]
    has_navigation = "navigate" in actions
    has_confirmation = "ask_confirm" in actions
    has_hold = "hold_position" in actions

    decision = "reject"
    reason_code = "invalid_task_queue"
    reason = queue_error or "planner did not produce a valid TaskQueue"
    motion_allowed = False

    if queue_valid and has_confirmation:
        decision = "await_confirmation"
        reason_code = "human_confirmation_required"
        reason = "TaskQueue requires human confirmation before any execution"
    elif queue_valid and has_hold and not has_navigation:
        decision = "hold"
        reason_code = "hold_position"
        reason = "TaskQueue requests hold_position"
    elif queue_valid and not has_navigation:
        decision = "reject"
        reason_code = "no_navigation_task"
        reason = "TaskQueue contains no executable navigation task"
    elif queue_valid and not execute_requested:
        decision = "dry_run_queue"
        reason_code = "dry_run"
        reason = "TaskQueue validated for preview; motion is not requested"
    elif queue_valid and not registry_allowed:
        decision = "reject"
        reason_code = "registry_blocked"
        reason = registry_reason
    elif queue_valid and not topology_allowed:
        decision = "reject"
        reason_code = "topology_blocked"
        reason = topology_reason
    elif queue_valid and not gateway_checked:
        decision = "reject"
        reason_code = "gateway_not_checked"
        reason = "real execution requires an authoritative Gateway preflight"
    elif queue_valid and not gateway_allowed:
        decision = "hold"
        reason_code = "gateway_blocked"
        reason = gateway_reason
    elif queue_valid:
        decision = "execute_queue"
        reason_code = "all_gates_passed"
        reason = "TaskQueue may enter the Python supervised executor"
        motion_allowed = True

    gateway_record = build_gateway_decision_record(
        gateway_state,
        checked=gateway_checked,
        allowed=gateway_allowed,
        reason=gateway_reason,
    )
    return {
        "schema_version": 1,
        "schema": "go2w_mission_decision_v1",
        "decision_id": f"decision_{now_ms}",
        "timestamp_ms": now_ms,
        "queue_id": queue_id,
        "queue_valid": queue_valid,
        "decision": decision,
        "reason_code": reason_code,
        "reason": reason,
        "execute_requested": bool(execute_requested),
        "motion_allowed": motion_allowed,
        "execution_owner": "python_persistent_supervisor",
        "execution_chain": EXECUTION_CHAIN,
        "llm_direct_motion": False,
        "gateway_final_authority": True,
        "preflight": {
            "registry": {"allowed": bool(registry_allowed), "reason": registry_reason},
            "topology": {"allowed": bool(topology_allowed), "reason": topology_reason},
            "gateway": gateway_record,
        },
    }


def build_recovery_decision(
    recovery_analysis: dict[str, Any],
    strategy_proposal: dict[str, Any],
    *,
    timestamp_ms: int | None = None,
) -> dict[str, Any]:
    now_ms = int(timestamp_ms if timestamp_ms is not None else time.time() * 1000)
    candidates = {
        str(item.get("direction")): item
        for item in recovery_analysis.get("candidates", [])
        if isinstance(item, dict)
        and str(item.get("direction")) in REPOSITION_DIRECTIONS
    }
    action = str(strategy_proposal.get("action") or "")
    direction = str(strategy_proposal.get("direction") or "")
    reason = str(strategy_proposal.get("reason") or "")
    accepted = False
    decision = "hold"
    reason_code = "invalid_recovery_strategy"
    authorized_command = None

    if action not in RECOVERY_ACTIONS:
        reason = reason or f"unsupported recovery action: {action or 'missing'}"
    elif action == "hold":
        accepted = True
        decision = "hold"
        reason_code = "strategy_requested_hold"
        reason = reason or "recovery strategy requested hold"
    elif direction not in candidates:
        reason_code = "reposition_direction_not_in_candidates"
        reason = f"reposition direction {direction!r} is not authorized"
    else:
        candidate = candidates[direction]
        try:
            requested_distance_m = float(strategy_proposal.get("distance_m"))
            max_distance_m = float(candidate.get("distance_m"))
        except (TypeError, ValueError):
            requested_distance_m = -1.0
            max_distance_m = -1.0
        if requested_distance_m < 0.20 or requested_distance_m > max_distance_m:
            reason_code = "reposition_distance_outside_candidate"
            reason = (
                f"requested reposition distance {requested_distance_m:.3f} "
                f"exceeds candidate range 0.20..{max_distance_m:.3f}"
            )
        else:
            accepted = True
            decision = "execute_reposition"
            reason_code = "bounded_reposition_authorized"
            reason = reason or "bounded reposition authorized"
            authorized_command = {
                "action": "supervised_reposition",
                "direction": direction,
                "distance_m": requested_distance_m,
                "speed_mps": 0.20,
                "operator_ack": True,
            }

    return {
        "schema_version": 1,
        "schema": "go2w_recovery_decision_v1",
        "decision_id": f"recovery_{now_ms}",
        "timestamp_ms": now_ms,
        "authority": "mission_decision_engine",
        "accepted": accepted,
        "decision": decision,
        "reason_code": reason_code,
        "reason": reason,
        "strategy_source": strategy_proposal.get("source"),
        "strategy_proposal": strategy_proposal,
        "recovery_translation_required": recovery_analysis.get("required") is True,
        "authorized_command": authorized_command,
        "gateway_final_authority": True,
        "llm_direct_motion": False,
    }
