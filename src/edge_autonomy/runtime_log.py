from __future__ import annotations

import time
from typing import Any


def build_runtime_log_record(
    *,
    world_state: dict[str, Any],
    operator_display: dict[str, Any] | None = None,
    task_queue: dict[str, Any] | None = None,
    mission_decision: dict[str, Any] | None = None,
    queue_execution: dict[str, Any] | None = None,
    user_command: str = "",
    llm_result: dict[str, Any] | None = None,
) -> dict[str, Any]:
    """Build one bounded JSONL record for replay, evaluation, and later SFT.

    The record stores semantic state and decisions. It should not contain raw
    video, dense point clouds, or long high-rate logs.
    """

    return {
        "schema_version": 1,
        "timestamp_ms": int(world_state.get("timestamp_ms") or time.time() * 1000),
        "user_command": user_command,
        "world_state": world_state,
        "operator_display": operator_display,
        "task_queue": task_queue,
        "mission_decision": mission_decision,
        "queue_execution": queue_execution,
        "llm_result": llm_result,
        "artifact_policy": {
            "allow_raw_video": False,
            "allow_dense_pointcloud": False,
            "allow_keyframe": True,
            "allow_semantic_summary": True,
        },
    }
