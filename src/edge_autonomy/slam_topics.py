from __future__ import annotations

import json
import math
import time
from dataclasses import replace
from typing import Any

from .models import Pose2D
from .slam_state import CurrentPose, NavigationTaskState


class SlamTopicParseError(ValueError):
    pass


def now_ms() -> int:
    return int(time.time() * 1000)


def quaternion_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def _load_json(raw: str | dict[str, Any]) -> dict[str, Any]:
    if isinstance(raw, dict):
        return raw
    try:
        data = json.loads(raw)
    except json.JSONDecodeError as exc:
        raise SlamTopicParseError(f"invalid JSON: {exc}") from exc
    if not isinstance(data, dict):
        raise SlamTopicParseError("topic payload must be a JSON object")
    return data


def parse_slam_info(
    raw: str | dict[str, Any],
    *,
    timestamp_ms: int | None = None,
    map_id: str = "unknown",
    frame_id: str = "map",
) -> CurrentPose | None:
    data = _load_json(raw)
    if data.get("errorCode", 0) != 0:
        return None
    if data.get("type") != "pos_info":
        return None

    try:
        current_pose = data["data"]["currentPose"]
    except KeyError as exc:
        raise SlamTopicParseError("pos_info is missing data.currentPose") from exc
    if not isinstance(current_pose, dict):
        raise SlamTopicParseError("pos_info data.currentPose must be an object")

    values: dict[str, float] = {}
    for key in ("x", "y", "z", "q_x", "q_y", "q_z", "q_w"):
        if key not in current_pose:
            raise SlamTopicParseError(f"pos_info currentPose is missing {key}")
        try:
            value = float(current_pose[key])
        except (TypeError, ValueError) as exc:
            raise SlamTopicParseError(f"pos_info currentPose.{key} must be numeric") from exc
        if not math.isfinite(value):
            raise SlamTopicParseError(f"pos_info currentPose.{key} must be finite")
        values[key] = value
    quaternion_norm = math.sqrt(sum(values[key] ** 2 for key in ("q_x", "q_y", "q_z", "q_w")))
    if not 0.5 <= quaternion_norm <= 1.5:
        raise SlamTopicParseError("pos_info quaternion norm is invalid")
    return CurrentPose(
        timestamp_ms=timestamp_ms if timestamp_ms is not None else now_ms(),
        map_id=map_id,
        frame_id=frame_id,
        pose=Pose2D(
            x=values["x"],
            y=values["y"],
            yaw=quaternion_to_yaw(values["q_x"], values["q_y"], values["q_z"], values["q_w"]),
        ),
        z=values["z"],
        source="rt/slam_info",
    )


def parse_slam_key_info(
    raw: str | dict[str, Any],
    *,
    timestamp_ms: int | None = None,
    current_state: NavigationTaskState | None = None,
) -> NavigationTaskState | None:
    data = _load_json(raw)
    ts = timestamp_ms if timestamp_ms is not None else now_ms()
    base = current_state or NavigationTaskState(timestamp_ms=ts)

    if data.get("errorCode", 0) != 0:
        return replace(
            base,
            timestamp_ms=ts,
            state="failed",
            failure_reason=str(data.get("info", "slam_key_info_error")),
            last_service_reply=json.dumps(data, ensure_ascii=False),
        )

    if data.get("type") != "task_result":
        return None

    task_data = data.get("data", {})
    arrived = bool(task_data.get("is_arrived", False))
    target_node = str(task_data.get("targetNodeName", base.target_node))
    return replace(
        base,
        timestamp_ms=ts,
        target_node=target_node,
        state="arrived" if arrived else "failed",
        is_arrived=arrived,
        failure_reason="" if arrived else "task_result_not_arrived",
        last_service_reply=json.dumps(data, ensure_ascii=False),
    )


def parse_slam_ctrl_info(
    raw: str | dict[str, Any],
    *,
    timestamp_ms: int | None = None,
    current_state: NavigationTaskState | None = None,
) -> NavigationTaskState | None:
    data = _load_json(raw)
    ts = timestamp_ms if timestamp_ms is not None else now_ms()
    base = current_state or NavigationTaskState(timestamp_ms=ts)

    if data.get("errorCode", 0) != 0:
        return replace(
            base,
            timestamp_ms=ts,
            state="failed",
            failure_reason=str(data.get("info", "slam_ctrl_info_error")),
            last_service_reply=json.dumps(data, ensure_ascii=False),
        )

    if data.get("type") != "ctrl_info":
        return None

    ctrl_data = data.get("data", {})
    target_pose_data = ctrl_data.get("targetPose", {})
    target_pose = base.target_pose
    if isinstance(target_pose_data, dict):
        target_pose = Pose2D(
            x=float(target_pose_data.get("x", base.target_pose.x)),
            y=float(target_pose_data.get("y", base.target_pose.y)),
            yaw=float(target_pose_data.get("yaw", base.target_pose.yaw)),
        )

    state_machine = ctrl_data.get("stateMachine", {})
    backend_state = str(state_machine.get("state", "")).upper() if isinstance(state_machine, dict) else ""
    arrived = bool(ctrl_data.get("is_arrived", False))
    if arrived or backend_state == "FINISHED":
        state = "arrived"
        failure_reason = ""
    elif backend_state in {"PAUSE", "PAUSED"}:
        state = "paused"
        failure_reason = ""
    elif backend_state in {"FAILED", "ERROR"}:
        state = "failed"
        failure_reason = str(data.get("info", "ctrl_info_failed"))
    else:
        state = "running"
        failure_reason = ""

    return replace(
        base,
        timestamp_ms=ts,
        target_node=str(ctrl_data.get("targetNodeName", base.target_node)),
        target_pose=target_pose,
        state=state,
        is_arrived=arrived,
        failure_reason=failure_reason,
        last_service_reply=json.dumps(data, ensure_ascii=False),
    )
