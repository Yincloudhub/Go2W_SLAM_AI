from __future__ import annotations

import math
import re
import time
from dataclasses import asdict, dataclass, field
from typing import Any


FLOAT_RE = r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?"


def now_ms() -> int:
    return int(time.time() * 1000)


def quaternion_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def _float_after(label: str, text: str) -> float | None:
    match = re.search(rf"{re.escape(label)}:\s*({FLOAT_RE})", text)
    return float(match.group(1)) if match else None


def _int_after(label: str, text: str) -> int | None:
    value = _float_after(label, text)
    return int(value) if value is not None else None


@dataclass(frozen=True)
class ProcessState:
    unitree_slam: bool = False
    xt16_driver: bool = False
    slam_keyboard_client: bool = False
    slam_llm_command_client: bool = False
    raw: str = ""

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True)
class LidarStateSummary:
    alive: bool
    cloud_frequency_hz: float | None = None
    imu_frequency_hz: float | None = None
    cloud_packet_loss_rate: float | None = None
    cloud_size: int | None = None
    error_state: int | None = None
    source_topic: str = "/utlidar/lidar_state"

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True)
class PointCloudSummary:
    alive: bool
    topic: str
    frame_id: str = ""
    width: int | None = None
    height: int | None = None
    point_step: int | None = None
    row_step: int | None = None
    is_dense: bool | None = None

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True)
class OdomPoseSummary:
    alive: bool
    topic: str = "/unitree/slam_relocation/odom"
    frame_id: str = ""
    child_frame_id: str = ""
    x: float | None = None
    y: float | None = None
    z: float | None = None
    q_x: float | None = None
    q_y: float | None = None
    q_z: float | None = None
    q_w: float | None = None
    yaw: float | None = None

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True)
class SlamTextTopicSummary:
    topic: str
    alive: bool
    raw_data: str = ""

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True)
class SlamRuntimeSnapshot:
    timestamp_ms: int
    host: str
    expected_map_id: str = "unknown"
    expected_map_path: str = ""
    processes: ProcessState = field(default_factory=ProcessState)
    lidar_state: LidarStateSummary = field(default_factory=lambda: LidarStateSummary(alive=False))
    live_pointcloud: PointCloudSummary = field(default_factory=lambda: PointCloudSummary(alive=False, topic="/unitree/slam_lidar/points"))
    relocation_odom: OdomPoseSummary = field(default_factory=lambda: OdomPoseSummary(alive=False))
    slam_info: SlamTextTopicSummary = field(default_factory=lambda: SlamTextTopicSummary(topic="/slam_info", alive=False))
    slam_key_info: SlamTextTopicSummary = field(default_factory=lambda: SlamTextTopicSummary(topic="/slam_key_info", alive=False))

    @property
    def localization_status(self) -> str:
        if not self.processes.unitree_slam:
            return "slam_not_running"
        if not self.lidar_state.alive and not self.live_pointcloud.alive:
            return "lidar_not_confirmed"
        if self.relocation_odom.alive:
            return "localized_or_tracking"
        return "relocation_odom_missing"

    @property
    def health_status(self) -> str:
        if self.processes.unitree_slam and self.processes.xt16_driver and self.live_pointcloud.alive and self.relocation_odom.alive:
            return "ok"
        if self.processes.unitree_slam and (self.live_pointcloud.alive or self.lidar_state.alive):
            return "degraded"
        return "failed"

    def to_dict(self) -> dict[str, Any]:
        data = asdict(self)
        data["health_status"] = self.health_status
        data["localization_status"] = self.localization_status
        return data


def parse_process_state(text: str) -> ProcessState:
    return ProcessState(
        unitree_slam=bool(re.search(r"(^|\s)(?:\./)?unitree_slam(\s|$)", text, re.MULTILINE)),
        xt16_driver=bool(re.search(r"(^|\s)(?:\./)?xt16_driver(\s|$)", text, re.MULTILINE)),
        slam_keyboard_client=bool(re.search(r"slam_keyboard_(?:client|c)", text)),
        slam_llm_command_client=bool(re.search(r"slam_llm_(?:command_client|comman)", text)),
        raw=text.strip(),
    )


def parse_lidar_state(text: str) -> LidarStateSummary:
    return LidarStateSummary(
        alive=bool(text.strip()),
        cloud_frequency_hz=_float_after("cloud_frequency", text),
        imu_frequency_hz=_float_after("imu_frequency", text),
        cloud_packet_loss_rate=_float_after("cloud_packet_loss_rate", text),
        cloud_size=_int_after("cloud_size", text),
        error_state=_int_after("error_state", text),
    )


def parse_pointcloud_summary(text: str, *, topic: str = "/unitree/slam_lidar/points") -> PointCloudSummary:
    frame_match = re.search(r"frame_id:\s*([^\n]+)", text)
    dense_match = re.search(r"is_dense:\s*(true|false|True|False)", text)
    return PointCloudSummary(
        alive=bool(text.strip()),
        topic=topic,
        frame_id=frame_match.group(1).strip().strip("'\"") if frame_match else "",
        width=_int_after("width", text),
        height=_int_after("height", text),
        point_step=_int_after("point_step", text),
        row_step=_int_after("row_step", text),
        is_dense=(dense_match.group(1).lower() == "true") if dense_match else None,
    )


def parse_odom_pose(text: str, *, topic: str = "/unitree/slam_relocation/odom") -> OdomPoseSummary:
    frame_match = re.search(r"frame_id:\s*([^\n]+)", text)
    child_match = re.search(r"child_frame_id:\s*([^\n]+)", text)
    position_match = re.search(
        rf"position:\s*\n\s*x:\s*({FLOAT_RE})\s*\n\s*y:\s*({FLOAT_RE})\s*\n\s*z:\s*({FLOAT_RE})",
        text,
    )
    orientation_match = re.search(
        rf"orientation:\s*\n\s*x:\s*({FLOAT_RE})\s*\n\s*y:\s*({FLOAT_RE})\s*\n\s*z:\s*({FLOAT_RE})\s*\n\s*w:\s*({FLOAT_RE})",
        text,
    )

    q_x = q_y = q_z = q_w = yaw = None
    if orientation_match:
        q_x = float(orientation_match.group(1))
        q_y = float(orientation_match.group(2))
        q_z = float(orientation_match.group(3))
        q_w = float(orientation_match.group(4))
        yaw = quaternion_to_yaw(q_x, q_y, q_z, q_w)

    return OdomPoseSummary(
        alive=bool(text.strip()),
        topic=topic,
        frame_id=frame_match.group(1).strip().strip("'\"") if frame_match else "",
        child_frame_id=child_match.group(1).strip().strip("'\"") if child_match else "",
        x=float(position_match.group(1)) if position_match else None,
        y=float(position_match.group(2)) if position_match else None,
        z=float(position_match.group(3)) if position_match else None,
        q_x=q_x,
        q_y=q_y,
        q_z=q_z,
        q_w=q_w,
        yaw=yaw,
    )


def parse_slam_text_topic(text: str, *, topic: str) -> SlamTextTopicSummary:
    data_match = re.search(r"data:\s*(.+)", text)
    return SlamTextTopicSummary(
        topic=topic,
        alive=bool(text.strip()),
        raw_data=data_match.group(1).strip().strip("'\"") if data_match else text.strip(),
    )


def build_runtime_snapshot(
    sections: dict[str, str],
    *,
    host: str,
    expected_map_id: str = "unknown",
    expected_map_path: str = "",
    timestamp_ms: int | None = None,
) -> SlamRuntimeSnapshot:
    return SlamRuntimeSnapshot(
        timestamp_ms=timestamp_ms if timestamp_ms is not None else now_ms(),
        host=host,
        expected_map_id=expected_map_id,
        expected_map_path=expected_map_path,
        processes=parse_process_state(sections.get("processes", "")),
        lidar_state=parse_lidar_state(sections.get("lidar_state", "")),
        live_pointcloud=parse_pointcloud_summary(sections.get("live_pointcloud", "")),
        relocation_odom=parse_odom_pose(sections.get("relocation_odom", "")),
        slam_info=parse_slam_text_topic(sections.get("slam_info", ""), topic="/slam_info"),
        slam_key_info=parse_slam_text_topic(sections.get("slam_key_info", ""), topic="/slam_key_info"),
    )
