#!/usr/bin/env python3
"""Preflight and record synchronized GO2W sensor topics into one ROS 2 bag."""

from __future__ import annotations

import argparse
import json
import os
import re
import signal
import subprocess
import sys
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Callable, Iterable


@dataclass(frozen=True)
class TopicRole:
    name: str
    candidates: tuple[str, ...]
    required: bool = True


TOPIC_ROLES = (
    TopicRole("front_camera", ("/frontvideostream",)),
    TopicRole("pointcloud", ("/utlidar/cloud", "/utlidar/cloud_deskewed")),
    TopicRole("imu", ("/utlidar/imu",)),
    TopicRole(
        "odometry",
        (
            "/utlidar/robot_odom",
            "/uslam/localization/odom",
            "/lio_sam_ros2/mapping/odometry",
            "/uslam/frontend/odom",
        ),
    ),
    TopicRole(
        "attitude",
        (
            "/utlidar/robot_pose",
            "/sportmodestate",
            "/lf/sportmodestate",
        ),
    ),
)

EXTRA_STATE_TOPICS = ("/sportmodestate", "/lf/sportmodestate")
TOPIC_LINE = re.compile(r"^(?P<topic>/\S+)\s+\[(?P<type>[^\]]+)\]\s*$")
ROLE_LABELS_ZH = {
    "front_camera": "前视相机",
    "pointcloud": "点云",
    "imu": "IMU",
    "odometry": "里程计",
    "attitude": "姿态",
}


def parse_topic_list(text: str) -> dict[str, str]:
    topics: dict[str, str] = {}
    for raw_line in text.splitlines():
        match = TOPIC_LINE.match(raw_line.strip())
        if match:
            topics[match.group("topic")] = match.group("type")
    return topics


def list_topics() -> dict[str, str]:
    completed = subprocess.run(
        ["ros2", "topic", "list", "-t"],
        check=True,
        text=True,
        capture_output=True,
    )
    return parse_topic_list(completed.stdout)


def topic_has_sample(topic: str, timeout_seconds: float) -> bool:
    timeout_value = max(1, int(round(timeout_seconds)))
    completed = subprocess.run(
        [
            "timeout",
            "--signal=INT",
            f"{timeout_value}s",
            "ros2",
            "topic",
            "echo",
            "--once",
            topic,
        ],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        check=False,
    )
    return completed.returncode == 0


def select_topics(
    available: dict[str, str],
    has_sample: Callable[[str], bool],
    allow_missing_state: bool,
) -> tuple[dict[str, str], list[dict[str, object]], list[str]]:
    selected: dict[str, str] = {}
    checks: list[dict[str, object]] = []
    missing_roles: list[str] = []
    for role in TOPIC_ROLES:
        role_required = role.required
        if allow_missing_state and role.name in {"odometry", "attitude"}:
            role_required = False

        selected_topic = ""
        for topic in role.candidates:
            topic_type = available.get(topic)
            if not topic_type:
                checks.append(
                    {
                        "role": role.name,
                        "topic": topic,
                        "type": "",
                        "listed": False,
                        "sample_received": False,
                    }
                )
                continue
            sample_received = has_sample(topic)
            checks.append(
                {
                    "role": role.name,
                    "topic": topic,
                    "type": topic_type,
                    "listed": True,
                    "sample_received": sample_received,
                }
            )
            if sample_received:
                selected_topic = topic
                break

        if selected_topic:
            selected[role.name] = selected_topic
        elif role_required:
            missing_roles.append(role.name)
    return selected, checks, missing_roles


def build_record_topics(
    selected: dict[str, str],
    available: dict[str, str],
    has_sample: Callable[[str], bool],
) -> list[str]:
    topics = list(dict.fromkeys(selected.values()))
    for topic in EXTRA_STATE_TOPICS:
        if topic not in topics and topic in available and has_sample(topic):
            topics.append(topic)
    return topics


def build_bag_command(output: Path, topics: Iterable[str]) -> list[str]:
    return [
        "ros2",
        "bag",
        "record",
        "-s",
        "sqlite3",
        "-o",
        str(output),
        *topics,
    ]


def read_message_counts(metadata_path: Path) -> dict[str, int]:
    try:
        import yaml
    except ImportError:
        yaml = None

    text = metadata_path.read_text(encoding="utf-8")
    if yaml is not None:
        metadata = yaml.safe_load(text)
        info = metadata.get("rosbag2_bagfile_information", {})
        counts: dict[str, int] = {}
        for entry in info.get("topics_with_message_count", []):
            topic_metadata = entry.get("topic_metadata", {})
            name = topic_metadata.get("name")
            if name:
                counts[name] = int(entry.get("message_count", 0))
        return counts

    counts = {}
    current_topic = ""
    for raw_line in text.splitlines():
        line = raw_line.strip()
        if line.startswith("name:"):
            value = line.split(":", 1)[1].strip().strip("'\"")
            if value.startswith("/"):
                current_topic = value
        elif current_topic and line.startswith("message_count:"):
            counts[current_topic] = int(line.split(":", 1)[1].strip())
            current_topic = ""
    return counts


def stop_process_group(process: subprocess.Popen[bytes], timeout_seconds: float = 20.0) -> None:
    if process.poll() is not None:
        return
    os.killpg(process.pid, signal.SIGINT)
    try:
        process.wait(timeout=timeout_seconds)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGTERM)
        process.wait(timeout=5)


def write_json(path: Path, data: dict[str, object]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def record(args: argparse.Namespace) -> int:
    started_at = datetime.now().astimezone()
    status_path = args.status_path.expanduser().resolve()
    live_checks: dict[str, bool] = {}

    def publish_status(
        *,
        phase: str,
        severity: str,
        message_zh: str,
        selected_roles: dict[str, str] | None = None,
        missing_roles: list[str] | None = None,
        output: Path | None = None,
        message_counts: dict[str, int] | None = None,
    ) -> None:
        selected_roles = selected_roles or {}
        missing_roles = missing_roles or []
        sensors = {}
        for role in TOPIC_ROLES:
            topic = selected_roles.get(role.name, "")
            sensors[role.name] = {
                "label_zh": ROLE_LABELS_ZH[role.name],
                "status": "online" if topic else ("offline" if role.name in missing_roles else "checking"),
                "status_zh": "在线" if topic else ("无数据" if role.name in missing_roles else "检查中"),
                "topic": topic,
            }
        payload: dict[str, object] = {
            "schema_version": 1,
            "timestamp_ms": int(time.time() * 1000),
            "available": True,
            "phase": phase,
            "severity": severity,
            "message_zh": message_zh,
            "name": args.name,
            "duration_seconds": args.duration,
            "sensors": sensors,
            "selected_roles": selected_roles,
            "missing_roles": missing_roles,
            "missing_labels_zh": [ROLE_LABELS_ZH[name] for name in missing_roles],
            "live_checks": live_checks,
            "output": str(output) if output else "",
        }
        if message_counts is not None:
            payload["message_counts"] = message_counts
        write_json(status_path, payload)

    publish_status(
        phase="preflight",
        severity="info",
        message_zh="正在检查前视相机、点云、IMU、里程计和姿态数据。",
    )
    available = list_topics()
    sample_cache: dict[str, bool] = {}

    def cached_sample_check(topic: str) -> bool:
        if topic not in sample_cache:
            print(f"[预检] 等待数据: {topic}", flush=True)
            sample_cache[topic] = topic_has_sample(topic, args.preflight_timeout)
            live_checks[topic] = sample_cache[topic]
            status = "在线" if sample_cache[topic] else "无数据"
            print(f"[预检] {topic}: {status}", flush=True)
            publish_status(
                phase="preflight",
                severity="info",
                message_zh=f"正在检查采集数据，{topic}：{status}。",
            )
        return sample_cache[topic]

    selected, checks, missing_roles = select_topics(
        available,
        cached_sample_check,
        args.allow_missing_state,
    )
    topics = build_record_topics(selected, available, cached_sample_check)

    timestamp = started_at.strftime("%Y%m%d_%H%M%S")
    output = args.output_root.expanduser().resolve() / f"go2w_multisensor_{timestamp}_{args.name}"
    command = build_bag_command(output, topics)
    manifest_path = output.with_name(f"{output.name}_capture_manifest.json")
    output.parent.mkdir(parents=True, exist_ok=True)
    missing_text = "、".join(ROLE_LABELS_ZH[name] for name in missing_roles)
    if topics:
        if missing_roles:
            publish_status(
                phase="recording",
                severity="warning",
                message_zh=f"{missing_text}当前无数据，继续录制其余在线数据。",
                selected_roles=selected,
                missing_roles=missing_roles,
                output=output,
            )
        else:
            publish_status(
                phase="recording",
                severity="ok",
                message_zh="五类数据均在线，开始同步录制。",
                selected_roles=selected,
                output=output,
            )
    else:
        publish_status(
            phase="no_data",
            severity="error",
            message_zh="当前没有可录制的传感器数据，请检查 ROS 2 驱动和 DDS 网络。",
            missing_roles=missing_roles,
            output=output,
        )
        print("[采集] 当前没有在线 topic，未创建空 bag。", flush=True)
        return 0

    preflight_manifest: dict[str, object] = {
        "status": "preflight_ok" if not missing_roles else "preflight_degraded",
        "name": args.name,
        "started_at": started_at.isoformat(),
        "duration_seconds": args.duration,
        "selected_roles": selected,
        "missing_roles": missing_roles,
        "record_topics": topics,
        "checks": checks,
        "output": str(output),
        "command": command,
    }
    write_json(manifest_path, preflight_manifest)

    print("[采集] " + " ".join(command), flush=True)
    if args.dry_run:
        publish_status(
            phase="dry_run_complete",
            severity="warning" if missing_roles else "ok",
            message_zh="采集预演完成，未实际写入 rosbag。",
            selected_roles=selected,
            missing_roles=missing_roles,
            output=output,
        )
        print(f"[采集] 预演清单: {manifest_path}", flush=True)
        return 0

    process = subprocess.Popen(command, start_new_session=True)
    record_error = ""
    try:
        deadline = time.monotonic() + args.duration
        while time.monotonic() < deadline:
            if process.poll() is not None:
                raise RuntimeError(f"ros2 bag record exited early with code {process.returncode}")
            time.sleep(min(0.25, max(0.0, deadline - time.monotonic())))
    except BaseException as exc:
        record_error = str(exc)
        raise
    finally:
        stop_process_group(process)
        preflight_manifest["finished_at"] = datetime.now().astimezone().isoformat()
        preflight_manifest["rosbag_returncode"] = process.returncode
        if record_error:
            preflight_manifest["record_error"] = record_error
        write_json(manifest_path, preflight_manifest)

    metadata_path = output / "metadata.yaml"
    if not metadata_path.exists():
        raise RuntimeError(f"rosbag metadata is missing: {metadata_path}")
    message_counts = read_message_counts(metadata_path)
    empty_topics = [topic for topic in topics if message_counts.get(topic, 0) <= 0]
    preflight_manifest.update(
        {
            "status": "ok" if not missing_roles and not empty_topics else "capture_degraded",
            "message_counts": message_counts,
            "empty_topics": empty_topics,
        }
    )
    write_json(manifest_path, preflight_manifest)
    if missing_roles or empty_topics:
        detail = missing_text or "部分 topic"
        publish_status(
            phase="completed",
            severity="warning",
            message_zh=f"采集完成，但 {detail} 缺失；其余数据已正常保存。",
            selected_roles=selected,
            missing_roles=missing_roles,
            output=output,
            message_counts=message_counts,
        )
    else:
        publish_status(
            phase="completed",
            severity="ok",
            message_zh="采集完成，前视相机、点云、IMU、里程计和姿态数据均已保存。",
            selected_roles=selected,
            output=output,
            message_counts=message_counts,
        )

    print(f"[采集] 已完成: {output}", flush=True)
    print(f"[采集] 清单: {manifest_path}", flush=True)
    return 0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--name", required=True, help="ASCII acquisition label")
    parser.add_argument("--duration", type=float, default=30.0)
    parser.add_argument(
        "--output-root",
        type=Path,
        default=Path("~/go2w_dataset/bags"),
    )
    parser.add_argument(
        "--status-path",
        type=Path,
        default=Path("~/go2w_dataset/collection_status.json"),
        help="JSON status file consumed by the operator UI",
    )
    parser.add_argument("--preflight-timeout", type=float, default=5.0)
    parser.add_argument(
        "--allow-missing-state",
        action="store_true",
        help="Allow recording without odometry and attitude. Camera/cloud/IMU remain required.",
    )
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()
    if args.duration <= 0:
        parser.error("--duration must be positive")
    if args.preflight_timeout <= 0:
        parser.error("--preflight-timeout must be positive")
    if not re.fullmatch(r"[A-Za-z0-9._-]+", args.name):
        parser.error("--name must contain only ASCII letters, digits, dot, underscore, or dash")
    return args


def main() -> int:
    try:
        return record(parse_args())
    except KeyboardInterrupt:
        print("capture interrupted", file=sys.stderr)
        return 130
    except Exception as exc:
        print(f"capture failed: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
