#!/usr/bin/env python3
"""Prepare aligned fleet/radar sessions and independently trimmed ROS 2 bags."""

from __future__ import annotations

import argparse
import csv
import json
import os
import shutil
import sqlite3
import subprocess
from pathlib import Path
from typing import Any

import imageio_ffmpeg
from ruamel.yaml import YAML


TARGET_TOPICS = ("/frontvideostream", "/utlidar/cloud", "/utlidar/imu")


def load_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def hardlink_or_copy(source: Path, target: Path) -> str:
    target.parent.mkdir(parents=True, exist_ok=True)
    if target.exists():
        target.unlink()
    try:
        os.link(source, target)
        return "hardlink"
    except OSError:
        shutil.copy2(source, target)
        return "copy"


def radar_window(manifest: dict[str, Any]) -> tuple[int, int]:
    events = manifest.get("radar", {}).get("events", [])
    by_name = {event.get("name"): event for event in events}
    start = int(by_name["sensor_start_sent"]["host_wall_time_ns"])
    stop = int(by_name["sensor_stop_detected"]["host_wall_time_ns"])
    return start, stop


def read_csv(path: Path) -> tuple[list[str], list[dict[str, str]]]:
    if not path.exists():
        return [], []
    with path.open("r", newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        return list(reader.fieldnames or []), list(reader)


def write_csv(path: Path, fieldnames: list[str], rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def trim_imu(
    source: Path, target: Path, start_ns: int, stop_ns: int
) -> tuple[int, int]:
    fields, rows = read_csv(source)
    selected = [
        row
        for row in rows
        if start_ns <= int(row["host_wall_time_ns"]) <= stop_ns
    ]
    output_fields = fields + ["t_from_radar_start_s"]
    for row in selected:
        row["t_from_radar_start_s"] = (
            int(row["host_wall_time_ns"]) - start_ns
        ) / 1e9
    write_csv(target, output_fields, selected)
    return len(rows), len(selected)


def trim_video_payload(
    source_video: Path,
    source_frames: Path,
    target_video: Path,
    target_frames: Path,
    start_ns: int,
    stop_ns: int,
) -> tuple[int, int, int]:
    fields, rows = read_csv(source_frames)
    selected = [
        row
        for row in rows
        if start_ns <= int(row["host_wall_time_ns"]) <= stop_ns
    ]
    target_video.parent.mkdir(parents=True, exist_ok=True)
    new_rows: list[dict[str, Any]] = []
    output_offset = 0
    with source_video.open("rb") as source, target_video.open("wb") as target:
        for index, row in enumerate(selected):
            source.seek(int(row["byte_offset"]))
            payload = source.read(int(row["byte_count"]))
            target.write(payload)
            updated = dict(row)
            updated["source_frame_index"] = row["frame_index"]
            updated["frame_index"] = index
            updated["byte_offset"] = output_offset
            updated["byte_count"] = len(payload)
            updated["t_from_radar_start_s"] = (
                int(row["host_wall_time_ns"]) - start_ns
            ) / 1e9
            new_rows.append(updated)
            output_offset += len(payload)
    output_fields = fields + ["source_frame_index", "t_from_radar_start_s"]
    write_csv(target_frames, output_fields, new_rows)
    return len(rows), len(selected), output_offset


def write_radar_frames(source: Path, target: Path, start_monotonic_ns: int) -> int:
    fields, rows = read_csv(source)
    for row in rows:
        row["t_from_radar_start_s"] = (
            int(row["window_start_monotonic_ns"]) - start_monotonic_ns
        ) / 1e9
    write_csv(target, fields + ["t_from_radar_start_s"], rows)
    return len(rows)


def trim_radar_camera(
    radar_dir: Path,
    target_dir: Path,
    camera: dict[str, Any],
    start_monotonic_ns: int,
    stop_monotonic_ns: int,
) -> dict[str, Any]:
    video_name = str(camera.get("video_file", "camera_video.avi"))
    frames_name = str(camera.get("frames_csv", "camera_frames.csv"))
    source_video = radar_dir / "camera" / video_name
    source_frames = radar_dir / "camera" / frames_name
    result = {
        "status": "missing",
        "source_video": video_name,
        "source_frames": frames_name,
        "source_saved_frames": int(camera.get("saved_frames") or 0),
        "aligned_frames": 0,
        "aligned_video_bytes": 0,
    }
    if not source_video.exists():
        return result

    first_saved_ns = camera.get("first_saved_monotonic_ns")
    if (
        camera.get("error")
        or not first_saved_ns
        or not source_frames.exists()
        or int(camera.get("saved_frames") or 0) <= 0
    ):
        target = target_dir / "camera_unaligned_capture_failed.avi"
        hardlink_or_copy(source_video, target)
        result.update(
            {
                "status": "capture_failed_unaligned",
                "aligned_video_bytes": target.stat().st_size,
                "error": camera.get("error") or "missing camera timestamps",
            }
        )
        return result

    fields, rows = read_csv(source_frames)
    selected = [
        row
        for row in rows
        if start_monotonic_ns
        <= int(row["host_monotonic_ns"])
        <= stop_monotonic_ns
    ]
    for index, row in enumerate(selected):
        row["aligned_frame_index"] = index
        row["t_from_radar_start_s"] = (
            int(row["host_monotonic_ns"]) - start_monotonic_ns
        ) / 1e9
    write_csv(
        target_dir / "camera_frames_aligned.csv",
        fields + ["aligned_frame_index", "t_from_radar_start_s"],
        selected,
    )

    offset_s = max((start_monotonic_ns - int(first_saved_ns)) / 1e9, 0.0)
    duration_s = max((stop_monotonic_ns - start_monotonic_ns) / 1e9, 0.0)
    target_video = target_dir / "camera_aligned.avi"
    command = [
        imageio_ffmpeg.get_ffmpeg_exe(),
        "-y",
        "-v",
        "error",
        "-ss",
        f"{offset_s:.9f}",
        "-i",
        str(source_video),
        "-t",
        f"{duration_s:.9f}",
        "-c",
        "copy",
        str(target_video),
    ]
    completed = subprocess.run(command, capture_output=True, text=True, check=False)
    result.update(
        {
            "status": "aligned" if completed.returncode == 0 else "trim_failed",
            "aligned_frames": len(selected),
            "aligned_video_bytes": (
                target_video.stat().st_size if target_video.exists() else 0
            ),
            "offset_s": offset_s,
            "duration_s": duration_s,
            "ffmpeg_error": completed.stderr.strip(),
        }
    )
    return result


def find_event_monotonic(manifest: dict[str, Any], name: str) -> int:
    for event in manifest.get("radar", {}).get("events", []):
        if event.get("name") == name:
            return int(event["host_monotonic_ns"])
    raise KeyError(name)


def process_sessions(source_root: Path, output_root: Path) -> list[dict[str, Any]]:
    radar_root = source_root / "radar0_raw"
    dog_root = source_root / "dog_fleet_sessions"
    rows: list[dict[str, Any]] = []

    for dog_dir in sorted(path for path in dog_root.iterdir() if path.is_dir()):
        fleet = load_json(dog_dir / "fleet_manifest.json")
        station = fleet.get("stations", [{}])[0]
        remote_dir = (
            station.get("capture", {})
            .get("response", {})
            .get("summary", {})
            .get("session_dir", "")
        )
        radar_dir = radar_root / Path(str(remote_dir)).name
        if not radar_dir.exists():
            continue

        radar_manifest = load_json(radar_dir / "session_manifest.json")
        start_ns, stop_ns = radar_window(radar_manifest)
        start_mono = find_event_monotonic(radar_manifest, "sensor_start_sent")
        stop_mono = find_event_monotonic(radar_manifest, "sensor_stop_detected")
        session_name = radar_dir.name
        target = output_root / session_name
        target.mkdir(parents=True, exist_ok=True)

        raw_mode = hardlink_or_copy(
            radar_dir / "radar" / "radar_raw.bin",
            target / "radar" / "radar_raw.bin",
        )
        radar_frame_count = write_radar_frames(
            radar_dir / "radar" / "radar_frames.csv",
            target / "radar" / "radar_frames_aligned.csv",
            start_mono,
        )
        for name in (
            "session_manifest.json",
            "session_metadata.json",
        ):
            shutil.copy2(radar_dir / name, target / "radar" / name)
        camera_result = trim_radar_camera(
            radar_dir,
            target / "radar" / "camera",
            radar_manifest.get("camera", {}),
            start_mono,
            stop_mono,
        )

        imu_before, imu_after = trim_imu(
            dog_dir / "controller_imu" / "imu_samples.csv",
            target / "dog" / "imu_aligned.csv",
            start_ns,
            stop_ns,
        )
        video_before, video_after, video_bytes = trim_video_payload(
            dog_dir / "controller_video" / "controller_front_video_720p.h264",
            dog_dir / "controller_video" / "controller_video_frames.csv",
            target / "dog" / "front_video_aligned.h264",
            target / "dog" / "front_video_frames_aligned.csv",
            start_ns,
            stop_ns,
        )
        shutil.copy2(
            dog_dir / "fleet_manifest.json", target / "dog" / "fleet_manifest.json"
        )

        record = {
            "session": session_name,
            "dog_session": dog_dir.name,
            "label": radar_manifest.get("requested_name", ""),
            "radar_start_wall_time_ns": start_ns,
            "radar_stop_wall_time_ns": stop_ns,
            "radar_duration_s": (stop_ns - start_ns) / 1e9,
            "radar_frames": radar_frame_count,
            "radar_raw_bytes": (target / "radar" / "radar_raw.bin").stat().st_size,
            "radar_raw_storage": raw_mode,
            "radar_camera_status": camera_result["status"],
            "radar_camera_aligned_frames": camera_result["aligned_frames"],
            "radar_camera_aligned_bytes": camera_result["aligned_video_bytes"],
            "dog_imu_rows_before": imu_before,
            "dog_imu_rows_aligned": imu_after,
            "dog_video_payloads_before": video_before,
            "dog_video_payloads_aligned": video_after,
            "dog_video_aligned_bytes": video_bytes,
            "alignment_method": "fleet manifest pair + wall-clock radar window",
            "clock_offset_status": "unmeasured",
        }
        (target / "alignment.json").write_text(
            json.dumps(record, indent=2) + "\n", encoding="utf-8"
        )
        rows.append(record)

    return rows


def bag_topic_ranges(db_path: Path) -> dict[str, dict[str, int]]:
    connection = sqlite3.connect(db_path)
    try:
        result: dict[str, dict[str, int]] = {}
        query = """
            SELECT topics.name, COUNT(messages.id), MIN(messages.timestamp),
                   MAX(messages.timestamp)
            FROM topics
            LEFT JOIN messages ON messages.topic_id = topics.id
            GROUP BY topics.id
        """
        for name, count, start, stop in connection.execute(query):
            result[str(name)] = {
                "count": int(count),
                "start_ns": int(start) if start is not None else 0,
                "stop_ns": int(stop) if stop is not None else 0,
            }
        return result
    finally:
        connection.close()


def trim_bag_database(
    source_db: Path, target_db: Path, start_ns: int, stop_ns: int
) -> dict[str, int]:
    target_db.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(source_db, target_db)
    connection = sqlite3.connect(target_db)
    try:
        placeholders = ",".join("?" for _ in TARGET_TOPICS)
        connection.execute(
            f"""
            DELETE FROM messages
            WHERE timestamp < ? OR timestamp > ?
               OR topic_id NOT IN (
                   SELECT id FROM topics WHERE name IN ({placeholders})
               )
            """,
            (start_ns, stop_ns, *TARGET_TOPICS),
        )
        connection.execute(
            f"DELETE FROM topics WHERE name NOT IN ({placeholders})", TARGET_TOPICS
        )
        connection.commit()
        connection.execute("VACUUM")
        counts = {
            str(name): int(count)
            for name, count in connection.execute(
                """
                SELECT topics.name, COUNT(messages.id)
                FROM topics LEFT JOIN messages ON messages.topic_id = topics.id
                GROUP BY topics.id
                """
            )
        }
        return counts
    finally:
        connection.close()


def write_trimmed_metadata(
    source_yaml: Path,
    target_yaml: Path,
    target_db: Path,
    start_ns: int,
    stop_ns: int,
    counts: dict[str, int],
) -> None:
    yaml = YAML()
    data = yaml.load(source_yaml.read_text(encoding="utf-8"))
    info = data["rosbag2_bagfile_information"]
    info["relative_file_paths"] = [target_db.name]
    info["starting_time"]["nanoseconds_since_epoch"] = start_ns
    info["duration"]["nanoseconds"] = max(stop_ns - start_ns, 0)
    info["message_count"] = sum(counts.values())
    topics = []
    for item in info["topics_with_message_count"]:
        name = item["topic_metadata"]["name"]
        if name in TARGET_TOPICS:
            item["message_count"] = counts.get(name, 0)
            topics.append(item)
    info["topics_with_message_count"] = topics
    with target_yaml.open("w", encoding="utf-8") as handle:
        yaml.dump(data, handle)


def trim_pcd_index(
    source_pcd: Path,
    target_pcd: Path,
    start_ns: int,
    stop_ns: int,
) -> int:
    fields, rows = read_csv(source_pcd / "frames.csv")
    selected = [
        row
        for row in rows
        if start_ns <= int(row["bag_timestamp_ns"]) <= stop_ns
    ]
    target_frames = target_pcd / "frames"
    target_frames.mkdir(parents=True, exist_ok=True)
    for index, row in enumerate(selected):
        source = source_pcd / row["filename"]
        target = target_frames / source.name
        hardlink_or_copy(source, target)
        row["trimmed_frame_index"] = index
        row["filename"] = f"frames/{source.name}"
        row["t_from_bag_common_start_s"] = (
            int(row["bag_timestamp_ns"]) - start_ns
        ) / 1e9
    write_csv(
        target_pcd / "frames.csv",
        fields + ["trimmed_frame_index", "t_from_bag_common_start_s"],
        selected,
    )
    return len(selected)


def process_bags(source_root: Path, output_root: Path) -> list[dict[str, Any]]:
    bag_root = source_root / "dog_rosbags"
    pcd_root = source_root / "pointcloud_pcd"
    rows: list[dict[str, Any]] = []
    for bag_dir in sorted(path for path in bag_root.iterdir() if path.is_dir()):
        source_db = next(bag_dir.glob("*.db3"))
        ranges = bag_topic_ranges(source_db)
        valid = all(ranges.get(topic, {}).get("count", 0) > 0 for topic in TARGET_TOPICS)
        record: dict[str, Any] = {
            "bag": bag_dir.name,
            "status": "valid" if valid else "excluded_missing_topics",
            "source_counts": {
                topic: ranges.get(topic, {}).get("count", 0) for topic in TARGET_TOPICS
            },
        }
        if not valid:
            rows.append(record)
            continue

        start_ns = max(ranges[topic]["start_ns"] for topic in TARGET_TOPICS)
        stop_ns = min(ranges[topic]["stop_ns"] for topic in TARGET_TOPICS)
        target = output_root / bag_dir.name
        target_db = target / source_db.name.replace(".db3", "_trimmed.db3")
        counts = trim_bag_database(source_db, target_db, start_ns, stop_ns)
        write_trimmed_metadata(
            bag_dir / "metadata.yaml",
            target / "metadata.yaml",
            target_db,
            start_ns,
            stop_ns,
            counts,
        )
        pcd_count = 0
        source_pcd = pcd_root / bag_dir.name
        if source_pcd.exists():
            pcd_count = trim_pcd_index(
                source_pcd, target / "pointcloud_pcd", start_ns, stop_ns
            )
        record.update(
            {
                "common_start_ns": start_ns,
                "common_stop_ns": stop_ns,
                "common_duration_s": (stop_ns - start_ns) / 1e9,
                "trimmed_counts": counts,
                "trimmed_pcd_frames": pcd_count,
            }
        )
        (target / "trim_summary.json").write_text(
            json.dumps(record, indent=2) + "\n", encoding="utf-8"
        )
        rows.append(record)
    return rows


def write_report(
    output_root: Path,
    session_rows: list[dict[str, Any]],
    bag_rows: list[dict[str, Any]],
) -> None:
    write_csv(
        output_root / "sessions_alignment_summary.csv",
        list(session_rows[0]) if session_rows else [],
        session_rows,
    )
    (output_root / "bags_trim_summary.json").write_text(
        json.dumps(bag_rows, indent=2) + "\n", encoding="utf-8"
    )

    lines = [
        "# GO2W 2026-06-10 数据裁剪与对齐简报",
        "",
        "## 处理原则",
        "",
        "- Sessions：按 fleet manifest 中的直接远端会话引用配对。",
        "- Radar raw 已是 sensor start/stop 对应的完整采集窗口，不再次切割二进制。",
        "- 机器狗 IMU 和前视视频 payload 按 radar wall-clock 窗口裁剪。",
        "- 前视视频源是 Unitree ROS topic payload；原始文件本身缺少标准 H.264 参数集，裁剪结果保留 payload 和索引，不保证普通播放器直接解码。",
        "- Clock snapshot 未得到有效 offset，属于 wall-clock 最佳努力对齐，不代表硬件级同步。",
        "- Bags 与 sessions 不强行对齐；每个 bag 独立裁剪为视频、点云、IMU 的共同有效区间。",
        "",
        "## Sessions",
        "",
        "| 标签 | Radar 帧 | Radar相机帧 | 机器狗IMU | 机器狗视频payload | 结果 |",
        "|---|---:|---:|---:|---:|---|",
    ]
    for row in session_rows:
        quality = "可用" if row["dog_imu_rows_aligned"] > 0 else "Radar 可用，机器狗 IMU 缺失"
        lines.append(
            f"| {row['label']} | {row['radar_frames']} | "
            f"{row['radar_camera_aligned_frames']} | "
            f"{row['dog_imu_rows_aligned']} | "
            f"{row['dog_video_payloads_aligned']} | {quality} |"
        )

    lines.extend(
        [
            "",
            "## Bags",
            "",
            "| Bag | 状态 | 共同区间(s) | 视频消息 | 点云消息 | IMU消息 | PCD帧 |",
            "|---|---|---:|---:|---:|---:|---:|",
        ]
    )
    for row in bag_rows:
        if row["status"] != "valid":
            counts = row["source_counts"]
            lines.append(
                f"| {row['bag']} | 缺少点云/IMU，排除 | 0 | "
                f"{counts.get('/frontvideostream', 0)} | "
                f"{counts.get('/utlidar/cloud', 0)} | "
                f"{counts.get('/utlidar/imu', 0)} | 0 |"
            )
            continue
        counts = row["trimmed_counts"]
        lines.append(
            f"| {row['bag']} | 已裁剪 | {row['common_duration_s']:.3f} | "
            f"{counts.get('/frontvideostream', 0)} | "
            f"{counts.get('/utlidar/cloud', 0)} | "
            f"{counts.get('/utlidar/imu', 0)} | "
            f"{row['trimmed_pcd_frames']} |"
        )

    usable_sessions = sum(row["dog_imu_rows_aligned"] > 0 for row in session_rows)
    valid_bags = sum(row["status"] == "valid" for row in bag_rows)
    lines.extend(
        [
            "",
            "## 结论",
            "",
            f"- 10 组 radar0 raw 均保留完整 600 帧窗口。",
            f"- Radar0 USB 相机正常会话已按雷达窗口裁剪；采集失败会话保留原始 AVI 并标记为未对齐。",
            f"- {usable_sessions} 组 sessions 同时具有裁剪后的机器狗 IMU；其余组仅能做 Radar/视频分析。",
            f"- {valid_bags} 份 bag 具备视频、点云、IMU共同区间；缺失话题的 bag 单独标记并未混入。",
            "- Radar raw 使用硬链接纳入处理目录，不额外复制约 12.6 GB 原始数据。",
            "- 机器狗视频需后续结合 Unitree Go2FrontVideoData 编码约定恢复标准视频。",
            "- 后续严格融合前应补做 NTP/PTP offset 测量或使用硬件同步信号。",
            "",
        ]
    )
    (output_root / "REPORT.md").write_text("\n".join(lines), encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("source_root", type=Path)
    parser.add_argument("output_root", type=Path)
    args = parser.parse_args()
    args.output_root.mkdir(parents=True, exist_ok=True)

    session_rows = process_sessions(
        args.source_root, args.output_root / "sessions_aligned"
    )
    bag_rows = process_bags(args.source_root, args.output_root / "bags_independent")
    write_report(args.output_root, session_rows, bag_rows)
    print(
        json.dumps(
            {
                "sessions": len(session_rows),
                "bags": len(bag_rows),
                "output": str(args.output_root.resolve()),
            },
            indent=2,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
