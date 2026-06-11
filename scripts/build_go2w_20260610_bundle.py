#!/usr/bin/env python3
"""Build the four-part GO2W data bundle for the 2026-06-10 collection."""

from __future__ import annotations

import argparse
import bisect
import csv
import json
import os
import shutil
import subprocess
from pathlib import Path


RAW_DIR = "01_所有原始数据包"
RADAR_DIR = "02_雷达采集包"
POINTCLOUD_DIR = "03_点云与对应视频"
REPORT_DIR = "04_报告"


def hardlink_or_copy(source: str, destination: str) -> str:
    try:
        os.link(source, destination)
        return destination
    except OSError:
        return shutil.copy2(source, destination)


def link_tree(source: Path, destination: Path) -> None:
    if not source.exists():
        raise FileNotFoundError(source)
    shutil.copytree(source, destination, copy_function=hardlink_or_copy)


def link_file(source: Path, destination: Path) -> None:
    destination.parent.mkdir(parents=True, exist_ok=True)
    hardlink_or_copy(str(source), str(destination))


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open("r", encoding="utf-8-sig", newline="") as handle:
        return list(csv.DictReader(handle))


def write_csv(path: Path, rows: list[dict[str, object]], fields: list[str]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8-sig", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def nearest_index(sorted_values: list[int], target: int) -> int:
    right = bisect.bisect_left(sorted_values, target)
    if right == 0:
        return 0
    if right == len(sorted_values):
        return len(sorted_values) - 1
    left = right - 1
    if target - sorted_values[left] <= sorted_values[right] - target:
        return left
    return right


def extract_video_frames(ffmpeg: Path, video: Path, output_dir: Path) -> int:
    output_dir.mkdir(parents=True, exist_ok=True)
    command = [
        str(ffmpeg),
        "-hide_banner",
        "-loglevel",
        "error",
        "-i",
        str(video),
        "-fps_mode",
        "passthrough",
        "-q:v",
        "3",
        "-start_number",
        "1",
        str(output_dir / "frame_%06d.jpg"),
    ]
    subprocess.run(command, check=True)
    return sum(1 for _ in output_dir.glob("frame_*.jpg"))


def transcode_camera_video(ffmpeg: Path, source: Path, destination: Path) -> None:
    command = [
        str(ffmpeg),
        "-y",
        "-hide_banner",
        "-loglevel",
        "error",
        "-i",
        str(source),
        "-c:v",
        "libx264",
        "-preset",
        "fast",
        "-crf",
        "20",
        "-pix_fmt",
        "yuv420p",
        "-movflags",
        "+faststart",
        str(destination),
    ]
    subprocess.run(command, check=True)


def build_mapping(pointcloud_csv: Path, video_csv: Path, output_csv: Path) -> dict[str, object]:
    clouds = read_csv(pointcloud_csv)
    videos = read_csv(video_csv)
    video_times = [int(row["bag_timestamp_ns"]) for row in videos]
    if not video_times:
        raise RuntimeError(f"No video timestamps in {video_csv}")

    rows: list[dict[str, object]] = []
    deltas_ms: list[float] = []
    for cloud in clouds:
        cloud_time = int(cloud["bag_timestamp_ns"])
        if cloud_time < video_times[0] or cloud_time > video_times[-1]:
            rows.append(
                {
                    "pointcloud_frame_index": cloud["frame_index"],
                    "pointcloud_file": cloud["filename"],
                    "pointcloud_bag_timestamp_ns": cloud_time,
                    "mapping_status": "outside_decodable_video_range",
                    "video_frame_index": "",
                    "video_frame_file": "",
                    "video_bag_timestamp_ns": "",
                    "video_minus_pointcloud_ms": "",
                }
            )
            continue
        video_position = nearest_index(video_times, cloud_time)
        video = videos[video_position]
        video_time = video_times[video_position]
        delta_ms = (video_time - cloud_time) / 1_000_000.0
        deltas_ms.append(abs(delta_ms))
        rows.append(
            {
                "pointcloud_frame_index": cloud["frame_index"],
                "pointcloud_file": cloud["filename"],
                "pointcloud_bag_timestamp_ns": cloud_time,
                "mapping_status": "matched_nearest_timestamp",
                "video_frame_index": video["frame_index"],
                "video_frame_file": f"video_frames/frame_{video_position + 1:06d}.jpg",
                "video_bag_timestamp_ns": video_time,
                "video_minus_pointcloud_ms": f"{delta_ms:.3f}",
            }
        )

    fields = [
        "pointcloud_frame_index",
        "pointcloud_file",
        "pointcloud_bag_timestamp_ns",
        "mapping_status",
        "video_frame_index",
        "video_frame_file",
        "video_bag_timestamp_ns",
        "video_minus_pointcloud_ms",
    ]
    write_csv(output_csv, rows, fields)
    deltas_ms.sort()
    return {
        "pointcloud_frames": len(clouds),
        "video_frames": len(videos),
        "matched_pointcloud_frames": len(deltas_ms),
        "unmatched_pointcloud_frames": len(clouds) - len(deltas_ms),
        "median_abs_delta_ms": round(deltas_ms[len(deltas_ms) // 2], 3),
        "max_abs_delta_ms": round(max(deltas_ms), 3),
    }


def directory_stats(path: Path) -> tuple[int, int]:
    count = 0
    total = 0
    for item in path.rglob("*"):
        if item.is_file():
            count += 1
            total += item.stat().st_size
    return count, total


def build_bundle(source: Path, output: Path, repo: Path, ffmpeg: Path) -> None:
    if output.exists():
        raise FileExistsError(f"Output already exists: {output}")
    output.mkdir(parents=True)

    raw_output = output / RAW_DIR
    radar_output = output / RADAR_DIR
    pointcloud_output = output / POINTCLOUD_DIR
    report_output = output / REPORT_DIR
    for path in (raw_output, radar_output, pointcloud_output, report_output):
        path.mkdir()

    raw_sources = {
        "radar0_raw": source / "radar0_raw",
        "dog_fleet_sessions": source / "dog_fleet_sessions",
        "dog_rosbags": source / "dog_rosbags",
    }
    for name, path in raw_sources.items():
        link_tree(path, raw_output / name)

    sessions_source = source / "processed_alignment" / "sessions_aligned"
    for session in sorted(sessions_source.iterdir()):
        if session.is_dir():
            session_output = radar_output / session.name
            link_tree(session, session_output)
            camera_dir = session_output / "radar" / "camera"
            for camera_video in camera_dir.glob("*.avi"):
                transcode_camera_video(
                    ffmpeg,
                    camera_video,
                    camera_video.with_name(f"{camera_video.stem}_playable.mp4"),
                )

    alignment_csv = source / "processed_alignment" / "sessions_alignment_summary.csv"
    link_file(alignment_csv, radar_output / "sessions_alignment_summary.csv")

    trim_summary_path = source / "processed_alignment" / "bags_trim_summary.json"
    trim_summary = json.loads(trim_summary_path.read_text(encoding="utf-8"))
    video_source = source / "bag_video_exports_fixed"
    bag_results: list[dict[str, object]] = []

    for bag_info in trim_summary:
        bag_name = bag_info["bag"]
        bag_output = pointcloud_output / bag_name
        bag_output.mkdir()
        video_prefix = f"{bag_name}_frontvideo_720p"
        video_dir = bag_output / "video"
        video_dir.mkdir()
        for suffix in (".mp4", ".h264", ".csv"):
            link_file(video_source / f"{video_prefix}{suffix}", video_dir / f"{video_prefix}{suffix}")

        if bag_info["status"] == "valid":
            processed_bag = source / "processed_alignment" / "bags_independent" / bag_name
            bag_dir = bag_output / "trimmed_rosbag"
            bag_dir.mkdir()
            for item in processed_bag.iterdir():
                if item.name == "pointcloud_pcd":
                    continue
                if item.is_file():
                    link_file(item, bag_dir / item.name)
            link_tree(processed_bag / "pointcloud_pcd", bag_output / "pointcloud_pcd")

            extracted_count = extract_video_frames(
                ffmpeg,
                video_dir / f"{video_prefix}.mp4",
                bag_output / "video_frames",
            )
            mapping_stats = build_mapping(
                bag_output / "pointcloud_pcd" / "frames.csv",
                video_dir / f"{video_prefix}.csv",
                bag_output / "pointcloud_video_mapping.csv",
            )
            if extracted_count != mapping_stats["video_frames"]:
                raise RuntimeError(
                    f"{bag_name}: extracted {extracted_count} video frames, "
                    f"timestamp CSV contains {mapping_stats['video_frames']}"
                )
            result = {
                "bag": bag_name,
                "status": "valid_aligned",
                "common_duration_s": bag_info["common_duration_s"],
                **mapping_stats,
            }
            note = (
                f"# {bag_name}\n\n"
                f"- 状态：有效，已按三话题公共时间段裁剪。\n"
                f"- 公共时长：{bag_info['common_duration_s']:.3f} 秒。\n"
                f"- PCD 帧：{mapping_stats['pointcloud_frames']}。\n"
                f"- 解码视频帧：{mapping_stats['video_frames']}，1280x720 JPEG。\n"
                f"- 对应关系：`pointcloud_video_mapping.csv`，按 bag 时间戳取最近视频帧。\n"
                f"- 成功匹配点云：{mapping_stats['matched_pointcloud_frames']}；"
                f"关键帧范围外未匹配：{mapping_stats['unmatched_pointcloud_frames']}。\n"
                f"- 中位绝对时间差：{mapping_stats['median_abs_delta_ms']:.3f} ms。\n"
                f"- 最大绝对时间差：{mapping_stats['max_abs_delta_ms']:.3f} ms。\n"
            )
        else:
            result = {
                "bag": bag_name,
                "status": "video_only",
                "pointcloud_frames": 0,
                "video_messages": bag_info["source_counts"]["/frontvideostream"],
            }
            note = (
                f"# {bag_name}\n\n"
                "- 状态：仅视频。\n"
                "- 原始 bag 中 `/utlidar/cloud` 和 `/utlidar/imu` 都是 0 条，"
                "因此没有可生成或对齐的点云帧。\n"
                "- 本目录保留修复后 MP4、原始 H.264 码流和视频时间戳 CSV。\n"
            )
        (bag_output / "README.md").write_text(note, encoding="utf-8")
        bag_results.append(result)

    reports_to_link = [
        source / "processed_alignment" / "REPORT.md",
        alignment_csv,
        trim_summary_path,
    ]
    for report in reports_to_link:
        link_file(report, report_output / report.name)

    scripts_dir = report_output / "处理脚本"
    scripts_dir.mkdir()
    for script_name in (
        "process_go2w_collected_dataset.py",
        "rosbag_pointcloud2_to_pcd.py",
        "rosbag_unitree_video_export.py",
        "build_go2w_20260610_bundle.py",
    ):
        link_file(repo / "scripts" / script_name, scripts_dir / script_name)

    (report_output / "点云视频对应汇总.json").write_text(
        json.dumps(bag_results, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )

    section_rows: list[dict[str, object]] = []
    for section in (raw_output, radar_output, pointcloud_output, report_output):
        file_count, byte_count = directory_stats(section)
        if section == report_output:
            file_count += 1
        section_rows.append(
            {
                "section": section.name,
                "file_count": file_count,
                "logical_bytes": byte_count,
                "logical_gib": f"{byte_count / (1024 ** 3):.3f}",
            }
        )
    write_csv(
        report_output / "总包文件统计.csv",
        section_rows,
        ["section", "file_count", "logical_bytes", "logical_gib"],
    )

    overview = f"""# GO2W 2026-06-10 数据总包

## 目录

1. `{RAW_DIR}`：雷达侧 NX 原始数据、机器狗采集 session、机器狗 ROS 2 bag。
2. `{RADAR_DIR}`：10 次雷达采集，按 session 放置雷达 raw、雷达相机和对应机器狗数据。
3. `{POINTCLOUD_DIR}`：按每次 bag 放置裁剪 bag、PCD、修复视频、解码 JPEG 帧和时间映射表。
4. `{REPORT_DIR}`：对齐报告、汇总表、处理说明和复现脚本。

## 关键说明

- 数据总包使用 NTFS 硬链接复用大文件；删除原拉取目录不会影响总包中的硬链接。复制到外置盘时会生成独立文件。
- 两个有效点云 bag 已分别整理；第三个 bag 只有视频，没有点云和 IMU。
- 雷达侧相机文件是 MJPEG AVI，不是 H.264。8 次成功对齐，2 次采集失败文件保留为未对齐。
- 每个雷达侧 AVI 旁边都提供了 `_playable.mp4` 兼容版本，统一使用 H.264/YUV420P。
- `dog/front_video_aligned.h264` 是机器狗采集程序留下的残缺裸码流，缺少稳定的参数集/关键帧边界，不应作为正常可播放视频使用。
- 机器狗 IMU 只有前两次 session 有有效数据；后 8 次源端录制时已经为空。
- 点云到视频帧的对应关系见各 bag 的 `pointcloud_video_mapping.csv`。
"""
    (output / "README_总包说明.md").write_text(overview, encoding="utf-8")

    playback_note = """# 视频播放说明

## 雷达侧相机

- `radar/camera/*.avi`：原始或对齐后的 MJPEG AVI，数据完整，但部分 Windows 播放器不支持 MJPEG。
- `radar/camera/*_playable.mp4`：兼容版本，H.264/YUV420P，建议直接播放此文件。
- 文件名含 `unaligned_capture_failed` 的两次视频可以解码，但源端没有可信帧时间戳，因此不能用于严格对齐。

## 机器狗视频

- `dog/front_video_aligned.h264`：从机器狗采集 session 裁出的裸 H.264 碎片。
- 这些文件体积只有约 20-36 KB，缺少可靠的 SPS/PPS/IDR 起始边界，FFmpeg 会报告解码错误。
- 它们保留用于追溯原始采集问题，不作为可播放交付视频。
- 可正常播放的机器狗视频位于 `03_点云与对应视频` 各 bag 的 `video/*.mp4`。
    """
    (report_output / "视频播放说明.md").write_text(playback_note, encoding="utf-8")

    final_section_rows: list[dict[str, object]] = []
    for section in (raw_output, radar_output, pointcloud_output, report_output):
        file_count, byte_count = directory_stats(section)
        final_section_rows.append(
            {
                "section": section.name,
                "file_count": file_count,
                "logical_bytes": byte_count,
                "logical_gib": f"{byte_count / (1024 ** 3):.3f}",
            }
        )
    write_csv(
        report_output / "总包文件统计.csv",
        final_section_rows,
        ["section", "file_count", "logical_bytes", "logical_gib"],
    )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--source",
        type=Path,
        default=Path.home() / "Desktop" / "go2w_pull_20260610_110555",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path.home() / "Desktop" / "GO2W_20260610_数据总包",
    )
    parser.add_argument("--repo", type=Path, default=Path(r"E:\GO2W_0"))
    parser.add_argument("--ffmpeg", type=Path, required=True)
    args = parser.parse_args()
    build_bundle(args.source.resolve(), args.output.resolve(), args.repo.resolve(), args.ffmpeg.resolve())
    print(args.output.resolve())


if __name__ == "__main__":
    main()
