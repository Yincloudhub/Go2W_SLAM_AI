#!/usr/bin/env python3
"""Export Unitree Go2FrontVideoData video720p payloads from rosbag2 SQLite."""

from __future__ import annotations

import argparse
import csv
import sqlite3
import struct
import subprocess
from pathlib import Path

import imageio_ffmpeg


def align4(offset: int) -> int:
    return (offset + 3) & ~3


def decode_video_payload(raw: bytes) -> tuple[int, int, bytes]:
    if len(raw) < 20:
        raise ValueError("CDR message is too short")
    endian = "<" if raw[1] == 1 else ">"
    time_frame = struct.unpack_from(endian + "Q", raw, 4)[0]
    resolution = struct.unpack_from(endian + "I", raw, 12)[0]
    size = struct.unpack_from(endian + "I", raw, 16)[0]
    return time_frame, resolution, raw[20 : 20 + size]


def has_random_access_point(payload: bytes) -> bool:
    nal_types = []
    index = 0
    while index + 4 < len(payload):
        if payload[index : index + 4] == b"\x00\x00\x00\x01":
            nal_types.append(payload[index + 4] & 0x1F)
            index += 4
        elif payload[index : index + 3] == b"\x00\x00\x01":
            nal_types.append(payload[index + 3] & 0x1F)
            index += 3
        else:
            index += 1
    return 7 in nal_types and 8 in nal_types and 5 in nal_types


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("bag", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--resolution", type=int, default=720)
    parser.add_argument("--fps", type=float, default=0.0)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)

    db_path = next(args.bag.glob("*.db3"))
    raw_path = args.output / f"{args.bag.name}_frontvideo_720p.h264"
    csv_path = args.output / f"{args.bag.name}_frontvideo_720p.csv"
    mp4_path = args.output / f"{args.bag.name}_frontvideo_720p.mp4"

    messages = []
    connection = sqlite3.connect(db_path)
    try:
        query = """
            SELECT messages.timestamp, messages.data
            FROM messages
            JOIN topics ON messages.topic_id = topics.id
            WHERE topics.name = '/frontvideostream'
            ORDER BY messages.timestamp
        """
        for timestamp, raw in connection.execute(query):
            time_frame, resolution, payload = decode_video_payload(raw)
            if resolution == args.resolution:
                messages.append((timestamp, time_frame, resolution, payload))
    finally:
        connection.close()

    first_keyframe = next(
        (index for index, item in enumerate(messages) if has_random_access_point(item[3])),
        0,
    )
    messages = messages[first_keyframe:]

    rows = []
    byte_offset = 0
    with raw_path.open("wb") as output:
        for index, (timestamp, time_frame, resolution, payload) in enumerate(messages):
            output.write(payload)
            rows.append(
                {
                    "frame_index": index,
                    "source_resolution_index": index + first_keyframe,
                    "bag_timestamp_ns": timestamp,
                    "time_frame": time_frame,
                    "resolution": resolution,
                    "byte_offset": byte_offset,
                    "byte_count": len(payload),
                }
            )
            byte_offset += len(payload)

    with csv_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]) if rows else [])
        if rows:
            writer.writeheader()
            writer.writerows(rows)

    if args.fps > 0:
        fps = args.fps
    else:
        unique_times = sorted({int(row["bag_timestamp_ns"]) for row in rows})
        deltas = [
            later - earlier
            for earlier, later in zip(unique_times, unique_times[1:])
            if later > earlier
        ]
        median_delta = sorted(deltas)[len(deltas) // 2] if deltas else 66_667
        fps = 1_000_000_000.0 / median_delta

    command = [
        imageio_ffmpeg.get_ffmpeg_exe(),
        "-y",
        "-v",
        "warning",
        "-fflags",
        "+genpts",
        "-r",
        f"{fps:.6f}",
        "-f",
        "h264",
        "-i",
        str(raw_path),
        "-c:v",
        "copy",
        "-movflags",
        "+faststart",
        str(mp4_path),
    ]
    completed = subprocess.run(command, capture_output=True, text=True, check=False)
    print(
        f"messages={len(rows)} skipped_before_keyframe={first_keyframe} bytes={byte_offset} "
        f"fps={fps:.3f} mp4={'ok' if completed.returncode == 0 else 'failed'}"
    )
    if completed.stderr:
        print(completed.stderr[-2000:])
    return 0 if rows else 2


if __name__ == "__main__":
    raise SystemExit(main())
