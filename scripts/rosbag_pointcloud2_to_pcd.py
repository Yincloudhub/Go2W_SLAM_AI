#!/usr/bin/env python3
"""Export a ROS 2 PointCloud2 topic from a rosbag2 SQLite bag to PCD files."""

from __future__ import annotations

import argparse
import csv
import json
import re
from pathlib import Path

import numpy as np
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore


POINT_FIELD_TYPES = {
    1: ("i1", 1, "I"),
    2: ("u1", 1, "U"),
    3: ("i2", 2, "I"),
    4: ("u2", 2, "U"),
    5: ("i4", 4, "I"),
    6: ("u4", 4, "U"),
    7: ("f4", 4, "F"),
    8: ("f8", 8, "F"),
}


def safe_field_name(name: str, fallback: str) -> str:
    cleaned = re.sub(r"[^A-Za-z0-9_]", "_", name).strip("_")
    return cleaned or fallback


def point_array(msg: object) -> tuple[np.ndarray, list[dict[str, object]]]:
    endian = ">" if msg.is_bigendian else "<"
    names: list[str] = []
    formats: list[object] = []
    offsets: list[int] = []
    metadata: list[dict[str, object]] = []

    for index, field in enumerate(msg.fields):
        if field.datatype not in POINT_FIELD_TYPES:
            continue
        dtype_code, size, pcd_type = POINT_FIELD_TYPES[field.datatype]
        name = safe_field_name(field.name, f"field_{index}")
        count = max(1, int(field.count))
        field_dtype: object = endian + dtype_code
        if count > 1:
            field_dtype = (field_dtype, (count,))
        names.append(name)
        formats.append(field_dtype)
        offsets.append(int(field.offset))
        metadata.append(
            {"name": name, "size": size, "type": pcd_type, "count": count}
        )

    dtype = np.dtype(
        {
            "names": names,
            "formats": formats,
            "offsets": offsets,
            "itemsize": int(msg.point_step),
        }
    )
    raw = np.asarray(msg.data, dtype=np.uint8)
    rows = []
    for row in range(int(msg.height)):
        start = row * int(msg.row_step)
        stop = start + int(msg.width) * int(msg.point_step)
        rows.append(np.frombuffer(raw[start:stop], dtype=dtype, count=int(msg.width)))
    points = np.concatenate(rows) if rows else np.empty(0, dtype=dtype)
    return points, metadata


def write_binary_pcd(
    path: Path,
    points: np.ndarray,
    fields: list[dict[str, object]],
    frame_id: str,
    stamp_ns: int,
) -> None:
    output_names: list[str] = []
    output_formats: list[object] = []
    for field in fields:
        dtype_code = POINT_FIELD_TYPES_BY_PCD[(field["type"], field["size"])]
        count = int(field["count"])
        dtype: object = "<" + dtype_code
        if count > 1:
            dtype = (dtype, (count,))
        output_names.append(str(field["name"]))
        output_formats.append(dtype)

    packed = np.empty(points.shape[0], dtype=np.dtype(list(zip(output_names, output_formats))))
    for name in output_names:
        packed[name] = points[name]

    header = "\n".join(
        [
            "# .PCD v0.7 - Point Cloud Data file format",
            f"# frame_id={frame_id} stamp_ns={stamp_ns}",
            "VERSION 0.7",
            "FIELDS " + " ".join(output_names),
            "SIZE " + " ".join(str(field["size"]) for field in fields),
            "TYPE " + " ".join(str(field["type"]) for field in fields),
            "COUNT " + " ".join(str(field["count"]) for field in fields),
            f"WIDTH {points.shape[0]}",
            "HEIGHT 1",
            "VIEWPOINT 0 0 0 1 0 0 0",
            f"POINTS {points.shape[0]}",
            "DATA binary",
            "",
        ]
    ).encode("ascii")
    with path.open("wb") as handle:
        handle.write(header)
        handle.write(packed.tobytes(order="C"))


POINT_FIELD_TYPES_BY_PCD = {
    ("I", 1): "i1",
    ("U", 1): "u1",
    ("I", 2): "i2",
    ("U", 2): "u2",
    ("I", 4): "i4",
    ("U", 4): "u4",
    ("F", 4): "f4",
    ("F", 8): "f8",
}


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("bag", type=Path, help="Rosbag2 directory containing metadata.yaml")
    parser.add_argument("output", type=Path, help="Output directory")
    parser.add_argument("--topic", default="/utlidar/cloud")
    parser.add_argument("--every", type=int, default=1, help="Export every Nth message")
    parser.add_argument("--limit", type=int, default=0, help="Maximum exported frames")
    args = parser.parse_args()

    args.output.mkdir(parents=True, exist_ok=True)
    frames_dir = args.output / "frames"
    frames_dir.mkdir(exist_ok=True)

    typestore = get_typestore(Stores.ROS2_FOXY)
    with AnyReader([args.bag], default_typestore=typestore) as reader:
        connections = [c for c in reader.connections if c.topic == args.topic]
        if not connections:
            raise SystemExit(f"Topic not found: {args.topic}")

        rows: list[dict[str, object]] = []
        field_metadata: list[dict[str, object]] | None = None
        seen = 0
        for connection, bag_timestamp, rawdata in reader.messages(connections=connections):
            source_index = seen
            seen += 1
            if source_index % args.every:
                continue
            if args.limit and len(rows) >= args.limit:
                break

            msg = reader.deserialize(rawdata, connection.msgtype)
            points, fields = point_array(msg)
            field_metadata = field_metadata or fields
            stamp_ns = int(msg.header.stamp.sec) * 1_000_000_000 + int(
                msg.header.stamp.nanosec
            )
            filename = f"cloud_{len(rows):06d}_{stamp_ns}.pcd"
            write_binary_pcd(
                frames_dir / filename,
                points,
                fields,
                str(msg.header.frame_id),
                stamp_ns,
            )
            rows.append(
                {
                    "frame_index": len(rows),
                    "source_index": source_index,
                    "bag_timestamp_ns": bag_timestamp,
                    "header_timestamp_ns": stamp_ns,
                    "frame_id": msg.header.frame_id,
                    "point_count": points.shape[0],
                    "filename": f"frames/{filename}",
                }
            )
            if len(rows) % 100 == 0:
                print(f"Exported {len(rows)} frames")

    with (args.output / "frames.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]) if rows else [])
        if rows:
            writer.writeheader()
            writer.writerows(rows)

    summary = {
        "source_bag": str(args.bag.resolve()),
        "topic": args.topic,
        "source_messages_seen": seen,
        "exported_frames": len(rows),
        "every": args.every,
        "fields": field_metadata or [],
    }
    (args.output / "summary.json").write_text(
        json.dumps(summary, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps(summary, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
