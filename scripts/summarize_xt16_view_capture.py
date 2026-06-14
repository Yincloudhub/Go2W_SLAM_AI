#!/usr/bin/env python3
from __future__ import annotations

import argparse
import hashlib
import json
import math
import statistics
from collections import Counter
from pathlib import Path
from typing import Any


def percentile(values: list[float], q: float) -> float | None:
    ordered = sorted(value for value in values if math.isfinite(value))
    if not ordered:
        return None
    position = max(0.0, min(1.0, q)) * (len(ordered) - 1)
    lower = int(position)
    upper = min(lower + 1, len(ordered) - 1)
    fraction = position - lower
    return ordered[lower] * (1.0 - fraction) + ordered[upper] * fraction


def stats(values: list[float]) -> dict[str, Any]:
    return {
        "count": len(values),
        "min": min(values) if values else None,
        "p05": percentile(values, 0.05),
        "median": statistics.median(values) if values else None,
        "p95": percentile(values, 0.95),
        "max": max(values) if values else None,
    }


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("capture")
    parser.add_argument("--scene-id", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--note", default="")
    args = parser.parse_args()

    capture = Path(args.capture)
    frames = [
        json.loads(line)
        for line in capture.read_text(encoding="utf-8").splitlines()
        if line.strip()
    ]
    if not frames:
        raise SystemExit("capture contains no frames")

    clearances: dict[str, list[float]] = {
        direction: [] for direction in ("front", "left", "right", "rear")
    }
    excluded: list[float] = []
    raw_points: list[float] = []
    classes: Counter[str] = Counter()
    near_right_body: list[float] = []
    near_left_body: list[float] = []

    for frame in frames:
        geometry = frame.get("geometry", {})
        for direction in clearances:
            value = geometry.get(f"{direction}_clearance_m")
            if isinstance(value, (int, float)) and math.isfinite(float(value)):
                clearances[direction].append(float(value))
        value = geometry.get("points_excluded_footprint")
        if isinstance(value, (int, float)):
            excluded.append(float(value))
        raw_points.append(float(frame.get("raw_points") or 0))
        for forward, lateral, _vertical, point_class in frame.get("plot_points", []):
            classes[str(point_class)] += 1
            if point_class != "body_height" or abs(float(forward)) > 0.75:
                continue
            if float(lateral) < 0:
                near_right_body.append(abs(float(lateral)))
            elif float(lateral) > 0:
                near_left_body.append(abs(float(lateral)))

    result = {
        "schema_version": 1,
        "scene_id": args.scene_id,
        "capture_path": capture.as_posix(),
        "capture_sha256": sha256(capture),
        "frames": len(frames),
        "sequence": [frames[0].get("sequence"), frames[-1].get("sequence")],
        "timestamp_ms": [
            frames[0].get("timestamp_ms"),
            frames[-1].get("timestamp_ms"),
        ],
        "topic": frames[0].get("topic"),
        "frame_id": frames[0].get("frame_id"),
        "note": args.note or None,
        "geometry": frames[0].get("geometry", {}).get("footprint_m"),
        "clearance_m": {
            direction: stats(values) for direction, values in clearances.items()
        },
        "raw_points": stats(raw_points),
        "points_excluded_footprint": stats(excluded),
        "sampled_point_classes": dict(sorted(classes.items())),
        "nearest_sampled_body_center_distance_m": {
            "left": min(near_left_body) if near_left_body else None,
            "right": min(near_right_body) if near_right_body else None,
        },
    }
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(
        json.dumps(result, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
