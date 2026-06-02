#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Export a compact RealSense depth summary for the GO2W runtime.

The script intentionally emits only low-rate ROI clearances. It does not publish
raw frames, dense depth maps, or point clouds to the operator UI/LLM boundary.
"""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path
from typing import Any, Optional, Sequence


def now_ms() -> int:
    return int(time.time() * 1000)


def percentile(values: Sequence[float], q: float) -> Optional[float]:
    clean = sorted(float(v) for v in values if v is not None)
    if not clean:
        return None
    if len(clean) == 1:
        return clean[0]
    q = max(0.0, min(100.0, float(q)))
    pos = (q / 100.0) * (len(clean) - 1)
    lo = int(pos)
    hi = min(lo + 1, len(clean) - 1)
    frac = pos - lo
    return clean[lo] * (1.0 - frac) + clean[hi] * frac


def depth_shape(depth_mm: Sequence[Sequence[float]]) -> tuple[int, int]:
    h = len(depth_mm)
    w = len(depth_mm[0]) if h else 0
    return h, w


def roi_values_m(
    depth_mm: Sequence[Sequence[float]],
    *,
    x0: int,
    x1: int,
    y0: int,
    y1: int,
    min_m: float,
    max_m: float,
) -> list[float]:
    if hasattr(depth_mm, "shape"):
        import numpy as np

        region_m = np.asarray(depth_mm)[y0:y1, x0:x1].astype(np.float32, copy=False) * 0.001
        usable = region_m[(region_m >= min_m) & (region_m <= max_m)]
        return usable.tolist()

    out: list[float] = []
    for row in depth_mm[y0:y1]:
        for raw in row[x0:x1]:
            value_m = float(raw) * 0.001
            if min_m <= value_m <= max_m:
                out.append(value_m)
    return out


def roi_percentile_m(
    depth_mm: Sequence[Sequence[float]],
    *,
    x0: int,
    x1: int,
    y0: int,
    y1: int,
    min_m: float,
    max_m: float,
    q: float,
) -> Optional[float]:
    return percentile(
        roi_values_m(depth_mm, x0=x0, x1=x1, y0=y0, y1=y1, min_m=min_m, max_m=max_m),
        q,
    )


def valid_fraction(depth_mm: Sequence[Sequence[float]], *, min_m: float, max_m: float) -> float:
    if hasattr(depth_mm, "shape"):
        import numpy as np

        depth_m = np.asarray(depth_mm).astype(np.float32, copy=False) * 0.001
        if depth_m.size == 0:
            return 0.0
        return float(np.count_nonzero((depth_m >= min_m) & (depth_m <= max_m)) / depth_m.size)

    total = 0
    valid = 0
    for row in depth_mm:
        for raw in row:
            total += 1
            value_m = float(raw) * 0.001
            if min_m <= value_m <= max_m:
                valid += 1
    return 0.0 if total == 0 else valid / total


def valid_depth_m(raw: float, *, min_m: float, max_m: float) -> Optional[float]:
    value_m = float(raw) * 0.001
    if min_m <= value_m <= max_m:
        return value_m
    return None


def clipped_window(h: int, w: int, cx: int, cy: int, radius: int) -> tuple[int, int, int, int]:
    return max(0, cx - radius), min(w, cx + radius + 1), max(0, cy - radius), min(h, cy + radius + 1)


def build_depth_summary(
    depth_mm: Sequence[Sequence[float]],
    *,
    timestamp_ms: Optional[int] = None,
    frame_id: str = "camera_depth_optical_frame",
    source: str = "stereo_depth",
    min_m: float = 0.15,
    max_m: float = 8.0,
    q: float = 10.0,
    latency_ms: Optional[float] = None,
    stale: bool = False,
) -> dict[str, Any]:
    h, w = depth_shape(depth_mm)
    if h == 0 or w == 0:
        raise ValueError("empty depth image")

    y0, y1 = h // 3, (2 * h) // 3
    left = (0, w // 3)
    front = (w // 3, (2 * w) // 3)
    right = ((2 * w) // 3, w)
    cx, cy = w // 2, h // 2
    center_x0, center_x1, center_y0, center_y1 = clipped_window(h, w, cx, cy, 40)
    front_values = roi_values_m(depth_mm, x0=front[0], x1=front[1], y0=y0, y1=y1, min_m=min_m, max_m=max_m)
    left_values = roi_values_m(depth_mm, x0=left[0], x1=left[1], y0=y0, y1=y1, min_m=min_m, max_m=max_m)
    right_values = roi_values_m(depth_mm, x0=right[0], x1=right[1], y0=y0, y1=y1, min_m=min_m, max_m=max_m)
    center_values = roi_values_m(
        depth_mm,
        x0=center_x0,
        x1=center_x1,
        y0=center_y0,
        y1=center_y1,
        min_m=min_m,
        max_m=max_m,
    )
    front_total = max(1, (front[1] - front[0]) * (y1 - y0))
    left_total = max(1, (left[1] - left[0]) * (y1 - y0))
    right_total = max(1, (right[1] - right[0]) * (y1 - y0))
    center_total = max(1, (center_x1 - center_x0) * (center_y1 - center_y0))

    return {
        "source": source,
        "timestamp_ms": int(timestamp_ms or now_ms()),
        "frame_id": frame_id,
        "center_distance_m": valid_depth_m(depth_mm[cy][cx], min_m=min_m, max_m=max_m),
        "center_window_m": percentile(center_values, 50.0),
        "front_clearance_m": percentile(front_values, q),
        "left_clearance_m": percentile(left_values, q),
        "right_clearance_m": percentile(right_values, q),
        "rear_clearance_m": None,
        "confidence": round(valid_fraction(depth_mm, min_m=min_m, max_m=max_m), 3),
        "roi_confidence": {
            "front": round(len(front_values) / front_total, 3),
            "left": round(len(left_values) / left_total, 3),
            "right": round(len(right_values) / right_total, 3),
            "center_window": round(len(center_values) / center_total, 3),
        },
        "latency_ms": latency_ms,
        "stale": bool(stale),
        "summary": {
            "shape": [h, w],
            "percentile": q,
            "valid_range_m": [min_m, max_m],
            "confidence_scope": "whole_image_valid_fraction",
            "center_window_px": [center_x1 - center_x0, center_y1 - center_y0],
        },
    }


def load_depth_npy(path: Path) -> Any:
    import numpy as np  # Imported lazily so unit tests do not require NumPy.

    return np.load(path)


def capture_realsense_depth(*, width: int, height: int, fps: int, frames: int, timeout_ms: int) -> tuple[Any, float]:
    import numpy as np  # Imported lazily on the robot.
    import pyrealsense2 as rs

    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
    start = time.time()
    try:
        pipeline.start(config)
        depth_frame = None
        for _ in range(max(1, frames)):
            frameset = pipeline.wait_for_frames(timeout_ms)
            depth_frame = frameset.get_depth_frame()
        if depth_frame is None:
            raise RuntimeError("no depth frame received")
        depth = np.asanyarray(depth_frame.get_data())
        latency_ms = (time.time() - start) * 1000.0
        return depth, latency_ms
    finally:
        pipeline.stop()


def write_summary(summary: dict[str, Any], output: Optional[Path], *, pretty: bool = False) -> str:
    text = json.dumps(summary, ensure_ascii=False, indent=2 if pretty else None, separators=None if pretty else (",", ":"))
    if output:
        output.parent.mkdir(parents=True, exist_ok=True)
        tmp = output.with_name(output.name + ".tmp")
        tmp.write_text(text + "\n", encoding="utf-8")
        tmp.replace(output)
    return text


def emit_depth_summary(depth: Any, args: argparse.Namespace, *, latency_ms: Optional[float]) -> str:
    summary = build_depth_summary(
        depth,
        min_m=args.min_m,
        max_m=args.max_m,
        q=args.percentile,
        latency_ms=latency_ms,
    )
    text = write_summary(summary, args.output, pretty=args.pretty)
    if not getattr(args, "quiet", False):
        print(text, flush=True)
    return text


def run_saved_depth(args: argparse.Namespace) -> int:
    samples = 0
    while True:
        depth = load_depth_npy(args.from_npy)
        emit_depth_summary(depth, args, latency_ms=None)
        samples += 1
        if args.loop_interval_s <= 0 or (args.max_samples > 0 and samples >= args.max_samples):
            return 0
        time.sleep(args.loop_interval_s)


def run_realsense_stream(args: argparse.Namespace) -> int:
    import numpy as np  # Imported lazily on the robot.
    import pyrealsense2 as rs

    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, args.width, args.height, rs.format.z16, args.fps)
    samples = 0
    try:
        pipeline.start(config)
        while True:
            sample_start = time.time()
            depth_frame = None
            for _ in range(max(1, args.frames)):
                frameset = pipeline.wait_for_frames(args.timeout_ms)
                depth_frame = frameset.get_depth_frame()
            if depth_frame is None:
                raise RuntimeError("no depth frame received")
            depth = np.asanyarray(depth_frame.get_data())
            emit_depth_summary(depth, args, latency_ms=(time.time() - sample_start) * 1000.0)
            samples += 1
            if args.loop_interval_s <= 0 or (args.max_samples > 0 and samples >= args.max_samples):
                return 0
            elapsed = time.time() - sample_start
            time.sleep(max(0.0, args.loop_interval_s - elapsed))
    finally:
        pipeline.stop()


def make_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Capture/export compact RealSense depth ROI summary")
    parser.add_argument("--from-npy", type=Path, help="Read uint16 millimeter depth image from .npy instead of live camera")
    parser.add_argument("--output", type=Path, default=Path("artifacts/stereo_depth_summary.json"))
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=15)
    parser.add_argument("--frames", type=int, default=3)
    parser.add_argument("--timeout-ms", type=int, default=10000)
    parser.add_argument("--min-m", type=float, default=0.15)
    parser.add_argument("--max-m", type=float, default=8.0)
    parser.add_argument("--percentile", type=float, default=10.0)
    parser.add_argument("--loop-interval-s", type=float, default=0.0, help="Repeat capture at this interval; 0 keeps one-shot behavior")
    parser.add_argument("--max-samples", type=int, default=1, help="Maximum summaries to emit when looping; 0 means run until stopped")
    parser.add_argument("--pretty", action="store_true")
    parser.add_argument("--quiet", action="store_true", help="Write the summary without printing every sample")
    return parser


def main(argv: Optional[list[str]] = None) -> int:
    args = make_parser().parse_args(argv)
    if args.from_npy:
        return run_saved_depth(args)
    if args.loop_interval_s > 0:
        return run_realsense_stream(args)
    depth, latency_ms = capture_realsense_depth(
        width=args.width,
        height=args.height,
        fps=args.fps,
        frames=args.frames,
        timeout_ms=args.timeout_ms,
    )
    emit_depth_summary(depth, args, latency_ms=latency_ms)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
