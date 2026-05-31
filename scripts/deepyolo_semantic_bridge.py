#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Bridge DeepYOLO RealSense JSONL events into a compact GO2W summary.

The DeepYOLO example under librealsense writes event-oriented JSONL packets.
This bridge keeps that detector as an optional side process and exports only a
bounded latest-value summary for the UI, LLM context, and future C++ fusion.
It never forwards raw images, dense depth frames, or unbounded logs.
"""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path
from typing import Any, Optional


DEFAULT_INPUT_DIR = Path("/home/unitree/librealsense/examples/DeepYolo_test/output")
DEFAULT_OUTPUT = Path("artifacts/vision_semantic_summary.json")


def now_ms() -> int:
    return int(time.time() * 1000)


def newest_jsonl(input_dir: Path, pattern: str = "semantic_stream_*.jsonl") -> Optional[Path]:
    if not input_dir.exists():
        return None
    candidates = [p for p in input_dir.glob(pattern) if p.is_file() and p.stat().st_size > 0]
    if not candidates:
        return None
    return max(candidates, key=lambda p: (p.stat().st_mtime_ns, p.name))


def read_last_json_line(path: Path, *, max_scan_bytes: int = 1_000_000) -> Optional[dict[str, Any]]:
    if not path.exists() or path.stat().st_size == 0:
        return None
    size = path.stat().st_size
    offset = max(0, size - max_scan_bytes)
    with path.open("rb") as f:
        f.seek(offset)
        data = f.read().decode("utf-8", errors="replace")
    for raw in reversed(data.splitlines()):
        line = raw.strip()
        if not line:
            continue
        try:
            parsed = json.loads(line)
        except json.JSONDecodeError:
            continue
        if isinstance(parsed, dict):
            return parsed
    return None


def risk_rank(level: str) -> int:
    return {"high": 3, "medium": 2, "low": 1}.get(str(level), 0)


def normalize_object(obj: dict[str, Any]) -> dict[str, Any]:
    distance = obj.get("distance_m")
    return {
        "track_id": obj.get("track_id"),
        "class_name": str(obj.get("class_name", "unknown")),
        "confidence": obj.get("confidence"),
        "risk_level": str(obj.get("risk_level", "unknown")),
        "region": str(obj.get("region", "unknown")),
        "distance_m": distance if isinstance(distance, (int, float)) and distance >= 0 else None,
        "depth_valid": bool(obj.get("depth_valid", False)),
        "bbox": obj.get("bbox", []),
        "cx_norm": obj.get("cx_norm"),
        "cy_norm": obj.get("cy_norm"),
        "area_norm": obj.get("area_norm"),
        "persist_frames": obj.get("persist_frames"),
        "is_new": bool(obj.get("is_new", False)),
    }


def sorted_objects(objects: list[dict[str, Any]], max_objects: int) -> list[dict[str, Any]]:
    normalized = [normalize_object(obj) for obj in objects if isinstance(obj, dict)]
    normalized.sort(
        key=lambda item: (
            risk_rank(str(item.get("risk_level", ""))),
            float(item.get("confidence") or 0.0),
            float(item.get("area_norm") or 0.0),
        ),
        reverse=True,
    )
    return normalized[: max(0, max_objects)]


def recommended_action(objects: list[dict[str, Any]], *, center_stop_m: float, near_watch_m: float) -> str:
    for obj in objects:
        if obj.get("risk_level") != "high":
            continue
        distance = obj.get("distance_m")
        if obj.get("region") == "center" and isinstance(distance, (int, float)) and distance <= center_stop_m:
            return "hold_or_pause"
    for obj in objects:
        if obj.get("risk_level") == "high":
            return "slow_and_watch"
    for obj in objects:
        distance = obj.get("distance_m")
        if isinstance(distance, (int, float)) and distance <= near_watch_m:
            return "semantic_observe"
    return "normal"


def build_semantic_summary(
    packet: dict[str, Any],
    *,
    source_path: Optional[Path],
    max_objects: int = 8,
    stale_ms: int = 3000,
    bridge_timestamp_ms: Optional[int] = None,
    center_stop_m: float = 1.5,
    near_watch_m: float = 2.5,
) -> dict[str, Any]:
    bridge_ts = int(bridge_timestamp_ms or now_ms())
    packet_ts = packet.get("timestamp_ms")
    packet_ts = int(packet_ts) if isinstance(packet_ts, (int, float)) else bridge_ts
    age_ms = max(0, bridge_ts - packet_ts)
    objects = sorted_objects(packet.get("objects", []) if isinstance(packet.get("objects"), list) else [], max_objects)
    action = recommended_action(objects, center_stop_m=center_stop_m, near_watch_m=near_watch_m)
    return {
        "available": True,
        "source": "deepyolo_realsense",
        "timestamp_ms": packet_ts,
        "bridge_timestamp_ms": bridge_ts,
        "age_ms": age_ms,
        "stale_ms": stale_ms,
        "stale": age_ms > stale_ms,
        "source_path": str(source_path) if source_path else "",
        "session_id": packet.get("session_id", ""),
        "frame_id": packet.get("frame_id"),
        "image_width": packet.get("image_width"),
        "image_height": packet.get("image_height"),
        "object_count": int(packet.get("object_count") or len(objects)),
        "high_risk_count": int(packet.get("high_risk_count") or 0),
        "has_high_risk": bool(packet.get("has_high_risk", False)),
        "dominant_class": str(packet.get("dominant_class", "none")),
        "main_region": str(packet.get("main_region", "none")),
        "scene_state": str(packet.get("scene_state", "empty")),
        "depth_valid": bool(packet.get("depth_valid", False)),
        "center_depth_m": packet.get("center_depth_m"),
        "ir_left_valid": bool(packet.get("ir_left_valid", False)),
        "ir_right_valid": bool(packet.get("ir_right_valid", False)),
        "recommended_action": action,
        "objects": objects,
        "summary": {
            "schema": "go2w_deepyolo_semantic_summary_v1",
            "bounded_objects": max_objects,
            "policy": "optional_latest_value_only",
        },
    }


def unavailable_summary(reason: str, *, output: Optional[Path] = None, stale_ms: int = 3000) -> dict[str, Any]:
    ts = now_ms()
    return {
        "available": False,
        "source": "deepyolo_realsense",
        "timestamp_ms": ts,
        "bridge_timestamp_ms": ts,
        "age_ms": None,
        "stale_ms": stale_ms,
        "stale": True,
        "source_path": str(output) if output else "",
        "reason": reason,
        "object_count": 0,
        "high_risk_count": 0,
        "has_high_risk": False,
        "dominant_class": "none",
        "main_region": "none",
        "scene_state": "unavailable",
        "depth_valid": False,
        "center_depth_m": None,
        "recommended_action": "normal",
        "objects": [],
        "summary": {
            "schema": "go2w_deepyolo_semantic_summary_v1",
            "policy": "optional_latest_value_only",
        },
    }


def write_summary(summary: dict[str, Any], output: Path, *, pretty: bool = False) -> str:
    text = json.dumps(summary, ensure_ascii=False, indent=2 if pretty else None, separators=None if pretty else (",", ":"))
    output.parent.mkdir(parents=True, exist_ok=True)
    tmp = output.with_name(output.name + ".tmp")
    tmp.write_text(text + "\n", encoding="utf-8")
    tmp.replace(output)
    return text


def emit_once(args: argparse.Namespace) -> dict[str, Any]:
    source = args.input_jsonl or newest_jsonl(args.input_dir)
    if source is None:
        return unavailable_summary("no_deepyolo_jsonl", output=args.input_dir, stale_ms=args.stale_ms)
    packet = read_last_json_line(source)
    if packet is None:
        return unavailable_summary("no_valid_deepyolo_packet", output=source, stale_ms=args.stale_ms)
    return build_semantic_summary(
        packet,
        source_path=source,
        max_objects=args.max_objects,
        stale_ms=args.stale_ms,
        center_stop_m=args.center_stop_m,
        near_watch_m=args.near_watch_m,
    )


def run(args: argparse.Namespace) -> int:
    samples = 0
    while True:
        summary = emit_once(args)
        text = write_summary(summary, args.output, pretty=args.pretty)
        if args.print:
            print(text, flush=True)
        samples += 1
        if args.loop_interval_s <= 0 or (args.max_samples > 0 and samples >= args.max_samples):
            return 0
        time.sleep(args.loop_interval_s)


def make_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Bridge DeepYOLO RealSense JSONL into compact GO2W semantic summary")
    parser.add_argument("--input-dir", type=Path, default=DEFAULT_INPUT_DIR)
    parser.add_argument("--input-jsonl", type=Path)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--loop-interval-s", type=float, default=0.0)
    parser.add_argument("--max-samples", type=int, default=1)
    parser.add_argument("--max-objects", type=int, default=8)
    parser.add_argument("--stale-ms", type=int, default=3000)
    parser.add_argument("--center-stop-m", type=float, default=1.5)
    parser.add_argument("--near-watch-m", type=float, default=2.5)
    parser.add_argument("--pretty", action="store_true")
    parser.add_argument("--print", action="store_true")
    return parser


def main(argv: Optional[list[str]] = None) -> int:
    return run(make_parser().parse_args(argv))


if __name__ == "__main__":
    raise SystemExit(main())
