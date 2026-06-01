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
SCHEMA_VERSION = 1
DEFAULT_SOURCE_STALE_MS = 30_000
DEFAULT_CLOCK_SKEW_LIMIT_MS = 5_000


def now_ms() -> int:
    return int(time.time() * 1000)


def timestamp_ms(value: Any) -> Optional[int]:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    return int(value)


def nonnegative_number(value: Any) -> bool:
    return not isinstance(value, bool) and isinstance(value, (int, float)) and value >= 0


def file_mtime_ms(path: Path) -> Optional[int]:
    try:
        return path.stat().st_mtime_ns // 1_000_000
    except OSError:
        return None


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
        "distance_m": distance if nonnegative_number(distance) else None,
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
        if obj.get("region") == "center" and nonnegative_number(distance) and distance <= center_stop_m:
            return "hold_or_pause"
    for obj in objects:
        if obj.get("risk_level") == "high":
            return "slow_and_watch"
    for obj in objects:
        distance = obj.get("distance_m")
        if nonnegative_number(distance) and distance <= near_watch_m:
            return "semantic_observe"
    return "normal"


def action_has_sufficient_depth(
    action: str,
    objects: list[dict[str, Any]],
    packet_depth_valid: bool,
    *,
    center_stop_m: float,
    near_watch_m: float,
) -> bool:
    if action == "normal":
        return True
    if not packet_depth_valid:
        return False
    valid = [obj for obj in objects if bool(obj.get("depth_valid")) and nonnegative_number(obj.get("distance_m"))]
    if action == "hold_or_pause":
        return any(
            obj.get("risk_level") == "high"
            and obj.get("region") == "center"
            and obj["distance_m"] <= center_stop_m
            for obj in valid
        )
    if action == "slow_and_watch":
        return any(obj.get("risk_level") == "high" for obj in valid)
    if action == "semantic_observe":
        return any(obj["distance_m"] <= near_watch_m for obj in valid)
    return False


def effective_action_for(source_status: str, action: str) -> tuple[str, str]:
    if source_status == "fresh":
        return action, "fresh_source"
    return "ignored", source_status


def build_semantic_summary(
    packet: dict[str, Any],
    *,
    source_path: Optional[Path],
    max_objects: int = 8,
    stale_ms: int = 3000,
    source_stale_ms: int = DEFAULT_SOURCE_STALE_MS,
    clock_skew_limit_ms: int = DEFAULT_CLOCK_SKEW_LIMIT_MS,
    source_file_mtime_ms: Optional[int] = None,
    bridge_timestamp_ms: Optional[int] = None,
    center_stop_m: float = 1.5,
    near_watch_m: float = 2.5,
) -> dict[str, Any]:
    bridge_ts = int(now_ms() if bridge_timestamp_ms is None else bridge_timestamp_ms)
    packet_ts = timestamp_ms(packet.get("timestamp_ms"))
    if packet_ts is None:
        packet_ts = bridge_ts
    stale_ms = max(0, int(stale_ms))
    source_stale_ms = max(stale_ms, int(source_stale_ms))
    clock_skew_limit_ms = max(0, int(clock_skew_limit_ms))
    if source_file_mtime_ms is None and source_path is not None:
        source_file_mtime_ms = file_mtime_ms(source_path)
    inferred_source_mtime = source_file_mtime_ms is None
    if source_file_mtime_ms is None:
        source_file_mtime_ms = packet_ts
    source_file_mtime_ms = int(source_file_mtime_ms)
    packet_age_ms = bridge_ts - packet_ts
    source_file_age_ms = bridge_ts - source_file_mtime_ms
    clock_skew_ms = packet_ts - source_file_mtime_ms
    objects = sorted_objects(packet.get("objects", []) if isinstance(packet.get("objects"), list) else [], max_objects)
    action = recommended_action(objects, center_stop_m=center_stop_m, near_watch_m=near_watch_m)
    packet_depth_valid = bool(packet.get("depth_valid", False))
    sufficient_depth = action_has_sufficient_depth(
        action,
        objects,
        packet_depth_valid,
        center_stop_m=center_stop_m,
        near_watch_m=near_watch_m,
    )
    if source_file_age_ms > source_stale_ms:
        source_status = "stale"
    elif abs(clock_skew_ms) > clock_skew_limit_ms:
        source_status = "clock_skew"
    elif source_file_age_ms > stale_ms:
        source_status = "event_only_idle"
    elif not sufficient_depth:
        source_status = "depth_insufficient"
    else:
        source_status = "fresh"
    effective_action, effective_action_reason = effective_action_for(source_status, action)
    return {
        "schema_version": SCHEMA_VERSION,
        "available": True,
        "source": "deepyolo_realsense",
        "source_status": source_status,
        "timestamp_ms": packet_ts,
        "packet_timestamp_ms": packet_ts,
        "bridge_timestamp_ms": bridge_ts,
        "age_ms": max(0, packet_age_ms),
        "packet_age_ms": packet_age_ms,
        "source_file_mtime_ms": source_file_mtime_ms,
        "source_file_age_ms": source_file_age_ms,
        "source_file_mtime_inferred": inferred_source_mtime,
        "clock_skew_ms": clock_skew_ms,
        "clock_skew_limit_ms": clock_skew_limit_ms,
        "stale_ms": stale_ms,
        "source_stale_ms": source_stale_ms,
        "stale": source_status in {"event_only_idle", "stale", "clock_skew"},
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
        "depth_valid": packet_depth_valid,
        "center_depth_m": packet.get("center_depth_m"),
        "ir_left_valid": bool(packet.get("ir_left_valid", False)),
        "ir_right_valid": bool(packet.get("ir_right_valid", False)),
        "recommended_action": action,
        "effective_action": effective_action,
        "effective_action_reason": effective_action_reason,
        "action_depth_sufficient": sufficient_depth,
        "objects": objects,
        "summary": {
            "schema": "go2w_deepyolo_semantic_summary_v1",
            "bounded_objects": max_objects,
            "policy": "optional_event_only_latest_value",
        },
    }


def unavailable_summary(
    reason: str,
    *,
    output: Optional[Path] = None,
    stale_ms: int = 3000,
    source_stale_ms: int = DEFAULT_SOURCE_STALE_MS,
    clock_skew_limit_ms: int = DEFAULT_CLOCK_SKEW_LIMIT_MS,
) -> dict[str, Any]:
    ts = now_ms()
    stale_ms = max(0, int(stale_ms))
    source_stale_ms = max(stale_ms, int(source_stale_ms))
    clock_skew_limit_ms = max(0, int(clock_skew_limit_ms))
    return {
        "schema_version": SCHEMA_VERSION,
        "available": False,
        "source": "deepyolo_realsense",
        "source_status": "unavailable",
        "timestamp_ms": ts,
        "packet_timestamp_ms": None,
        "bridge_timestamp_ms": ts,
        "age_ms": None,
        "packet_age_ms": None,
        "source_file_mtime_ms": None,
        "source_file_age_ms": None,
        "source_file_mtime_inferred": False,
        "clock_skew_ms": None,
        "clock_skew_limit_ms": clock_skew_limit_ms,
        "stale_ms": stale_ms,
        "source_stale_ms": source_stale_ms,
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
        "effective_action": "ignored",
        "effective_action_reason": "unavailable",
        "action_depth_sufficient": False,
        "objects": [],
        "summary": {
            "schema": "go2w_deepyolo_semantic_summary_v1",
            "policy": "optional_event_only_latest_value",
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
    source_stale_ms = getattr(args, "source_stale_ms", DEFAULT_SOURCE_STALE_MS)
    clock_skew_limit_ms = getattr(args, "clock_skew_limit_ms", DEFAULT_CLOCK_SKEW_LIMIT_MS)
    if source is None:
        return unavailable_summary(
            "no_deepyolo_jsonl",
            output=args.input_dir,
            stale_ms=args.stale_ms,
            source_stale_ms=source_stale_ms,
            clock_skew_limit_ms=clock_skew_limit_ms,
        )
    packet = read_last_json_line(source)
    if packet is None:
        return unavailable_summary(
            "no_valid_deepyolo_packet",
            output=source,
            stale_ms=args.stale_ms,
            source_stale_ms=source_stale_ms,
            clock_skew_limit_ms=clock_skew_limit_ms,
        )
    return build_semantic_summary(
        packet,
        source_path=source,
        max_objects=args.max_objects,
        stale_ms=args.stale_ms,
        source_stale_ms=source_stale_ms,
        clock_skew_limit_ms=clock_skew_limit_ms,
        source_file_mtime_ms=file_mtime_ms(source),
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
    parser.add_argument("--source-stale-ms", type=int, default=DEFAULT_SOURCE_STALE_MS)
    parser.add_argument("--clock-skew-limit-ms", type=int, default=DEFAULT_CLOCK_SKEW_LIMIT_MS)
    parser.add_argument("--center-stop-m", type=float, default=1.5)
    parser.add_argument("--near-watch-m", type=float, default=2.5)
    parser.add_argument("--pretty", action="store_true")
    parser.add_argument("--print", action="store_true")
    return parser


def main(argv: Optional[list[str]] = None) -> int:
    return run(make_parser().parse_args(argv))


if __name__ == "__main__":
    raise SystemExit(main())
