#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Reduce one D435 capture stream and optional YOLO events into stable summaries."""

from __future__ import annotations

import argparse
import json
import os
import time
import uuid
from pathlib import Path
from typing import Any, Optional

import deepyolo_semantic_bridge as semantic_bridge


SCHEMA_VERSION = 1
DEFAULT_DEPTH_STALE_MS = 1000
DEFAULT_YOLO_STALE_MS = 3000
DEFAULT_YOLO_SOURCE_STALE_MS = 30_000


def now_ms() -> int:
    return int(time.time() * 1000)


def read_json(path: Path) -> Optional[dict[str, Any]]:
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return None
    return data if isinstance(data, dict) else None


def read_pid(path: Path) -> Optional[int]:
    try:
        value = int(path.read_text(encoding="ascii").strip())
    except (OSError, ValueError):
        return None
    return value if value > 0 else None


def capture_owner_status(
    pid_file: Path,
    *,
    expected_fragment: str,
    proc_root: Path = Path("/proc"),
) -> dict[str, Any]:
    pid = read_pid(pid_file)
    cmdline = ""
    if pid is not None:
        try:
            cmdline = (proc_root / str(pid) / "cmdline").read_bytes().replace(b"\0", b" ").decode(
                "utf-8", errors="replace"
            )
        except OSError:
            cmdline = ""
    running = bool(pid is not None and cmdline and expected_fragment in cmdline)
    process_start_ticks = None
    boot_id = None
    if running and pid is not None:
        try:
            stat_fields = (proc_root / str(pid) / "stat").read_text(encoding="ascii").split()
            process_start_ticks = int(stat_fields[21])
        except (OSError, ValueError, IndexError):
            process_start_ticks = None
        try:
            boot_id = (proc_root / "sys" / "kernel" / "random" / "boot_id").read_text(encoding="ascii").strip()
        except OSError:
            boot_id = None
    return {
        "pid": pid,
        "running": running,
        "expected_process": expected_fragment,
        "process_start_ticks": process_start_ticks,
        "boot_id": boot_id,
        "status": "owned" if running else "offline",
    }


def _integer(value: Any) -> Optional[int]:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    return int(value)


def _number(value: Any) -> Optional[float]:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    return float(value)


def _nonnegative_number(value: Any) -> Optional[float]:
    parsed = _number(value)
    return parsed if parsed is not None and parsed >= 0 else None


def _valid_depth_packet(depth: dict[str, Any]) -> bool:
    captured_at_ms = _integer(depth.get("captured_at_ms") or depth.get("timestamp_ms"))
    sequence = _integer(depth.get("frame_sequence") or depth.get("sequence"))
    sensor_timestamp_ms = _nonnegative_number(depth.get("sensor_timestamp_ms"))
    confidence = _number(depth.get("confidence"))
    roi = depth.get("roi_confidence")
    if (
        captured_at_ms is None
        or sequence is None
        or sequence <= 0
        or sensor_timestamp_ms is None
        or confidence is None
        or not 0 <= confidence <= 1
        or not isinstance(depth.get("frame_id"), str)
        or not depth.get("frame_id")
        or not isinstance(roi, dict)
        or not isinstance(depth.get("stale"), bool)
    ):
        return False
    for key in ("front_clearance_m", "left_clearance_m", "right_clearance_m", "center_distance_m"):
        value = depth.get(key)
        if value is not None and _nonnegative_number(value) is None:
            return False
    for key in ("front", "left", "right", "center_window"):
        value = _number(roi.get(key))
        if value is None or not 0 <= value <= 1:
            return False
    return True


def _depth_status(
    depth: Optional[dict[str, Any]],
    *,
    generated_at_ms: int,
    stale_ms: int,
    owner_running: bool,
) -> tuple[str, Optional[int]]:
    if not owner_running:
        return "offline", None
    if depth is None:
        return "offline", None
    if not _valid_depth_packet(depth):
        return "invalid", None
    captured_at_ms = _integer(depth.get("captured_at_ms") or depth.get("timestamp_ms"))
    sequence = _integer(depth.get("frame_sequence") or depth.get("sequence"))
    if captured_at_ms is None or sequence is None or sequence <= 0:
        return "invalid", None
    age_ms = generated_at_ms - captured_at_ms
    if age_ms < 0:
        return "invalid", age_ms
    if age_ms > stale_ms or bool(depth.get("stale", False)):
        return "stale", age_ms
    return "fresh", age_ms


def _build_yolo_summary(
    packet: Optional[dict[str, Any]],
    *,
    source_path: Optional[Path],
    generated_at_ms: int,
    stale_ms: int,
    source_stale_ms: int,
    max_objects: int,
) -> dict[str, Any]:
    if packet is None:
        return semantic_bridge.unavailable_summary(
            "no_deepyolo_packet",
            output=source_path,
            stale_ms=stale_ms,
            source_stale_ms=source_stale_ms,
        )
    source_mtime_ms = semantic_bridge.file_mtime_ms(source_path) if source_path else None
    return semantic_bridge.build_semantic_summary(
        packet,
        source_path=source_path,
        max_objects=max_objects,
        stale_ms=stale_ms,
        source_stale_ms=source_stale_ms,
        source_file_mtime_ms=source_mtime_ms,
        bridge_timestamp_ms=generated_at_ms,
    )


def build_summaries(
    depth_packet: Optional[dict[str, Any]],
    yolo_packet: Optional[dict[str, Any]],
    *,
    owner: dict[str, Any],
    generated_at_ms: Optional[int] = None,
    yolo_source_path: Optional[Path] = None,
    depth_stale_ms: int = DEFAULT_DEPTH_STALE_MS,
    yolo_stale_ms: int = DEFAULT_YOLO_STALE_MS,
    yolo_source_stale_ms: int = DEFAULT_YOLO_SOURCE_STALE_MS,
    max_objects: int = 8,
    generation_id: Optional[str] = None,
    previous_frame_sequence: Optional[int] = None,
) -> tuple[dict[str, Any], dict[str, Any], dict[str, Any]]:
    generated_at_ms = int(now_ms() if generated_at_ms is None else generated_at_ms)
    generation_id = generation_id or f"{generated_at_ms}-{uuid.uuid4().hex[:12]}"
    depth_stale_ms = max(1, int(depth_stale_ms))
    yolo_stale_ms = max(1, int(yolo_stale_ms))
    yolo_source_stale_ms = max(yolo_stale_ms, int(yolo_source_stale_ms))
    depth_status, depth_age_ms = _depth_status(
        depth_packet,
        generated_at_ms=generated_at_ms,
        stale_ms=depth_stale_ms,
        owner_running=bool(owner.get("running", False)),
    )

    depth = dict(depth_packet or {})
    capture_sequence = _integer(depth.get("frame_sequence") or depth.get("sequence"))
    if (
        depth_status == "fresh"
        and previous_frame_sequence is not None
        and capture_sequence is not None
        and capture_sequence < previous_frame_sequence
    ):
        depth_status = "invalid"
    sensor_timestamp_ms = _nonnegative_number(depth.get("sensor_timestamp_ms"))
    raw_confidence = depth.get("confidence")
    confidence = (
        max(0.0, min(1.0, float(raw_confidence)))
        if isinstance(raw_confidence, (int, float)) and not isinstance(raw_confidence, bool)
        else 0.0
    )
    captured_at_ms = _integer(depth.get("captured_at_ms") or depth.get("timestamp_ms"))
    if captured_at_ms is None:
        captured_at_ms = generated_at_ms

    yolo = _build_yolo_summary(
        yolo_packet,
        source_path=yolo_source_path,
        generated_at_ms=generated_at_ms,
        stale_ms=yolo_stale_ms,
        source_stale_ms=yolo_source_stale_ms,
        max_objects=max_objects,
    )
    yolo["frame_sequence"] = _integer((yolo_packet or {}).get("frame_sequence"))
    yolo["sensor_timestamp_ms"] = (yolo_packet or {}).get("sensor_timestamp_ms")
    yolo["captured_at_ms"] = _integer((yolo_packet or {}).get("captured_at_ms"))
    yolo["generation_id"] = generation_id
    yolo_sequence = _integer(yolo.get("frame_sequence"))
    yolo["capture_frame_lag"] = (
        capture_sequence - yolo_sequence
        if capture_sequence is not None and yolo_sequence is not None
        else None
    )
    yolo["envelope_status"] = (
        "fresh"
        if yolo.get("source_status") in {"fresh", "event_only_idle", "depth_insufficient"}
        else ("stale" if yolo.get("source_status") in {"stale", "clock_skew"} else "offline")
    )

    roi = depth.get("roi_confidence") if isinstance(depth.get("roi_confidence"), dict) else {}
    depth_compat = dict(depth)
    depth_compat.update(
        {
            "source": "stereo_depth",
            "timestamp_ms": captured_at_ms,
            "frame_sequence": capture_sequence,
            "sensor_timestamp_ms": sensor_timestamp_ms,
            "sensor_timestamp_domain": (
                depth.get("sensor_timestamp_domain")
                if isinstance(depth.get("sensor_timestamp_domain"), str)
                else None
            ),
            "frame_id": (
                depth.get("frame_id")
                if isinstance(depth.get("frame_id"), str) and depth.get("frame_id")
                else "camera_color_optical_frame"
            ),
            "front_clearance_m": _nonnegative_number(depth.get("front_clearance_m")),
            "left_clearance_m": _nonnegative_number(depth.get("left_clearance_m")),
            "right_clearance_m": _nonnegative_number(depth.get("right_clearance_m")),
            "center_distance_m": _nonnegative_number(depth.get("center_distance_m")),
            "confidence": confidence,
            "roi_confidence": {
                key: max(0.0, min(1.0, _number(roi.get(key)) or 0.0))
                for key in ("front", "left", "right", "center_window")
            },
            "age_ms": depth_age_ms,
            "stale_ms": depth_stale_ms,
            "stale": depth_status != "fresh",
            "source_status": depth_status,
            "producer": "d435_perception_sidecar",
            "owner_pid": owner.get("pid"),
            "generation_id": generation_id,
        }
    )

    yolo_compat = dict(yolo)
    yolo_compat["producer"] = "d435_perception_sidecar"

    degraded: list[str] = []
    if depth_status != "fresh":
        degraded.append("d435_depth")
    if yolo.get("source_status") not in {"fresh", "event_only_idle", "depth_insufficient"}:
        degraded.append("d435_yolo")

    main = {
        "schema_version": SCHEMA_VERSION,
        "schema": "go2w_d435_perception_summary_v1",
        "source": "d435_perception",
        "source_id": "d435_rgbd",
        "source_kind": "rgbd_perception",
        "producer": "d435_perception_sidecar",
        "generated_at_ms": generated_at_ms,
        "timestamp_ms": captured_at_ms,
        "generation_id": generation_id,
        "status": depth_status,
        "stale": depth_status != "fresh",
        "frame_sequence": capture_sequence,
        "sensor_timestamp_ms": sensor_timestamp_ms,
        "sensor_timestamp_domain": depth_compat["sensor_timestamp_domain"],
        "confidence": confidence,
        "owner": dict(owner),
        "capture": {
            "status": depth_status,
            "frame_sequence": capture_sequence,
            "sensor_timestamp_ms": sensor_timestamp_ms,
            "captured_at_ms": captured_at_ms,
            "age_ms": depth_age_ms,
            "stale_ms": depth_stale_ms,
            "frame_id": depth_compat["frame_id"],
        },
        "depth": depth_compat,
        "yolo": yolo_compat,
        "degraded_capabilities": degraded,
        "policy": {
            "motion_authority": "slam_gateway",
            "depth_role": "conservative_forward_supplement",
            "yolo_role": "semantic_caution_only",
            "llm_direct_motion": False,
        },
    }
    return main, depth_compat, yolo_compat


def write_outputs(
    main: dict[str, Any],
    depth_compat: dict[str, Any],
    yolo_compat: dict[str, Any],
    *,
    output: Path,
    depth_output: Path,
    yolo_output: Path,
    pretty: bool = False,
) -> None:
    targets = ((depth_output, depth_compat), (yolo_output, yolo_compat), (output, main))
    prepared: list[tuple[Path, Path]] = []
    for path, payload in targets:
        path.parent.mkdir(parents=True, exist_ok=True)
        tmp = path.with_name(f".{path.name}.{os.getpid()}.tmp")
        text = json.dumps(
            payload,
            ensure_ascii=False,
            indent=2 if pretty else None,
            separators=None if pretty else (",", ":"),
        )
        tmp.write_text(text + "\n", encoding="utf-8")
        prepared.append((tmp, path))
    for tmp, path in prepared:
        tmp.replace(path)


def emit_once(
    args: argparse.Namespace,
    *,
    previous_frame_sequence: Optional[int] = None,
    previous_owner_identity: Optional[tuple[Any, Any, Any]] = None,
) -> tuple[dict[str, Any], dict[str, Any], dict[str, Any]]:
    depth_packet = read_json(args.depth_input)
    yolo_path = args.yolo_jsonl or semantic_bridge.newest_jsonl(args.yolo_input_dir)
    yolo_packet = semantic_bridge.read_last_json_line(yolo_path) if yolo_path else None
    owner = capture_owner_status(
        args.capture_owner_pid_file,
        expected_fragment=args.capture_owner_process,
        proc_root=args.proc_root,
    )
    owner_identity = (owner.get("pid"), owner.get("process_start_ticks"), owner.get("boot_id"))
    if owner_identity != previous_owner_identity:
        previous_frame_sequence = None
    return build_summaries(
        depth_packet,
        yolo_packet,
        owner=owner,
        yolo_source_path=yolo_path,
        depth_stale_ms=args.depth_stale_ms,
        yolo_stale_ms=args.yolo_stale_ms,
        yolo_source_stale_ms=args.yolo_source_stale_ms,
        max_objects=args.max_objects,
        previous_frame_sequence=previous_frame_sequence,
    )


def run(args: argparse.Namespace) -> int:
    samples = 0
    previous_frame_sequence: Optional[int] = None
    previous_owner_identity: Optional[tuple[Any, Any, Any]] = None
    while True:
        main, depth, yolo = emit_once(
            args,
            previous_frame_sequence=previous_frame_sequence,
            previous_owner_identity=previous_owner_identity,
        )
        write_outputs(
            main,
            depth,
            yolo,
            output=args.output,
            depth_output=args.depth_output,
            yolo_output=args.yolo_output,
            pretty=args.pretty,
        )
        if args.print:
            print(json.dumps(main, ensure_ascii=False), flush=True)
        owner = main.get("owner", {})
        owner_identity = (owner.get("pid"), owner.get("process_start_ticks"), owner.get("boot_id"))
        if owner_identity != previous_owner_identity:
            previous_frame_sequence = None
        sequence = _integer(main.get("frame_sequence"))
        if main.get("status") == "fresh" and sequence is not None:
            previous_frame_sequence = sequence
        previous_owner_identity = owner_identity
        samples += 1
        if args.loop_interval_s <= 0 or (args.max_samples > 0 and samples >= args.max_samples):
            return 0
        time.sleep(args.loop_interval_s)


def make_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Build unified D435 depth and YOLO summaries")
    parser.add_argument("--depth-input", type=Path, required=True)
    parser.add_argument("--yolo-input-dir", type=Path, required=True)
    parser.add_argument("--yolo-jsonl", type=Path)
    parser.add_argument("--output", type=Path, default=Path("artifacts/d435_perception_summary.json"))
    parser.add_argument("--depth-output", type=Path, default=Path("artifacts/stereo_depth_summary.json"))
    parser.add_argument("--yolo-output", type=Path, default=Path("artifacts/vision_semantic_summary.json"))
    parser.add_argument("--capture-owner-pid-file", type=Path, required=True)
    parser.add_argument("--capture-owner-process", default="yolo_test_realsense_headless")
    parser.add_argument("--proc-root", type=Path, default=Path("/proc"))
    parser.add_argument("--depth-stale-ms", type=int, default=DEFAULT_DEPTH_STALE_MS)
    parser.add_argument("--yolo-stale-ms", type=int, default=DEFAULT_YOLO_STALE_MS)
    parser.add_argument("--yolo-source-stale-ms", type=int, default=DEFAULT_YOLO_SOURCE_STALE_MS)
    parser.add_argument("--max-objects", type=int, default=8)
    parser.add_argument("--loop-interval-s", type=float, default=0.1)
    parser.add_argument("--max-samples", type=int, default=0)
    parser.add_argument("--pretty", action="store_true")
    parser.add_argument("--print", action="store_true")
    return parser


def main(argv: Optional[list[str]] = None) -> int:
    return run(make_parser().parse_args(argv))


if __name__ == "__main__":
    raise SystemExit(main())
