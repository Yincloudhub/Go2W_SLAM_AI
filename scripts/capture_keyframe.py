#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path
from typing import Any


REPO_ROOT = Path(__file__).resolve().parents[1]
try:
    sys.stdout.reconfigure(encoding="utf-8")
    sys.stderr.reconfigure(encoding="utf-8")
except (AttributeError, ValueError):
    pass


def now_ms() -> int:
    return int(time.time() * 1000)


def safe_token(value: str, fallback: str) -> str:
    cleaned = "".join(ch if ch.isalnum() or ch in {"-", "_"} else "_" for ch in value.strip())
    cleaned = cleaned.strip("_")
    return cleaned or fallback


def json_line(payload: dict[str, Any]) -> str:
    return json.dumps(payload, ensure_ascii=False, separators=(",", ":"))


def first_json_object(text: str) -> dict[str, Any] | None:
    for line in reversed(text.splitlines()):
        line = line.strip()
        if not line.startswith("{"):
            continue
        try:
            value = json.loads(line)
        except json.JSONDecodeError:
            continue
        if isinstance(value, dict):
            return value
    return None


def build_paths(args: argparse.Namespace, timestamp_ms: int) -> tuple[str, Path, Path]:
    run_id = args.run_id or os.environ.get("GO2W_RUN_ID") or time.strftime("run_%Y%m%d_%H%M%S")
    target_node = args.target_node or os.environ.get("GO2W_TARGET_NODE") or "unknown_target"
    output_image_env = os.environ.get("GO2W_OUTPUT_IMAGE", "")
    if args.output_image or output_image_env:
        image_path = Path(args.output_image or output_image_env).expanduser()
        if not image_path.is_absolute():
            image_path = (REPO_ROOT / image_path).resolve()
        image_path.parent.mkdir(parents=True, exist_ok=True)
    else:
        output_dir = Path(
            args.output_dir
            or os.environ.get("GO2W_KEYFRAME_DIR")
            or REPO_ROOT / "artifacts" / "keyframes" / run_id
        ).expanduser()
        if not output_dir.is_absolute():
            output_dir = (REPO_ROOT / output_dir).resolve()
        output_dir.mkdir(parents=True, exist_ok=True)
        image_path = output_dir / f"{timestamp_ms}_{safe_token(target_node, 'target')}.jpg"
    sidecar_path = image_path.with_suffix(".json")
    return run_id, image_path, sidecar_path


def write_sidecar(payload: dict[str, Any], sidecar_path: Path) -> None:
    sidecar_path.parent.mkdir(parents=True, exist_ok=True)
    sidecar_path.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def copy_source_image(source_image: str, output_image: Path) -> tuple[bool, str]:
    source = Path(source_image).expanduser()
    if not source.is_absolute():
        source = (Path.cwd() / source).resolve()
    if not source.exists() or not source.is_file():
        return False, f"source image not found: {source}"
    shutil.copyfile(source, output_image)
    return True, "source_image"


def capture_with_external_command(args: argparse.Namespace, output_image: Path, run_id: str) -> dict[str, Any]:
    command = args.command or os.environ.get("GO2W_CAMERA_CAPTURE_COMMAND", "")
    env = os.environ.copy()
    env["GO2W_OUTPUT_IMAGE"] = str(output_image)
    env["GO2W_KEYFRAME_DIR"] = str(output_image.parent)
    env["GO2W_RUN_ID"] = run_id
    env["GO2W_TARGET_NODE"] = args.target_node or env.get("GO2W_TARGET_NODE", "")
    env["GO2W_TARGET_NAME"] = args.target_name or env.get("GO2W_TARGET_NAME", "")
    completed = subprocess.run(
        ["bash", "-lc", command],
        text=True,
        encoding="utf-8",
        errors="replace",
        capture_output=True,
        timeout=args.timeout_s,
        env=env,
    )
    parsed = first_json_object(completed.stdout)
    captured = completed.returncode == 0 and output_image.exists() and output_image.stat().st_size > 0
    payload: dict[str, Any] = {
        "captured": captured,
        "source": "external_command",
        "returncode": completed.returncode,
        "stdout": completed.stdout,
        "stderr": completed.stderr,
    }
    if isinstance(parsed, dict):
        payload["command_payload"] = parsed
        if not captured and parsed.get("captured") is True and parsed.get("image_path"):
            candidate = Path(str(parsed["image_path"]))
            payload["captured"] = candidate.exists() and candidate.stat().st_size > 0
            payload["image_path"] = str(candidate)
    if not payload["captured"]:
        payload["reason"] = "capture command did not create a non-empty image"
    return payload


def capture_with_opencv(args: argparse.Namespace, output_image: Path) -> dict[str, Any]:
    try:
        import cv2  # type: ignore
    except Exception as exc:  # pragma: no cover - depends on robot image
        return {"captured": False, "source": "opencv", "reason": f"opencv unavailable: {exc}"}

    cap = cv2.VideoCapture(args.device_index)
    if args.width > 0:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    if args.height > 0:
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    try:
        if not cap.isOpened():
            return {"captured": False, "source": "opencv", "reason": f"camera index {args.device_index} not opened"}
        ok, frame = cap.read()
        if not ok or frame is None:
            return {"captured": False, "source": "opencv", "reason": "failed to read camera frame"}
        if not cv2.imwrite(str(output_image), frame):
            return {"captured": False, "source": "opencv", "reason": "failed to write image"}
    finally:
        cap.release()
    return {"captured": True, "source": "opencv", "device_index": args.device_index}


def capture_with_ffmpeg(args: argparse.Namespace, output_image: Path) -> dict[str, Any]:
    ffmpeg = shutil.which("ffmpeg")
    video_device = args.video_device or os.environ.get("GO2W_VIDEO_DEVICE") or "/dev/video0"
    if not ffmpeg:
        return {"captured": False, "source": "ffmpeg", "reason": "ffmpeg unavailable"}
    if not Path(video_device).exists():
        return {"captured": False, "source": "ffmpeg", "reason": f"video device not found: {video_device}"}
    command = [
        ffmpeg,
        "-hide_banner",
        "-loglevel",
        "error",
        "-y",
        "-f",
        "v4l2",
        "-i",
        video_device,
        "-frames:v",
        "1",
        str(output_image),
    ]
    completed = subprocess.run(command, text=True, capture_output=True, timeout=args.timeout_s)
    captured = completed.returncode == 0 and output_image.exists() and output_image.stat().st_size > 0
    return {
        "captured": captured,
        "source": "ffmpeg",
        "video_device": video_device,
        "returncode": completed.returncode,
        "stderr": completed.stderr,
        "reason": "" if captured else "ffmpeg did not create a non-empty image",
    }


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Capture one GO2W inspection keyframe and emit JSON.")
    parser.add_argument("--target-node", default=os.environ.get("GO2W_TARGET_NODE", ""))
    parser.add_argument("--target-name", default=os.environ.get("GO2W_TARGET_NAME", ""))
    parser.add_argument("--run-id", default=os.environ.get("GO2W_RUN_ID", ""))
    parser.add_argument("--output-dir", default=os.environ.get("GO2W_KEYFRAME_DIR", ""))
    parser.add_argument("--output-image", default=os.environ.get("GO2W_OUTPUT_IMAGE", ""))
    parser.add_argument("--source-image", default="", help="Copy an existing image; mainly for tests and replay.")
    parser.add_argument("--command", default="", help="External bash capture command. It receives GO2W_OUTPUT_IMAGE.")
    parser.add_argument("--device-index", type=int, default=int(os.environ.get("GO2W_CAMERA_DEVICE_INDEX", "0") or 0))
    parser.add_argument("--video-device", default=os.environ.get("GO2W_VIDEO_DEVICE", "/dev/video0"))
    parser.add_argument("--width", type=int, default=int(os.environ.get("GO2W_CAMERA_WIDTH", "0") or 0))
    parser.add_argument("--height", type=int, default=int(os.environ.get("GO2W_CAMERA_HEIGHT", "0") or 0))
    parser.add_argument("--timeout-s", type=int, default=int(os.environ.get("GO2W_CAPTURE_TIMEOUT_S", "8") or 8))
    parser.add_argument("--dry-run", action="store_true")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    timestamp_ms = now_ms()
    run_id, image_path, sidecar_path = build_paths(args, timestamp_ms)
    target_node = args.target_node or os.environ.get("GO2W_TARGET_NODE", "")
    target_name = args.target_name or os.environ.get("GO2W_TARGET_NAME", "")

    payload: dict[str, Any] = {
        "schema_version": 1,
        "captured": False,
        "target_node": target_node,
        "target_name": target_name,
        "run_id": run_id,
        "timestamp_ms": timestamp_ms,
        "image_path": str(image_path),
        "sidecar_path": str(sidecar_path),
    }

    if args.dry_run:
        payload["reason"] = "dry_run"
        write_sidecar(payload, sidecar_path)
        print(json_line(payload))
        return 0

    if args.source_image:
        ok, source = copy_source_image(args.source_image, image_path)
        payload.update({"captured": ok, "source": source if ok else "source_image"})
        if not ok:
            payload["reason"] = source
    elif args.command or os.environ.get("GO2W_CAMERA_CAPTURE_COMMAND"):
        payload.update(capture_with_external_command(args, image_path, run_id))
    else:
        opencv_payload = capture_with_opencv(args, image_path)
        if opencv_payload.get("captured") is True:
            payload.update(opencv_payload)
        else:
            ffmpeg_payload = capture_with_ffmpeg(args, image_path)
            payload.update(ffmpeg_payload if ffmpeg_payload.get("captured") is True else opencv_payload)
            if ffmpeg_payload.get("captured") is not True:
                payload["fallback_failure"] = ffmpeg_payload

    if payload.get("captured") is True:
        try:
            payload["image_bytes"] = image_path.stat().st_size
        except OSError:
            payload["captured"] = False
            payload["reason"] = "image disappeared before sidecar write"

    write_sidecar(payload, sidecar_path)
    print(json_line(payload))
    return 0 if payload.get("captured") is True else 1


if __name__ == "__main__":
    raise SystemExit(main())
