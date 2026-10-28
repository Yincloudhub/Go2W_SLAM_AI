#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import os
import signal
import subprocess
import sys
import time
from pathlib import Path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Record TI radar live imaging and an optical USB camera into one experiment directory."
    )
    parser.add_argument("--metrics-dir", type=Path, default=Path("/home/radar/fft_live_metrics"))
    parser.add_argument("--experiment-name", default="", help="Optional suffix for the timestamped experiment folder.")

    parser.add_argument("--camera-device", default="/dev/video0")
    parser.add_argument("--camera-width", type=int, default=1920)
    parser.add_argument("--camera-height", type=int, default=1080)
    parser.add_argument("--camera-fps", type=int, default=30)
    parser.add_argument("--camera-input-format", default="mjpeg")
    parser.add_argument("--disable-camera", action="store_true")
    parser.add_argument("--timestamp-overlay", action="store_true", default=True)
    parser.add_argument("--no-timestamp-overlay", dest="timestamp_overlay", action="store_false")

    parser.add_argument("--radar-config", type=Path, default=Path("/home/radar/pyRadar-main/configFiles/mmwaveconfig_0508_30f128_safe.txt"))
    parser.add_argument("--radar-numframes", type=int, default=0)
    parser.add_argument("--velocity-mps", type=float, default=1.0)
    parser.add_argument("--fft-step", type=int, default=200)
    parser.add_argument("--fft-frag", type=int, default=2000)
    parser.add_argument("--video-fps", type=float, default=15.0)
    parser.add_argument("--mjpeg-port", type=int, default=8090)
    parser.add_argument("--skip-usb-reset", action="store_true", default=True)
    parser.add_argument("--no-skip-usb-reset", dest="skip_usb_reset", action="store_false")
    parser.add_argument("--launcher", type=Path, default=Path("/home/radar/fft_image_NX_live/scripts/live_dca1000_launcher.py"))
    return parser.parse_args()


def stop_process(proc: subprocess.Popen[bytes] | None, *, interrupt: bool) -> None:
    if proc is None or proc.poll() is not None:
        return
    signum = signal.SIGINT if interrupt else signal.SIGTERM
    try:
        os.killpg(proc.pid, signum)
    except ProcessLookupError:
        return
    try:
        proc.wait(timeout=8)
        return
    except subprocess.TimeoutExpired:
        pass
    try:
        os.killpg(proc.pid, signal.SIGTERM)
    except ProcessLookupError:
        return
    try:
        proc.wait(timeout=5)
        return
    except subprocess.TimeoutExpired:
        pass
    try:
        os.killpg(proc.pid, signal.SIGKILL)
    except ProcessLookupError:
        return
    proc.wait(timeout=5)


def make_experiment_dir(root: Path, suffix: str) -> Path:
    stamp = time.strftime("%Y%m%d_%H%M%S")
    name = f"{stamp}_{suffix}" if suffix else stamp
    path = root.expanduser() / name
    (path / "radar").mkdir(parents=True, exist_ok=False)
    (path / "camera").mkdir(parents=True, exist_ok=True)
    (path / "logs").mkdir(parents=True, exist_ok=True)
    return path


def relaunch_with_sudo_if_needed() -> None:
    if os.name == "posix" and hasattr(os, "geteuid") and os.geteuid() != 0:
        os.execvp("sudo", ["sudo", "-E", sys.executable, *sys.argv])


def main() -> int:
    args = parse_args()
    relaunch_with_sudo_if_needed()
    experiment_dir = make_experiment_dir(args.metrics_dir, args.experiment_name.strip())
    radar_dir = experiment_dir / "radar"
    camera_dir = experiment_dir / "camera"
    logs_dir = experiment_dir / "logs"

    radar_video = radar_dir / "radar_live.avi"
    camera_video = camera_dir / "camera_live.avi"
    metadata_path = experiment_dir / "experiment_meta.json"
    metadata = {
        "created_at": time.strftime("%Y-%m-%d %H:%M:%S"),
        "experiment_dir": str(experiment_dir),
        "radar_config": str(args.radar_config),
        "radar_video": str(radar_video),
        "camera_video": str(camera_video),
        "camera_device": args.camera_device,
        "camera_width": args.camera_width,
        "camera_height": args.camera_height,
        "camera_fps": args.camera_fps,
        "camera_input_format": args.camera_input_format,
        "timestamp_overlay": args.timestamp_overlay,
        "mjpeg_bind": f"0.0.0.0:{args.mjpeg_port}" if args.mjpeg_port > 0 else "",
        "mjpeg_path": "/radar.mjpg" if args.mjpeg_port > 0 else "",
    }
    metadata_path.write_text(json.dumps(metadata, indent=2, ensure_ascii=False), encoding="utf-8")

    camera_proc: subprocess.Popen[bytes] | None = None
    radar_proc: subprocess.Popen[bytes] | None = None
    shutdown_requested = False

    def request_shutdown(signum: int, _frame: object) -> None:
        nonlocal shutdown_requested
        shutdown_requested = True
        print(f"\n[INFO][multi] received {signal.Signals(signum).name}; stopping camera and radar...", flush=True)
        stop_process(radar_proc, interrupt=True)
        stop_process(camera_proc, interrupt=True)

    signal.signal(signal.SIGINT, request_shutdown)
    signal.signal(signal.SIGTERM, request_shutdown)

    print(f"[INFO][multi] experiment_dir={experiment_dir}", flush=True)
    print(f"[INFO][multi] radar_video={radar_video}", flush=True)
    print(f"[INFO][multi] camera_video={camera_video}", flush=True)

    try:
        if not args.disable_camera:
            camera_cmd = [
                "ffmpeg",
                "-hide_banner",
                "-loglevel",
                "info",
                "-f",
                "v4l2",
                "-thread_queue_size",
                "512",
                "-input_format",
                args.camera_input_format,
                "-video_size",
                f"{args.camera_width}x{args.camera_height}",
                "-framerate",
                str(args.camera_fps),
                "-i",
                args.camera_device,
            ]
            if args.timestamp_overlay:
                camera_cmd.extend([
                    "-vf",
                    "drawtext=fontfile=/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf:"
                    "expansion=strftime:"
                    "text='%Y-%m-%d %H\\:%M\\:%S':"
                    "x=18:y=18:fontsize=30:fontcolor=white:"
                    "box=1:boxcolor=0x14232dcc:boxborderw=8",
                    "-c:v",
                    "libx264",
                    "-preset",
                    "ultrafast",
                    "-tune",
                    "zerolatency",
                    "-pix_fmt",
                    "yuv420p",
                ])
            else:
                camera_cmd.extend([
                    "-c:v",
                    "copy",
                ])
            camera_cmd.extend([
                "-y",
                str(camera_video),
            ])
            camera_log = (logs_dir / "camera_ffmpeg.log").open("wb", buffering=0)
            print("[INFO][multi] starting camera:", " ".join(camera_cmd), flush=True)
            camera_proc = subprocess.Popen(
                camera_cmd,
                stdout=camera_log,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
            time.sleep(1.0)
            if camera_proc.poll() is not None:
                raise RuntimeError(f"camera ffmpeg exited early with code {camera_proc.returncode}; see {logs_dir / 'camera_ffmpeg.log'}")

        radar_cmd = [
            sys.executable,
            str(args.launcher),
        ]
        if args.skip_usb_reset:
            radar_cmd.append("--skip-usb-reset")
        radar_cmd.extend([
            "--radar-config",
            str(args.radar_config),
            "--radar-numframes",
            str(args.radar_numframes),
            "--velocity-mps",
            str(args.velocity_mps),
            "--fft-step",
            str(args.fft_step),
            "--fft-frag",
            str(args.fft_frag),
            "--video-fps",
            str(args.video_fps),
            "--video-output",
            str(radar_video),
            "--mjpeg-port",
            str(args.mjpeg_port),
            "--metrics-dir",
            str(radar_dir),
        ])
        if args.timestamp_overlay:
            radar_cmd.append("--timestamp-overlay")
        radar_log = (logs_dir / "radar_launcher.log").open("wb", buffering=0)
        print("[INFO][multi] starting radar:", " ".join(radar_cmd), flush=True)
        radar_proc = subprocess.Popen(
            radar_cmd,
            stdout=radar_log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )

        while True:
            if shutdown_requested:
                return 130
            radar_code = radar_proc.poll() if radar_proc is not None else 0
            camera_code = camera_proc.poll() if camera_proc is not None else None
            if radar_code is not None:
                if camera_proc is not None and camera_proc.poll() is None:
                    stop_process(camera_proc, interrupt=True)
                print(f"[INFO][multi] radar exited with code {radar_code}", flush=True)
                return int(radar_code)
            if camera_code is not None:
                print(f"[WARN][multi] camera exited with code {camera_code}; radar continues", flush=True)
                camera_proc = None
            time.sleep(0.5)
    finally:
        stop_process(radar_proc, interrupt=shutdown_requested)
        stop_process(camera_proc, interrupt=shutdown_requested)


if __name__ == "__main__":
    raise SystemExit(main())
