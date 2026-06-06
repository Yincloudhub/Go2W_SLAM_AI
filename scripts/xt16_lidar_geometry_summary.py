#!/usr/bin/env python3
"""Build a compact XT16 PointCloud2 local-obstacle summary for GO2W.

The script emits the same `lidar_pointcloud` JSON contract consumed by the
SLAM gateway. It intentionally keeps raw point clouds outside the UI/LLM
boundary.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path
from typing import Any, Iterable

REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = REPO_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from edge_autonomy.xt16_geometry import Xt16GeometryConfig, build_xt16_geometry_summary, now_ms  # noqa: E402


DEFAULT_OUTPUT = REPO_ROOT / "artifacts" / "lidar_geometry_summary.json"
DEFAULT_TOPIC = "/unitree/slam_lidar/points"


def write_summary(summary: dict[str, Any], output: Path, *, pretty: bool = False) -> str:
    text = json.dumps(
        summary,
        ensure_ascii=False,
        indent=2 if pretty else None,
        separators=None if pretty else (",", ":"),
    )
    output.parent.mkdir(parents=True, exist_ok=True)
    tmp = output.with_name(output.name + ".tmp")
    tmp.write_text(text + "\n", encoding="utf-8")
    tmp.replace(output)
    return text


def load_points_json(path: Path) -> tuple[list[Any], str]:
    data = json.loads(path.read_text(encoding="utf-8"))
    if isinstance(data, dict):
        frame_id = str(data.get("frame_id") or "rslidar")
        points = data.get("points", [])
    else:
        frame_id = "rslidar"
        points = data
    if not isinstance(points, list):
        raise ValueError("input JSON must be a point list or object with points")
    return points, frame_id


def config_from_args(args: argparse.Namespace) -> Xt16GeometryConfig:
    return Xt16GeometryConfig(
        range_m=args.range_m,
        percentile=args.percentile,
        min_points_per_roi=args.min_points_per_roi,
        front_half_width_m=args.front_half_width_m,
        side_forward_m=args.side_forward_m,
        rear_half_width_m=args.rear_half_width_m,
        footprint_front_m=args.footprint_front_m,
        footprint_rear_m=args.footprint_rear_m,
        footprint_half_width_m=args.footprint_half_width_m,
        min_z_m=args.min_z_m,
        body_min_z_m=args.body_min_z_m,
        max_z_m=args.max_z_m,
        clearance_cluster_gap_m=args.clearance_cluster_gap_m,
        support_bin_m=args.support_bin_m,
        min_spatial_bins=args.min_spatial_bins,
        pending_min_points=args.pending_min_points,
        min_cloud_points_for_no_return=args.min_cloud_points_for_no_return,
        no_return_confidence=args.no_return_confidence,
        forward_axis=args.forward_axis,
        lateral_axis=args.lateral_axis,
        vertical_axis=args.vertical_axis,
        forward_sign=args.forward_sign,
        lateral_sign=args.lateral_sign,
        vertical_sign=args.vertical_sign,
        calibrated=args.calibrated,
    )


def sampled_points(points: Iterable[Any], *, max_points: int, sample_stride: int) -> Iterable[Any]:
    emitted = 0
    stride = max(1, int(sample_stride))
    for index, point in enumerate(points):
        if index % stride:
            continue
        yield point
        emitted += 1
        if max_points > 0 and emitted >= max_points:
            break


def run_offline(args: argparse.Namespace) -> int:
    points, frame_id = load_points_json(Path(args.input_json))
    start = time.time()
    summary = build_xt16_geometry_summary(
        sampled_points(points, max_points=args.max_points, sample_stride=args.sample_stride),
        config=config_from_args(args),
        timestamp_ms=now_ms(),
        frame_id=frame_id,
        latency_ms=(time.time() - start) * 1000.0,
    )
    text = write_summary(summary, Path(args.output), pretty=args.pretty)
    if args.print_summary:
        print(text)
    return 0


def header_frame_id(msg: Any) -> str:
    header = getattr(msg, "header", None)
    return str(getattr(header, "frame_id", "") or "rslidar")


def header_latency_ms(msg: Any, received_ms: int) -> float | None:
    header = getattr(msg, "header", None)
    stamp = getattr(header, "stamp", None)
    sec = int(getattr(stamp, "sec", 0) or 0)
    nanosec = int(getattr(stamp, "nanosec", 0) or 0)
    if sec < 1_000_000_000:
        return None
    stamp_ms = sec * 1000 + nanosec / 1_000_000.0
    return max(0.0, received_ms - stamp_ms)


def run_ros(args: argparse.Namespace) -> int:
    try:
        import rclpy
        from sensor_msgs.msg import PointCloud2
        from sensor_msgs_py import point_cloud2
    except Exception as exc:  # pragma: no cover - depends on robot ROS2 env.
        raise RuntimeError("ROS2 Python packages are required for live PointCloud2 mode") from exc

    cfg = config_from_args(args)
    output = Path(args.output)
    min_interval_s = 0.0 if args.rate_limit_hz <= 0 else 1.0 / float(args.rate_limit_hz)
    state = {"last_write_s": 0.0, "count": 0}

    rclpy.init(args=None)
    node = rclpy.create_node("go2w_xt16_lidar_geometry_summary")

    def on_cloud(msg: PointCloud2) -> None:
        current_s = time.time()
        if min_interval_s and current_s - state["last_write_s"] < min_interval_s:
            return
        received_ms = now_ms()
        start = time.time()
        raw_points = point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        summary = build_xt16_geometry_summary(
            sampled_points(raw_points, max_points=args.max_points, sample_stride=args.sample_stride),
            config=cfg,
            timestamp_ms=received_ms,
            frame_id=header_frame_id(msg),
            latency_ms=header_latency_ms(msg, received_ms),
        )
        summary["summary"]["processing_latency_ms"] = round((time.time() - start) * 1000.0, 3)
        write_summary(summary, output, pretty=args.pretty)
        state["last_write_s"] = current_s
        state["count"] += 1
        if args.print_summary:
            print(json.dumps(summary, ensure_ascii=False, separators=(",", ":")), flush=True)
        if args.once:
            raise KeyboardInterrupt

    node.create_subscription(PointCloud2, args.topic, on_cloud, 10)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0 if state["count"] > 0 or not args.once else 2


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Emit GO2W XT16 lidar_pointcloud LocalObstacleSummary JSON.")
    parser.add_argument("--topic", default=DEFAULT_TOPIC)
    parser.add_argument("--output", default=str(DEFAULT_OUTPUT))
    parser.add_argument("--input-json", default="", help="Offline point list JSON for smoke tests.")
    parser.add_argument("--once", action="store_true", help="Live ROS mode: exit after one written cloud summary.")
    parser.add_argument("--print-summary", action="store_true")
    parser.add_argument("--pretty", action="store_true")
    parser.add_argument("--calibrated", action="store_true", help="Allow fresh summaries. Without this, output is stale.")
    parser.add_argument("--range-m", type=float, default=6.0)
    parser.add_argument("--percentile", type=float, default=10.0)
    parser.add_argument("--min-points-per-roi", type=int, default=8)
    parser.add_argument("--front-half-width-m", type=float, default=0.45)
    parser.add_argument("--side-forward-m", type=float, default=0.75)
    parser.add_argument("--rear-half-width-m", type=float, default=0.45)
    parser.add_argument("--footprint-front-m", type=float, default=0.35)
    parser.add_argument("--footprint-rear-m", type=float, default=0.45)
    parser.add_argument("--footprint-half-width-m", type=float, default=0.40)
    parser.add_argument("--min-z-m", type=float, default=-0.25)
    parser.add_argument("--body-min-z-m", type=float, default=-0.10)
    parser.add_argument("--max-z-m", type=float, default=1.20)
    parser.add_argument("--clearance-cluster-gap-m", type=float, default=0.15)
    parser.add_argument("--support-bin-m", type=float, default=0.05)
    parser.add_argument("--min-spatial-bins", type=int, default=2)
    parser.add_argument("--pending-min-points", type=int, default=3)
    parser.add_argument("--min-cloud-points-for-no-return", type=int, default=1000)
    parser.add_argument("--no-return-confidence", type=float, default=0.5)
    parser.add_argument("--forward-axis", choices=("x", "y", "z"), default="y")
    parser.add_argument("--lateral-axis", choices=("x", "y", "z"), default="x")
    parser.add_argument("--vertical-axis", choices=("x", "y", "z"), default="z")
    parser.add_argument("--forward-sign", type=float, choices=(-1.0, 1.0), default=-1.0)
    parser.add_argument("--lateral-sign", type=float, choices=(-1.0, 1.0), default=1.0)
    parser.add_argument("--vertical-sign", type=float, choices=(-1.0, 1.0), default=1.0)
    parser.add_argument("--max-points", type=int, default=80000)
    parser.add_argument("--sample-stride", type=int, default=1)
    parser.add_argument("--rate-limit-hz", type=float, default=5.0)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if args.input_json:
        return run_offline(args)
    return run_ros(args)


if __name__ == "__main__":
    raise SystemExit(main())
