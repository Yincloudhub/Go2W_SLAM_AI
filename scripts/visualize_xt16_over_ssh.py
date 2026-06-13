#!/usr/bin/env python3
"""Display a read-only XT16 top view by streaming sampled PointCloud2 over SSH."""

from __future__ import annotations

import argparse
import base64
import getpass
import json
import math
import os
import queue
import shlex
import threading
import time
import tkinter as tk
from pathlib import Path
from typing import Any


DEFAULT_HOST = os.environ.get("GO2W_SSH_HOST", "192.168.123.18")
DEFAULT_TOPIC = "/utlidar/cloud"


REMOTE_STREAMER = r"""
import base64
import json
import math
import sys
import time

config = json.loads(base64.b64decode(sys.argv[1]).decode("utf-8"))
sys.path.insert(0, config["repo_root"] + "/src")

import rclpy
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

from edge_autonomy.xt16_geometry import Xt16GeometryConfig, build_xt16_geometry_summary


def finite_point(value):
    try:
        point = tuple(float(item) for item in value[:3])
    except (TypeError, ValueError):
        return None
    return point if all(math.isfinite(item) for item in point) else None


def body_point(point):
    x, y, z = point
    return -y, x, z


geometry = Xt16GeometryConfig(
    range_m=float(config["range_m"]),
    footprint_front_m=float(config["footprint_front_m"]),
    footprint_rear_m=float(config["footprint_rear_m"]),
    footprint_half_width_m=float(config["footprint_half_width_m"]),
    footprint_filter_margin_m=float(config["footprint_filter_margin_m"]),
    min_z_m=float(config["min_z_m"]),
    body_min_z_m=float(config["body_min_z_m"]),
    max_z_m=float(config["max_z_m"]),
    calibrated=False,
)
interval_s = 1.0 / max(0.1, float(config["rate_hz"]))
max_plot_points = max(100, int(config["max_plot_points"]))
state = {"last_emit": 0.0, "sequence": 0}

rclpy.init(args=None)
node = rclpy.create_node("go2w_xt16_ssh_visualizer")


def on_cloud(msg):
    now_s = time.time()
    if now_s - state["last_emit"] < interval_s:
        return

    raw_points = []
    for value in point_cloud2.read_points(
        msg,
        field_names=("x", "y", "z"),
        skip_nans=True,
    ):
        point = finite_point(value)
        if point is not None:
            raw_points.append(point)

    state["sequence"] += 1
    timestamp_ms = int(now_s * 1000)
    summary = build_xt16_geometry_summary(
        raw_points,
        config=geometry,
        timestamp_ms=timestamp_ms,
        sequence=state["sequence"],
        frame_id=str(msg.header.frame_id or "unknown"),
    )

    stride = max(1, int(math.ceil(len(raw_points) / max_plot_points)))
    plot_points = []
    margin = geometry.footprint_filter_margin_m
    for raw in raw_points[::stride]:
        forward, lateral, vertical = body_point(raw)
        if math.hypot(forward, lateral) > geometry.range_m:
            continue
        if vertical < geometry.min_z_m or vertical > geometry.max_z_m:
            point_class = "height_rejected"
        elif (
            -(geometry.footprint_rear_m + margin)
            <= forward
            <= geometry.footprint_front_m + margin
            and abs(lateral) <= geometry.footprint_half_width_m + margin
        ):
            point_class = "footprint_rejected"
        elif vertical < geometry.body_min_z_m:
            point_class = "low_hazard"
        else:
            point_class = "body_height"
        plot_points.append(
            [
                round(forward, 4),
                round(lateral, 4),
                round(vertical, 4),
                point_class,
            ]
        )

    detail = summary.get("summary") if isinstance(summary.get("summary"), dict) else {}
    packet = {
        "timestamp_ms": timestamp_ms,
        "sequence": state["sequence"],
        "topic": config["topic"],
        "frame_id": str(msg.header.frame_id or "unknown"),
        "raw_width": int(msg.width),
        "raw_points": len(raw_points),
        "plot_stride": stride,
        "plot_points": plot_points,
        "geometry": {
            "front_clearance_m": summary.get("front_clearance_m"),
            "left_clearance_m": summary.get("left_clearance_m"),
            "right_clearance_m": summary.get("right_clearance_m"),
            "rear_clearance_m": summary.get("rear_clearance_m"),
            "body_clearance_m": summary.get("body_clearance_m"),
            "low_hazard_clearance_m": summary.get("low_hazard_clearance_m"),
            "low_hazard_directions": summary.get("low_hazard_directions"),
            "pending_low_hazard_directions": summary.get("pending_low_hazard_directions"),
            "blocked_directions": summary.get("blocked_directions"),
            "points_excluded_footprint": detail.get("points_excluded_footprint"),
            "points_in_height_band": detail.get("points_in_height_band"),
            "stale": summary.get("stale"),
            "stale_reasons": summary.get("stale_reasons"),
        },
    }
    print(json.dumps(packet, separators=(",", ":")), flush=True)
    state["last_emit"] = now_s


qos = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)
node.create_subscription(PointCloud2, config["topic"], on_cloud, qos)
try:
    rclpy.spin(node)
finally:
    node.destroy_node()
    rclpy.shutdown()
"""


POINT_COLORS = {
    "footprint_rejected": "#ef4444",
    "body_height": "#22d3ee",
    "low_hazard": "#f59e0b",
    "height_rejected": "#4b5563",
}


def classify_body_point(
    forward: float,
    lateral: float,
    vertical: float,
    *,
    footprint_front_m: float,
    footprint_rear_m: float,
    footprint_half_width_m: float,
    footprint_filter_margin_m: float,
    min_z_m: float,
    body_min_z_m: float,
    max_z_m: float,
) -> str:
    if vertical < min_z_m or vertical > max_z_m:
        return "height_rejected"
    if (
        -(footprint_rear_m + footprint_filter_margin_m)
        <= forward
        <= footprint_front_m + footprint_filter_margin_m
        and abs(lateral) <= footprint_half_width_m + footprint_filter_margin_m
    ):
        return "footprint_rejected"
    return "low_hazard" if vertical < body_min_z_m else "body_height"


def canvas_point(
    forward: float,
    lateral: float,
    *,
    width: int,
    height: int,
    range_m: float,
) -> tuple[float, float]:
    scale = min(width, height) / (2.0 * max(0.1, range_m))
    return width / 2.0 - lateral * scale, height / 2.0 - forward * scale


def build_remote_command(config: dict[str, Any]) -> str:
    script_payload = base64.b64encode(REMOTE_STREAMER.encode("utf-8")).decode("ascii")
    config_payload = base64.b64encode(
        json.dumps(config, separators=(",", ":")).encode("utf-8")
    ).decode("ascii")
    inner = (
        "source /opt/ros/foxy/setup.bash >/dev/null 2>&1 || true; "
        f"echo {shlex.quote(script_payload)} | base64 -d | "
        f"python3 - {shlex.quote(config_payload)}"
    )
    return "bash -lc " + shlex.quote(inner)


def format_clearance(value: Any) -> str:
    return "?" if not isinstance(value, (int, float)) else f"{float(value):.2f}"


class StreamReader(threading.Thread):
    def __init__(
        self,
        stdout: Any,
        stderr: Any,
        messages: queue.Queue[tuple[str, Any]],
        record_path: Path | None,
    ) -> None:
        super().__init__(daemon=True)
        self.stdout = stdout
        self.stderr = stderr
        self.messages = messages
        self.record_path = record_path

    def run(self) -> None:
        record = None
        try:
            if self.record_path is not None:
                self.record_path.parent.mkdir(parents=True, exist_ok=True)
                record = self.record_path.open("a", encoding="utf-8")
            while True:
                line = self.stdout.readline()
                if not line:
                    break
                text = line.decode("utf-8", "replace").strip() if isinstance(line, bytes) else line.strip()
                if not text:
                    continue
                try:
                    packet = json.loads(text)
                except json.JSONDecodeError:
                    self.messages.put(("error", f"remote output is not JSON: {text[:200]}"))
                    continue
                if record is not None:
                    record.write(json.dumps(packet, separators=(",", ":")) + "\n")
                    record.flush()
                self.messages.put(("packet", packet))
            error_value = self.stderr.read()
            error_text = (
                error_value.decode("utf-8", "replace")
                if isinstance(error_value, bytes)
                else str(error_value)
            ).strip()
            if error_text:
                self.messages.put(("error", error_text))
        except Exception as exc:
            self.messages.put(("error", str(exc)))
        finally:
            if record is not None:
                record.close()
            self.messages.put(("closed", None))


class Xt16Viewer:
    def __init__(
        self,
        root: tk.Tk,
        client: Any,
        reader: StreamReader,
        messages: queue.Queue[tuple[str, Any]],
        args: argparse.Namespace,
    ) -> None:
        self.root = root
        self.client = client
        self.reader = reader
        self.messages = messages
        self.args = args
        self.packet: dict[str, Any] | None = None
        self.closed = False
        self.started_s = time.time()

        root.title("GO2W XT16 read-only top view")
        root.geometry("1000x820")
        root.configure(bg="#111827")
        root.protocol("WM_DELETE_WINDOW", self.close)

        self.status = tk.StringVar(value="Connecting to XT16 stream...")
        tk.Label(
            root,
            textvariable=self.status,
            bg="#111827",
            fg="#e5e7eb",
            font=("Consolas", 11),
            anchor="w",
            justify="left",
        ).pack(fill="x", padx=12, pady=(10, 4))

        self.canvas = tk.Canvas(root, bg="#030712", highlightthickness=0)
        self.canvas.pack(fill="both", expand=True, padx=12, pady=(4, 12))
        self.canvas.bind("<Configure>", lambda _event: self.draw())

        self.reader.start()
        self.root.after(50, self.poll)

    def poll(self) -> None:
        while True:
            try:
                kind, value = self.messages.get_nowait()
            except queue.Empty:
                break
            if kind == "packet":
                self.packet = value
                self.update_status()
                self.draw()
            elif kind == "error":
                self.status.set(f"ERROR: {value}")
            elif kind == "closed" and not self.closed:
                self.status.set(self.status.get() + "\nSSH point-cloud stream closed.")

        if (
            self.args.duration_s > 0
            and time.time() - self.started_s >= self.args.duration_s
        ):
            self.close()
            return
        if not self.closed:
            self.root.after(50, self.poll)

    def update_status(self) -> None:
        assert self.packet is not None
        geometry = self.packet.get("geometry", {})
        self.status.set(
            f"host={self.args.host}  topic={self.packet.get('topic')}  "
            f"frame={self.packet.get('frame_id')}  sequence={self.packet.get('sequence')}\n"
            f"raw={self.packet.get('raw_points')}  plotted={len(self.packet.get('plot_points', []))}  "
            f"footprint_removed={geometry.get('points_excluded_footprint')}  "
            f"front={format_clearance(geometry.get('front_clearance_m'))}m  "
            f"left={format_clearance(geometry.get('left_clearance_m'))}m  "
            f"right={format_clearance(geometry.get('right_clearance_m'))}m  "
            f"rear={format_clearance(geometry.get('rear_clearance_m'))}m\n"
            f"low_hazard={geometry.get('low_hazard_directions') or []}  "
            f"pending_low={geometry.get('pending_low_hazard_directions') or []}  "
            f"blocked={geometry.get('blocked_directions') or []}"
        )

    def draw(self) -> None:
        width = max(100, self.canvas.winfo_width())
        height = max(100, self.canvas.winfo_height())
        self.canvas.delete("all")
        center = canvas_point(
            0.0,
            0.0,
            width=width,
            height=height,
            range_m=self.args.range_m,
        )
        self.canvas.create_line(center[0], 0, center[0], height, fill="#374151")
        self.canvas.create_line(0, center[1], width, center[1], fill="#374151")

        for radius in range(1, int(self.args.range_m) + 1):
            edge = canvas_point(
                float(radius),
                0.0,
                width=width,
                height=height,
                range_m=self.args.range_m,
            )
            pixel_radius = abs(center[1] - edge[1])
            self.canvas.create_oval(
                center[0] - pixel_radius,
                center[1] - pixel_radius,
                center[0] + pixel_radius,
                center[1] + pixel_radius,
                outline="#1f2937",
            )
            self.canvas.create_text(
                center[0] + 5,
                center[1] - pixel_radius + 10,
                text=f"{radius}m",
                fill="#6b7280",
                anchor="w",
            )

        self.draw_footprint(width, height)

        if self.packet is not None:
            for value in self.packet.get("plot_points", []):
                if not isinstance(value, list) or len(value) < 4:
                    continue
                forward, lateral, _vertical, point_class = value
                x, y = canvas_point(
                    float(forward),
                    float(lateral),
                    width=width,
                    height=height,
                    range_m=self.args.range_m,
                )
                if 0 <= x <= width and 0 <= y <= height:
                    color = POINT_COLORS.get(str(point_class), "#9ca3af")
                    radius = 2 if point_class != "height_rejected" else 1
                    self.canvas.create_oval(
                        x - radius,
                        y - radius,
                        x + radius,
                        y + radius,
                        fill=color,
                        outline="",
                    )

        self.canvas.create_text(12, 12, text="FRONT", fill="#f9fafb", anchor="nw")
        self.canvas.create_text(
            12,
            height - 12,
            text="REAR",
            fill="#f9fafb",
            anchor="sw",
        )
        self.canvas.create_text(
            12,
            center[1] - 8,
            text="LEFT",
            fill="#f9fafb",
            anchor="w",
        )
        self.canvas.create_text(
            width - 12,
            center[1] - 8,
            text="RIGHT",
            fill="#f9fafb",
            anchor="e",
        )
        legend = (
            "red=footprint rejected   cyan=body-height retained   "
            "orange=low hazard   gray=height rejected"
        )
        self.canvas.create_text(
            width - 12,
            height - 12,
            text=legend,
            fill="#d1d5db",
            anchor="se",
        )

    def draw_footprint(self, width: int, height: int) -> None:
        margin = self.args.footprint_filter_margin_m
        nominal = (
            self.args.footprint_front_m,
            self.args.footprint_rear_m,
            self.args.footprint_half_width_m,
        )
        expanded = (
            nominal[0] + margin,
            nominal[1] + margin,
            nominal[2] + margin,
        )
        for front, rear, half_width, color, dash in (
            (*expanded, "#ef4444", (4, 3)),
            (*nominal, "#f9fafb", ()),
        ):
            x1, y1 = canvas_point(
                front,
                half_width,
                width=width,
                height=height,
                range_m=self.args.range_m,
            )
            x2, y2 = canvas_point(
                -rear,
                -half_width,
                width=width,
                height=height,
                range_m=self.args.range_m,
            )
            self.canvas.create_rectangle(x1, y1, x2, y2, outline=color, dash=dash)

    def close(self) -> None:
        if self.closed:
            return
        self.closed = True
        try:
            self.client.close()
        finally:
            self.root.destroy()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default=DEFAULT_HOST)
    parser.add_argument("--username", default="unitree")
    parser.add_argument("--topic", default=DEFAULT_TOPIC)
    parser.add_argument("--repo-root", default="/home/unitree/Go2W_SLAM_AI")
    parser.add_argument("--password-env", default="GO2W_SSH_PASSWORD")
    parser.add_argument("--range-m", type=float, default=4.0)
    parser.add_argument("--rate-hz", type=float, default=2.0)
    parser.add_argument("--max-plot-points", type=int, default=6000)
    parser.add_argument("--duration-s", type=float, default=0.0)
    parser.add_argument("--record-jsonl", default="")
    parser.add_argument("--footprint-front-m", type=float, default=0.30)
    parser.add_argument("--footprint-rear-m", type=float, default=0.30)
    parser.add_argument("--footprint-half-width-m", type=float, default=0.30)
    parser.add_argument("--footprint-filter-margin-m", type=float, default=0.02)
    parser.add_argument("--min-z-m", type=float, default=-0.25)
    parser.add_argument("--body-min-z-m", type=float, default=-0.10)
    parser.add_argument("--max-z-m", type=float, default=1.20)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        import paramiko
    except ImportError as exc:
        raise SystemExit(
            "paramiko is required; run with the repository .venv Python"
        ) from exc

    password = os.environ.get(args.password_env) or getpass.getpass(
        f"{args.username}@{args.host} password: "
    )
    remote_config = {
        "topic": args.topic,
        "repo_root": args.repo_root,
        "range_m": args.range_m,
        "rate_hz": args.rate_hz,
        "max_plot_points": args.max_plot_points,
        "footprint_front_m": args.footprint_front_m,
        "footprint_rear_m": args.footprint_rear_m,
        "footprint_half_width_m": args.footprint_half_width_m,
        "footprint_filter_margin_m": args.footprint_filter_margin_m,
        "min_z_m": args.min_z_m,
        "body_min_z_m": args.body_min_z_m,
        "max_z_m": args.max_z_m,
    }

    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    client.connect(
        args.host,
        username=args.username,
        password=password,
        timeout=10,
        banner_timeout=10,
        auth_timeout=10,
    )
    _stdin, stdout, stderr = client.exec_command(
        build_remote_command(remote_config),
        timeout=None,
    )
    messages: queue.Queue[tuple[str, Any]] = queue.Queue()
    record_path = Path(args.record_jsonl) if args.record_jsonl else None
    reader = StreamReader(stdout, stderr, messages, record_path)

    root = tk.Tk()
    Xt16Viewer(root, client, reader, messages, args)
    root.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
