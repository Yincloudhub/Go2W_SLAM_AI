from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Any, Iterable, Mapping, Sequence


def now_ms() -> int:
    return int(time.time() * 1000)


@dataclass(frozen=True)
class Xt16GeometryConfig:
    range_m: float = 6.0
    percentile: float = 10.0
    min_points_per_roi: int = 8
    front_half_width_m: float = 0.45
    side_forward_m: float = 0.75
    rear_half_width_m: float = 0.45
    footprint_front_m: float = 0.35
    footprint_rear_m: float = 0.35
    footprint_half_width_m: float = 0.32
    min_z_m: float = -0.25
    max_z_m: float = 1.20
    forward_axis: str = "x"
    lateral_axis: str = "y"
    vertical_axis: str = "z"
    forward_sign: float = 1.0
    lateral_sign: float = 1.0
    vertical_sign: float = 1.0
    calibrated: bool = False


def _point_value(point: Any, key: str) -> float | None:
    if isinstance(point, Mapping):
        value = point.get(key)
    else:
        index = {"x": 0, "y": 1, "z": 2}.get(key)
        if index is None:
            return None
        try:
            value = point[index]
        except (IndexError, TypeError):
            return None
    try:
        value_f = float(value)
    except (TypeError, ValueError):
        return None
    return value_f if math.isfinite(value_f) else None


def _body_point(point: Any, config: Xt16GeometryConfig) -> tuple[float, float, float] | None:
    forward = _point_value(point, config.forward_axis)
    lateral = _point_value(point, config.lateral_axis)
    vertical = _point_value(point, config.vertical_axis)
    if forward is None or lateral is None or vertical is None:
        return None
    return (
        forward * config.forward_sign,
        lateral * config.lateral_sign,
        vertical * config.vertical_sign,
    )


def _percentile(values: Sequence[float], q: float) -> float | None:
    clean = sorted(float(value) for value in values if math.isfinite(float(value)))
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


def _roi_confidence(count: int, config: Xt16GeometryConfig) -> float:
    return round(min(1.0, count / max(1, config.min_points_per_roi)), 3)


def _clearance(values: Sequence[float], config: Xt16GeometryConfig) -> float | None:
    if len(values) < config.min_points_per_roi:
        return None
    value = _percentile(values, config.percentile)
    if value is None:
        return None
    return round(max(0.0, float(value)), 3)


def _blocked_directions(summary: dict[str, Any]) -> list[str]:
    blocked: list[str] = []
    thresholds = {
        "front": 0.8,
        "left": 0.8,
        "right": 0.8,
        "rear": 0.6,
    }
    for direction, threshold in thresholds.items():
        value = summary.get(f"{direction}_clearance_m")
        if isinstance(value, (int, float)) and math.isfinite(float(value)) and 0 <= float(value) < threshold:
            blocked.append(direction)
    return blocked


def _recommended_action(summary: dict[str, Any]) -> str:
    front = summary.get("front_clearance_m")
    left = summary.get("left_clearance_m")
    right = summary.get("right_clearance_m")
    if isinstance(front, (int, float)) and 0 <= float(front) < 0.8:
        return "pause"
    if isinstance(left, (int, float)) and 0 <= float(left) < 0.8:
        return "pause"
    if isinstance(right, (int, float)) and 0 <= float(right) < 0.8:
        return "pause"
    if isinstance(front, (int, float)) and 0 <= float(front) < 1.5:
        return "go_slow"
    if isinstance(left, (int, float)) and 0 <= float(left) < 1.0:
        return "go_slow"
    if isinstance(right, (int, float)) and 0 <= float(right) < 1.0:
        return "go_slow"
    return "normal"


def build_xt16_geometry_summary(
    points: Iterable[Any],
    *,
    config: Xt16GeometryConfig | None = None,
    timestamp_ms: int | None = None,
    frame_id: str = "rslidar",
    latency_ms: float | None = None,
) -> dict[str, Any]:
    cfg = config or Xt16GeometryConfig()
    front_values: list[float] = []
    left_values: list[float] = []
    right_values: list[float] = []
    rear_values: list[float] = []
    total_points = 0
    finite_points = 0
    height_filtered_points = 0
    footprint_filtered_points = 0

    for point in points:
        total_points += 1
        body = _body_point(point, cfg)
        if body is None:
            continue
        finite_points += 1
        forward, lateral, vertical = body
        if vertical < cfg.min_z_m or vertical > cfg.max_z_m:
            continue
        height_filtered_points += 1
        in_footprint = (
            -cfg.footprint_rear_m <= forward <= cfg.footprint_front_m
            and abs(lateral) <= cfg.footprint_half_width_m
        )
        if in_footprint:
            footprint_filtered_points += 1
            continue

        if cfg.footprint_front_m < forward <= cfg.range_m and abs(lateral) <= cfg.front_half_width_m:
            front_values.append(forward - cfg.footprint_front_m)
        if cfg.footprint_half_width_m < lateral <= cfg.range_m and abs(forward) <= cfg.side_forward_m:
            left_values.append(lateral - cfg.footprint_half_width_m)
        if -cfg.range_m <= lateral < -cfg.footprint_half_width_m and abs(forward) <= cfg.side_forward_m:
            right_values.append(-lateral - cfg.footprint_half_width_m)
        if -cfg.range_m <= forward < -cfg.footprint_rear_m and abs(lateral) <= cfg.rear_half_width_m:
            rear_values.append(-forward - cfg.footprint_rear_m)

    front = _clearance(front_values, cfg)
    left = _clearance(left_values, cfg)
    right = _clearance(right_values, cfg)
    rear = _clearance(rear_values, cfg)
    roi_confidence = {
        "front": _roi_confidence(len(front_values), cfg),
        "left": _roi_confidence(len(left_values), cfg),
        "right": _roi_confidence(len(right_values), cfg),
        "rear": _roi_confidence(len(rear_values), cfg),
    }
    confidence = round(min(roi_confidence["front"], roi_confidence["left"], roi_confidence["right"]), 3)
    missing_required = [name for name, value in {"front": front, "left": left, "right": right}.items() if value is None]
    stale_reasons: list[str] = []
    if not cfg.calibrated:
        stale_reasons.append("uncalibrated_xt16_geometry")
    if missing_required:
        stale_reasons.append("missing_required_roi:" + ",".join(missing_required))
    if total_points == 0 or finite_points == 0:
        stale_reasons.append("empty_or_invalid_pointcloud")

    summary: dict[str, Any] = {
        "type": "local_obstacle_summary",
        "source": "lidar_pointcloud",
        "timestamp_ms": int(timestamp_ms if timestamp_ms is not None else now_ms()),
        "frame_id": frame_id,
        "range_m": cfg.range_m,
        "front_clearance_m": front,
        "left_clearance_m": left,
        "right_clearance_m": right,
        "rear_clearance_m": rear,
        "confidence": confidence,
        "roi_confidence": roi_confidence,
        "latency_ms": latency_ms,
        "stale": bool(stale_reasons),
        "stale_reasons": stale_reasons,
        "blocked_directions": [],
        "narrow_passage": False,
        "recommended_action": "normal",
        "summary": {
            "points_total": total_points,
            "points_finite": finite_points,
            "points_in_height_band": height_filtered_points,
            "points_excluded_footprint": footprint_filtered_points,
            "roi_counts": {
                "front": len(front_values),
                "left": len(left_values),
                "right": len(right_values),
                "rear": len(rear_values),
            },
            "percentile": cfg.percentile,
            "calibrated": cfg.calibrated,
            "axes": {
                "forward": cfg.forward_axis,
                "lateral": cfg.lateral_axis,
                "vertical": cfg.vertical_axis,
                "forward_sign": cfg.forward_sign,
                "lateral_sign": cfg.lateral_sign,
                "vertical_sign": cfg.vertical_sign,
            },
            "footprint_m": {
                "front": cfg.footprint_front_m,
                "rear": cfg.footprint_rear_m,
                "half_width": cfg.footprint_half_width_m,
            },
        },
    }
    summary["blocked_directions"] = _blocked_directions(summary)
    summary["narrow_passage"] = (
        isinstance(left, (int, float))
        and isinstance(right, (int, float))
        and 0 <= float(left) < 0.8
        and 0 <= float(right) < 0.8
    )
    summary["recommended_action"] = _recommended_action(summary)
    return summary
