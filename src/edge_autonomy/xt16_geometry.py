from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Any, Iterable, Mapping, Sequence, Tuple

from .obstacle_policy import (
    FRONT_PAUSE_M,
    REAR_PAUSE_M,
    REAR_SLOW_M,
    SIDE_PAUSE_M,
    SIDE_SLOW_M,
    blocked_directions,
    narrow_passage,
    recommended_action,
)


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
    footprint_front_m: float = 0.30
    footprint_rear_m: float = 0.30
    footprint_half_width_m: float = 0.30
    footprint_filter_margin_m: float = 0.05
    footprint_lateral_filter_margin_m: float = 0.0
    min_z_m: float = -0.25
    body_min_z_m: float = -0.10
    max_z_m: float = 1.20
    clearance_cluster_gap_m: float = 0.15
    support_bin_m: float = 0.05
    min_spatial_bins: int = 2
    pending_min_points: int = 3
    min_cloud_points_for_no_return: int = 1000
    no_return_confidence: float = 0.5
    forward_axis: str = "y"
    lateral_axis: str = "x"
    vertical_axis: str = "z"
    forward_sign: float = -1.0
    lateral_sign: float = 1.0
    vertical_sign: float = 1.0
    calibrated: bool = False
    calibration_id: str = ""
    supervised_release: bool = False
    supervised_release_id: str = ""
    supervised_max_speed_mps: float = 0.0


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


DirectionalSample = Tuple[float, float, float]


@dataclass(frozen=True)
class ClusterResult:
    clearance_m: float | None
    confidence: float
    selected_points: int
    selected_spatial_bins: int
    cluster_count: int
    pending_clearance_m: float | None


def _spatial_bin(sample: DirectionalSample, config: Xt16GeometryConfig) -> tuple[int, int]:
    bin_m = max(0.01, float(config.support_bin_m))
    return (math.floor(sample[1] / bin_m), math.floor(sample[2] / bin_m))


def _clearance_clusters(
    samples: Sequence[DirectionalSample],
    config: Xt16GeometryConfig,
) -> list[list[DirectionalSample]]:
    ordered = sorted(samples, key=lambda sample: sample[0])
    if not ordered:
        return []
    gap_m = max(0.01, float(config.clearance_cluster_gap_m))
    clusters: list[list[DirectionalSample]] = [[ordered[0]]]
    for sample in ordered[1:]:
        if sample[0] - clusters[-1][-1][0] <= gap_m:
            clusters[-1].append(sample)
        else:
            clusters.append([sample])
    return clusters


def _cluster_result(samples: Sequence[DirectionalSample], config: Xt16GeometryConfig) -> ClusterResult:
    clusters = _clearance_clusters(samples, config)
    min_points = max(1, int(config.min_points_per_roi))
    min_bins = max(1, int(config.min_spatial_bins))
    pending_min_points = max(1, min(min_points - 1 if min_points > 1 else 1, int(config.pending_min_points)))
    pending_clearance: float | None = None

    for cluster in clusters:
        bins = {_spatial_bin(sample, config) for sample in cluster}
        clearance = _percentile([sample[0] for sample in cluster], config.percentile)
        if clearance is None:
            continue
        clearance = round(max(0.0, float(clearance)), 3)
        if len(cluster) >= min_points and len(bins) >= min_bins:
            confidence = round(
                min(1.0, len(cluster) / min_points) * min(1.0, len(bins) / min_bins),
                3,
            )
            return ClusterResult(
                clearance_m=clearance,
                confidence=confidence,
                selected_points=len(cluster),
                selected_spatial_bins=len(bins),
                cluster_count=len(clusters),
                pending_clearance_m=pending_clearance,
            )
        if pending_clearance is None and len(cluster) >= pending_min_points:
            pending_clearance = clearance

    return ClusterResult(
        clearance_m=None,
        confidence=0.0,
        selected_points=0,
        selected_spatial_bins=0,
        cluster_count=len(clusters),
        pending_clearance_m=pending_clearance,
    )


def _conservative_clearance(*values: float | None) -> float | None:
    known = [float(value) for value in values if isinstance(value, (int, float)) and float(value) >= 0.0]
    return min(known) if known else None


def _directional_results(
    values: Mapping[str, Sequence[DirectionalSample]],
    config: Xt16GeometryConfig,
) -> dict[str, ClusterResult]:
    return {direction: _cluster_result(direction_values, config) for direction, direction_values in values.items()}


def _low_hazard_directions(
    low_hazard_clearance: Mapping[str, float | None],
) -> list[str]:
    thresholds = {
        "front": FRONT_PAUSE_M,
        "left": SIDE_SLOW_M,
        "right": SIDE_SLOW_M,
        "rear": REAR_SLOW_M,
    }
    return [
        direction
        for direction in ("front", "left", "right", "rear")
        if isinstance(low_hazard_clearance.get(direction), (int, float))
        and float(low_hazard_clearance[direction]) < thresholds[direction]
    ]


def _pending_low_hazard_directions(
    pending_clearance: Mapping[str, float | None],
) -> list[str]:
    thresholds = {
        "front": FRONT_PAUSE_M,
        "left": SIDE_SLOW_M,
        "right": SIDE_SLOW_M,
        "rear": REAR_SLOW_M,
    }
    return [
        direction
        for direction in ("front", "left", "right", "rear")
        if isinstance(pending_clearance.get(direction), (int, float))
        and float(pending_clearance[direction]) < thresholds[direction]
    ]


def build_xt16_geometry_summary(
    points: Iterable[Any],
    *,
    config: Xt16GeometryConfig | None = None,
    timestamp_ms: int | None = None,
    sequence: int | None = None,
    frame_id: str = "rslidar",
    latency_ms: float | None = None,
) -> dict[str, Any]:
    cfg = config or Xt16GeometryConfig()
    calibration_id = cfg.calibration_id.strip()
    calibration_verified = bool(cfg.calibrated and calibration_id)
    supervised_release_id = cfg.supervised_release_id.strip()
    supervised_release_active = bool(
        cfg.supervised_release
        and supervised_release_id
        and math.isclose(
            float(cfg.supervised_max_speed_mps),
            0.2,
            rel_tol=0.0,
            abs_tol=1e-9,
        )
    )
    operationally_released = calibration_verified or supervised_release_active
    body_values: dict[str, list[DirectionalSample]] = {
        direction: [] for direction in ("front", "left", "right", "rear")
    }
    low_hazard_values: dict[str, list[DirectionalSample]] = {
        direction: [] for direction in ("front", "left", "right", "rear")
    }
    body_far_support_values: dict[str, list[DirectionalSample]] = {
        direction: [] for direction in ("front", "left", "right", "rear")
    }
    total_points = 0
    finite_points = 0
    height_filtered_points = 0
    body_height_points = 0
    low_hazard_height_points = 0
    footprint_filtered_points = 0
    body_min_z_m = max(cfg.min_z_m, min(cfg.body_min_z_m, cfg.max_z_m))
    footprint_filter_margin_m = max(0.0, float(cfg.footprint_filter_margin_m))
    footprint_lateral_filter_margin_m = max(
        0.0,
        float(cfg.footprint_lateral_filter_margin_m),
    )

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
        height_values = body_values if vertical >= body_min_z_m else low_hazard_values
        if height_values is body_values:
            body_height_points += 1
        else:
            low_hazard_height_points += 1
        longitudinal_filter_margin_m = (
            footprint_filter_margin_m if height_values is body_values else 0.0
        )
        lateral_filter_margin_m = (
            footprint_lateral_filter_margin_m
            if height_values is body_values
            else 0.0
        )
        in_footprint = (
            -(cfg.footprint_rear_m + longitudinal_filter_margin_m)
            <= forward
            <= cfg.footprint_front_m + longitudinal_filter_margin_m
            and abs(lateral)
            <= cfg.footprint_half_width_m + lateral_filter_margin_m
        )
        if in_footprint:
            footprint_filtered_points += 1
            continue

        front_clearance = forward - cfg.footprint_front_m
        if front_clearance > 0 and abs(lateral) <= cfg.front_half_width_m:
            if front_clearance <= cfg.range_m:
                height_values["front"].append((front_clearance, lateral, vertical))
            elif height_values is body_values:
                body_far_support_values["front"].append((front_clearance, lateral, vertical))

        left_clearance = lateral - cfg.footprint_half_width_m
        if left_clearance > 0 and abs(forward) <= cfg.side_forward_m:
            if left_clearance <= cfg.range_m:
                height_values["left"].append((left_clearance, forward, vertical))
            elif height_values is body_values:
                body_far_support_values["left"].append((left_clearance, forward, vertical))

        right_clearance = -lateral - cfg.footprint_half_width_m
        if right_clearance > 0 and abs(forward) <= cfg.side_forward_m:
            if right_clearance <= cfg.range_m:
                height_values["right"].append((right_clearance, forward, vertical))
            elif height_values is body_values:
                body_far_support_values["right"].append((right_clearance, forward, vertical))

        rear_clearance = -forward - cfg.footprint_rear_m
        if rear_clearance > 0 and abs(lateral) <= cfg.rear_half_width_m:
            if rear_clearance <= cfg.range_m:
                height_values["rear"].append((rear_clearance, lateral, vertical))
            elif height_values is body_values:
                body_far_support_values["rear"].append((rear_clearance, lateral, vertical))

    body_results = _directional_results(body_values, cfg)
    low_hazard_results = _directional_results(low_hazard_values, cfg)
    body_far_support_results = _directional_results(body_far_support_values, cfg)
    body_clearance = {direction: result.clearance_m for direction, result in body_results.items()}
    low_hazard_clearance = {direction: result.clearance_m for direction, result in low_hazard_results.items()}
    body_roi_confidence = {direction: result.confidence for direction, result in body_results.items()}
    low_hazard_roi_confidence = {direction: result.confidence for direction, result in low_hazard_results.items()}
    pending_body_clearance = {
        direction: result.pending_clearance_m for direction, result in body_results.items()
    }
    pending_low_hazard_clearance = {
        direction: result.pending_clearance_m for direction, result in low_hazard_results.items()
    }
    pending_body_directions = _pending_low_hazard_directions(pending_body_clearance)
    pending_low_hazard_directions = _pending_low_hazard_directions(pending_low_hazard_clearance)
    cloud_supports_no_return = finite_points >= max(1, int(cfg.min_cloud_points_for_no_return))
    body_no_return_directions: list[str] = []
    if cloud_supports_no_return:
        for direction in ("front", "left", "right", "rear"):
            far_support = body_far_support_results[direction]
            if (
                body_clearance[direction] is None
                and direction not in pending_body_directions
                and far_support.clearance_m is not None
            ):
                body_clearance[direction] = round(float(cfg.range_m), 3)
                body_roi_confidence[direction] = round(
                    max(0.0, min(1.0, float(cfg.no_return_confidence))),
                    3,
                )
                body_no_return_directions.append(direction)
    clearance = {
        direction: _conservative_clearance(body_clearance[direction], low_hazard_clearance[direction])
        for direction in ("front", "left", "right", "rear")
    }
    roi_confidence = {
        direction: max(body_roi_confidence[direction], low_hazard_roi_confidence[direction])
        for direction in ("front", "left", "right", "rear")
    }
    front = clearance["front"]
    left = clearance["left"]
    right = clearance["right"]
    rear = clearance["rear"]
    confidence = round(
        min(
            roi_confidence["front"],
            roi_confidence["left"],
            roi_confidence["right"],
            roi_confidence["rear"],
        ),
        3,
    )
    missing_required = [
        name
        for name, value in {
            "front": front,
            "left": left,
            "right": right,
            "rear": rear,
        }.items()
        if value is None
    ]
    stale_reasons: list[str] = []
    if not operationally_released:
        stale_reasons.append("uncalibrated_xt16_geometry")
    if cfg.calibrated and not calibration_id:
        stale_reasons.append("missing_xt16_calibration_id")
    if cfg.supervised_release and not supervised_release_id:
        stale_reasons.append("missing_xt16_supervised_release_id")
    if cfg.supervised_release and not math.isclose(
        float(cfg.supervised_max_speed_mps),
        0.2,
        rel_tol=0.0,
        abs_tol=1e-9,
    ):
        stale_reasons.append("invalid_xt16_supervised_speed_limit")
    if missing_required:
        stale_reasons.append("missing_required_roi:" + ",".join(missing_required))
    advisory_reasons: list[str] = []
    pending_body_hard = list(pending_body_directions)
    pending_low_hazard_hard = list(pending_low_hazard_directions)
    if supervised_release_active:
        pending_body_hard = [
            direction for direction in pending_body_directions if direction == "front"
        ]
        pending_low_hazard_hard = [
            direction for direction in pending_low_hazard_directions if direction == "front"
        ]
        pending_body_advisory = [
            direction for direction in pending_body_directions if direction != "front"
        ]
        pending_low_hazard_advisory = [
            direction for direction in pending_low_hazard_directions if direction != "front"
        ]
        if pending_body_advisory:
            advisory_reasons.append(
                "pending_body_obstacle:" + ",".join(pending_body_advisory)
            )
        if pending_low_hazard_advisory:
            advisory_reasons.append(
                "pending_low_hazard:" + ",".join(pending_low_hazard_advisory)
            )
    if pending_body_hard:
        stale_reasons.append("pending_body_obstacle:" + ",".join(pending_body_hard))
    if pending_low_hazard_hard:
        stale_reasons.append(
            "pending_low_hazard:" + ",".join(pending_low_hazard_hard)
        )
    if total_points == 0 or finite_points == 0:
        stale_reasons.append("empty_or_invalid_pointcloud")

    summary: dict[str, Any] = {
        "type": "local_obstacle_summary",
        "schema_version": 2,
        "source": "lidar_pointcloud",
        "timestamp_ms": int(timestamp_ms if timestamp_ms is not None else now_ms()),
        "sequence": sequence,
        "frame_id": frame_id,
        "range_m": cfg.range_m,
        "parameters": {
            "calibrated": calibration_verified,
            "calibration_id": calibration_id or None,
            "supervised_release": {
                "active": supervised_release_active,
                "release_id": supervised_release_id or None,
                "max_speed_mps": (
                    float(cfg.supervised_max_speed_mps)
                    if supervised_release_active
                    else None
                ),
            },
        },
        "front_clearance_m": front,
        "left_clearance_m": left,
        "right_clearance_m": right,
        "rear_clearance_m": rear,
        "body_clearance_m": body_clearance,
        "low_hazard_clearance_m": low_hazard_clearance,
        "low_hazard_directions": _low_hazard_directions(low_hazard_clearance),
        "pending_body_clearance_m": pending_body_clearance,
        "pending_body_directions": pending_body_directions,
        "pending_low_hazard_clearance_m": pending_low_hazard_clearance,
        "pending_low_hazard_directions": pending_low_hazard_directions,
        "confidence": confidence,
        "roi_confidence": roi_confidence,
        "body_roi_confidence": body_roi_confidence,
        "low_hazard_roi_confidence": low_hazard_roi_confidence,
        "latency_ms": latency_ms,
        "stale": bool(stale_reasons),
        "stale_reasons": stale_reasons,
        "advisory_reasons": advisory_reasons,
        "blocked_directions": [],
        "narrow_passage": False,
        "recommended_action": "normal",
        "summary": {
            "points_total": total_points,
            "points_finite": finite_points,
            "points_in_height_band": height_filtered_points,
            "points_in_body_height_band": body_height_points,
            "points_in_low_hazard_band": low_hazard_height_points,
            "points_excluded_footprint": footprint_filtered_points,
            "cloud_supports_no_return": cloud_supports_no_return,
            "body_no_return_directions": body_no_return_directions,
            "body_far_support_counts": {
                direction: len(values)
                for direction, values in body_far_support_values.items()
            },
            "body_far_support": {
                direction: {
                    "clusters": result.cluster_count,
                    "selected_points": result.selected_points,
                    "selected_spatial_bins": result.selected_spatial_bins,
                }
                for direction, result in body_far_support_results.items()
            },
            "roi_counts": {
                direction: len(body_values[direction]) + len(low_hazard_values[direction])
                for direction in ("front", "left", "right", "rear")
            },
            "body_roi_counts": {direction: len(values) for direction, values in body_values.items()},
            "low_hazard_roi_counts": {direction: len(values) for direction, values in low_hazard_values.items()},
            "body_cluster_support": {
                direction: {
                    "clusters": result.cluster_count,
                    "selected_points": result.selected_points,
                    "selected_spatial_bins": result.selected_spatial_bins,
                }
                for direction, result in body_results.items()
            },
            "low_hazard_cluster_support": {
                direction: {
                    "clusters": result.cluster_count,
                    "selected_points": result.selected_points,
                    "selected_spatial_bins": result.selected_spatial_bins,
                }
                for direction, result in low_hazard_results.items()
            },
            "percentile": cfg.percentile,
            "clearance_cluster_gap_m": cfg.clearance_cluster_gap_m,
            "support_bin_m": cfg.support_bin_m,
            "min_spatial_bins": cfg.min_spatial_bins,
            "min_cloud_points_for_no_return": cfg.min_cloud_points_for_no_return,
            "no_return_confidence": cfg.no_return_confidence,
            "calibrated": calibration_verified,
            "calibration_id": calibration_id or None,
            "supervised_release": {
                "active": supervised_release_active,
                "release_id": supervised_release_id or None,
                "max_speed_mps": (
                    float(cfg.supervised_max_speed_mps)
                    if supervised_release_active
                    else None
                ),
            },
            "height_bands_m": {
                "low_hazard": [cfg.min_z_m, body_min_z_m],
                "body": [body_min_z_m, cfg.max_z_m],
            },
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
                "filter_margin": footprint_filter_margin_m,
                "longitudinal_filter_margin": footprint_filter_margin_m,
                "lateral_filter_margin": footprint_lateral_filter_margin_m,
                "filter_margin_applies_to": "body_height_only_per_axis",
            },
        },
    }
    directional_clearance = {
        "front": front,
        "left": left,
        "right": right,
        "rear": rear,
    }
    summary["blocked_directions"] = blocked_directions(directional_clearance)
    summary["narrow_passage"] = narrow_passage(directional_clearance)
    summary["recommended_action"] = recommended_action(directional_clearance)
    return summary
