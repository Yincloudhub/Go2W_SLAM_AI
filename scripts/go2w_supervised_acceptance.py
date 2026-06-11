#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
import subprocess
import sys
import time
from pathlib import Path
from typing import Any, Callable


REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = REPO_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from edge_autonomy.chassis_controller import (  # noqa: E402
    GatewayConfig,
    PersistentGatewaySession,
    run_gateway_command,
)
from edge_autonomy.gateway_safety import gateway_allows_navigation  # noqa: E402
from edge_autonomy.map_registry import MapProfile, MapRegistry, RelocalizationAnchor, TopologyNode  # noqa: E402
from edge_autonomy.runtime_readiness import assess_runtime_readiness, load_json_summary  # noqa: E402


DEFAULT_REGISTRY = REPO_ROOT / "configs" / "maps" / "go2w_real_site_map_registry.json"
DEFAULT_GATEWAY_CLIENT = REPO_ROOT / "robot" / "slam_gateway_refactor" / "build" / "slam_llm_command_client"
DEFAULT_LIDAR_SUMMARY = REPO_ROOT / "artifacts" / "lidar_geometry_summary.json"
BLOCKING_TARGET_TAGS = {
    "disabled",
    "ui_disabled",
    "deleted",
    "needs_calibration",
    "needs_standing_verification",
    "requires_standing_verification",
}
REQUIRED_NAVIGATION_TAG = "live_verified"
LOCALIZATION_HEALTHY_STATUSES = {
    "localized",
    "localized_or_tracking",
    "tracking",
    "degraded",
}


def finite_number(value: Any) -> float | None:
    if isinstance(value, bool):
        return None
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def angle_error_rad(current: float, target: float) -> float:
    return abs((current - target + math.pi) % (2.0 * math.pi) - math.pi)


def verified_anchor(anchor: RelocalizationAnchor) -> bool:
    status = anchor.status.strip().lower()
    return status == "verified" or status.startswith("verified_")


def runtime_process_status() -> dict[str, Any]:
    processes: dict[str, bool] = {}
    errors: dict[str, str] = {}
    for name in ("xt16_driver", "unitree_slam"):
        try:
            result = subprocess.run(
                ["pidof", name],
                text=True,
                capture_output=True,
                timeout=2,
                check=False,
            )
            processes[name] = result.returncode == 0 and bool(result.stdout.strip())
        except (OSError, subprocess.SubprocessError) as exc:
            processes[name] = False
            errors[name] = str(exc)
    return {
        "ready": all(processes.values()),
        "processes": processes,
        "errors": errors,
    }


def gateway_config(args: argparse.Namespace) -> GatewayConfig:
    return GatewayConfig(
        client_path=args.gateway_client,
        network_interface=args.network_interface,
        timeout_s=args.timeout_s,
        startup_wait_s=args.gateway_startup_wait_s,
    )


def load_profile(args: argparse.Namespace) -> MapProfile:
    return MapRegistry.from_file(args.registry).get_map(args.map_id)


def world_state(args: argparse.Namespace) -> dict[str, Any]:
    return run_gateway_command({"action": "get_world_state"}, gateway_config(args))


def localization_is_healthy(response: dict[str, Any], *, max_pose_age_ms: float) -> bool:
    world = response.get("world_state") if isinstance(response, dict) else None
    if not isinstance(world, dict):
        return False
    localization = world.get("localization")
    health = world.get("slam_health")
    if not isinstance(localization, dict) or not isinstance(health, dict):
        return False
    pose_age_ms = finite_number(localization.get("pose_age_ms"))
    return (
        str(localization.get("status") or "") in LOCALIZATION_HEALTHY_STATUSES
        and pose_age_ms is not None
        and 0 <= pose_age_ms <= max_pose_age_ms
        and str(health.get("status") or "") in {"ok", "degraded"}
        and health.get("slam_alive") is True
        and health.get("localization_alive") is True
    )


def map_identity_check(response: dict[str, Any], profile: MapProfile) -> dict[str, Any]:
    world = response.get("world_state") if isinstance(response, dict) else None
    current_pose = world.get("current_pose") if isinstance(world, dict) else None
    localization = world.get("localization") if isinstance(world, dict) else None
    current_map_id = str(current_pose.get("map_id") or "").strip() if isinstance(current_pose, dict) else ""
    pose_map_path = str(current_pose.get("map_path") or "").strip() if isinstance(current_pose, dict) else ""
    localization_map_id = (
        str(localization.get("map_id") or "").strip() if isinstance(localization, dict) else ""
    )
    localization_map_path = (
        str(localization.get("map_path") or "").strip() if isinstance(localization, dict) else ""
    )
    expected_ids = {profile.map_id, Path(profile.pcd_path).stem}
    reported_ids = {
        source: value
        for source, value in (
            ("current_pose", current_map_id),
            ("localization", localization_map_id),
        )
        if value
    }
    reported_paths = {
        source: value
        for source, value in (
            ("current_pose", pose_map_path),
            ("localization", localization_map_path),
        )
        if value
    }
    conflicting_ids = {
        source: value
        for source, value in reported_ids.items()
        if value == "debug_map" or value not in expected_ids
    }
    conflicting_paths = {
        source: value
        for source, value in reported_paths.items()
        if Path(value) != Path(profile.pcd_path)
    }
    if not isinstance(current_pose, dict):
        matches = False
        reason = "missing current_pose map identity"
    elif conflicting_ids:
        matches = False
        reason = "active SLAM map id does not match registry"
    elif conflicting_paths:
        matches = False
        reason = "active SLAM map path does not match registry"
    elif not reported_paths:
        matches = False
        reason = "active SLAM map path was not reported; map id alone is insufficient"
    else:
        matches = True
        reason = "active SLAM map identity matches registry"
    return {
        "matches": matches,
        "current_map_id": current_map_id,
        "current_map_path": pose_map_path or localization_map_path,
        "current_pose_map_path": pose_map_path,
        "localization_map_id": localization_map_id,
        "localization_map_path": localization_map_path,
        "conflicting_map_ids": conflicting_ids,
        "conflicting_map_paths": conflicting_paths,
        "expected_map_id": profile.map_id,
        "expected_map_ids": sorted(expected_ids),
        "expected_map_path": profile.pcd_path,
        "reason": reason,
    }


def gateway_failure_reason(result: dict[str, Any]) -> str:
    reason = str(result.get("reason") or "").strip()
    status_code = result.get("status_code")
    data = str(result.get("data") or "").strip()
    parts = [reason or "gateway rejected request"]
    if status_code is not None:
        parts.append(f"status_code={status_code}")
    if data:
        parts.append(f"data={data}")
    return "; ".join(parts)


def compact_world(response: dict[str, Any]) -> dict[str, Any]:
    world = response.get("world_state") if isinstance(response, dict) else None
    if not isinstance(world, dict):
        return {"available": False}
    current_pose = world.get("current_pose", {})
    pose = current_pose.get("pose", {}) if isinstance(current_pose, dict) else {}
    localization = world.get("localization", {})
    health = world.get("slam_health", {})
    obstacle = world.get("local_obstacle", {})
    safety = world.get("safety", {})
    navigation = world.get("navigation", {})
    return {
        "available": True,
        "map": {
            "map_id": current_pose.get("map_id"),
            "map_path": current_pose.get("map_path"),
        }
        if isinstance(current_pose, dict)
        else None,
        "pose": {
            "x": pose.get("x"),
            "y": pose.get("y"),
            "yaw": pose.get("yaw"),
        }
        if isinstance(pose, dict)
        else None,
        "localization": {
            "status": localization.get("status"),
            "pose_age_ms": localization.get("pose_age_ms"),
            "confidence": localization.get("confidence"),
        }
        if isinstance(localization, dict)
        else None,
        "slam": {
            "status": health.get("status"),
            "slam_alive": health.get("slam_alive"),
            "localization_alive": health.get("localization_alive"),
        }
        if isinstance(health, dict)
        else None,
        "obstacle": {
            "source": obstacle.get("source"),
            "stale": obstacle.get("stale"),
            "age_ms": obstacle.get("age_ms"),
            "recommended_action": obstacle.get("recommended_action"),
        }
        if isinstance(obstacle, dict)
        else None,
        "safety": {
            "allow_navigation": safety.get("allow_navigation"),
            "reason": safety.get("reason"),
        }
        if isinstance(safety, dict)
        else None,
        "navigation": {
            "state": navigation.get("state"),
            "target_node": navigation.get("target_node"),
            "is_arrived": navigation.get("is_arrived"),
        }
        if isinstance(navigation, dict)
        else None,
    }


def anchor_error(response: dict[str, Any], anchor: RelocalizationAnchor) -> dict[str, Any]:
    world = response.get("world_state") if isinstance(response, dict) else None
    pose = world.get("current_pose", {}).get("pose") if isinstance(world, dict) else None
    if not isinstance(pose, dict):
        return {"valid": False, "reason": "missing current pose"}
    x = finite_number(pose.get("x"))
    y = finite_number(pose.get("y"))
    yaw = finite_number(pose.get("yaw"))
    if x is None or y is None or yaw is None:
        return {"valid": False, "reason": "current pose x/y/yaw is invalid"}
    distance_m = math.hypot(x - anchor.pose.x, y - anchor.pose.y)
    yaw_error = angle_error_rad(yaw, float(anchor.pose.yaw or 0.0))
    return {
        "valid": True,
        "distance_m": distance_m,
        "yaw_error_deg": math.degrees(yaw_error),
        "within_radius": distance_m <= anchor.allowed_radius_m,
        "within_yaw": math.degrees(yaw_error) <= anchor.allowed_yaw_error_deg,
    }


def validate_localization_sample(
    response: dict[str, Any],
    anchor: RelocalizationAnchor,
    *,
    max_pose_age_ms: float,
) -> tuple[bool, dict[str, Any]]:
    world = response.get("world_state") if isinstance(response, dict) else None
    if not isinstance(world, dict):
        return False, {"reason": "missing world_state"}
    localization = world.get("localization")
    health = world.get("slam_health")
    if not isinstance(localization, dict) or not isinstance(health, dict):
        return False, {"reason": "missing localization or slam health"}
    pose_age_ms = finite_number(localization.get("pose_age_ms"))
    anchor_check = anchor_error(response, anchor)
    checks = {
        "localization_status": localization.get("status"),
        "pose_age_ms": pose_age_ms,
        "slam_status": health.get("status"),
        "slam_alive": health.get("slam_alive"),
        "localization_alive": health.get("localization_alive"),
        "anchor_error": anchor_check,
    }
    ok = (
        localization.get("status") == "localized"
        and pose_age_ms is not None
        and 0 <= pose_age_ms <= max_pose_age_ms
        and health.get("status") == "ok"
        and health.get("slam_alive") is True
        and health.get("localization_alive") is True
        and anchor_check.get("valid") is True
        and anchor_check.get("within_radius") is True
        and anchor_check.get("within_yaw") is True
    )
    return ok, checks


def status_stage(args: argparse.Namespace) -> tuple[int, dict[str, Any]]:
    profile = load_profile(args)
    processes = runtime_process_status()
    response = world_state(args)
    lidar_summary = load_json_summary(args.lidar_summary)
    readiness = assess_runtime_readiness(
        response,
        startup_ok=processes["ready"],
        lidar_summary=lidar_summary,
        llm_configured=Path(args.llm_model).is_file() and Path(args.llm_ask_script).is_file(),
    )
    return 0, {
        "stage": "status",
        "motion_commands_sent": False,
        "runtime_processes": processes,
        "readiness": readiness,
        "world": compact_world(response),
        "build_map_origin": profile.mapping_origin_anchor_id or None,
        "active_relocalization_anchors": [
            {
                "anchor_id": anchor.anchor_id,
                "name": anchor.name,
                "status": anchor.status,
                "verified": verified_anchor(anchor),
                "is_build_map_origin": anchor.anchor_id == profile.mapping_origin_anchor_id,
            }
            for anchor in profile.relocalization_anchors
        ],
        "next_command": None,
        "operator_action": (
            "select the active verified relocation anchor matching the robot's physical pose, "
            "align the robot there, then pass that anchor ID explicitly"
        )
        if readiness.get("relocalization_ready") and not readiness.get("localization_ready")
        else None,
    }


def verify_samples(
    args: argparse.Namespace,
    profile: MapProfile,
    anchor: RelocalizationAnchor,
    *,
    get_world: Callable[[], dict[str, Any]],
    minimum_timestamp_ms: float | None = None,
) -> tuple[bool, list[dict[str, Any]]]:
    samples: list[dict[str, Any]] = []
    last_timestamp = minimum_timestamp_ms
    all_ok = True
    for index in range(args.samples):
        try:
            response = get_world()
        except Exception as exc:
            samples.append(
                {
                    "index": index + 1,
                    "ok": False,
                    "reason": f"world-state query failed: {exc}",
                    "timestamp_ms": None,
                    "timestamp_advanced": False,
                }
            )
            all_ok = False
            if index + 1 < args.samples:
                time.sleep(args.interval_s)
            continue
        ok, checks = validate_localization_sample(
            response,
            anchor,
            max_pose_age_ms=args.max_pose_age_ms,
        )
        world = response.get("world_state") if isinstance(response, dict) else None
        current_pose = world.get("current_pose") if isinstance(world, dict) else None
        timestamp = finite_number(current_pose.get("timestamp_ms")) if isinstance(current_pose, dict) else None
        timestamp_valid = timestamp is not None and timestamp > 0
        timestamp_advanced = timestamp_valid and (last_timestamp is None or timestamp > last_timestamp)
        map_identity = map_identity_check(response, profile)
        sample_ok = ok and timestamp_advanced and map_identity["matches"]
        samples.append(
            {
                "index": index + 1,
                "ok": sample_ok,
                "timestamp_ms": timestamp,
                "timestamp_advanced": timestamp_advanced,
                "map_identity": map_identity,
                **checks,
            }
        )
        all_ok = all_ok and sample_ok
        if timestamp_valid:
            last_timestamp = timestamp
        if index + 1 < args.samples:
            time.sleep(args.interval_s)
    return all_ok, samples


def verify_persistent_samples(
    args: argparse.Namespace,
    profile: MapProfile,
    anchor: RelocalizationAnchor,
    *,
    minimum_timestamp_ms: float | None = None,
) -> tuple[bool, list[dict[str, Any]]]:
    try:
        with PersistentGatewaySession(
            client_path=args.gateway_client,
            network_interface=args.network_interface,
            timeout_s=args.timeout_s,
            startup_wait_s=args.gateway_startup_wait_s,
        ) as session:
            return verify_samples(
                args,
                profile,
                anchor,
                get_world=lambda: session.command({"action": "get_world_state"}),
                minimum_timestamp_ms=minimum_timestamp_ms,
            )
    except Exception as exc:
        return False, [
            {
                "index": 1,
                "ok": False,
                "reason": f"persistent gateway session failed: {exc}",
                "timestamp_ms": None,
                "timestamp_advanced": False,
            }
        ]


def relocate_stage(args: argparse.Namespace) -> tuple[int, dict[str, Any]]:
    processes = runtime_process_status()
    if not processes["ready"]:
        return 7, {
            "stage": "relocate",
            "accepted": False,
            "motion_commands_sent": False,
            "reason": "XT16 and Unitree SLAM processes must be running before relocation",
            "runtime_processes": processes,
        }
    if not args.anchor:
        return 2, {
            "stage": "relocate",
            "accepted": False,
            "motion_commands_sent": False,
            "reason": "--anchor is required; choose the active verified anchor matching the physical pose",
        }
    profile = load_profile(args)
    anchor = profile.get_anchor(args.anchor)
    if not verified_anchor(anchor):
        return 3, {
            "stage": "relocate",
            "accepted": False,
            "motion_commands_sent": False,
            "reason": f"anchor {anchor.anchor_id} is not verified: {anchor.status}",
        }
    before = world_state(args)
    before_map_identity = map_identity_check(before, profile)
    if (
        localization_is_healthy(before, max_pose_age_ms=args.max_pose_age_ms)
        and before_map_identity["matches"]
    ):
        return 3, {
            "stage": "relocate",
            "accepted": False,
            "motion_commands_sent": False,
            "reason": "localization is already healthy and fresh; recovery relocation is not allowed",
            "map_identity": before_map_identity,
            "world": compact_world(before),
        }
    if args.confirm_relocation != anchor.anchor_id:
        return 2, {
            "stage": "relocate",
            "accepted": False,
            "motion_commands_sent": False,
            "reason": f"type --confirm-relocation {anchor.anchor_id} after physically aligning the robot",
            "anchor": {
                "anchor_id": anchor.anchor_id,
                "status": anchor.status,
                "allowed_radius_m": anchor.allowed_radius_m,
                "allowed_yaw_error_deg": anchor.allowed_yaw_error_deg,
            },
        }

    try:
        result = run_gateway_command(profile.relocate_command(anchor.anchor_id), gateway_config(args))
    except Exception as exc:
        return 7, {
            "stage": "relocate",
            "accepted": False,
            "motion_commands_sent": False,
            "relocation_request_state": "unknown",
            "reason": f"relocation gateway call failed: {exc}",
            "next_command": "do not navigate; query status before retrying relocation",
        }
    if result.get("accepted") is not True:
        return 4, {
            "stage": "relocate",
            "accepted": False,
            "motion_commands_sent": False,
            "reason": gateway_failure_reason(result),
            "gateway_result": result,
        }
    if args.relocation_settle_s > 0:
        time.sleep(args.relocation_settle_s)
    relocation_world = result.get("world_state") if isinstance(result, dict) else None
    relocation_pose = (
        relocation_world.get("current_pose")
        if isinstance(relocation_world, dict)
        else None
    )
    relocation_timestamp = (
        finite_number(relocation_pose.get("timestamp_ms"))
        if isinstance(relocation_pose, dict)
        else None
    )
    verified, samples = verify_persistent_samples(
        args,
        profile,
        anchor,
        minimum_timestamp_ms=relocation_timestamp,
    )
    return (0 if verified else 5), {
        "stage": "relocate",
        "accepted": True,
        "motion_commands_sent": False,
        "anchor_id": anchor.anchor_id,
        "localization_verified": verified,
        "samples": samples,
        "next_command": (
            f"python3 scripts/go2w_supervised_acceptance.py --stage prepare-navigation "
            f"--target TARGET_NODE"
        )
        if verified
        else "do not navigate; inspect SLAM/ICP and anchor alignment",
    }


def verify_localization_stage(args: argparse.Namespace) -> tuple[int, dict[str, Any]]:
    if not args.anchor:
        return 2, {
            "stage": "verify-localization",
            "motion_commands_sent": False,
            "localization_verified": False,
            "reason": "--anchor is required; verification compares the pose against that explicit anchor",
        }
    profile = load_profile(args)
    anchor = profile.get_anchor(args.anchor)
    if not verified_anchor(anchor):
        return 3, {
            "stage": "verify-localization",
            "motion_commands_sent": False,
            "anchor_id": anchor.anchor_id,
            "localization_verified": False,
            "reason": f"anchor {anchor.anchor_id} is not verified: {anchor.status}",
        }
    verified, samples = verify_persistent_samples(args, profile, anchor)
    return (0 if verified else 5), {
        "stage": "verify-localization",
        "motion_commands_sent": False,
        "anchor_id": anchor.anchor_id,
        "localization_verified": verified,
        "samples": samples,
    }


def target_blockers(node: TopologyNode) -> list[str]:
    blockers = set(node.tags) & BLOCKING_TARGET_TAGS
    if REQUIRED_NAVIGATION_TAG not in node.tags:
        blockers.add(f"missing_{REQUIRED_NAVIGATION_TAG}")
    return sorted(blockers)


def prepare_navigation_stage(args: argparse.Namespace) -> tuple[int, dict[str, Any]]:
    if not args.target:
        return 2, {
            "stage": "prepare-navigation",
            "motion_commands_sent": False,
            "ready": False,
            "reason": "--target is required",
        }
    profile = load_profile(args)
    node = profile.get_node(args.target)
    blockers = target_blockers(node)
    response = world_state(args)
    processes = runtime_process_status()
    lidar_summary = load_json_summary(args.lidar_summary)
    readiness = assess_runtime_readiness(
        response,
        startup_ok=processes["ready"],
        lidar_summary=lidar_summary,
        llm_configured=Path(args.llm_model).is_file() and Path(args.llm_ask_script).is_file(),
        max_localization_pose_age_ms=args.max_pose_age_ms,
        max_perception_age_ms=int(args.max_obstacle_age_ms),
    )
    navigation_ready, reason = gateway_allows_navigation(response, max_obstacle_age_ms=args.max_obstacle_age_ms)
    map_identity = map_identity_check(response, profile)
    ready = (
        processes["ready"]
        and readiness.get("localization_ready") is True
        and readiness.get("perception_ready") is True
        and navigation_ready
        and map_identity["matches"]
        and not blockers
    )
    blocking_reasons: list[str] = []
    if not processes["ready"]:
        blocking_reasons.append("runtime processes are not ready")
    if readiness.get("localization_ready") is not True:
        blocking_reasons.append(str(readiness.get("localization_reason") or "localization is not ready"))
    if readiness.get("perception_ready") is not True:
        blocking_reasons.append(str(readiness.get("perception_reason") or "perception is not ready"))
    if not navigation_ready:
        blocking_reasons.append(reason)
    if not map_identity["matches"]:
        blocking_reasons.append(map_identity["reason"])
    blocking_reasons.extend(blockers)
    command = (
        "python3 scripts/go2w_agent_entry.py "
        f"--go {node.node_id} --execute --nav-speed-mps {args.nav_speed_mps:.2f} "
        "--no-auto-start-slam --no-auto-relocate --human"
    )
    return (0 if ready else 6), {
        "stage": "prepare-navigation",
        "motion_commands_sent": False,
        "ready": ready,
        "reason": "; ".join(dict.fromkeys(blocking_reasons)) if not ready else "all supervised navigation gates passed",
        "runtime_processes": processes,
        "readiness": readiness,
        "map_identity": map_identity,
        "target": {
            "node_id": node.node_id,
            "name": node.name,
            "tags": list(node.tags),
            "blocking_tags": blockers,
        },
        "navigation_gate": {"allowed": navigation_ready, "reason": reason},
        "world": compact_world(response),
        "supervised_motion_command": command if ready else None,
        "operator_requirements": [
            "keep the emergency stop available",
            "keep the robot in sight",
            "run only one short target at 0.1 m/s",
            "stop if clearance, localization, or pose disagrees with the scene",
        ],
    }


def human_output(payload: dict[str, Any]) -> str:
    stage = payload.get("stage")
    lines = [f"阶段: {stage}", f"底盘运动命令: {'已发送' if payload.get('motion_commands_sent') else '未发送'}"]
    if stage == "status":
        readiness = payload.get("readiness", {})
        anchors = payload.get("active_relocalization_anchors", [])
        anchor_labels = [
            f"{item.get('anchor_id')}{'（建图原点）' if item.get('is_build_map_origin') else ''}"
            for item in anchors
            if item.get("verified")
        ]
        lines.extend(
            [
                f"服务: {readiness.get('services_ready')}",
                f"可重定位: {readiness.get('relocalization_ready')}",
                f"定位: {readiness.get('localization_ready')}",
                f"感知: {readiness.get('perception_ready')}",
                f"可导航: {readiness.get('navigation_ready')}",
                f"下一步: {readiness.get('next_action')}",
                f"建图原点: {payload.get('build_map_origin') or '未登记'}",
                f"可选重定位锚点: {', '.join(anchor_labels) if anchor_labels else '无'}",
            ]
        )
        if payload.get("operator_action"):
            lines.append("操作要求: 按机器人的真实物理位置选择对应锚点，现场对齐后显式确认")
    else:
        for key in ("accepted", "localization_verified", "ready", "reason", "anchor_id"):
            if key in payload:
                lines.append(f"{key}: {payload[key]}")
    if payload.get("next_command"):
        lines.append(f"下一条命令: {payload['next_command']}")
    if payload.get("supervised_motion_command"):
        lines.append(f"人工确认后运行: {payload['supervised_motion_command']}")
    return "\n".join(lines)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Guided GO2W supervised acceptance. Navigation is never executed by this tool.")
    parser.add_argument(
        "--stage",
        choices=["status", "relocate", "verify-localization", "prepare-navigation"],
        default="status",
    )
    parser.add_argument("--anchor", default="")
    parser.add_argument("--target", default="")
    parser.add_argument("--confirm-relocation", default="")
    parser.add_argument("--samples", type=int, default=5)
    parser.add_argument("--interval-s", type=float, default=1.0)
    parser.add_argument("--relocation-settle-s", type=float, default=3.0)
    parser.add_argument("--max-pose-age-ms", type=float, default=500.0)
    parser.add_argument("--max-obstacle-age-ms", type=float, default=1500.0)
    parser.add_argument("--nav-speed-mps", type=float, default=0.1)
    parser.add_argument("--registry", default=str(DEFAULT_REGISTRY))
    parser.add_argument("--map-id", default="go2w_real_site")
    parser.add_argument("--gateway-client", default=str(DEFAULT_GATEWAY_CLIENT))
    parser.add_argument("--network-interface", default="eth0")
    parser.add_argument("--timeout-s", type=int, default=30)
    parser.add_argument("--gateway-startup-wait-s", type=float, default=1.0)
    parser.add_argument("--lidar-summary", default=str(DEFAULT_LIDAR_SUMMARY))
    parser.add_argument("--llm-model", default="/home/unitree/models/Qwen_Qwen3-4B-Instruct-2507-Q4_K_M.gguf")
    parser.add_argument("--llm-ask-script", default="/home/unitree/llm_runtime/scripts/ask_qwen.sh")
    parser.add_argument("--json", action="store_true")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if args.samples < 1:
        raise SystemExit("--samples must be at least 1")
    if not 0 < args.nav_speed_mps <= 0.1:
        raise SystemExit("--nav-speed-mps must be in (0, 0.1] for supervised acceptance")
    stages = {
        "status": status_stage,
        "relocate": relocate_stage,
        "verify-localization": verify_localization_stage,
        "prepare-navigation": prepare_navigation_stage,
    }
    try:
        code, payload = stages[args.stage](args)
    except Exception as exc:
        code = 7
        payload = {
            "stage": args.stage,
            "accepted": False,
            "ready": False,
            "motion_commands_sent": False,
            "reason": f"acceptance stage failed safely: {exc}",
        }
    if args.json:
        print(json.dumps(payload, ensure_ascii=False, indent=2))
    else:
        print(human_output(payload))
    return code


if __name__ == "__main__":
    raise SystemExit(main())
