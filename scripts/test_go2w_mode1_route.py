#!/usr/bin/env python3
"""Run a supervised mode=1 route test against a GO2W robot.

This script temporarily patches the robot-side navigation registry long enough
for slam_llm_command_client to load a mode=1 snapshot, restores the original
registry immediately after the persistent navigation session is ready, then
drives a waypoint sequence with heartbeats and world-state monitoring.
"""

from __future__ import annotations

import argparse
import datetime as dt
import json
import math
import os
import sys
import time
from pathlib import Path
from typing import Any

import paramiko


REGISTRY_PATH = "/home/unitree/Go2W_SLAM_AI/configs/maps/go2w_real_site_map_registry.json"
REPO_DIR = "/home/unitree/Go2W_SLAM_AI"
GATEWAY_CLIENT = "./robot/slam_gateway_refactor/build/slam_llm_command_client"
MAP_ID = "go2w_real_site"
MAP_PATH = "/home/unitree/maps/staging/map_701.pcd"
ROUTE = [
    "wp_60497f",
    "wp_b738c7",
    "wp_bbcb4c",
    "wp_a684c6",
    "wp_2ae043",
    "wp_e643bc",
    "transition_701_to_701_left",
    "transition_701_to_terrace",
]


class JsonlLogger:
    def __init__(self, path: Path) -> None:
        self.path = path
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._fh = self.path.open("a", encoding="utf-8")

    def close(self) -> None:
        self._fh.close()

    def event(self, event: str, **payload: Any) -> None:
        row = {
            "ts": dt.datetime.now(dt.timezone.utc).isoformat(),
            "event": event,
            **payload,
        }
        line = json.dumps(row, ensure_ascii=False, separators=(",", ":"))
        print(line, flush=True)
        self._fh.write(line + "\n")
        self._fh.flush()


def now_tag() -> str:
    return dt.datetime.now().strftime("%Y%m%d_%H%M%S")


def pose_distance(pose: dict[str, Any], target: dict[str, Any]) -> float | None:
    try:
        return math.hypot(float(pose["x"]) - float(target["x"]), float(pose["y"]) - float(target["y"]))
    except Exception:
        return None


def recv_json_lines(chan: paramiko.Channel, buffer: bytearray) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    while chan.recv_ready():
        buffer.extend(chan.recv(65536))
    while True:
        try:
            idx = buffer.index(10)
        except ValueError:
            break
        raw = bytes(buffer[:idx])
        del buffer[: idx + 1]
        text = raw.decode("utf-8", "replace").strip()
        if not text:
            continue
        try:
            rows.append(json.loads(text))
        except json.JSONDecodeError:
            rows.append({"_non_json": text})
    return rows


def send_json(chan: paramiko.Channel, command: dict[str, Any]) -> None:
    chan.send((json.dumps(command, ensure_ascii=False, separators=(",", ":")) + "\n").encode("utf-8"))


def wait_for_response(
    chan: paramiko.Channel,
    buffer: bytearray,
    request_id: str,
    logger: JsonlLogger,
    timeout_s: float,
) -> dict[str, Any] | None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        for row in recv_json_lines(chan, buffer):
            logger.event("gateway_rx", row=row)
            if row.get("request_id") == request_id:
                return row
        if chan.exit_status_ready():
            return None
        time.sleep(0.05)
    return None


def load_and_patch_registry(
    sftp: paramiko.SFTPClient,
    logger: JsonlLogger,
    speed_mps: float,
) -> tuple[bytes, str, dict[str, dict[str, Any]]]:
    with sftp.open(REGISTRY_PATH, "rb") as fh:
        original = fh.read()
    backup_path = f"{REGISTRY_PATH}.codex_mode1_backup_{now_tag()}"
    with sftp.open(backup_path, "wb") as fh:
        fh.write(original)

    data = json.loads(original.decode("utf-8"))
    patched_targets: dict[str, dict[str, Any]] = {}
    found: set[str] = set()
    for m in data.get("maps", []):
        if m.get("map_id") != MAP_ID:
            continue
        for node in m.get("topology_nodes", []):
            node_id = node.get("node_id")
            if node_id not in ROUTE:
                continue
            pose = node.get("pose")
            if not isinstance(pose, dict):
                raise RuntimeError(f"missing pose for {node_id}")
            found.add(node_id)
            pose["mode"] = 1
            if float(pose.get("speed", 0.0) or 0.0) <= 0.0:
                pose["speed"] = speed_mps
            patched_pose = dict(pose)
            patched_pose["name"] = node_id
            patched_pose["speed"] = min(float(patched_pose.get("speed", speed_mps)), speed_mps)
            patched_pose["mode"] = 1
            patched_targets[node_id] = patched_pose
    missing = [node_id for node_id in ROUTE if node_id not in found]
    if missing:
        raise RuntimeError(f"route nodes missing from registry: {missing}")

    encoded = (json.dumps(data, ensure_ascii=False, indent=2) + "\n").encode("utf-8")
    json.loads(encoded.decode("utf-8"))
    with sftp.open(REGISTRY_PATH, "wb") as fh:
        fh.write(encoded)
    logger.event(
        "registry_patched",
        registry_path=REGISTRY_PATH,
        backup_path=backup_path,
        route=ROUTE,
        speed_mps=speed_mps,
    )
    return original, backup_path, patched_targets


def restore_registry(sftp: paramiko.SFTPClient, original: bytes, logger: JsonlLogger) -> None:
    with sftp.open(REGISTRY_PATH, "wb") as fh:
        fh.write(original)
    with sftp.open(REGISTRY_PATH, "rb") as fh:
        restored = fh.read()
    if restored != original:
        raise RuntimeError("registry restore verification failed")
    logger.event("registry_restored", registry_path=REGISTRY_PATH)


def start_navigation_session(client: paramiko.SSHClient, logger: JsonlLogger) -> tuple[paramiko.Channel, bytearray, str]:
    command = f"cd {REPO_DIR} && {GATEWAY_CLIENT} eth0 --persistent-navigation-session"
    chan = client.get_transport().open_session()
    chan.exec_command(command)
    buffer = bytearray()
    deadline = time.monotonic() + 12.0
    while time.monotonic() < deadline:
        for row in recv_json_lines(chan, buffer):
            logger.event("gateway_rx", row=row)
            if row.get("type") == "navigation_session_ready":
                return chan, buffer, str(row["session_token"])
            if row.get("type") == "navigation_session_unready":
                raise RuntimeError(f"navigation session unready: {row}")
        if chan.exit_status_ready():
            raise RuntimeError("navigation session exited before ready")
        time.sleep(0.05)
    raise RuntimeError("navigation session startup timed out")


def pause_navigation(chan: paramiko.Channel, buffer: bytearray, logger: JsonlLogger, reason: str) -> None:
    request_id = f"pause_{int(time.time() * 1000)}"
    logger.event("pause_request", reason=reason, request_id=request_id)
    try:
        send_json(chan, {"action": "pause_navigation", "request_id": request_id})
        wait_for_response(chan, buffer, request_id, logger, 6.0)
    except Exception as exc:
        logger.event("pause_error", reason=reason, error=str(exc))


def test_leg(
    chan: paramiko.Channel,
    buffer: bytearray,
    logger: JsonlLogger,
    token: str,
    target_node: str,
    target_pose: dict[str, Any],
    leg_index: int,
    arrival_radius_m: float,
    speed_mps: float,
) -> dict[str, Any]:
    request_id = f"nav_{leg_index}_{target_node}_{int(time.time() * 1000)}"
    command = {
        "action": "navigate_to_pose",
        "request_id": request_id,
        "navigation_session_token": token,
        "operator_ack": True,
        "map_id": MAP_ID,
        "map_path": MAP_PATH,
        "target_node": target_node,
        "target_pose": {**target_pose, "speed": speed_mps, "mode": 1, "name": target_node},
    }
    logger.event("leg_start", index=leg_index, target_node=target_node, target_pose=command["target_pose"])
    send_json(chan, command)
    response = wait_for_response(chan, buffer, request_id, logger, 12.0)
    if not response:
        pause_navigation(chan, buffer, logger, "submit_timeout")
        return {"target_node": target_node, "status": "submit_timeout"}
    if response.get("accepted") is not True:
        pause_navigation(chan, buffer, logger, "submit_rejected")
        return {
            "target_node": target_node,
            "status": "submit_rejected",
            "reason": response.get("reason"),
            "response": response,
        }

    first_distance: float | None = None
    best_distance: float | None = None
    last_progress = time.monotonic()
    reached_count = 0
    submitted = time.monotonic()
    timeout_s = 45.0
    heartbeat_due = 0.0
    world_state_due = 0.0
    heartbeat_index = 0
    world_state_index = 0
    last_world_state: dict[str, Any] | None = None
    last_distance: float | None = None

    while True:
        now = time.monotonic()
        if now - submitted > timeout_s:
            pause_navigation(chan, buffer, logger, "leg_timeout")
            return {
                "target_node": target_node,
                "status": "timeout",
                "first_distance_m": first_distance,
                "best_distance_m": best_distance,
                "last_distance_m": last_distance,
                "last_world_state": last_world_state,
            }
        if now - last_progress > 16.0 and (last_distance is None or last_distance > arrival_radius_m):
            pause_navigation(chan, buffer, logger, "no_progress")
            return {
                "target_node": target_node,
                "status": "no_progress",
                "first_distance_m": first_distance,
                "best_distance_m": best_distance,
                "last_distance_m": last_distance,
                "last_world_state": last_world_state,
            }

        if now >= heartbeat_due:
            heartbeat_index += 1
            send_json(
                chan,
                {
                    "action": "navigation_heartbeat",
                    "request_id": f"hb_{leg_index}_{heartbeat_index}_{int(time.time() * 1000)}",
                    "navigation_session_token": token,
                },
            )
            heartbeat_due = now + 0.45
        if now >= world_state_due:
            world_state_index += 1
            send_json(
                chan,
                {
                    "action": "get_world_state",
                    "request_id": f"ws_{leg_index}_{world_state_index}_{int(time.time() * 1000)}",
                },
            )
            world_state_due = now + 0.75

        for row in recv_json_lines(chan, buffer):
            action = row.get("action")
            if row.get("type") in {"navigation_lease_expired", "navigation_pause_retry"}:
                logger.event("session_event", row=row)
                return {
                    "target_node": target_node,
                    "status": "session_event",
                    "event": row,
                    "first_distance_m": first_distance,
                    "best_distance_m": best_distance,
                    "last_distance_m": last_distance,
                }
            if action == "navigation_heartbeat" and row.get("accepted") is not True:
                logger.event("heartbeat_rejected", row=row)
                pause_navigation(chan, buffer, logger, "heartbeat_rejected")
                return {
                    "target_node": target_node,
                    "status": "heartbeat_rejected",
                    "response": row,
                    "first_distance_m": first_distance,
                    "best_distance_m": best_distance,
                    "last_distance_m": last_distance,
                }
            if action != "get_world_state" or row.get("accepted") is not True:
                logger.event("gateway_rx", row=row)
                continue

            world = row.get("world_state") or {}
            last_world_state = world
            current_pose = ((world.get("current_pose") or {}).get("pose") or {})
            distance = pose_distance(current_pose, target_pose)
            last_distance = distance
            if first_distance is None and distance is not None:
                first_distance = distance
                best_distance = distance
                last_progress = now
            if distance is not None and (best_distance is None or distance < best_distance - 0.05):
                best_distance = distance
                last_progress = now

            localization = world.get("localization") or {}
            safety = world.get("safety") or {}
            navigation = world.get("navigation") or {}
            logger.event(
                "leg_sample",
                index=leg_index,
                target_node=target_node,
                distance_m=distance,
                best_distance_m=best_distance,
                loc_status=localization.get("status"),
                pose_age_ms=localization.get("pose_age_ms"),
                safety_allow=safety.get("allow_navigation"),
                safety_reason=safety.get("reason"),
                nav_state=navigation.get("state"),
                nav_arrived=navigation.get("is_arrived"),
                nav_failure=navigation.get("failure_reason"),
            )

            if localization.get("status") != "localized":
                pause_navigation(chan, buffer, logger, "localization_not_localized")
                return {
                    "target_node": target_node,
                    "status": "localization_not_localized",
                    "last_world_state": world,
                    "last_distance_m": distance,
                }
            if safety.get("allow_navigation") is False:
                pause_navigation(chan, buffer, logger, "safety_blocked")
                return {
                    "target_node": target_node,
                    "status": "safety_blocked",
                    "safety": safety,
                    "last_distance_m": distance,
                }
            if navigation.get("failure_reason"):
                pause_navigation(chan, buffer, logger, "navigation_failure")
                return {
                    "target_node": target_node,
                    "status": "navigation_failure",
                    "navigation": navigation,
                    "last_distance_m": distance,
                }
            if navigation.get("is_arrived") is True or (distance is not None and distance <= arrival_radius_m):
                reached_count += 1
                if reached_count >= 3:
                    pause_navigation(chan, buffer, logger, "arrived")
                    return {
                        "target_node": target_node,
                        "status": "arrived",
                        "first_distance_m": first_distance,
                        "best_distance_m": best_distance,
                        "last_distance_m": distance,
                        "elapsed_s": time.monotonic() - submitted,
                    }
            else:
                reached_count = 0

        if chan.exit_status_ready():
            return {
                "target_node": target_node,
                "status": "session_exited",
                "first_distance_m": first_distance,
                "best_distance_m": best_distance,
                "last_distance_m": last_distance,
            }
        time.sleep(0.05)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="192.168.3.17")
    parser.add_argument("--user", default="unitree")
    parser.add_argument("--password-env", default="GO2W_SSH_PASSWORD")
    parser.add_argument("--speed-mps", type=float, default=0.2)
    parser.add_argument("--arrival-radius-m", type=float, default=0.35)
    parser.add_argument("--log", type=Path, default=None)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    password = os.environ.get(args.password_env)
    if not password:
        print(f"missing password env var: {args.password_env}", file=sys.stderr)
        return 2

    default_log = Path.home() / "Desktop" / f"go2w_mode1_route_test_{now_tag()}.jsonl"
    logger = JsonlLogger(args.log or default_log)
    client: paramiko.SSHClient | None = None
    sftp: paramiko.SFTPClient | None = None
    chan: paramiko.Channel | None = None
    buffer = bytearray()
    original: bytes | None = None
    results: list[dict[str, Any]] = []
    exit_code = 1
    try:
        logger.event("test_start", host=args.host, user=args.user, log=str(logger.path))
        client = paramiko.SSHClient()
        client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        client.connect(
            hostname=args.host,
            username=args.user,
            password=password,
            timeout=8,
            banner_timeout=8,
            auth_timeout=8,
        )
        sftp = client.open_sftp()
        original, backup_path, targets = load_and_patch_registry(sftp, logger, args.speed_mps)
        chan, buffer, token = start_navigation_session(client, logger)
        restore_registry(sftp, original, logger)
        original = None
        logger.event("navigation_session_active", route=ROUTE, backup_path=backup_path)
        for index, target_node in enumerate(ROUTE, start=1):
            result = test_leg(
                chan,
                buffer,
                logger,
                token,
                target_node,
                targets[target_node],
                index,
                args.arrival_radius_m,
                args.speed_mps,
            )
            results.append(result)
            logger.event("leg_result", index=index, result=result)
            if result.get("status") != "arrived":
                break
            time.sleep(1.0)
        exit_code = 0 if len(results) == len(ROUTE) and all(r.get("status") == "arrived" for r in results) else 3
        logger.event("test_result", exit_code=exit_code, results=results)
        return exit_code
    except Exception as exc:
        logger.event("test_error", error=str(exc), results=results)
        return exit_code
    finally:
        if chan is not None and not chan.closed:
            try:
                pause_navigation(chan, buffer, logger, "final_cleanup")
            except Exception as exc:
                logger.event("final_pause_error", error=str(exc))
            try:
                chan.shutdown_write()
            except Exception:
                pass
            try:
                chan.close()
            except Exception:
                pass
        if original is not None and sftp is not None:
            try:
                restore_registry(sftp, original, logger)
            except Exception as exc:
                logger.event("registry_restore_error", error=str(exc))
        if sftp is not None:
            sftp.close()
        if client is not None:
            client.close()
        logger.event("test_end")
        logger.close()


if __name__ == "__main__":
    raise SystemExit(main())
