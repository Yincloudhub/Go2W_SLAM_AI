from __future__ import annotations

import json
import math
import queue
import subprocess
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from .gateway_safety import gateway_allows_navigation


@dataclass(frozen=True)
class GatewayConfig:
    client_path: str
    network_interface: str = "eth0"
    timeout_s: int = 30
    startup_wait_s: float = 0.0


def run_gateway_command(command: dict[str, Any], config: GatewayConfig) -> dict[str, Any]:
    payload = json.dumps(command, ensure_ascii=False, separators=(",", ":")) + "\n"
    process = subprocess.Popen(
        [config.client_path, config.network_interface],
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    if config.startup_wait_s > 0:
        time.sleep(config.startup_wait_s)
    try:
        stdout, stderr = process.communicate(payload, timeout=config.timeout_s)
    except subprocess.TimeoutExpired as exc:
        process.kill()
        stdout, stderr = process.communicate()
        detail = (stderr or "").strip()
        suffix = f": {detail}" if detail else ""
        raise RuntimeError(f"gateway command timed out after {config.timeout_s}s{suffix}") from exc
    if process.returncode != 0:
        raise RuntimeError(stderr.strip() or f"gateway command failed with exit {process.returncode}")

    decoder = json.JSONDecoder()
    objects: list[dict[str, Any]] = []
    for index, char in enumerate(stdout):
        if char != "{":
            continue
        try:
            value, _ = decoder.raw_decode(stdout[index:])
        except json.JSONDecodeError:
            continue
        if isinstance(value, dict):
            objects.append(value)
    if not objects:
        raise RuntimeError(f"gateway did not return JSON: {stdout[-1000:]}")
    for value in objects:
        if {"accepted", "action", "world_state"}.issubset(value.keys()):
            return value
    for value in objects:
        if "accepted" in value:
            return value
    return objects[-1]


class PersistentGatewaySession:
    """Persistent gateway client with request correlation and a navigation lease."""

    def __init__(
        self,
        *,
        client_path: str,
        network_interface: str,
        timeout_s: int,
        startup_wait_s: float = 0.0,
        navigation_session: bool = True,
    ) -> None:
        self.timeout_s = max(1, timeout_s)
        self.navigation_session = navigation_session
        session_flag = (
            "--persistent-navigation-session"
            if navigation_session
            else "--persistent-world-state-session"
        )
        self.process = subprocess.Popen(
            [client_path, network_interface, session_flag],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            encoding="utf-8",
            errors="replace",
            bufsize=1,
        )
        self.messages: queue.Queue[dict[str, Any]] = queue.Queue()
        self.reader = threading.Thread(target=self._read_output, daemon=True)
        self.reader.start()
        self.session_token = ""
        self.lease_timeout_ms = 0
        self._request_sequence = 0
        self.active = False
        self.async_events: list[dict[str, Any]] = []
        if startup_wait_s > 0:
            time.sleep(startup_wait_s)
        try:
            ready_type = (
                "navigation_session_ready"
                if navigation_session
                else "world_state_session_ready"
            )
            ready = self._wait_for(
                lambda value: value.get("type") == ready_type,
                timeout_s=self.timeout_s,
            )
        except Exception:
            self.close()
            raise
        self.session_token = str(ready.get("session_token") or "")
        self.lease_timeout_ms = int(ready.get("lease_timeout_ms") or 0)
        if navigation_session and (not self.session_token or self.lease_timeout_ms <= 0):
            self.close()
            raise RuntimeError("gateway did not provide a valid navigation session lease")

    def _read_output(self) -> None:
        assert self.process.stdout is not None
        for line in self.process.stdout:
            text = line.strip()
            if not text:
                continue
            try:
                value = json.loads(text)
            except json.JSONDecodeError:
                continue
            if isinstance(value, dict):
                self.messages.put(value)
        self.messages.put({"type": "_session_eof", "returncode": self.process.poll()})

    def _wait_for(self, predicate: Any, *, timeout_s: float) -> dict[str, Any]:
        deadline = time.monotonic() + timeout_s
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError("persistent gateway session response timed out")
            try:
                value = self.messages.get(timeout=remaining)
            except queue.Empty as exc:
                raise TimeoutError("persistent gateway session response timed out") from exc
            if value.get("type") == "_session_eof":
                raise RuntimeError(
                    f"persistent gateway session exited unexpectedly: {value.get('returncode')}"
                )
            if value.get("type") == "navigation_session_unready":
                raise RuntimeError(
                    "persistent gateway session is unready: "
                    f"{value.get('reason') or 'unknown reason'}"
                )
            if predicate(value):
                return value
            self.async_events.append(value)

    def command(self, command: dict[str, Any]) -> dict[str, Any]:
        if self.process.poll() is not None:
            raise RuntimeError("persistent gateway session is not running")
        action = str(command.get("action") or "")
        self._request_sequence += 1
        request_id = f"session-request-{self._request_sequence}"
        payload = {**command, "request_id": request_id}
        if getattr(self, "navigation_session", True):
            payload["navigation_session_token"] = self.session_token
        assert self.process.stdin is not None
        self.process.stdin.write(
            json.dumps(payload, ensure_ascii=False, separators=(",", ":")) + "\n"
        )
        self.process.stdin.flush()
        result = self._wait_for(
            lambda value: (
                "accepted" in value
                and not value.get("type")
                and value.get("request_id") == request_id
            ),
            timeout_s=self.timeout_s,
        )
        if action in {
            "navigate_to_pose",
            "supervised_reposition",
        } and result.get("accepted") is True:
            self.active = True
        elif (
            action in {"pause_navigation", "stop_slam"}
            and result.get("accepted") is True
        ):
            self.active = False
        return result

    def heartbeat(self) -> dict[str, Any]:
        if not getattr(self, "navigation_session", True):
            raise RuntimeError("world-state session does not support navigation heartbeat")
        return self.command({"action": "navigation_heartbeat"})

    def close(self) -> None:
        if not hasattr(self, "process"):
            return
        try:
            if self.active and self.process.poll() is None:
                for _ in range(3):
                    result = self.command({"action": "pause_navigation"})
                    if result.get("accepted") is True:
                        break
                    time.sleep(0.1)
        except Exception:
            pass
        try:
            if self.process.stdin is not None and not self.process.stdin.closed:
                self.process.stdin.close()
        except OSError:
            pass
        try:
            self.process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            self.process.kill()
            self.process.wait(timeout=3)
        self.active = False

    def __enter__(self) -> "PersistentGatewaySession":
        return self

    def __exit__(self, exc_type: Any, exc: Any, traceback: Any) -> None:
        self.close()


def pose_distance(pose: dict[str, Any], target_pose: dict[str, Any]) -> float | None:
    try:
        return math.hypot(float(pose["x"]) - float(target_pose["x"]), float(pose["y"]) - float(target_pose["y"]))
    except (KeyError, TypeError, ValueError):
        return None


def yaw_error(pose: dict[str, Any], target_pose: dict[str, Any]) -> float | None:
    try:
        current_yaw = float(pose["yaw"])
        target_yaw = float(target_pose.get("yaw", 0.0))
    except (KeyError, TypeError, ValueError):
        return None
    diff = (current_yaw - target_yaw + math.pi) % (2.0 * math.pi) - math.pi
    return abs(diff)


class ChassisController:
    """Small deterministic wrapper around registry and the SLAM gateway."""

    def __init__(self, *, registry_path: str | Path, map_id: str, gateway: GatewayConfig):
        self.registry_path = Path(registry_path)
        self.map_id = map_id
        self.gateway = gateway

    def registry_map(self) -> dict[str, Any] | None:
        registry = json.loads(self.registry_path.read_text(encoding="utf-8"))
        for item in registry.get("maps", []):
            if item.get("map_id") == self.map_id:
                return item
        return None

    def map_path(self, fallback: str) -> str:
        profile = self.registry_map()
        if isinstance(profile, dict) and profile.get("pcd_path"):
            return str(profile["pcd_path"])
        return fallback

    def node(self, node_id: str) -> dict[str, Any] | None:
        profile = self.registry_map()
        if not profile:
            return None
        for node in profile.get("topology_nodes", []):
            if node.get("node_id") == node_id:
                return node
        return None

    def nodes(self) -> list[dict[str, Any]]:
        profile = self.registry_map()
        if not profile:
            return []
        nodes = profile.get("topology_nodes", [])
        return nodes if isinstance(nodes, list) else []

    def node_summary(self) -> list[dict[str, Any]]:
        out: list[dict[str, Any]] = []
        for node in self.nodes():
            pose = node.get("pose", {})
            tags = node.get("tags", [])
            out.append(
                {
                    "node_id": node.get("node_id"),
                    "name": node.get("name"),
                    "aliases": node.get("aliases", []),
                    "tags": tags if isinstance(tags, list) else [],
                    "needs_calibration": isinstance(tags, list) and "needs_calibration" in tags,
                    "x": pose.get("x") if isinstance(pose, dict) else None,
                    "y": pose.get("y") if isinstance(pose, dict) else None,
                }
            )
        return out

    def resolve_node(self, text: str) -> dict[str, Any]:
        query = text.strip().lower()
        if not query:
            return {"matched": False, "reason": "empty query", "matches": []}
        matches = []
        for node in self.nodes():
            node_id = str(node.get("node_id", ""))
            terms = [node_id, str(node.get("name", ""))]
            aliases = node.get("aliases", [])
            if isinstance(aliases, list):
                terms.extend(str(item) for item in aliases if item)
            hit_terms = []
            first_index: int | None = None
            for term in dict.fromkeys(term for term in terms if term):
                term_lower = term.lower()
                index = query.find(term_lower)
                if index < 0:
                    continue
                hit_terms.append(term)
                first_index = index if first_index is None else min(first_index, index)
            if hit_terms:
                matches.append(
                    {
                        "node_id": node_id,
                        "name": node.get("name"),
                        "matched_terms": hit_terms,
                        "first_index": first_index,
                        "needs_calibration": "needs_calibration" in node.get("tags", []) if isinstance(node.get("tags"), list) else False,
                    }
                )
        if not matches:
            return {"matched": False, "reason": "no registry node alias matched", "matches": []}
        exact = [item for item in matches if item["node_id"].lower() == query]
        ordered = sorted(matches, key=lambda item: (item["first_index"] if item.get("first_index") is not None else 10**9, -max(len(term) for term in item["matched_terms"])))
        selected = exact[0] if exact else ordered[0]
        return {
            "matched": True,
            "selected": selected,
            "ambiguous": len(matches) > 1,
            "multi_target": len(matches) > 1,
            "matches": ordered,
        }

    def world_state(self) -> dict[str, Any]:
        return run_gateway_command({"action": "get_world_state"}, self.gateway)

    def pause(self) -> dict[str, Any]:
        return run_gateway_command({"action": "pause_navigation"}, self.gateway)

    def relocate_to_anchor(self, anchor_id: str, *, map_path_fallback: str) -> dict[str, Any]:
        profile = self.registry_map()
        if not profile:
            return {"accepted": False, "action": "relocate", "reason": f"map_id {self.map_id!r} not found"}
        anchors = profile.get("relocalization_anchors", [])
        anchor = next(
            (
                item
                for item in anchors
                if isinstance(item, dict) and item.get("anchor_id") == anchor_id
            ),
            None,
        )
        if not anchor:
            return {
                "accepted": False,
                "action": "relocate",
                "reason": f"active relocalization anchor {anchor_id!r} not found",
            }
        status = str(anchor.get("status") or "").strip().lower()
        if status != "verified" and not status.startswith("verified_"):
            return {
                "accepted": False,
                "action": "relocate",
                "reason": f"relocalization anchor {anchor_id!r} is not verified: {status or 'missing'}",
            }
        pose = anchor.get("pose")
        if not isinstance(pose, dict):
            return {"accepted": False, "action": "relocate", "reason": f"anchor_id {anchor_id!r} has no pose"}
        return run_gateway_command(
            {
                "action": "relocate",
                "operator_ack": True,
                "map_id": self.map_id,
                "map_path": self.map_path(map_path_fallback),
                "anchor_id": anchor_id,
                "initial_pose": pose,
            },
            self.gateway,
        )

    def preflight(self) -> dict[str, Any]:
        state = self.world_state()
        allowed, reason = gateway_allows_navigation(state)
        return {"allowed": allowed, "reason": reason, "result": state}


def compact_world_state(world_state_result: dict[str, Any]) -> dict[str, Any]:
    world = world_state_result.get("world_state", {})
    if not isinstance(world, dict):
        return {}
    pose = world.get("current_pose", {}).get("pose", {})
    nav = world.get("navigation", {})
    loc = world.get("localization", {})
    health = world.get("slam_health", {})
    safety = world.get("safety", {})
    return {
        "pose": {
            "x": pose.get("x"),
            "y": pose.get("y"),
            "yaw": pose.get("yaw"),
        }
        if isinstance(pose, dict)
        else None,
        "localization": loc.get("status") if isinstance(loc, dict) else None,
        "slam_health": health.get("status") if isinstance(health, dict) else None,
        "safety": safety.get("reason") if isinstance(safety, dict) else None,
        "navigation": nav.get("state") if isinstance(nav, dict) else None,
        "target_node": nav.get("target_node") if isinstance(nav, dict) else None,
    }
