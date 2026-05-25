from __future__ import annotations

import json
import math
import subprocess
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any


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
    stdout, stderr = process.communicate(payload, timeout=config.timeout_s)
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


def gateway_allows_navigation(world_state_result: dict[str, Any]) -> tuple[bool, str]:
    world_state = world_state_result.get("world_state", {})
    if not isinstance(world_state, dict):
        return False, "missing world_state"
    safety = world_state.get("safety", {})
    if isinstance(safety, dict) and safety.get("allow_navigation") is not True:
        return False, f"safety disallows navigation: {safety.get('reason', 'unknown')}"
    slam_health = world_state.get("slam_health", {})
    if isinstance(slam_health, dict) and slam_health.get("status") not in (None, "ok"):
        return False, f"slam health is {slam_health.get('status')}"
    localization = world_state.get("localization", {})
    if isinstance(localization, dict) and localization.get("status") not in (None, "localized_or_tracking", "tracking", "localized"):
        return False, f"localization is {localization.get('status')}"
    return True, "gateway allows navigation"


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

    def relocate_to_node(self, node_id: str, *, map_path_fallback: str) -> dict[str, Any]:
        node = self.node(node_id)
        if not node:
            return {"accepted": False, "action": "relocate", "reason": f"node_id {node_id!r} not found"}
        pose = node.get("pose")
        if not isinstance(pose, dict):
            return {"accepted": False, "action": "relocate", "reason": f"node_id {node_id!r} has no pose"}
        return run_gateway_command(
            {"action": "relocate", "map_path": self.map_path(map_path_fallback), "init_pose": pose},
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
