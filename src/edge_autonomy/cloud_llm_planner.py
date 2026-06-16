"""Cloud LLM planner — builds context for remote LLMs and converts hop plans to task queues.

Architecture:
  1. build_cloud_planner_context()  — generates compact context for cloud LLM
  2. build_waypoint_connectivity()  — pre-computes reachable waypoint pairs
  3. hops_to_plan()                 — converts cloud LLM hop output to plan schema
  4. validate_cloud_hops()          — validates hop sequence against connectivity

The cloud LLM only outputs {"hops": [...], "reason": "..."}.
All schema conversion happens on the robot side here.
"""

from __future__ import annotations

import json
import time
from typing import Any, Dict, List, Optional

from .llm_context import build_planner_context
from .local_llm_planner import (
    _COARSE_MAP,
    _weak_communication_policy,
    _normal_communication_policy,
    PLAN_REQUIRED_KEYS,
    build_lightweight_planner_context,
    validate_local_llm_plan,
    validate_execution_contract,
)

# ---------------------------------------------------------------------------
# Waypoint connectivity
# ---------------------------------------------------------------------------

_WAYPOINT_CONNECTIVITY_CACHE: Optional[Dict[str, Any]] = None


def build_waypoint_connectivity(
    registry_path: str,
    map_id: str = "go2w_real_site",
    *,
    pcd_path: Optional[str] = None,
    force_refresh: bool = False,
) -> Dict[str, Any]:
    """Pre-compute which topology node pairs are directly reachable.

    Returns:
        {
            "nodes": ["wp_a", "wp_b", ...],
            "adjacency": {"wp_a": ["wp_b", "wp_c"], "wp_b": ["wp_a"], ...},
            "grid_positions": {"wp_a": [gx, gy], ...},   # coarse_map grid coords
            "computed_at_ms": 1234567890,
        }
    """
    global _WAYPOINT_CONNECTIVITY_CACHE
    if _WAYPOINT_CONNECTIVITY_CACHE is not None and not force_refresh:
        return _WAYPOINT_CONNECTIVITY_CACHE

    # Load registry to get node list
    with open(registry_path, "r", encoding="utf-8") as fh:
        reg = json.load(fh)

    nodes: List[Dict[str, Any]] = []
    map_pcd: Optional[str] = pcd_path
    for m in reg.get("maps", []):
        if m.get("map_id") == map_id:
            if map_pcd is None:
                map_pcd = m.get("pcd_path")
            nodes = m.get("topology_nodes", [])
            break

    if not nodes:
        result: Dict[str, Any] = {"nodes": [], "adjacency": {}, "grid_positions": {}, "computed_at_ms": int(time.time() * 1000)}
        _WAYPOINT_CONNECTIVITY_CACHE = result
        return result

    node_ids = [str(n["node_id"]) for n in nodes]

    # Coarse map node positions (matching the grid overlay in path_validator)
    grid_positions: Dict[str, List[int]] = {}
    if _COARSE_MAP and isinstance(_COARSE_MAP, dict):
        cm_nodes = _COARSE_MAP.get("nodes", {})
        if isinstance(cm_nodes, dict):
            for nid, pos in cm_nodes.items():
                if isinstance(pos, (list, tuple)) and len(pos) >= 2:
                    grid_positions[str(nid)] = [int(pos[0]), int(pos[1])]

    # Build adjacency by checking each pair
    adjacency: Dict[str, List[str]] = {nid: [] for nid in node_ids}
    try:
        from .path_validator import check_path_between_nodes

        for i, a in enumerate(node_ids):
            for b in node_ids[i + 1 :]:
                try:
                    r = check_path_between_nodes(a, b, registry_path=registry_path)
                    if r.get("passable"):
                        adjacency[a].append(b)
                        adjacency[b].append(a)
                except Exception:
                    # Pair not passable or error — skip edge
                    pass
    except ImportError:
        # path_validator not available (e.g., dry-run without PCD)
        pass

    result = {
        "nodes": node_ids,
        "adjacency": adjacency,
        "grid_positions": grid_positions,
        "computed_at_ms": int(time.time() * 1000),
    }
    _WAYPOINT_CONNECTIVITY_CACHE = result
    return result


# ---------------------------------------------------------------------------
# Cloud planner context
# ---------------------------------------------------------------------------


def build_cloud_planner_context(
    snapshot: Dict[str, Any],
    registry: Any,  # MapRegistry
    *,
    user_command: str,
    map_id: Optional[str] = None,
    perception_context: Optional[Dict[str, Any]] = None,
    registry_path: str = "configs/maps/go2w_real_site_map_registry.json",
) -> Dict[str, Any]:
    """Build a compact context for the cloud LLM.

    Extends the lightweight planner context with:
      - waypoint_connectivity (adjacency graph)
      - compact coarse_map with node overlay
    """
    # Base context (reuse existing builder)
    full_context = build_planner_context(
        snapshot,
        registry,
        user_command=user_command,
        map_id=map_id,
        perception_context=perception_context,
    )
    light_context = build_lightweight_planner_context(full_context)

    # Waypoint connectivity
    actual_map_id = map_id or full_context.get("world_state_summary", {}).get("map", {}).get("map_id", "go2w_real_site")
    pcd_path = full_context.get("world_state_summary", {}).get("map", {}).get("pcd_path")
    connectivity = build_waypoint_connectivity(registry_path, map_id=actual_map_id, pcd_path=pcd_path)

    # Compact coarse_map
    coarse_map = ""
    coarse_map_nodes: Dict[str, Any] = {}
    coarse_map_grid_size = 0
    if _COARSE_MAP and isinstance(_COARSE_MAP, dict):
        coarse_map = _COARSE_MAP.get("grid_string", "")
        coarse_map_nodes = _COARSE_MAP.get("nodes", {})
        coarse_map_grid_size = _COARSE_MAP.get("grid_size", 0)

    # Current pose
    current_pose = full_context.get("world_state_summary", {}).get("robot", {}).get("pose")
    current_node = light_context.get("nearest_node")

    # Target candidates
    candidates = light_context.get("candidates", [])
    matched_targets = light_context.get("matched_targets", [])
    requested_target = light_context.get("requested_target_guess")

    return {
        "schema_version": 1,
        "user_command": user_command,
        "map_id": actual_map_id,
        "current_pose": current_pose,
        "current_node": current_node,
        "requested_target": requested_target,
        "candidates": candidates,
        "matched_targets": matched_targets,
        "coarse_map": coarse_map,
        "coarse_map_nodes": coarse_map_nodes,
        "coarse_map_grid_size": coarse_map_grid_size,
        "waypoint_connectivity": {
            "nodes": connectivity.get("nodes", []),
            "adjacency": connectivity.get("adjacency", {}),
            "grid_positions": connectivity.get("grid_positions", {}),
        },
        "navigation_hint": (
            "Multi-hop routing: examine the coarse_map. "
            "Walls are shown as ██, open space as ··. "
            "Topology nodes are overlaid on the grid. "
            "Use waypoint_connectivity.adjacency to find reachable neighbor pairs. "
            "Plan a sequence of hops from current_node to requested_target through "
            "intermediate nodes. Each hop must be a directly connected pair in the adjacency graph. "
            "Output format: {\"hops\": [\"wp_a\", \"wp_b\", \"wp_c\"], \"reason\": \"short explanation\"}"
        ),
    }


# ---------------------------------------------------------------------------
# Hop → Plan conversion
# ---------------------------------------------------------------------------


def validate_cloud_hops(
    hops: List[str],
    connectivity: Dict[str, Any],
    known_nodes: List[str],
) -> Optional[str]:
    """Validate a hop sequence. Returns None if valid, or an error string."""
    if not hops or len(hops) < 1:
        return "hops list is empty"
    if len(hops) < 2:
        return "need at least 2 hops (from and to)"

    adjacency = connectivity.get("adjacency", {})
    if not isinstance(adjacency, dict):
        adjacency = {}

    for hop in hops:
        if hop not in known_nodes:
            return f"hop '{hop}' is not a registered topology node"

    for i in range(len(hops) - 1):
        a, b = hops[i], hops[i + 1]
        neighbors = adjacency.get(a, [])
        if not isinstance(neighbors, list):
            neighbors = []
        if b not in neighbors:
            return f"hops '{a}' → '{b}' are not directly connected in waypoint connectivity graph"

    return None


def hops_to_plan(
    hops: List[str],
    map_id: str,
    *,
    reason: str = "",
    connectivity: Optional[Dict[str, Any]] = None,
) -> Dict[str, Any]:
    """Convert a hop sequence to a full plan schema.

    Args:
        hops: Ordered list of waypoint node IDs, e.g. ["current", "wp_a", "target"]
        map_id: Map identifier
        reason: Human-readable explanation for the plan
        connectivity: Optional connectivity graph for validation

    Returns:
        A plan dict matching the PLAN_REQUIRED_KEYS schema,
        suitable for plan_to_task_queue().
    """
    # Validate if connectivity provided
    if connectivity:
        known_nodes = connectivity.get("nodes", [])
        err = validate_cloud_hops(hops, connectivity, known_nodes)
        if err:
            return {
                "plan_id": f"cloud_rejected_{int(time.time() * 1000)}",
                "mode": "human_confirm",
                "confidence": 0.0,
                "reason": f"Cloud hop plan rejected: {err}",
                "steps": [
                    {
                        "step_id": "ask_1",
                        "tool": "request_human_confirm",
                        "arguments": {"reason": f"Cloud plan invalid: {err}", "hops": hops},
                    }
                ],
                "communication_policy": _normal_communication_policy(),
                "requires_human_ack": True,
            }

    # Build steps: for each adjacent pair, create navigate + wait
    plan_steps: List[Dict[str, Any]] = []
    nav_count = 0
    for i in range(len(hops) - 1):
        target_node = hops[i + 1]
        nav_count += 1
        plan_steps.append(
            {
                "step_id": f"nav_{nav_count}",
                "tool": "create_navigation_subgoal",
                "arguments": {"map_id": map_id, "target_node": target_node},
            }
        )
        plan_steps.append(
            {
                "step_id": f"wait_{nav_count}",
                "tool": "wait_until",
                "arguments": {"condition": "arrived"},
            }
        )

    if not plan_steps:
        plan_steps = [
            {
                "step_id": "hold_1",
                "tool": "hold_position",
                "arguments": {"reason": "no valid hops to execute"},
            }
        ]

    # Cap at 6 steps (safety limit)
    plan_steps = plan_steps[:6]

    plan_reason = reason or f"Cloud multi-hop: {' → '.join(hops)}"

    return {
        "plan_id": f"cloud_plan_{int(time.time() * 1000)}",
        "mode": "mapped_navigation" if nav_count > 0 else "safe_hold",
        "confidence": 0.9,
        "reason": plan_reason,
        "steps": plan_steps,
        "communication_policy": _normal_communication_policy(),
        "requires_human_ack": False,
    }
