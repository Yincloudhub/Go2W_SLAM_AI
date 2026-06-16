#!/usr/bin/env python3
"""select_waypoint.py — Multi-PCD waypoint selector with BFS path planning.

Reads go2w_multi_map_registry_v2.json, builds a cross-PCD connectivity
graph, and computes the shortest navigation path between any two
topology nodes, including PCD switch instructions at transition anchors.

Usage:
  python3 scripts/select_waypoint.py <from_node> <to_node>
  python3 scripts/select_waypoint.py --list                       # list all nodes
  python3 scripts/select_waypoint.py --graph                      # print graph
  python3 scripts/select_waypoint.py --map <map_id> --list         # nodes in map
  python3 scripts/select_waypoint.py --help

Output:
  A step-by-step navigation plan: navigate → switch → navigate ...
"""

from __future__ import annotations

import json
import sys
from collections import deque
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

REGISTRY_PATH = Path(__file__).resolve().parents[1] / "configs" / "maps" / "go2w_multi_map_registry_v2.json"


def load_registry() -> Dict[str, Any]:
    with open(REGISTRY_PATH) as f:
        return json.load(f)


def find_node(reg: Dict[str, Any], node_query: str) -> Tuple[Optional[str], Optional[Dict[str, Any]]]:
    """Find a topology node by node_id, name, or alias across all maps.
    Returns (map_id, node_dict) or (None, None).
    """
    q = node_query.strip().casefold()
    for m in reg["maps"]:
        for n in m["topology_nodes"]:
            ids = [n["node_id"].casefold(), n.get("name", "").casefold()]
            ids.extend(a.casefold() for a in n.get("aliases", []))
            if q in ids:
                return m["map_id"], n
    return None, None


def build_map_graph(reg: Dict[str, Any]) -> Dict[str, List[Tuple[str, str, str]]]:
    """Build adjacency list: map_id → [(connects_to, transition_anchor, reverse_anchor), ...].

    Each entry means: from this map, you can go to `connects_to` by relocating
    at `transition_anchor` (in current map's transition_anchors), landing at
    `reverse_anchor` (in target map's relocalization_anchors).
    """
    maps = {m["map_id"]: m for m in reg["maps"]}
    graph: Dict[str, List[Tuple[str, str, str]]] = {mid: [] for mid in maps}

    for mid, m in maps.items():
        for ta in m.get("transition_anchors", []):
            target_map = ta.get("connects_to", "")
            if target_map and target_map in maps:
                # Use explicit reverse_anchor field if present
                reverse = ta.get("reverse_anchor", "")
                if not reverse:
                    # Fallback: try to derive from anchor_id naming convention
                    parts = ta["anchor_id"].split("_to_")
                    if len(parts) == 2:
                        reverse = parts[1] + "_to_" + parts[0]
                    else:
                        reverse = ta["anchor_id"]

                # Verify reverse anchor exists in target map's relocalization_anchors
                target = maps[target_map]
                found = False
                for ra in target.get("relocalization_anchors", []):
                    if ra["anchor_id"] == reverse:
                        found = True
                        break
                if found:
                    graph[mid].append((target_map, ta["anchor_id"], reverse))
    return graph


def bfs_path(
    graph: Dict[str, List[Tuple[str, str, str]]],
    start_map: str,
    end_map: str,
) -> Optional[List[Tuple[str, str, str, str]]]:
    """BFS from start_map to end_map.
    Returns list of (from_map, to_map, transition_anchor, reverse_anchor) steps,
    or None if unreachable.
    """
    if start_map == end_map:
        return []

    visited = {start_map}
    q: deque[List[Tuple[str, str, str, str]]] = deque()
    q.append([])

    current_map = start_map
    # Level-order BFS
    frontier: List[Tuple[str, List[Tuple[str, str, str, str]]]] = [(start_map, [])]

    while frontier:
        next_frontier = []
        for cur, path in frontier:
            for next_map, ta, reverse_ta in graph.get(cur, []):
                if next_map == end_map:
                    return path + [(cur, next_map, ta, reverse_ta)]
                if next_map not in visited:
                    visited.add(next_map)
                    next_frontier.append((next_map, path + [(cur, next_map, ta, reverse_ta)]))
        frontier = next_frontier
    return None


def build_plan(
    reg: Dict[str, Any],
    from_node: str,
    to_node: str,
) -> str:
    """Build a human-readable navigation plan."""
    from_map, from_n = find_node(reg, from_node)
    to_map, to_n = find_node(reg, to_node)

    if not from_n:
        return f"ERROR: from_node '{from_node}' not found in any map"
    if not to_n:
        return f"ERROR: to_node '{to_node}' not found in any map"

    assert from_map and to_map  # for type checker

    lines: List[str] = []
    lines.append(f"Plan: {from_n['name']} ({from_map}) → {to_n['name']} ({to_map})")
    lines.append("=" * 60)

    if from_map == to_map:
        # Same map — direct navigation
        lines.append("")
        lines.append(f"  Same PCD ({from_map}) — direct navigation.")
        lines.append(f"  1. Navigate: {from_n['node_id']} → {to_n['node_id']}")
        fx, fy = from_n["pose"]["x"], from_n["pose"]["y"]
        tx, ty = to_n["pose"]["x"], to_n["pose"]["y"]
        if fx != 0.0 or fy != 0.0 or tx != 0.0 or ty != 0.0:
            dist = ((tx - fx) ** 2 + (ty - fy) ** 2) ** 0.5
            lines.append(f"     Distance: ~{dist:.1f}m")
        lines.append(f"     Command:")
        lines.append(f"       python3 scripts/run_robot_closed_loop.py \\")
        target_pcd = _get_map(reg, from_map)["pcd_path"]
        lines.append(f"         --command {to_n['node_id']} \\")
        lines.append(f"         --registry {REGISTRY_PATH} \\")
        lines.append(f"         --map-id {from_map} \\")
        lines.append(f"         --map-path {target_pcd} \\")
        lines.append(f"         --execute --nav-speed-mps 0.2 --nav-mode 0 \\")
        lines.append(f"         --robot-password 123 --summary")
        return "\n".join(lines)

    # Cross-map path
    graph = build_map_graph(reg)
    path = bfs_path(graph, from_map, to_map)

    if path is None:
        lines.append("")
        lines.append(f"  ERROR: No transition path from {from_map} to {to_map}")
        lines.append(f"  Available maps: {list(graph.keys())}")
        for mid, edges in graph.items():
            for target, ta, _ in edges:
                lines.append(f"    {mid} → {target}  via {ta}")
        return "\n".join(lines)

    lines.append(f"  Cross-PCD path ({len(path)} switch{'es' if len(path) > 1 else ''}):")
    lines.append("")

    current_map = from_map
    current_node = from_n

    step = 1
    for i, (frm, to, ta, reverse_ta) in enumerate(path):
        # Step A: Navigate from current node to transition anchor in current map
        ta_node = None
        for tn in _get_map(reg, frm).get("topology_nodes", []):
            # Transition anchor itself might not be a topology node
            pass

        # Find transition anchor pose in current map
        ta_pose = None
        for t in _get_map(reg, frm).get("transition_anchors", []):
            if t["anchor_id"] == ta:
                ta_pose = t["pose"]
                break
        # Fallback: look in relocalization_anchors
        if not ta_pose:
            for ra in _get_map(reg, frm).get("relocalization_anchors", []):
                if ra["anchor_id"] == ta:
                    ta_pose = ra["pose"]
                    break

        lines.append(f"  Step {step}: Navigate to transition spot in {frm}")
        lines.append(f"    From: {current_node['node_id']}")
        lines.append(f"    To:   {ta} (transition to {to})")
        if ta_pose:
            fx2, fy2 = current_node["pose"]["x"], current_node["pose"]["y"]
            tx2, ty2 = ta_pose["x"], ta_pose["y"]
            if fx2 != 0.0 or fy2 != 0.0 or tx2 != 0.0 or ty2 != 0.0:
                dist = ((tx2 - fx2) ** 2 + (ty2 - fy2) ** 2) ** 0.5
                lines.append(f"    Distance: ~{dist:.1f}m")
        lines.append(f"    Command: navigate to transition location within {frm}")
        step += 1

        # Step B: Switch PCD
        lines.append(f"")
        lines.append(f"  Step {step}: SWITCH PCD: {frm} → {to}")
        lines.append(f"    Transition anchor: {reverse_ta} (in {to})")
        lines.append(f"    Command:")
        lines.append(f"      bash scripts/switch_pcd.sh {frm} {to}")
        step += 1

        current_map = to
        # After switch, current "position" is at the transition anchor in the new map
        current_node = {"node_id": reverse_ta, "pose": _get_anchor_pose(reg, to, reverse_ta) or {"x": 0, "y": 0}}

    # Final step: Navigate to destination
    lines.append(f"")
    lines.append(f"  Step {step}: Navigate to destination in {to_map}")
    lines.append(f"    From: {current_node['node_id']} (after PCD switch)")
    lines.append(f"    To:   {to_n['node_id']}")
    tx3, ty3 = to_n["pose"]["x"], to_n["pose"]["y"]
    cx, cy = current_node["pose"]["x"], current_node["pose"]["y"]
    if cx != 0.0 or cy != 0.0 or tx3 != 0.0 or ty3 != 0.0:
        dist = ((tx3 - cx) ** 2 + (ty3 - cy) ** 2) ** 0.5
        lines.append(f"    Distance: ~{dist:.1f}m")
    lines.append(f"    Command:")
    target_pcd = _get_map(reg, to_map)["pcd_path"]
    lines.append(f"      python3 scripts/run_robot_closed_loop.py \\")
    lines.append(f"        --command {to_n['node_id']} \\")
    lines.append(f"        --registry {REGISTRY_PATH} \\")
    lines.append(f"        --map-id {to_map} \\")
    lines.append(f"        --map-path {target_pcd} \\")
    lines.append(f"        --execute --nav-speed-mps 0.2 --nav-mode 0 \\")
    lines.append(f"        --robot-password 123 --summary")

    lines.append("")
    lines.append("=" * 60)
    lines.append("NOTE: All coordinates are currently placeholders (0,0).")
    lines.append("Build PCDs and snapshot nodes before navigating.")

    return "\n".join(lines)


def _get_map(reg: Dict[str, Any], map_id: str) -> Dict[str, Any]:
    for m in reg["maps"]:
        if m["map_id"] == map_id:
            return m
    return {}


def _get_anchor_pose(reg: Dict[str, Any], map_id: str, anchor_id: str) -> Optional[Dict[str, Any]]:
    m = _get_map(reg, map_id)
    for ra in m.get("relocalization_anchors", []):
        if ra["anchor_id"] == anchor_id:
            return ra["pose"]
    for ta in m.get("transition_anchors", []):
        if ta["anchor_id"] == anchor_id:
            return ta["pose"]
    return None


def list_all_nodes(reg: Dict[str, Any], map_filter: Optional[str] = None) -> str:
    lines = []
    for m in reg["maps"]:
        if map_filter and m["map_id"] != map_filter:
            continue
        lines.append(f"\n{m['map_id']} — {m['name']}  (PCD: {m['pcd_path']})")
        lines.append("-" * 60)
        for n in m["topology_nodes"]:
            p = n["pose"]
            has_coords = p["x"] != 0.0 or p["y"] != 0.0
            status = "✓" if has_coords else "○"
            lines.append(f"  {status} {n['node_id']:<30s} {n.get('name', ''):<20s} ({p['x']:.2f}, {p['y']:.2f})")
        lines.append(f"  Transition anchors:")
        for ta in m.get("transition_anchors", []):
            lines.append(f"    → {ta['anchor_id']}  → {ta.get('connects_to', '?')}")
    return "\n".join(lines)


def print_graph(reg: Dict[str, Any]) -> str:
    graph = build_map_graph(reg)
    lines = ["Cross-PCD Connectivity Graph", "=" * 40]
    for mid, edges in graph.items():
        if edges:
            for target, ta, rev in edges:
                lines.append(f"  {mid}  ──[{ta}]──►  {target}  (land at {rev})")
        else:
            lines.append(f"  {mid}  (no outgoing transitions)")
    return "\n".join(lines)


def main():
    args = sys.argv[1:]

    if not args or "--help" in args or "-h" in args:
        print(__doc__)
        return

    reg = load_registry()

    if "--list" in args:
        map_filter = None
        for i, a in enumerate(args):
            if a == "--map" and i + 1 < len(args):
                map_filter = args[i + 1]
        print(list_all_nodes(reg, map_filter))
        return

    if "--graph" in args:
        print(print_graph(reg))
        return

    if len(args) >= 2:
        from_node = args[0]
        to_node = args[1]
        print(build_plan(reg, from_node, to_node))
        return

    print("Usage: select_waypoint.py <from_node> <to_node>")
    print("       select_waypoint.py --list [--map <map_id>]")
    print("       select_waypoint.py --graph")
    sys.exit(2)


if __name__ == "__main__":
    main()
