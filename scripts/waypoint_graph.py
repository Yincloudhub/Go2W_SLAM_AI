"""Waypoint graph utilities for registry-first routing.

The measured registry is the source of truth for live navigation edges. PCD
line-of-sight checks are kept as a fallback for maps that do not yet declare
topology_edges.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import struct
import sys
from collections import deque
from typing import Any, Dict, Iterable, List, Optional, Tuple


REPO = os.environ.get("GO2W_REPO", os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
REGISTRY_PATH = os.path.join(REPO, "configs", "maps", "go2w_real_site_map_registry.json")
GRAPH_PATH = os.path.join(REPO, "artifacts", "waypoint_graph.json")

SAMPLE_STEP_M = 0.1
CHECK_RADIUS_M = 0.25
DENSITY_WALL_THRESHOLD = 80


def load_registry(registry_path: str = REGISTRY_PATH) -> Dict[str, Any]:
    with open(registry_path, "r", encoding="utf-8") as fh:
        return json.load(fh)


def get_map_profile(registry: Dict[str, Any], map_id: str) -> Dict[str, Any]:
    for item in registry.get("maps", []):
        if item.get("map_id") == map_id:
            return item
    raise KeyError(f"map_id not found in registry: {map_id}")


def _node_to_waypoint(node: Dict[str, Any]) -> Dict[str, Any]:
    pose = node.get("pose", {})
    return {
        "name": node.get("name", node["node_id"]),
        "x": float(pose.get("x", 0.0)),
        "y": float(pose.get("y", 0.0)),
        "yaw": float(pose.get("yaw", 0.0)),
        "type": node.get("node_type", "unknown"),
        "point_category": node.get("point_category", ""),
    }


def load_waypoints(
    map_id: str = "go2w_real_site",
    registry_path: str = REGISTRY_PATH,
) -> Tuple[Dict[str, Dict[str, Any]], Dict[str, Any]]:
    registry = load_registry(registry_path)
    map_profile = get_map_profile(registry, map_id)
    waypoints = {
        str(node["node_id"]): _node_to_waypoint(node)
        for node in map_profile.get("topology_nodes", [])
    }
    return waypoints, map_profile


def _distance_between(waypoints: Dict[str, Dict[str, Any]], a: str, b: str) -> float:
    pa = waypoints[a]
    pb = waypoints[b]
    return math.hypot(pa["x"] - pb["x"], pa["y"] - pb["y"])


def load_declared_edges(
    map_profile: Dict[str, Any],
    waypoints: Dict[str, Dict[str, Any]],
) -> List[Dict[str, Any]]:
    edges: List[Dict[str, Any]] = []
    known = set(waypoints)
    for item in map_profile.get("topology_edges", []):
        a = str(item.get("from", ""))
        b = str(item.get("to", ""))
        if not a or not b or a not in known or b not in known:
            continue
        dist = item.get("expected_distance_m")
        if dist is None:
            dist = _distance_between(waypoints, a, b)
        edges.append(
            {
                "a": a,
                "b": b,
                "dist_m": round(float(dist), 3),
                "bidirectional": bool(item.get("bidirectional", True)),
                "source": "registry",
                "distance_verified": bool(item.get("distance_verified", False)),
            }
        )
    return edges


def load_pcd_points(pcd_path: str) -> List[Tuple[float, float]]:
    with open(pcd_path, "rb") as fh:
        header = b""
        while True:
            line = fh.readline()
            if not line:
                break
            header += line
            if line.strip().startswith(b"DATA"):
                break

        points_count = 0
        for line in header.splitlines():
            if line.startswith(b"POINTS"):
                parts = line.split()
                if len(parts) >= 2:
                    points_count = int(parts[1])
                break
        raw = fh.read()

    points: List[Tuple[float, float]] = []
    for idx in range(points_count):
        offset = idx * 12
        if offset + 12 > len(raw):
            break
        x, y, _z = struct.unpack_from("fff", raw, offset)
        if math.isfinite(x) and math.isfinite(y):
            points.append((x, y))
    return points


def check_path_clear(
    pcd_points: Iterable[Tuple[float, float]],
    x1: float,
    y1: float,
    x2: float,
    y2: float,
) -> Tuple[bool, int]:
    points = list(pcd_points)
    dx = x2 - x1
    dy = y2 - y1
    dist = math.hypot(dx, dy)
    if dist < 0.1:
        return True, 0

    steps = max(1, int(dist / SAMPLE_STEP_M))
    r2 = CHECK_RADIUS_M * CHECK_RADIUS_M
    max_density = 0
    for idx in range(steps + 1):
        t = idx / steps
        sx = x1 + dx * t
        sy = y1 + dy * t
        density = 0
        for px, py in points:
            if (px - sx) ** 2 + (py - sy) ** 2 <= r2:
                density += 1
                if density >= DENSITY_WALL_THRESHOLD:
                    break
        max_density = max(max_density, density)
        if density >= DENSITY_WALL_THRESHOLD:
            return False, max_density
    return True, max_density


def build_pcd_edges(
    waypoints: Dict[str, Dict[str, Any]],
    pcd_path: str,
) -> Tuple[List[Dict[str, Any]], List[Dict[str, Any]]]:
    if not pcd_path or not os.path.exists(pcd_path):
        return [], [
            {
                "error": f"PCD not available: {pcd_path}",
                "source": "pcd_fallback",
            }
        ]

    pcd_points = load_pcd_points(pcd_path)
    node_ids = list(waypoints)
    edges: List[Dict[str, Any]] = []
    blocked: List[Dict[str, Any]] = []
    for i, a in enumerate(node_ids):
        for b in node_ids[i + 1 :]:
            pa = waypoints[a]
            pb = waypoints[b]
            dist = _distance_between(waypoints, a, b)
            clear, density = check_path_clear(pcd_points, pa["x"], pa["y"], pb["x"], pb["y"])
            record = {
                "a": a,
                "b": b,
                "dist_m": round(dist, 3),
                "max_density": density,
                "source": "pcd_fallback",
                "bidirectional": True,
            }
            if clear:
                edges.append(record)
            else:
                blocked.append(record)
    return edges, blocked


def build_graph(
    map_id: str = "go2w_real_site",
    registry_path: str = REGISTRY_PATH,
    graph_path: str = GRAPH_PATH,
    *,
    force_pcd: bool = False,
) -> Dict[str, Any]:
    waypoints, map_profile = load_waypoints(map_id=map_id, registry_path=registry_path)
    pcd_path = str(map_profile.get("pcd_path", ""))

    edges = [] if force_pcd else load_declared_edges(map_profile, waypoints)
    blocked: List[Dict[str, Any]] = []
    edge_source = "registry" if edges else "pcd_fallback"
    if not edges:
        edges, blocked = build_pcd_edges(waypoints, pcd_path)

    graph = {
        "map_id": map_id,
        "registry_path": registry_path,
        "pcd_path": pcd_path,
        "edge_source": edge_source,
        "waypoints": waypoints,
        "edges": edges,
        "blocked": blocked,
    }

    os.makedirs(os.path.dirname(graph_path), exist_ok=True)
    with open(graph_path, "w", encoding="utf-8") as fh:
        json.dump(graph, fh, ensure_ascii=False, indent=2)
        fh.write("\n")
    return graph


def load_graph(graph_path: str = GRAPH_PATH) -> Optional[Dict[str, Any]]:
    if os.path.exists(graph_path):
        with open(graph_path, "r", encoding="utf-8") as fh:
            return json.load(fh)
    return None


def build_adjacency(graph: Dict[str, Any]) -> Dict[str, List[str]]:
    adjacency: Dict[str, List[str]] = {nid: [] for nid in graph.get("waypoints", {})}
    for edge in graph.get("edges", []):
        a = str(edge["a"])
        b = str(edge["b"])
        adjacency.setdefault(a, []).append(b)
        if edge.get("bidirectional", True):
            adjacency.setdefault(b, []).append(a)
    return adjacency


def bfs_route(graph: Dict[str, Any], from_id: str, to_id: str) -> Optional[List[str]]:
    if from_id == to_id:
        return [from_id]
    if from_id not in graph.get("waypoints", {}) or to_id not in graph.get("waypoints", {}):
        return None

    adjacency = build_adjacency(graph)
    queue = deque([[from_id]])
    visited = {from_id}
    while queue:
        path = queue.popleft()
        last = path[-1]
        for nxt in adjacency.get(last, []):
            if nxt == to_id:
                return path + [nxt]
            if nxt not in visited:
                visited.add(nxt)
                queue.append(path + [nxt])
    return None


def nearest_waypoint(graph: Dict[str, Any], x: float, y: float) -> Tuple[Optional[str], float]:
    best_id: Optional[str] = None
    best_dist = float("inf")
    for node_id, info in graph.get("waypoints", {}).items():
        dist = math.hypot(float(info["x"]) - x, float(info["y"]) - y)
        if dist < best_dist:
            best_id = node_id
            best_dist = dist
    return best_id, best_dist


def route_distance(graph: Dict[str, Any], route: List[str]) -> float:
    total = 0.0
    waypoints = graph.get("waypoints", {})
    for idx in range(1, len(route)):
        total += _distance_between(waypoints, route[idx - 1], route[idx])
    return total


def list_graph(graph: Dict[str, Any]) -> None:
    print(f"Map: {graph.get('map_id')} ({graph.get('edge_source')})")
    print(f"Waypoints: {len(graph.get('waypoints', {}))}")
    for node_id, info in graph.get("waypoints", {}).items():
        print(f"  {node_id}: ({info['x']:.2f}, {info['y']:.2f}) {info['name']}")

    print(f"Edges: {len(graph.get('edges', []))}")
    for edge in graph.get("edges", []):
        arrow = "<->" if edge.get("bidirectional", True) else "->"
        print(f"  {edge['a']} {arrow} {edge['b']} ({edge['dist_m']}m, {edge.get('source', '')})")

    blocked = graph.get("blocked", [])
    if blocked:
        print(f"Blocked/unknown: {len(blocked)}")
        for edge in blocked:
            if "error" in edge:
                print(f"  {edge['error']}")
            else:
                print(f"  {edge['a']} x {edge['b']} ({edge['dist_m']}m)")


def print_route(graph: Dict[str, Any], route: List[str]) -> None:
    print(f"Route ({len(route) - 1} hops, {route_distance(graph, route):.2f}m):")
    for idx, node_id in enumerate(route):
        info = graph["waypoints"][node_id]
        marker = "start" if idx == 0 else "hop"
        print(f"  {marker}: {node_id} ({info['name']}) [{info['x']:.2f}, {info['y']:.2f}]")


def _load_or_build(map_id: str, registry_path: str, graph_path: str, force_pcd: bool) -> Dict[str, Any]:
    graph = load_graph(graph_path)
    if not graph or graph.get("map_id") != map_id or graph.get("registry_path") != registry_path or force_pcd:
        graph = build_graph(map_id=map_id, registry_path=registry_path, graph_path=graph_path, force_pcd=force_pcd)
    return graph


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="Build and inspect GO2W waypoint graph.")
    parser.add_argument("--map-id", default="go2w_real_site")
    parser.add_argument("--registry", default=REGISTRY_PATH)
    parser.add_argument("--graph", default=GRAPH_PATH)
    parser.add_argument("--force-pcd", action="store_true", help="Ignore declared topology_edges.")
    sub = parser.add_subparsers(dest="command")
    sub.add_parser("build")
    sub.add_parser("list")
    route_parser = sub.add_parser("route")
    route_parser.add_argument("from_node")
    route_parser.add_argument("to_node")

    args = parser.parse_args(argv)
    command = args.command or "list"

    if command == "build":
        graph = build_graph(args.map_id, args.registry, args.graph, force_pcd=args.force_pcd)
        print(f"Graph saved: {args.graph}")
        print(f"Waypoints: {len(graph['waypoints'])}, edges: {len(graph['edges'])}, source: {graph['edge_source']}")
        return 0

    graph = _load_or_build(args.map_id, args.registry, args.graph, args.force_pcd)
    if command == "list":
        list_graph(graph)
        return 0

    if command == "route":
        route = bfs_route(graph, args.from_node, args.to_node)
        if not route:
            print(f"No route from {args.from_node} to {args.to_node}")
            return 2
        print_route(graph, route)
        return 0

    parser.print_help()
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
