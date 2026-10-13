"""Waypoint-to-waypoint navigation on top of nav_core.NavigationSession.

Dry-run mode only validates graph routing and never starts Gateway or
sport_bridge. Live mode opens one persistent NavigationSession and sends each
route hop to the closed-loop move_to_xy primitive.
"""

from __future__ import annotations

import argparse
import math
import os
import sys
from typing import Any, Dict, List, Optional, Tuple


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

import waypoint_graph


DEFAULT_SPEED_MPS = 0.20
DEFAULT_ARRIVAL_RADIUS_M = 0.30
DEFAULT_START_TOLERANCE_M = 3.00
SEGMENT_TIMEOUT_MIN_S = 20.0
SEGMENT_TIMEOUT_MARGIN_S = 12.0


def load_or_build_graph(args: argparse.Namespace) -> Dict[str, Any]:
    graph = None if args.rebuild_graph else waypoint_graph.load_graph(args.graph)
    if (
        not graph
        or graph.get("map_id") != args.map_id
        or graph.get("registry_path") != args.registry
    ):
        graph = waypoint_graph.build_graph(
            map_id=args.map_id,
            registry_path=args.registry,
            graph_path=args.graph,
        )
    return graph


def node_distance_from_pose(graph: Dict[str, Any], node_id: str, pose: Tuple[float, float, float]) -> float:
    node = graph["waypoints"][node_id]
    return math.hypot(float(node["x"]) - pose[0], float(node["y"]) - pose[1])


def resolve_dry_run_start(args: argparse.Namespace, graph: Dict[str, Any]) -> Optional[str]:
    if args.from_node:
        if args.from_node not in graph["waypoints"]:
            print(f"Unknown start node: {args.from_node}")
            return None
        return args.from_node
    if args.start_x is not None and args.start_y is not None:
        node_id, dist = waypoint_graph.nearest_waypoint(graph, args.start_x, args.start_y)
        if node_id:
            print(f"Dry-run nearest start: {node_id} ({dist:.2f}m)")
        return node_id
    print("Dry-run requires --from-node or both --start-x/--start-y.")
    return None


def resolve_live_start(
    args: argparse.Namespace,
    graph: Dict[str, Any],
    pose: Tuple[float, float, float],
) -> Optional[str]:
    nearest_id, nearest_dist = waypoint_graph.nearest_waypoint(graph, pose[0], pose[1])
    if not nearest_id:
        print("No waypoint is available in the graph.")
        return None

    if not args.from_node:
        print(f"Nearest live start: {nearest_id} ({nearest_dist:.2f}m)")
        return nearest_id

    if args.from_node not in graph["waypoints"]:
        print(f"Unknown start node: {args.from_node}")
        return None
    override_dist = node_distance_from_pose(graph, args.from_node, pose)
    if override_dist > args.start_tolerance:
        print(
            f"Start override rejected: current pose is {override_dist:.2f}m from "
            f"{args.from_node}, tolerance is {args.start_tolerance:.2f}m."
        )
        print(f"Falling back to nearest live start: {nearest_id} ({nearest_dist:.2f}m)")
        return nearest_id
    print(f"Using live start override: {args.from_node} ({override_dist:.2f}m)")
    return args.from_node


def print_route(graph: Dict[str, Any], route: List[str]) -> None:
    print(
        f"Route on {graph.get('map_id')} "
        f"({graph.get('edge_source')}, {len(route) - 1} hops, "
        f"{waypoint_graph.route_distance(graph, route):.2f}m):"
    )
    for idx, node_id in enumerate(route):
        node = graph["waypoints"][node_id]
        label = "start" if idx == 0 else f"hop {idx}"
        print(f"  {label}: {node_id} ({node['name']}) [{node['x']:.2f}, {node['y']:.2f}]")


def plan_route(args: argparse.Namespace, graph: Dict[str, Any], start_node: str) -> Optional[List[str]]:
    if args.target_node not in graph["waypoints"]:
        print(f"Unknown target node: {args.target_node}")
        print("Available:", ", ".join(sorted(graph["waypoints"])))
        return None
    route = waypoint_graph.bfs_route(graph, start_node, args.target_node)
    if not route:
        print(f"No route from {start_node} to {args.target_node}")
        return None
    return route


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="Route to a GO2W waypoint.")
    parser.add_argument("target_node")
    parser.add_argument("legacy_speed", nargs="?", type=float, help="Backward-compatible speed positional.")
    parser.add_argument("--map-id", default="go2w_real_site")
    parser.add_argument("--registry", default=waypoint_graph.REGISTRY_PATH)
    parser.add_argument("--graph", default=waypoint_graph.GRAPH_PATH)
    parser.add_argument("--rebuild-graph", action="store_true")
    parser.add_argument("--dry-run", action="store_true", help="Validate route only; do not start robot processes.")
    parser.add_argument("--from-node", help="Start node for dry-run, or a live override within tolerance.")
    parser.add_argument("--start-x", type=float)
    parser.add_argument("--start-y", type=float)
    parser.add_argument("--speed", type=float)
    parser.add_argument("--arrival-radius", type=float, default=DEFAULT_ARRIVAL_RADIUS_M)
    parser.add_argument("--start-tolerance", type=float, default=DEFAULT_START_TOLERANCE_M)
    parser.add_argument("--segment-timeout-min", type=float, default=SEGMENT_TIMEOUT_MIN_S)
    parser.add_argument("--segment-timeout-margin", type=float, default=SEGMENT_TIMEOUT_MARGIN_S)
    args = parser.parse_args(argv)

    graph = load_or_build_graph(args)
    print(f"Graph: {len(graph['waypoints'])} waypoints, {len(graph['edges'])} edges, source={graph.get('edge_source')}")

    if args.dry_run:
        start_node = resolve_dry_run_start(args, graph)
        if not start_node:
            return 2
        route = plan_route(args, graph, start_node)
        if not route:
            return 2
        print_route(graph, route)
        print("Dry-run complete: no Gateway or sport_bridge process was started.")
        return 0

    from nav_core import NavigationSession

    # Start the session before route planning in live mode so the route seed is
    # based on the current SLAM pose, not on a stale command-line assumption.
    speed = args.speed if args.speed is not None else args.legacy_speed or DEFAULT_SPEED_MPS
    print(f"Starting live NavigationSession, speed cap {speed:.2f} m/s")
    with NavigationSession() as nav:
        pose = nav.pose
        print(f"Current pose: x={pose[0]:.2f} y={pose[1]:.2f} yaw={math.degrees(pose[2]):.1f}deg")
        start_node = resolve_live_start(args, graph, pose)
        if not start_node:
            return 2
        route = plan_route(args, graph, start_node)
        if not route:
            return 2
        print_route(graph, route)

        for index, node_id in enumerate(route[1:], start=1):
            target = graph["waypoints"][node_id]
            previous = graph["waypoints"][route[index - 1]]
            segment_dist = math.hypot(float(target["x"]) - float(previous["x"]), float(target["y"]) - float(previous["y"]))
            max_time = max(
                args.segment_timeout_min,
                segment_dist / max(speed, 0.05) + args.segment_timeout_margin,
            )
            print(
                f"Hop {index}/{len(route) - 1}: {node_id} ({target['name']}) "
                f"dist={segment_dist:.2f}m timeout={max_time:.1f}s"
            )
            result = nav.move_to_xy(
                float(target["x"]),
                float(target["y"]),
                goal_tol=args.arrival_radius,
                max_speed=speed,
                max_time=max_time,
            )
            print(f"  result={result}")
            if result != "ARRIVED":
                return 2
        return 0


if __name__ == "__main__":
    raise SystemExit(main())
