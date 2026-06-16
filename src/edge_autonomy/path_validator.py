"""
Path pre-validator and coarse PCD map builder for LLM context.
Supports multi-PCD maps via go2w_multi_map_registry_v2.json.

Capabilities:
1. check_path(from_xy, to_xy, pcd_path) -> bool: sample PCD density along line
2. check_path_between_nodes(from_node, to_node) -> dict: multi-PCD aware
3. build_coarse_map(grid_size, pcd_path) -> 2D occupancy grid
4. build_multi_pcd_reachability_graph() -> full cross-map reachability matrix
"""

from __future__ import annotations

import json
import math
import struct
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

# ── Multi-PCD configuration ──
DEFAULT_REGISTRY = str(
    Path(__file__).resolve().parents[2]
    / "configs"
    / "maps"
    / "go2w_multi_map_registry_v2.json"
)

# Cache: pcd_path -> list of (x,y) points
_pcd_cache: Dict[str, List[Tuple[float, float]]] = {}


def _load_pcd(pcd_path: str) -> List[Tuple[float, float]]:
    """Load PCD points, cached per path."""
    if pcd_path in _pcd_cache:
        return _pcd_cache[pcd_path]

    pts: List[Tuple[float, float]] = []
    try:
        with open(pcd_path, "rb") as f:
            for _ in range(11):
                f.readline()
            data = f.read()
    except FileNotFoundError:
        print(f"Warning: PCD not found: {pcd_path}", file=sys.stderr)
        _pcd_cache[pcd_path] = pts
        return pts

    for i in range(0, len(data) - 12, 12):
        x, y, z = struct.unpack_from("fff", data, i)
        if math.isfinite(x) and math.isfinite(y):
            pts.append((x, y))
    _pcd_cache[pcd_path] = pts
    return pts


def _pcd_density_at(
    x: float, y: float, pcd_path: str, radius_m: float = 0.2
) -> int:
    """Count PCD points within radius_m of (x,y) for a given PCD."""
    pts = _load_pcd(pcd_path)
    if not pts:
        return 0
    r2 = radius_m * radius_m
    count = 0
    for px, py in pts:
        if (px - x) ** 2 + (py - y) ** 2 <= r2:
            count += 1
    return count


def check_path(
    from_x: float,
    from_y: float,
    to_x: float,
    to_y: float,
    *,
    pcd_path: str = "/home/unitree/test.pcd",
    num_samples: int = 20,
    density_threshold: int = 15,
    radius_m: float = 0.25,
    start_offset_m: float = 0.5,
) -> Dict[str, Any]:
    """
    Check if a straight-line path is clear of obstacles.

    Args:
        pcd_path: Path to the PCD file to sample from.
    """
    pts = _load_pcd(pcd_path)
    if not pts:
        return {
            "passable": None,
            "error": f"PCD not available: {pcd_path}",
            "max_density": 0,
            "density_threshold": density_threshold,
            "samples": 0,
            "blocked_at": None,
            "from": (round(from_x, 3), round(from_y, 3)),
            "to": (round(to_x, 3), round(to_y, 3)),
            "distance_m": round(
                math.sqrt((to_x - from_x) ** 2 + (to_y - from_y) ** 2), 3
            ),
        }

    max_density = 0
    blocked_at = None

    total_dist = math.sqrt((to_x - from_x) ** 2 + (to_y - from_y) ** 2)
    if total_dist <= start_offset_m:
        return {
            "passable": True,
            "max_density": 0,
            "density_threshold": density_threshold,
            "samples": 0,
            "blocked_at": None,
            "from": (round(from_x, 3), round(from_y, 3)),
            "to": (round(to_x, 3), round(to_y, 3)),
            "distance_m": round(total_dist, 3),
            "pcd_path": pcd_path,
            "note": "path shorter than start offset, trivially passable",
        }

    for i in range(num_samples + 1):
        t = start_offset_m / total_dist + i / num_samples * (
            1 - start_offset_m / total_dist
        )
        x = from_x + t * (to_x - from_x)
        y = from_y + t * (to_y - from_y)
        density = _pcd_density_at(x, y, pcd_path, radius_m)

        if density > max_density:
            max_density = density

        if density >= density_threshold:
            blocked_at = (round(x, 3), round(y, 3))
            break

    return {
        "passable": blocked_at is None,
        "max_density": max_density,
        "density_threshold": density_threshold,
        "samples": num_samples,
        "blocked_at": blocked_at,
        "from": (round(from_x, 3), round(from_y, 3)),
        "to": (round(to_x, 3), round(to_y, 3)),
        "distance_m": round(total_dist, 3),
        "pcd_path": pcd_path,
    }


def load_multi_registry(
    registry_path: str = DEFAULT_REGISTRY,
) -> Dict[str, Any]:
    """Load multi-PCD registry from JSON file."""
    with open(registry_path) as f:
        return json.load(f)


def _get_node_map_info(
    reg: Dict[str, Any], node_query: str
) -> Optional[Tuple[str, str, Dict[str, Any]]]:
    """Find a topology node across all maps.
    Returns (map_id, pcd_path, node_dict) or None.
    """
    q = node_query.strip().casefold()
    for m in reg.get("maps", []):
        for n in m.get("topology_nodes", []):
            ids = [n["node_id"].casefold(), n.get("name", "").casefold()]
            ids.extend(a.casefold() for a in n.get("aliases", []))
            if q in ids:
                return m["map_id"], m["pcd_path"], n
    return None


def check_path_between_nodes(
    from_node: str,
    to_node: str,
    *,
    registry_path: str = DEFAULT_REGISTRY,
    density_threshold: int = 15,
    num_samples: int = 20,
) -> Dict[str, Any]:
    """
    Multi-PCD aware path check between two topology nodes.

    Args:
        from_node, to_node: Node IDs or aliases.
        registry_path: Path to multi-PCD registry JSON.
    """
    reg = load_multi_registry(registry_path)

    from_info = _get_node_map_info(reg, from_node)
    to_info = _get_node_map_info(reg, to_node)

    if not from_info:
        return {"error": f"from_node '{from_node}' not found", "nodes_checked": []}
    if not to_info:
        return {"error": f"to_node '{to_node}' not found", "nodes_checked": []}

    from_map, from_pcd, from_n = from_info
    to_map, to_pcd, to_n = to_info

    if from_map != to_map:
        return {
            "passable": None,
            "error": "nodes are in different PCD maps",
            "from_map": from_map,
            "to_map": to_map,
            "from_node": from_n["node_id"],
            "to_node": to_n["node_id"],
            "note": "Use select_waypoint.py to plan cross-PCD path, then check each segment.",
        }

    from_pose = from_n["pose"]
    to_pose = to_n["pose"]

    return check_path(
        from_pose["x"],
        from_pose["y"],
        to_pose["x"],
        to_pose["y"],
        pcd_path=to_pcd,
        density_threshold=density_threshold,
        num_samples=num_samples,
    )


def build_multi_pcd_reachability_graph(
    registry_path: str = DEFAULT_REGISTRY,
    *,
    density_threshold: int = 15,
) -> Dict[str, Any]:
    """
    Build a full reachability graph across all PCD maps.

    For each map:
      - Checks all node pairs within the map
      - Classifies edges as: passable, blocked, unknown (no PCD)
    Cross-map transitions are NOT checked (they represent same physical spot).

    Returns:
        {
            "maps": {map_id: {"pcd_path": ..., "nodes": [...], "edges": [...]}},
            "cross_map_transitions": [...],
            "summary": {...}
        }
    """
    reg = load_multi_registry(registry_path)
    result: Dict[str, Any] = {"maps": {}, "cross_map_transitions": [], "summary": {}}

    total_nodes = 0
    passable_edges = 0
    blocked_edges = 0
    unknown_edges = 0

    for m in reg.get("maps", []):
        map_id = m["map_id"]
        pcd_path = m["pcd_path"]
        nodes = m.get("topology_nodes", [])
        total_nodes += len(nodes)
        
        map_nodes = []
        for n in nodes:
            p = n["pose"]
            map_nodes.append(
                {
                    "node_id": n["node_id"],
                    "name": n.get("name", ""),
                    "x": p["x"],
                    "y": p["y"],
                }
            )

        edges = []
        # Check all node pairs within the map
        for i, n1 in enumerate(nodes):
            for j, n2 in enumerate(nodes):
                if j <= i:
                    continue
                p1 = n1["pose"]
                p2 = n2["pose"]
                r = check_path(
                    p1["x"],
                    p1["y"],
                    p2["x"],
                    p2["y"],
                    pcd_path=pcd_path,
                    density_threshold=density_threshold,
                )
                edge = {
                    "from": n1["node_id"],
                    "to": n2["node_id"],
                    "passable": r["passable"],
                    "distance_m": r.get("distance_m", 0),
                    "max_density": r.get("max_density", 0),
                }
                if r["passable"] is True:
                    passable_edges += 1
                elif r["passable"] is False:
                    blocked_edges += 1
                    edge["blocked_at"] = r.get("blocked_at")
                else:
                    unknown_edges += 1
                    edge["error"] = r.get("error", "unknown")
                edges.append(edge)

        result["maps"][map_id] = {
            "pcd_path": pcd_path,
            "status": m.get("status", "candidate"),
            "nodes": map_nodes,
            "edges": edges,
        }

    # Collect cross-map transitions
    for m in reg.get("maps", []):
        for ta in m.get("transition_anchors", []):
            result["cross_map_transitions"].append(
                {
                    "from_map": m["map_id"],
                    "to_map": ta.get("connects_to", ""),
                    "anchor_id": ta["anchor_id"],
                    "reverse_anchor": ta.get("reverse_anchor", ""),
                }
            )

    result["summary"] = {
        "total_maps": len(reg.get("maps", [])),
        "total_nodes": total_nodes,
        "total_edges": passable_edges + blocked_edges + unknown_edges,
        "passable": passable_edges,
        "blocked": blocked_edges,
        "unknown": unknown_edges,
        "density_threshold": density_threshold,
    }

    return result


def build_coarse_map(
    *,
    pcd_path: str = "/home/unitree/test.pcd",
    grid_size: int = 30,
    x_range: Tuple[float, float] = (-5.0, 5.0),
    y_range: Tuple[float, float] = (-7.0, 2.0),
    registry_path: str = DEFAULT_REGISTRY,
    map_id: Optional[str] = None,
) -> Dict[str, Any]:
    """
    Build a coarse occupancy grid suitable for LLM context.

    Args:
        pcd_path: Path to the PCD file.
        registry_path: Path to registry for topology node overlay.
        map_id: If set, only show nodes from this map.
    """
    pts = _load_pcd(pcd_path)
    x_min, x_max = x_range
    y_min, y_max = y_range
    dx = (x_max - x_min) / grid_size
    dy = (y_max - y_min) / grid_size

    # Count points per cell
    grid = [[0 for _ in range(grid_size)] for _ in range(grid_size)]
    for px, py in pts:
        if x_min <= px <= x_max and y_min <= py <= y_max:
            gx = int((px - x_min) / dx)
            gy = int((y_max - py) / dy)
            if 0 <= gx < grid_size and 0 <= gy < grid_size:
                grid[gy][gx] += 1

    # Binarize: threshold scales with cell area
    cell_area = dx * dy
    base_area = 0.11
    threshold = max(2, int(3 * base_area / cell_area))
    occupied = [
        [1 if grid[gy][gx] >= threshold else 0 for gx in range(grid_size)]
        for gy in range(grid_size)
    ]

    # Load topology nodes
    nodes: Dict[str, Any] = {}
    try:
        reg = load_multi_registry(registry_path)
        for m in reg.get("maps", []):
            if map_id and m["map_id"] != map_id:
                continue
            for n in m.get("topology_nodes", []):
                p = n["pose"]
                gx = int((p["x"] - x_min) / dx)
                gy = int((y_max - p["y"]) / dy)
                if 0 <= gx < grid_size and 0 <= gy < grid_size:
                    nodes[n["node_id"]] = {
                        "x": p["x"],
                        "y": p["y"],
                        "name": n.get("name", ""),
                        "map_id": m["map_id"],
                        "grid_x": gx,
                        "grid_y": gy,
                    }
    except (OSError, json.JSONDecodeError, KeyError):
        pass

    return {
        "grid_size": grid_size,
        "x_range": list(x_range),
        "y_range": list(y_range),
        "cell_size_m": round(dx, 3),
        "grid": occupied,
        "nodes": nodes,
        "pcd_path": pcd_path,
        "grid_string": _render_grid(occupied, nodes, grid_size),
    }


def build_adaptive_coarse_map(
    *,
    pcd_path: str = "/home/unitree/test.pcd",
    grid_size: int = 40,
    margin_m: float = 3.0,
    registry_path: str = DEFAULT_REGISTRY,
    map_id: Optional[str] = None,
) -> Dict[str, Any]:
    """Build coarse map with range auto-fitted to topology nodes + margin."""
    reg = load_multi_registry(registry_path)
    xs, ys = [], []

    for m in reg.get("maps", []):
        if map_id and m["map_id"] != map_id:
            continue
        for n in m.get("topology_nodes", []):
            p = n["pose"]
            xs.append(p["x"])
            ys.append(p["y"])

    if not xs:
        return build_coarse_map(
            pcd_path=pcd_path, grid_size=grid_size, registry_path=registry_path
        )

    x_min = math.floor(min(xs) - margin_m)
    x_max = math.ceil(max(xs) + margin_m)
    y_min = math.floor(min(ys) - margin_m)
    y_max = math.ceil(max(ys) + margin_m)

    return build_coarse_map(
        pcd_path=pcd_path,
        grid_size=grid_size,
        x_range=(x_min, x_max),
        y_range=(y_min, y_max),
        registry_path=registry_path,
        map_id=map_id,
    )


def _render_grid(
    occupied: List[List[int]], nodes: Dict[str, Any], grid_size: int
) -> str:
    """Render grid as ASCII for LLM prompt."""
    chars = {0: "·", 1: "█"}
    result = []
    for gy in range(grid_size):
        row = []
        for gx in range(grid_size):
            row.append(chars.get(occupied[gy][gx], "?"))

        for nid, nd in nodes.items():
            if nd["grid_y"] == gy and 0 <= nd["grid_x"] < grid_size - 4:
                short = nid.replace("_", "")[:6]
                for j, ch in enumerate(short):
                    tx = nd["grid_x"] + j
                    if tx < grid_size:
                        row[tx] = ch

        result.append("".join(row))

    result.append("")
    result.append("█ = wall/obstacle  · = open space")
    for nid, nd in sorted(nodes.items()):
        result.append(
            f"  [{nd.get('map_id', '?')}] {nd['name']} ({nid}): grid({nd['grid_x']},{nd['grid_y']})"
        )
    return "\n".join(result)


# ── CLI ──
if __name__ == "__main__":
    if len(sys.argv) >= 2 and sys.argv[1] == "reachability":
        # Build full reachability graph
        threshold = int(sys.argv[2]) if len(sys.argv) > 2 else 15
        result = build_multi_pcd_reachability_graph(
            density_threshold=threshold
        )
        print(json.dumps(result["summary"], indent=2))
        for map_id, mdata in result["maps"].items():
            print(f"\n--- {map_id} ---")
            for e in mdata["edges"]:
                status = (
                    "✅" if e["passable"] is True
                    else "❌" if e["passable"] is False
                    else "❓"
                )
                extra = ""
                if not e["passable"] and e.get("blocked_at"):
                    extra = f" blocked_at={e['blocked_at']}"
                elif "error" in e:
                    extra = f" {e['error']}"
                print(
                    f"  {status} {e['from']} → {e['to']}: "
                    f"dist={e['distance_m']}m, max_density={e['max_density']}{extra}"
                )

    elif len(sys.argv) >= 5 and sys.argv[1] == "check":
        fx, fy, tx, ty = map(float, sys.argv[2:6])
        pcd = sys.argv[6] if len(sys.argv) > 6 else "/home/unitree/test.pcd"
        result = check_path(fx, fy, tx, ty, pcd_path=pcd)
        print(json.dumps(result, indent=2))

    elif len(sys.argv) >= 3 and sys.argv[1] == "between":
        from_node = sys.argv[2]
        # to_node is argv[3], unless argv[3] is a digit (threshold)
        # and argv[4] exists, in which case argv[4] is to_node
        if len(sys.argv) >= 5 and sys.argv[3].lstrip('-').isdigit():
            to_node = sys.argv[4]
            threshold = int(sys.argv[3])
        elif len(sys.argv) >= 4:
            to_node = sys.argv[3]
            threshold = int(sys.argv[4]) if len(sys.argv) > 4 and sys.argv[4].isdigit() else 15
        else:
            print("Usage: path_validator.py between <from_node> <to_node> [threshold]")
            sys.exit(2)
        result = check_path_between_nodes(
            from_node, to_node,
            density_threshold=threshold,
        )
        print(json.dumps(result, indent=2))

    elif len(sys.argv) >= 2 and sys.argv[1] == "map":
        size = int(sys.argv[2]) if len(sys.argv) > 2 else 30
        pcd = sys.argv[3] if len(sys.argv) > 3 else "/home/unitree/test.pcd"
        map_id = sys.argv[4] if len(sys.argv) > 4 else None
        result = build_coarse_map(
            pcd_path=pcd, grid_size=size, map_id=map_id,
        )
        print(result["grid_string"])
        result.pop("grid_string")
        result.pop("grid")
        print(json.dumps(result, indent=2))

    else:
        # Default: build reachability graph
        print("=== Multi-PCD Reachability Graph ===\n")
        result = build_multi_pcd_reachability_graph()
        s = result["summary"]
        print(
            f"Maps: {s['total_maps']} | Nodes: {s['total_nodes']} | "
            f"Edges: {s['total_edges']} "
            f"(✅{s['passable']} ❌{s['blocked']} ❓{s['unknown']})"
        )
        for map_id, mdata in result["maps"].items():
            print(f"\n--- {map_id} ({mdata['status']}) ---")
            for e in mdata["edges"]:
                status = (
                    "✅" if e["passable"] is True
                    else "❌" if e["passable"] is False
                    else "❓"
                )
                print(
                    f"  {status} {e['from']} → {e['to']}: "
                    f"dist={e['distance_m']:.1f}m, max_density={e['max_density']}"
                    + (f", blocked_at={e['blocked_at']}" if e.get("blocked_at") else "")
                )
