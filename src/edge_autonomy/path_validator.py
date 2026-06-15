"""
Path pre-validator and coarse PCD map builder for LLM context.
Adds two capabilities:
1. check_path(from_xy, to_xy) -> bool: sample PCD density along line
2. build_coarse_map(grid_size=30) -> 2D occupancy grid for LLM
"""

import struct, math, json
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

PCD_PATH = "/home/unitree/test.pcd"
REGISTRY_PATH = "/home/unitree/Go2W_SLAM_AI/configs/maps/go2w_real_site_map_registry.json"

# Cache PCD points in memory (49K points, ~600KB)
_pcd_points: Optional[List[Tuple[float, float]]] = None


def _load_pcd() -> List[Tuple[float, float]]:
    global _pcd_points
    if _pcd_points is not None:
        return _pcd_points

    with open(PCD_PATH, "rb") as f:
        for _ in range(11):
            f.readline()
        data = f.read()

    pts = []
    for i in range(0, len(data) - 12, 12):
        x, y, z = struct.unpack_from("fff", data, i)
        if math.isfinite(x) and math.isfinite(y):
            pts.append((x, y))
    _pcd_points = pts
    return pts


def _pcd_density_at(x: float, y: float, radius_m: float = 0.2) -> int:
    """Count PCD points within radius_m of (x,y)."""
    pts = _load_pcd()
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
    num_samples: int = 20,
    density_threshold: int = 15,
    radius_m: float = 0.25,
    start_offset_m: float = 0.5,
) -> Dict[str, Any]:
    """
    Check if a straight-line path is clear of obstacles.
    Skips first start_offset_m to allow robot to exit its immediate area.
    """
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
            "note": "path shorter than start offset, trivially passable",
        }

    for i in range(num_samples + 1):
        t = start_offset_m / total_dist + i / num_samples * (1 - start_offset_m / total_dist)
        x = from_x + t * (to_x - from_x)
        y = from_y + t * (to_y - from_y)
        density = _pcd_density_at(x, y, radius_m)

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
        "distance_m": round(
            math.sqrt((to_x - from_x) ** 2 + (to_y - from_y) ** 2), 3
        ),
    }


def build_adaptive_coarse_map(*, grid_size: int = 40, margin_m: float = 3.0) -> Dict[str, Any]:
    """Build coarse map with range auto-fitted to topology nodes + margin."""
    with open(REGISTRY_PATH) as f:
        reg = json.load(f)
    
    xs, ys = [], []
    for m in reg["maps"]:
        if m["map_id"] == "go2w_real_site":
            for n in m["topology_nodes"]:
                p = n["pose"]
                xs.extend([p["x"], p["x"]])
                ys.extend([p["y"], p["y"]])
    
    if not xs:
        return build_coarse_map(grid_size=grid_size)
    
    x_min = min(xs) - margin_m
    x_max = max(xs) + margin_m
    y_min = min(ys) - margin_m
    y_max = max(ys) + margin_m
    
    # Ensure integer bounds for clean grid alignment
    x_min = math.floor(x_min)
    x_max = math.ceil(x_max)
    y_min = math.floor(y_min)
    y_max = math.ceil(y_max)
    
    return build_coarse_map(
        grid_size=grid_size,
        x_range=(x_min, x_max),
        y_range=(y_min, y_max),
    )


def build_coarse_map(
    *,
    grid_size: int = 30,
    x_range: Tuple[float, float] = (-5.0, 5.0),
    y_range: Tuple[float, float] = (-7.0, 2.0),
) -> Dict[str, Any]:
    """
    Build a coarse occupancy grid suitable for LLM context.
    Returns a 2D list of 0/1 values + metadata.
    """
    pts = _load_pcd()
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

    # Binarize: cell is occupied if > threshold points
    threshold = 3
    occupied = [
        [1 if grid[gy][gx] >= threshold else 0 for gx in range(grid_size)]
        for gy in range(grid_size)
    ]

    # Load topology nodes for overlay
    nodes = {}
    with open(REGISTRY_PATH) as f:
        reg = json.load(f)
    for m in reg["maps"]:
        if m["map_id"] == "go2w_real_site":
            for n in m["topology_nodes"]:
                p = n["pose"]
                nodes[n["node_id"]] = {
                    "x": p["x"],
                    "y": p["y"],
                    "name": n["name"],
                    "grid_x": int((p["x"] - x_min) / dx),
                    "grid_y": int((y_max - p["y"]) / dy),
                }

    return {
        "grid_size": grid_size,
        "x_range": list(x_range),
        "y_range": list(y_range),
        "cell_size_m": round(dx, 3),
        "grid": occupied,
        "nodes": nodes,
        "grid_string": _render_grid(occupied, nodes, grid_size),
    }


def _render_grid(
    occupied: List[List[int]], nodes: Dict, grid_size: int
) -> str:
    """Render grid as ASCII for LLM prompt."""
    chars = {0: "·", 1: "█"}
    result = []
    for gy in range(grid_size):
        row = []
        for gx in range(grid_size):
            row.append(chars.get(occupied[gy][gx], "?"))

        # Overlay node labels
        for nid, nd in nodes.items():
            if (
                nd["grid_y"] == gy
                and 0 <= nd["grid_x"] < grid_size - 4
            ):
                short = nid.replace("_", "")[:6]
                for j, ch in enumerate(short):
                    tx = nd["grid_x"] + j
                    if tx < grid_size:
                        row[tx] = ch

        result.append("".join(row))

    # Legend
    result.append("")
    result.append("█ = wall/obstacle  · = open space")
    for nid, nd in sorted(nodes.items()):
        result.append(
            f"  {nd['name']} ({nid}): grid({nd['grid_x']},{nd['grid_y']})"
        )
    return "\n".join(result)


def check_path_between_nodes(
    from_node: str, to_node: str
) -> Dict[str, Any]:
    """Convenience: check path between two registered topology nodes."""
    with open(REGISTRY_PATH) as f:
        reg = json.load(f)

    from_pose = to_pose = None
    for m in reg["maps"]:
        if m["map_id"] == "go2w_real_site":
            for n in m["topology_nodes"]:
                if n["node_id"] == from_node:
                    from_pose = n["pose"]
                if n["node_id"] == to_node:
                    to_pose = n["pose"]

    if not from_pose or not to_pose:
        return {"error": "node not found"}

    return check_path(
        from_pose["x"], from_pose["y"], to_pose["x"], to_pose["y"]
    )


# ── CLI for testing ──
if __name__ == "__main__":
    import sys

    if len(sys.argv) >= 5 and sys.argv[1] == "check":
        fx, fy, tx, ty = map(float, sys.argv[2:6])
        result = check_path(fx, fy, tx, ty)
        print(json.dumps(result, indent=2))

    elif len(sys.argv) >= 3 and sys.argv[1] == "between":
        result = check_path_between_nodes(sys.argv[2], sys.argv[3])
        print(json.dumps(result, indent=2))

    elif len(sys.argv) >= 2 and sys.argv[1] == "map":
        size = int(sys.argv[2]) if len(sys.argv) > 2 else 30
        result = build_coarse_map(grid_size=size)
        print(result["grid_string"])
        # Also print JSON without the string (too large)
        result.pop("grid_string")
        result.pop("grid")
        print(json.dumps(result, indent=2))

    else:
        # Default: check critical paths
        print("=== Path checks ===\n")
        for from_n, to_n in [
            ("yin_siyuan_station", "zhao_bo_office_front"),
            ("yin_siyuan_station", "nie_guoli_office_front"),
            ("nie_guoli_office_front", "zhao_bo_office_front"),
            ("yin_siyuan_station", "chen_jiayu_station"),
            ("initial_point", "yin_siyuan_station"),
        ]:
            r = check_path_between_nodes(from_n, to_n)
            status = "✅" if r["passable"] else "❌"
            print(
                f"  {status} {from_n} -> {to_n}: "
                f"dist={r['distance_m']}m, "
                f"max_density={r['max_density']}"
                + (f", blocked_at={r['blocked_at']}" if not r["passable"] else "")
            )

        print("\n=== Coarse Map (30x30) ===\n")
        print(build_coarse_map(grid_size=30)["grid_string"])
