"""Simple waypoint graph with PCD path validation.

Builds a connectivity graph from all registered waypoints by checking
straight-line paths on the PCD, then provides BFS routing.

Usage:
  python3 waypoint_graph.py build     # Build/reload graph
  python3 waypoint_graph.py list      # List nodes and edges
  python3 waypoint_graph.py route <from_node> <to_node>  # Find path
"""
import json
import struct
import math
import sys
import os
from collections import deque

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))  # up from scripts/
REGISTRY_PATH = os.path.join(REPO, 'configs', 'maps', 'go2w_real_site_map_registry.json')
GRAPH_PATH = os.path.join(REPO, 'artifacts', 'waypoint_graph.json')

# PCD path check parameters
SAMPLE_STEP_M = 0.1          # sample every 10cm
CHECK_RADIUS_M = 0.25        # check in 25cm radius
DENSITY_WALL_THRESHOLD = 80  # points in 25cm radius → wall (PCD ~100k pts, ~25 avg)


def load_registry():
    with open(REGISTRY_PATH) as f:
        return json.load(f)


def load_waypoints():
    """Extract all waypoints from go2w_real_site topology_nodes."""
    reg = load_registry()
    waypoints = {}
    for m in reg.get('maps', []):
        if m['map_id'] == 'go2w_real_site':
            for n in m.get('topology_nodes', []):
                nid = n['node_id']
                pose = n.get('pose', {})
                waypoints[nid] = {
                    'name': n.get('name', nid),
                    'x': pose.get('x', 0),
                    'y': pose.get('y', 0),
                    'yaw': pose.get('yaw', 0),
                    'type': n.get('node_type', 'unknown'),
                }
    return waypoints


def load_pcd_points(pcd_path):
    """Load PCD points as list of (x, y)."""
    with open(pcd_path, 'rb') as f:
        header = b''
        while True:
            line = f.readline()
            header += line
            if line.strip().startswith(b'DATA'):
                break

        # Parse header for point count
        points_count = 0
        for line in header.split(b'\n'):
            if line.startswith(b'POINTS'):
                points_count = int(line.split(b' ')[1])

        raw = f.read()
    pts = []
    for i in range(points_count):
        off = i * 12
        if off + 12 > len(raw):
            break
        x, y, z = struct.unpack_from('fff', raw, off)
        if math.isfinite(x) and math.isfinite(y):
            pts.append((x, y))
    return pts


def check_path_clear(pcd_points, x1, y1, x2, y2):
    """Check if straight-line path between two points is clear of PCD points."""
    dx = x2 - x1
    dy = y2 - y1
    dist = math.sqrt(dx * dx + dy * dy)
    if dist < 0.1:
        return True, 0  # Too close, assume clear

    steps = max(1, int(dist / SAMPLE_STEP_M))
    r2 = CHECK_RADIUS_M * CHECK_RADIUS_M

    max_density = 0
    for i in range(steps + 1):
        t = i / steps
        sx = x1 + dx * t
        sy = y1 + dy * t

        # Count points within radius
        count = 0
        for px, py in pcd_points:
            if (px - sx) ** 2 + (py - sy) ** 2 <= r2:
                count += 1
                if count >= DENSITY_WALL_THRESHOLD:
                    break

        max_density = max(max_density, count)
        if count >= DENSITY_WALL_THRESHOLD:
            return False, max_density

    return True, max_density


def build_graph():
    """Build connectivity graph from waypoints + PCD validation."""
    waypoints = load_waypoints()
    wp_ids = list(waypoints.keys())

    # Get PCD path from registry
    reg = load_registry()
    pcd_path = None
    for m in reg.get('maps', []):
        if m['map_id'] == 'go2w_real_site':
            pcd_path = m.get('pcd_path', '')
            break

    if not pcd_path or not os.path.exists(pcd_path):
        print(f'PCD not found: {pcd_path}')
        # Try default
        pcd_path = '/home/unitree/maps/staging/map_701.pcd'

    print(f'Loading PCD: {pcd_path}')
    pcd_points = load_pcd_points(pcd_path)
    print(f'  Points: {len(pcd_points)}')

    print(f'Checking {len(wp_ids)} waypoints...')
    edges = []
    blocked = []

    for i, a in enumerate(wp_ids):
        for j, b in enumerate(wp_ids):
            if j <= i:
                continue
            ax, ay = waypoints[a]['x'], waypoints[a]['y']
            bx, by = waypoints[b]['x'], waypoints[b]['y']
            clear, density = check_path_clear(pcd_points, ax, ay, bx, by)

            dist = math.sqrt((bx - ax) ** 2 + (by - ay) ** 2)
            if clear:
                edges.append({'a': a, 'b': b, 'dist_m': round(dist, 2), 'max_density': density})
                print(f'  ✅ {a} ↔ {b} ({dist:.1f}m, dens={density})')
            else:
                blocked.append({'a': a, 'b': b, 'dist_m': round(dist, 2), 'max_density': density})
                print(f'  ❌ {a} ↔ {b} ({dist:.1f}m, dens={density})')

    graph = {
        'waypoints': waypoints,
        'edges': edges,
        'blocked': blocked,
        'pcd_path': pcd_path,
    }

    with open(GRAPH_PATH, 'w') as f:
        json.dump(graph, f, indent=2)
    print(f'\nGraph saved: {GRAPH_PATH}')
    print(f'Edges: {len(edges)} passable, {len(blocked)} blocked')

    return graph


def load_graph():
    if os.path.exists(GRAPH_PATH):
        with open(GRAPH_PATH) as f:
            return json.load(f)
    return None


def build_adjacency(graph):
    adj = {}
    for e in graph['edges']:
        a, b = e['a'], e['b']
        adj.setdefault(a, []).append(b)
        adj.setdefault(b, []).append(a)
    return adj


def bfs_route(graph, from_id, to_id):
    """Find shortest path via BFS."""
    if from_id == to_id:
        return [from_id]

    adj = build_adjacency(graph)
    if from_id not in adj or to_id not in adj:
        return None

    q = deque([[from_id]])
    visited = {from_id}

    while q:
        path = q.popleft()
        last = path[-1]
        for nxt in adj.get(last, []):
            if nxt == to_id:
                return path + [nxt]
            if nxt not in visited:
                visited.add(nxt)
                q.append(path + [nxt])
    return None


def list_graph(graph):
    wp = graph['waypoints']
    print(f"Waypoints: {len(wp)}")
    for nid, info in wp.items():
        print(f"  {nid}: ({info['x']:.2f}, {info['y']:.2f}) {info['name']}")

    print(f"\nPassable edges: {len(graph['edges'])}")
    for e in graph['edges']:
        print(f"  {e['a']} ↔ {e['b']} ({e['dist_m']}m)")

    print(f"\nBlocked edges: {len(graph.get('blocked', []))}")
    for e in graph.get('blocked', []):
        print(f"  {e['a']} ↔ {e['b']} ({e['dist_m']}m, dens={e['max_density']})")


def print_route(graph, route):
    wp = graph['waypoints']
    total_dist = 0
    print(f"\nRoute ({len(route)-1} hops):")
    for i, nid in enumerate(route):
        info = wp[nid]
        marker = '→' if i > 0 else '📍'
        print(f"  {marker} {nid} ({info['name']}) [{info['x']:.2f}, {info['y']:.2f}]")
        if i > 0:
            prev = wp[route[i-1]]
            seg = math.sqrt((info['x']-prev['x'])**2 + (info['y']-prev['y'])**2)
            total_dist += seg
            print(f"     └─ {seg:.2f}m")
    print(f"\nTotal: {total_dist:.2f}m")


if __name__ == '__main__':
    cmd = sys.argv[1] if len(sys.argv) > 1 else 'list'

    if cmd == 'build':
        build_graph()
    elif cmd == 'list':
        graph = load_graph()
        if graph:
            list_graph(graph)
        else:
            print('No graph found. Run: python3 waypoint_graph.py build')
    elif cmd == 'route':
        if len(sys.argv) < 4:
            print('Usage: python3 waypoint_graph.py route <from_node> <to_node>')
            sys.exit(1)
        graph = load_graph() or build_graph()
        route = bfs_route(graph, sys.argv[2], sys.argv[3])
        if route:
            print_route(graph, route)
        else:
            print(f'No route from {sys.argv[2]} to {sys.argv[3]}')
    else:
        print('Commands: build | list | route <from> <to>')
