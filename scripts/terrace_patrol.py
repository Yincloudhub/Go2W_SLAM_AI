#!/usr/bin/env python3
"""terrace_patrol — loop patrol in terrace PCD with cross-PCD navigation.

Full flow:
  1. Navigate in map_701: current → transition_701_to_terrace
  2. Switch PCD to map_terrace_wc + relocate
  3. Loop: wp_48cd1f → wp_4ff584 → wp_a98f89 → (repeat)
  4. On exit: switch back to map_701 + relocate
"""
from __future__ import annotations

import argparse, json, math, os, subprocess, sys, time, threading, queue
from collections import deque
from typing import Any, Dict, List, Optional, Tuple

REPO = os.environ.get("GO2W_REPO", os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
SCRIPTS = os.path.join(REPO, "scripts")
if SCRIPTS not in sys.path:
    sys.path.insert(0, SCRIPTS)

import nav_core
from nav_core import NavigationSession

GATEWAY_CLIENT = os.path.join(REPO, "robot", "slam_gateway_refactor", "build", "slam_llm_command_client")
REGISTRY_PATH = os.path.join(REPO, "configs", "maps", "go2w_real_site_map_registry.json")

MAP_701_PCD = "/home/unitree/maps/staging/map_701.pcd"
MAP_TERRACE_PCD = "/home/unitree/maps/staging/map_terrace_wc.pcd"

PATROL_NODES = ["wp_48cd1f", "wp_4ff584", "wp_a98f89"]  # 末端→门口→电梯口

DEFAULT_SPEED = 0.50
ARRIVAL_RADIUS = 0.30
PAUSE_S = 5.0
START_TOLERANCE = 3.00
TIMEOUT_MIN = 20.0
TIMEOUT_MARGIN = 35.0


# ═══════════════════════════════════
# Graph helpers
# ═══════════════════════════════════

def _dist_between(wp, a, b):
    pa, pb = wp[a], wp[b]
    return math.hypot(float(pa["x"]) - float(pb["x"]), float(pa["y"]) - float(pb["y"]))

def _nearest_waypoint(wp, x, y):
    best_id, best_dist = None, float("inf")
    for nid, info in wp.items():
        d = math.hypot(float(info["x"]) - x, float(info["y"]) - y)
        if d < best_dist:
            best_id, best_dist = nid, d
    return best_id, best_dist

def _route_distance(wp, route):
    return sum(_dist_between(wp, route[i-1], route[i]) for i in range(1, len(route)))

def load_waypoints_from_registry(pcd_owner=None):
    reg = json.load(open(REGISTRY_PATH))
    wp = {}
    for m in reg.get("maps", []):
        owner = m.get("map_id", "")
        for n in m.get("topology_nodes", []):
            nid = n["node_id"]
            if nid in wp:
                continue
            if pcd_owner and n.get("pcd_owner", owner) != pcd_owner:
                continue
            p = n.get("pose", {})
            wp[nid] = {
                "name": n.get("name", nid),
                "x": float(p.get("x", 0)), "y": float(p.get("y", 0)),
                "yaw": float(p.get("yaw", 0)),
            }
    return wp

def build_subgraph(pcd_owner=None):
    wp = load_waypoints_from_registry(pcd_owner)
    reg = json.load(open(REGISTRY_PATH))
    edges = []
    wp_set = set(wp)
    for m in reg.get("maps", []):
        for e in m.get("topology_edges", []):
            a, b = e["from"], e["to"]
            if a in wp_set and b in wp_set:
                dist = e.get("expected_distance_m") or _dist_between(wp, a, b)
                edges.append({"a": a, "b": b, "dist_m": round(dist, 3), "bidirectional": True})
    return {"waypoints": wp, "edges": edges}


# ═══════════════════════════════════
# PCD switch
# ═══════════════════════════════════

def update_pcd_path(pcd):
    reg = json.load(open(REGISTRY_PATH))
    for m in reg["maps"]:
        if m["map_id"] == "go2w_real_site":
            m["pcd_path"] = pcd
            break
    json.dump(reg, open(REGISTRY_PATH, "w"), indent=2, ensure_ascii=False)

def get_anchor_pose(anchor_id):
    reg = json.load(open(REGISTRY_PATH))
    for m in reg["maps"]:
        if m["map_id"] == "go2w_real_site":
            for a in m.get("relocalization_anchors", []):
                if a["anchor_id"] == anchor_id:
                    p = a["pose"]
                    return {
                        "x": p["x"], "y": p["y"], "z": p.get("z", 0.0),
                        "yaw": p["yaw"],
                        "q_x": p.get("q_x", 0.0), "q_y": p.get("q_y", 0.0),
                        "q_z": p.get("q_z", 0.0), "q_w": p.get("q_w", 1.0),
                        "name": anchor_id, "speed": 0.0, "mode": 0
                    }
    return None

def verify_localization(expected_x, expected_y, anchor_id, retries=3):
    p = subprocess.Popen([GATEWAY_CLIENT, "eth0", "--persistent-world-state-session"],
        stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        text=True, bufsize=1)
    q = queue.Queue()
    def reader():
        for line in iter(p.stdout.readline, ''):
            if line.strip().startswith('{'):
                try: q.put(json.loads(line.strip()))
                except: pass
    threading.Thread(target=reader, daemon=True).start()
    time.sleep(4)

    poses = []
    for i in range(3):
        p.stdin.write(json.dumps({"action": "get_world_state", "request_id": "v%d" % i}) + "\n")
        p.stdin.flush()
        deadline = time.time() + 5
        while time.time() < deadline:
            try:
                d = q.get(timeout=1)
                if "world_state" in d:
                    ws = d["world_state"]
                    pp = ws.get("pose", {})
                    loc = ws.get("localization", {})
                    x, y = pp.get("x", 0), pp.get("y", 0)
                    poses.append((x, y))
                    print("  verify[%d] x=%.3f y=%.3f loc=%s" % (i, x, y, loc.get("status")))
                    break
            except queue.Empty:
                pass
        time.sleep(1.5)
    p.stdin.close()

    if len(poses) < 2:
        print("  VERIFY FAIL: no pose data")
        return False
    dx = max(p[0] for p in poses) - min(p[0] for p in poses)
    dy = max(p[1] for p in poses) - min(p[1] for p in poses)
    if dx < 0.001 and dy < 0.001 and (abs(expected_x) > 0.01 or abs(expected_y) > 0.01):
        print("  VERIFY FAIL: pose frozen")
        return False
    print("  VERIFY OK: pose alive")
    return True

def gateway_relocate(anchor_id, map_path, pose):
    cmd = json.dumps({
        "action": "relocate", "map_id": "go2w_real_site",
        "map_path": map_path, "anchor_id": anchor_id,
        "initial_pose": pose, "operator_ack": True,
        "request_id": "reloc_%d" % int(time.time())
    })
    r = subprocess.run([GATEWAY_CLIENT, "eth0"], input=cmd+"\n",
                       capture_output=True, text=True, timeout=20)
    ok = '"accepted":true' in r.stdout.replace(" ", "")
    print("  relocate %s -> %s" % (anchor_id, "OK" if ok else "FAIL"))
    if not ok:
        return False
    return verify_localization(pose["x"], pose["y"], anchor_id)

def switch_to_terrace():
    """Switch PCD from map_701 to map_terrace_wc."""
    print("\n[PCD SWITCH] map_701 → map_terrace_wc")
    update_pcd_path(MAP_TERRACE_PCD)
    anchor_pose = get_anchor_pose("mapping_origin_terrace")
    if not anchor_pose:
        print("  ERROR: mapping_origin_terrace not found in registry")
        return False
    return gateway_relocate("mapping_origin_terrace", MAP_TERRACE_PCD, anchor_pose)

def switch_to_701():
    """Switch PCD from map_terrace_wc back to map_701."""
    print("\n[PCD SWITCH] map_terrace_wc → map_701")
    update_pcd_path(MAP_701_PCD)
    anchor_pose = get_anchor_pose("transition_701_to_terrace")
    if not anchor_pose:
        print("  ERROR: transition_701_to_terrace not found in registry")
        return False
    return gateway_relocate("transition_701_to_terrace", MAP_701_PCD, anchor_pose)


# ═══════════════════════════════════
# Navigation
# ═══════════════════════════════════

def navigate_to(nav, graph, target_id, speed, label=""):
    """Navigate from current pose to target using BFS route."""
    pose = nav.pose
    wp = graph["waypoints"]
    start_id, start_dist = _nearest_waypoint(wp, pose[0], pose[1])
    if not start_id or start_dist > START_TOLERANCE:
        print("  [%s] Cannot resolve start (nearest=%s dist=%.2fm)" % (label, start_id, start_dist))
        return False

    adj = {}
    for e in graph["edges"]:
        a, b = e["a"], e["b"]
        adj.setdefault(a, []).append(b)
        if e.get("bidirectional", True):
            adj.setdefault(b, []).append(a)

    q = deque([[start_id]])
    visited = {start_id}
    route = None
    while q:
        path = q.popleft()
        last = path[-1]
        for nxt in adj.get(last, []):
            if nxt == target_id:
                route = path + [nxt]
                break
            if nxt not in visited:
                visited.add(nxt)
                q.append(path + [nxt])
        if route:
            break

    if not route:
        print("  [%s] No route from %s to %s" % (label, start_id, target_id))
        return False

    total = _route_distance(wp, route)
    print("  [%s] Route (%d hops, %.2fm): %s" % (label, len(route)-1, total, " -> ".join(route)))

    for idx in range(1, len(route)):
        nid = route[idx]
        prev = route[idx-1]
        target = wp[nid]
        seg_dist = math.hypot(float(target["x"]) - float(wp[prev]["x"]),
                              float(target["y"]) - float(wp[prev]["y"]))
        max_t = max(TIMEOUT_MIN, seg_dist / max(speed, 0.05) + TIMEOUT_MARGIN)
        print("  [%s] Hop %d/%d: %s (%s) dist=%.2fm timeout=%.0fs" % (
            label, idx, len(route)-1, nid, target["name"], seg_dist, max_t))
        result = nav.move_to_xy(float(target["x"]), float(target["y"]),
                                goal_tol=ARRIVAL_RADIUS, max_speed=speed, max_time=max_t)
        print("  [%s] -> %s" % (label, result))
        if result != "ARRIVED":
            return False
    return True


def navigate_direct(nav, target_id, wp, speed):
    """Blind hop (no BFS) — for terrace where graph may be sparse."""
    t = wp[target_id]
    dist = math.hypot(float(t["x"]) - nav.pose[0], float(t["y"]) - nav.pose[1])
    max_t = max(TIMEOUT_MIN, dist / max(speed, 0.05) + TIMEOUT_MARGIN)
    print(f"  -> {target_id} ({t['name']}) dist={dist:.2f}m timeout={max_t:.0f}s")
    result = nav.move_to_xy(float(t["x"]), float(t["y"]),
                            goal_tol=ARRIVAL_RADIUS, max_speed=speed, max_time=max_t)
    print(f"  -> {result}")
    return result == "ARRIVED"


# ═══════════════════════════════════
# Main
# ═══════════════════════════════════

def main():
    parser = argparse.ArgumentParser(description="Terrace patrol loop with cross-PCD navigation.")
    parser.add_argument("--speed", type=float, default=DEFAULT_SPEED)
    parser.add_argument("--pause", type=float, default=PAUSE_S, help="Pause seconds at each waypoint.")
    parser.add_argument("--max-loops", type=int, default=0, help="Max loops (0=infinite).")
    args = parser.parse_args()

    print("=" * 60)
    print("TERRACE PATROL (cross-PCD)")
    route_str = " -> ".join(PATROL_NODES)
    print("Route: %s" % route_str)
    print("Speed: %.2f m/s  Pause: %.0fs each" % (args.speed, args.pause))
    print("=" * 60)

    # ── Step 1: Navigate within map_701 to transition point ──
    print("\n[1] NAVIGATE (map_701): current -> transition_701_to_terrace")
    g701 = build_subgraph(pcd_owner="map_701")
    print("  Graph: %d nodes, %d edges" % (len(g701["waypoints"]), len(g701["edges"])))

    with NavigationSession() as nav:
        p = nav.pose
        print("  Pose: x=%.2f y=%.2f yaw=%.0fdeg  loc=%s safety=%s" % (
            p[0], p[1], math.degrees(p[2]), nav.loc_status, nav.gateway_allows_motion()))
        ok = navigate_to(nav, g701, "transition_701_to_terrace", args.speed, "701")
        if not ok:
            print("FAILED: cannot reach transition point in map_701")
            return 2
    print("  ARRIVED at transition point.")

    # ── Step 2: Switch to terrace PCD ──
    if not switch_to_terrace():
        print("FAILED: PCD switch to terrace")
        return 2

    # ── Step 3: Patrol loop ──
    wp_terrace = load_waypoints_from_registry(pcd_owner="map_terrace_wc")
    available = set(wp_terrace)
    missing = [n for n in PATROL_NODES if n not in available]
    if missing:
        print("ERROR: missing terrace nodes:", missing)
        return 2

    loop_count = 0
    max_loops = args.max_loops if args.max_loops > 0 else float("inf")

    try:
        while loop_count < max_loops:
            loop_count += 1
            print("\n--- LOOP %d ---" % loop_count)

            with NavigationSession() as nav:
                p = nav.pose
                print("Pose: x=%.2f y=%.2f yaw=%.0fdeg  loc=%s safety=%s" % (
                    p[0], p[1], math.degrees(p[2]), nav.loc_status, nav.gateway_allows_motion()))

                for idx, node_id in enumerate(PATROL_NODES):
                    print("\n[%d.%d] %s (%s)" % (loop_count, idx + 1, node_id, wp_terrace[node_id]["name"]))
                    ok = navigate_direct(nav, node_id, wp_terrace, args.speed)
                    if not ok:
                        print("PATROL FAILED at %s — stopping" % node_id)
                        return 2
                    print("  Pausing %.0fs at %s..." % (args.pause, wp_terrace[node_id]["name"]))
                    time.sleep(args.pause)

            print("\nLoop %d complete." % loop_count)
    except KeyboardInterrupt:
        print("\nPatrol stopped by user.")

    # ── Step 4: Switch back to map_701 ──
    if not switch_to_701():
        print("WARNING: PCD switch back to map_701 failed")
    else:
        print("\nReturned to map_701.")

    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print("\nPatrol stopped by user.")
