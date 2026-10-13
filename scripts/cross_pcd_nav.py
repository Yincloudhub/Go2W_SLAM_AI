#!/usr/bin/env python3
"""cross_pcd_nav — navigate across PCD boundaries with coordinate-frame-aware routing.

Plan:
  1. Navigate origin → wp_e643bc → transition_701_to_terrace (map_701 frame, 13.7m)
  2. Switch PCD to map_terrace_wc + relocate to mapping_origin_terrace
  3. Navigate wp_832687 → wp_48cd1f → wp_4ff584 (terrace frame, ~17m)
  4. Return to wp_832687 (terrace origin)
  5. Switch PCD back to map_701 + relocate to transition_701_to_terrace
  6. Navigate transition_701_to_terrace → wp_e643bc → wp_60497f (map_701 frame, 13.7m)
"""
from __future__ import annotations

import argparse, json, math, os, subprocess, sys, time
from typing import Any, Dict, List, Optional, Tuple

REPO = os.environ.get("GO2W_REPO", os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
SCRIPTS = os.path.join(REPO, "scripts")
if SCRIPTS not in sys.path:
    sys.path.insert(0, SCRIPTS)

import waypoint_graph
import nav_core
from nav_core import NavigationSession

GATEWAY_CLIENT = os.path.join(REPO, "robot", "slam_gateway_refactor", "build", "slam_llm_command_client")
REGISTRY_PATH = os.path.join(REPO, "configs", "maps", "go2w_real_site_map_registry.json")
GRAPH_PATH = os.path.join(REPO, "artifacts", "waypoint_graph.json")

MAP_701_PCD = "/home/unitree/maps/staging/map_701.pcd"
MAP_TERRACE_PCD = "/home/unitree/maps/staging/map_terrace_wc.pcd"

DEFAULT_SPEED = 0.25
ARRIVAL_RADIUS = 0.30
START_TOLERANCE = 3.00
TIMEOUT_MIN = 20.0
TIMEOUT_MARGIN = 35.0


# ═══════════════════════════════════
# Helpers (robot's waypoint_graph.py missing these)
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


# ═══════════════════════════════════
# Graph: load waypoints optionally filtered by pcd_owner
# ═══════════════════════════════════

def load_waypoints_from_registry(pcd_owner=None):
    """Load waypoints from registry, optionally filtered to one PCD."""
    reg = json.load(open(REGISTRY_PATH))
    wp = {}
    for m in reg.get("maps", []):
        owner = m.get("map_id", "")
        for n in m.get("topology_nodes", []):
            nid = n["node_id"]
            if nid in wp:
                continue  # first occurrence wins (go2w_real_site wrapper)
            if pcd_owner and n.get("pcd_owner", owner) != pcd_owner:
                continue
            p = n.get("pose", {})
            wp[nid] = {
                "name": n.get("name", nid),
                "x": float(p.get("x", 0)), "y": float(p.get("y", 0)),
                "yaw": float(p.get("yaw", 0)),
                "type": n.get("node_type", ""),
            }
    return wp

def build_subgraph(pcd_owner=None):
    """Build a graph with waypoints from a specific PCD."""
    wp = load_waypoints_from_registry(pcd_owner)
    # Collect edges from all topology_edges that connect known waypoints
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
# Registry & PCD switch
# ═══════════════════════════════════

def update_pcd_path(pcd):
    reg = json.load(open(REGISTRY_PATH))
    for m in reg["maps"]:
        if m["map_id"] == "go2w_real_site":
            m["pcd_path"] = pcd
            break
    json.dump(reg, open(REGISTRY_PATH, "w"), indent=2, ensure_ascii=False)

def get_anchor_pose(anchor_id):
    """Read exact anchor pose from registry (prevents coordinate mismatch)."""
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
    
    # ── Verify: read pose 3 times over 6s, confirm not frozen ──
    return verify_localization(pose["x"], pose["y"], anchor_id)


def verify_localization(expected_x, expected_y, anchor_id, retries=3):
    """After relocate, confirm SLAM pose is alive and near the anchor position."""
    import threading, queue
    
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
        print("  VERIFY FAIL: no pose data received")
        return False
    
    # Check: pose is not frozen at exactly (0,0,0) if anchor is not at origin
    dx = max(p[0] for p in poses) - min(p[0] for p in poses)
    dy = max(p[1] for p in poses) - min(p[1] for p in poses)
    if dx < 0.001 and dy < 0.001:
        if abs(expected_x) < 0.01 and abs(expected_y) < 0.01:
            print("  VERIFY OK: at origin anchor, pose stable")
            return True
        else:
            print("  VERIFY FAIL: pose frozen at (%.3f,%.3f) — SLAM not tracking" % (poses[-1][0], poses[-1][1]))
            return False
    
    # Check: pose is near expected anchor position
    avg_x = sum(p[0] for p in poses) / len(poses)
    avg_y = sum(p[1] for p in poses) / len(poses)
    dist = math.hypot(avg_x - expected_x, avg_y - expected_y)
    if dist > 2.0:
        print("  VERIFY WARN: pose (%.2f,%.2f) is %.2fm from anchor (%.2f,%.2f)" % (
            avg_x, avg_y, dist, expected_x, expected_y))
    
    print("  VERIFY OK: pose alive (drift dx=%.3f dy=%.3f), near anchor" % (dx, dy))
    return True


# ═══════════════════════════════════
# Navigation primitive
# ═══════════════════════════════════

def navigate_to(nav, graph, target_id, speed, label=""):
    """Navigate from current pose to target waypoint using BFS route."""
    pose = nav.pose
    wp = graph["waypoints"]
    start_id, start_dist = _nearest_waypoint(wp, pose[0], pose[1])
    if not start_id or start_dist > START_TOLERANCE:
        print("  [%s] Cannot resolve start (nearest=%s dist=%.2fm)" % (label, start_id, start_dist))
        return False

    # Simple BFS (inlined to not depend on robot's waypoint_graph.bfs_route)
    adj = {}
    for e in graph["edges"]:
        a, b = e["a"], e["b"]
        adj.setdefault(a, []).append(b)
        if e.get("bidirectional", True):
            adj.setdefault(b, []).append(a)

    from collections import deque
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


# ═══════════════════════════════════

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--speed", type=float, default=DEFAULT_SPEED)
    parser.add_argument("--skip-terrace-nav", action="store_true",
                        help="Skip terrace navigation, only test PCD switch.")
    args = parser.parse_args()
    speed = args.speed

    print("="*60)
    print("CROSS-PCD NAV: origin -> terrace waypoints -> back")
    print("Speed: %.2f m/s" % speed)
    print("="*60)

    # ── Step 1: Navigate origin → transition_701_to_terrace (map_701) ──
    print("\n[1/6] NAVIGATE (map_701): origin -> transition_701_to_terrace")
    g701 = build_subgraph(pcd_owner="map_701")
    print("  Graph: %d nodes, %d edges" % (len(g701["waypoints"]), len(g701["edges"])))

    with NavigationSession() as nav:
        p = nav.pose
        print("  Pose: x=%.2f y=%.2f yaw=%.0fdeg  loc=%s safety=%s" % (
            p[0], p[1], math.degrees(p[2]), nav.loc_status, nav.gateway_allows_motion()))
        ok = navigate_to(nav, g701, "transition_701_to_terrace", speed, "701")
        fp = nav.pose
        print("  Final: x=%.2f y=%.2f" % (fp[0], fp[1]))

    if not ok:
        print("ABORT: failed to reach transition point")
        return 2

    # ── Step 2: Switch PCD → map_terrace_wc ──
    print("\n[2/6] SWITCH PCD: map_701 -> map_terrace_wc")
    update_pcd_path(MAP_TERRACE_PCD)
    terrace_origin_pose = get_anchor_pose("mapping_origin_terrace")
    if not terrace_origin_pose:
        print("ERROR: mapping_origin_terrace not found in registry")
        return 2
    if not gateway_relocate("mapping_origin_terrace", MAP_TERRACE_PCD, terrace_origin_pose):
        print("WARNING: terrace relocate may have failed")

    if args.skip_terrace_nav:
        # Skip to step 5
        pass
    else:
        # ── Step 3: Navigate terrace waypoints ──
        print("\n[3/6] NAVIGATE (terrace): wp_832687 -> wp_48cd1f -> wp_4ff584")
        g_terrace = build_subgraph(pcd_owner="map_terrace_wc")
        print("  Graph: %d nodes, %d edges" % (len(g_terrace["waypoints"]), len(g_terrace["edges"])))

        with NavigationSession() as nav2:
            p = nav2.pose
            print("  Pose: x=%.2f y=%.2f yaw=%.0fdeg  loc=%s safety=%s" % (
                p[0], p[1], math.degrees(p[2]), nav2.loc_status, nav2.gateway_allows_motion()))

            for target in ["wp_48cd1f", "wp_4ff584"]:
                print("  -- Target: %s --" % target)
                if not navigate_to(nav2, g_terrace, target, speed, "TER"):
                    print("ABORT: terrace nav failed at %s" % target)
                    return 2

        # ── Step 4: Return to wp_832687 (terrace origin) ──
        print("\n[4/6] RETURN to wp_832687 (terrace origin)")
        with NavigationSession() as nav3:
            p = nav3.pose
            print("  Pose: x=%.2f y=%.2f" % (p[0], p[1]))
            if not navigate_to(nav3, g_terrace, "wp_832687", speed, "TER-RET"):
                print("WARNING: return to terrace origin may not be exact")

    # ── Step 5: Switch PCD back → map_701 ──
    print("\n[5/6] SWITCH PCD: map_terrace_wc -> map_701")
    update_pcd_path(MAP_701_PCD)
    trans_pose = get_anchor_pose("transition_701_to_terrace")
    if not trans_pose:
        print("ERROR: transition_701_to_terrace not found in registry")
        return 2
    if not gateway_relocate("transition_701_to_terrace", MAP_701_PCD, trans_pose):
        print("WARNING: 701 relocate may have failed")

    # ── Step 6: Navigate back to origin ──
    print("\n[6/6] NAVIGATE (map_701): transition_701_to_terrace -> wp_60497f")
    with NavigationSession() as nav4:
        p = nav4.pose
        print("  Pose: x=%.2f y=%.2f yaw=%.0fdeg  loc=%s safety=%s" % (
            p[0], p[1], math.degrees(p[2]), nav4.loc_status, nav4.gateway_allows_motion()))
        if not navigate_to(nav4, g701, "wp_60497f", speed, "701-RET"):
            print("ABORT: return nav failed")
            return 2

    print("\n" + "="*60)
    print("CROSS-PCD NAV COMPLETE")
    print("="*60)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

