"""Waypoint-to-waypoint navigation using graph routing + sport_bridge.

Usage:
  python3 waypoint_nav.py <target_node_id> [speed_mps]

Flow:
  1. Get current SLAM pose via Gateway persistent session
  2. BFS route from nearest waypoint to target
  3. For each hop: rotate to face next → move forward → check arrival
"""

import subprocess
import json
import time
import threading
import queue
import math
import sys
import os

REPO = '/home/unitree/Go2W_SLAM_AI'
CLIENT = os.path.join(REPO, 'robot/slam_gateway_refactor/build/slam_llm_command_client')
BRIDGE = os.path.join(REPO, 'robot/slam_gateway_refactor/build/sport_bridge')
GRAPH_PATH = os.path.join(REPO, 'artifacts', 'waypoint_graph.json')

ARRIVE_THRESHOLD_M = 0.3  # consider arrived within 30cm
MOVE_SPEED_MPS = 0.2
ROTATE_SPEED = 0.5  # rad/s
ROTATE_THRESHOLD_RAD = 0.1  # consider aligned within ~6°

# ── Graph ──

def load_graph():
    with open(GRAPH_PATH) as f:
        return json.load(f)

def build_adjacency(graph):
    adj = {}
    for e in graph['edges']:
        a, b = e['a'], e['b']
        adj.setdefault(a, []).append(b)
        adj.setdefault(b, []).append(a)
    return adj

def bfs_route(graph, from_id, to_id):
    if from_id == to_id:
        return [from_id]
    adj = build_adjacency(graph)
    if from_id not in adj or to_id not in adj:
        return None
    from collections import deque
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

def nearest_waypoint(graph, x, y):
    best_id, best_dist = None, float('inf')
    for nid, info in graph['waypoints'].items():
        d = math.hypot(info['x'] - x, info['y'] - y)
        if d < best_dist:
            best_dist = d
            best_id = nid
    return best_id, best_dist

# ── SLAM Pose ──

def get_slam_pose():
    """Get current (x, y, yaw) from SLAM via Gateway persistent session."""
    p = subprocess.Popen(
        [CLIENT, 'eth0', '--persistent-world-state-session'],
        stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        text=True, bufsize=1
    )
    q = queue.Queue()
    def rdr():
        for line in iter(p.stdout.readline, ''):
            s = line.strip()
            if s.startswith('{'):
                try: q.put(json.loads(s))
                except: pass
    threading.Thread(target=rdr, daemon=True).start()
    time.sleep(3)

    p.stdin.write(json.dumps({"action": "get_world_state", "request_id": "pose1"}) + '\n')
    p.stdin.flush()

    pose = (0.0, 0.0, 0.0)
    deadline = time.time() + 8
    while time.time() < deadline:
        try:
            d = q.get(timeout=0.5)
            if 'world_state' in d:
                cp = d['world_state'].get('current_pose', {})
                pp = cp.get('pose', {})
                x = pp.get('x', 0)
                y = pp.get('y', 0)
                yaw = pp.get('yaw', 0)
                if x is not None and y is not None:
                    pose = (x, y, yaw)
                break
        except queue.Empty:
            pass

    p.stdin.close()
    p.terminate()
    return pose

# ── Motion ──

def send_sport(cmd_dict, timeout_s=3):
    """Send a command to sport_bridge, return response."""
    p = subprocess.Popen(
        [BRIDGE, 'eth0'],
        stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        text=True
    )
    out, _ = p.communicate(input=json.dumps(cmd_dict) + '\n', timeout=timeout_s)
    try:
        resp = json.loads(out.strip())
        return resp
    except:
        return {"error": "parse_failed", "raw": out}

def stop_robot():
    send_sport({"stop": True})

def rotate_to_yaw(target_yaw, current_yaw):
    """Rotate in place to target yaw using sport_bridge."""
    # Compute shortest rotation
    diff = target_yaw - current_yaw
    diff = math.atan2(math.sin(diff), math.cos(diff))  # normalize to [-pi, pi]

    if abs(diff) < ROTATE_THRESHOLD_RAD:
        return True  # Already aligned

    direction = 1.0 if diff > 0 else -1.0
    duration = int(abs(diff) / ROTATE_SPEED * 1000)
    duration = max(300, min(3000, duration))  # clamp

    print(f'  Rotating {math.degrees(diff):.0f}° ({duration}ms)...')
    resp = send_sport({
        "vx": 0.0, "vy": 0.0, "vyaw": direction * ROTATE_SPEED,
        "duration_ms": duration
    })
    return resp.get('ok') == 'true'

def move_forward(distance_m):
    """Move forward by distance_m meters."""
    duration = int(distance_m / MOVE_SPEED_MPS * 1000)
    duration = max(300, min(5000, duration))

    print(f'  Moving {distance_m:.2f}m ({duration}ms)...')
    resp = send_sport({
        "vx": MOVE_SPEED_MPS, "vy": 0.0, "vyaw": 0.0,
        "duration_ms": duration
    })
    return resp.get('ok') == 'true'

# ── Main ──

def main():
    if len(sys.argv) < 2:
        print('Usage: python3 waypoint_nav.py <target_node_id> [speed_mps]')
        sys.exit(1)

    target_node = sys.argv[1]
    global MOVE_SPEED_MPS
    if len(sys.argv) > 2:
        MOVE_SPEED_MPS = float(sys.argv[2])

    graph = load_graph()
    if target_node not in graph['waypoints']:
        print(f'Unknown node: {target_node}')
        print('Available:', list(graph['waypoints'].keys()))
        sys.exit(1)

    # Get current pose
    print('Getting SLAM pose...')
    x, y, yaw = get_slam_pose()
    print(f'Current: x={x:.2f} y={y:.2f} yaw={math.degrees(yaw):.0f}°')

    # Find nearest waypoint
    nearest_id, nearest_dist = nearest_waypoint(graph, x, y)
    print(f'Nearest: {nearest_id} ({nearest_dist:.2f}m)')

    # Route
    route = bfs_route(graph, nearest_id, target_node)
    if not route:
        print(f'No route from {nearest_id} to {target_node}')
        sys.exit(1)

    # Skip first waypoint if we're already at it
    if nearest_dist < ARRIVE_THRESHOLD_M and route[0] == nearest_id:
        pass  # keep it as we need it for direction

    print(f'\nRoute ({len(route)-1} hops):')
    for i, nid in enumerate(route):
        info = graph['waypoints'][nid]
        marker = '→' if i > 0 else '📍'
        print('  %s %s (%s) [%.2f, %.2f]' % (marker, nid, info['name'], info['x'], info['y']))

    try:
        # Execute hops
        current_x, current_y, current_yaw = x, y, yaw

        for i, node_id in enumerate(route):
            if i == 0:
                continue  # Skip starting node

            target = graph['waypoints'][node_id]
            tx, ty = target['x'], target['y']

            print('\n── Hop %d: → %s (%s) ──' % (i, node_id, target['name']))

            for attempt in range(3):
                # Get fresh pose
                current_x, current_y, current_yaw = get_slam_pose()

                dx = tx - current_x
                dy = ty - current_y
                dist = math.hypot(dx, dy)

                if dist < ARRIVE_THRESHOLD_M:
                    print(f'  ✅ Arrived ({dist:.2f}m)')
                    break

                target_bearing = math.atan2(dy, dx)
                print(f'  Dist: {dist:.2f}m  Bearing: {math.degrees(target_bearing):.0f}°')

                # Rotate to face target
                rotate_to_yaw(target_bearing, current_yaw)

                # Wait for rotation to settle
                time.sleep(0.5)

                # Get pose after rotation
                _, _, current_yaw = get_slam_pose()
                dx = tx - current_x
                dy = ty - current_y
                dist = math.hypot(dx, dy)

                if dist < ARRIVE_THRESHOLD_M:
                    print(f'  ✅ Arrived ({dist:.2f}m)')
                    break

                # Move forward
                move_forward(min(dist, 1.0))  # max 1m per hop segment

                # Check arrival
                time.sleep(0.5)
                current_x, current_y, current_yaw = get_slam_pose()
                dx = tx - current_x
                dy = ty - current_y
                dist = math.hypot(dx, dy)

                if dist < ARRIVE_THRESHOLD_M:
                    print(f'  ✅ Arrived ({dist:.2f}m)')
                    break

                print(f'  Remaining: {dist:.2f}m (attempt {attempt+1})')
            else:
                print(f'  ⚠️ Failed to reach {node_id} after 3 attempts')

        print(f'\n🎯 Navigation complete!')

    except KeyboardInterrupt:
        print('\nInterrupted.')
    finally:
        stop_robot()
        print('Stopped.')

if __name__ == '__main__':
    main()
