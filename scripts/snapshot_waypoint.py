#!/usr/bin/env python3
"""
snapshot_waypoint.py — Record current SLAM pose into multi-PCD registry.

Usage:
  python3 scripts/snapshot_waypoint.py <node_id> [--map <map_id>]

  If --map is omitted, the script searches all maps in the V2 registry
  for the given node_id. If found in exactly one map, it uses that one.

Examples:
  python3 scripts/snapshot_waypoint.py yin_siyuan_station
  python3 scripts/snapshot_waypoint.py terrace_entrance --map map_terrace_wc

The script:
  1. Reads current SLAM pose from Gateway (get_world_state)
  2. Updates the target node's pose in the V2 registry
  3. Adds 'live_calibrated' tag
  4. Prints the old vs new pose diff
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
from pathlib import Path
from typing import Optional

REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = REPO_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

REGISTRY_PATH = REPO_ROOT / "configs" / "maps" / "go2w_multi_map_registry_v2.json"


def get_current_pose(timeout_s: int = 10) -> dict:
    """Read current SLAM pose via Gateway get_world_state (persistent session)."""
    import subprocess
    import threading
    import queue

    client = str(REPO_ROOT / "robot" / "slam_gateway_refactor" / "build" / "slam_llm_command_client")

    if not os.path.exists(client):
        # Fallback: try without full path
        client = "robot/slam_gateway_refactor/build/slam_llm_command_client"

    p = subprocess.Popen(
        [client, "eth0"],
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        bufsize=1,
        cwd=str(REPO_ROOT),
    )

    q: queue.Queue = queue.Queue()

    def reader():
        for line in iter(p.stdout.readline, ""):
            line = line.strip()
            if line.startswith("{"):
                try:
                    q.put(json.loads(line))
                except json.JSONDecodeError:
                    pass

    t = threading.Thread(target=reader, daemon=True)
    t.start()
    time.sleep(1.5)  # Wait for DDS subscription

    p.stdin.write(json.dumps({"action": "get_world_state", "request_id": "snap"}) + "\n")
    p.stdin.flush()

    deadline = time.time() + timeout_s
    while time.time() < deadline:
        try:
            d = q.get(timeout=1)
            if "world_state" in d:
                ws = d["world_state"]
                pose = ws.get("current_pose", {}).get("pose", {})
                if pose.get("x") != 0.0 or pose.get("y") != 0.0:
                    p.stdin.close()
                    p.terminate()
                    return pose
        except queue.Empty:
            pass

    p.stdin.close()
    p.terminate()
    raise RuntimeError(
        "Failed to get valid pose from Gateway. Is SLAM localized? "
        "Run: bash scripts/build_multi_pcd.sh relocate"
    )


def load_registry() -> dict:
    with open(REGISTRY_PATH) as f:
        return json.load(f)


def save_registry(reg: dict):
    with open(REGISTRY_PATH, "w") as f:
        json.dump(reg, f, indent=2, ensure_ascii=False)
    print(f"  Registry saved: {REGISTRY_PATH}")


def find_node(reg: dict, node_id: str, target_map: Optional[str] = None):
    """Find node in registry. Returns (map_entry, node_entry)."""
    candidates = []
    for m in reg["maps"]:
        if target_map and m["map_id"] != target_map:
            continue
        for n in m.get("topology_nodes", []):
            if n["node_id"] == node_id:
                candidates.append((m, n))

    if not candidates:
        maps_with_nodes = [(m["map_id"], [n["node_id"] for n in m.get("topology_nodes", [])])
                          for m in reg["maps"]]
        print(f"ERROR: node '{node_id}' not found in registry.", file=sys.stderr)
        print(f"  Maps with topology_nodes:", file=sys.stderr)
        for map_id, nodes in maps_with_nodes:
            if nodes:
                print(f"    {map_id}: {nodes}", file=sys.stderr)
        sys.exit(1)

    if len(candidates) > 1:
        map_ids = [m["map_id"] for m, _ in candidates]
        print(f"ERROR: node '{node_id}' found in multiple maps: {map_ids}", file=sys.stderr)
        print(f"  Use --map to specify which one.", file=sys.stderr)
        sys.exit(1)

    return candidates[0]


def pose_to_dict(pose: dict) -> dict:
    """Extract standard pose fields from Gateway pose dict."""
    return {
        "x": float(pose.get("x", 0.0)),
        "y": float(pose.get("y", 0.0)),
        "z": float(pose.get("z", 0.0)),
        "yaw": float(pose.get("yaw", 0.0)),
        "q_x": float(pose.get("q_x", 0.0)),
        "q_y": float(pose.get("q_y", 0.0)),
        "q_z": float(pose.get("q_z", 0.0)),
        "q_w": float(pose.get("q_w", 1.0)),
        "name": pose.get("name", ""),
        "speed": float(pose.get("speed", 0.5)),
        "mode": int(pose.get("mode", 0)),
    }


def main():
    parser = argparse.ArgumentParser(
        description="Record current SLAM pose into multi-PCD topology node."
    )
    parser.add_argument("node_id", help="Topology node ID to update (e.g. yin_siyuan_station)")
    parser.add_argument("--map", dest="map_id", help="Target map_id (auto-detected if omitted)")
    parser.add_argument("--dry-run", action="store_true", help="Print pose but don't save")
    parser.add_argument("--timeout", type=int, default=10, help="Gateway timeout seconds")
    args = parser.parse_args()

    # 1. Load registry and find node
    reg = load_registry()
    map_entry, node_entry = find_node(reg, args.node_id, args.map_id)

    map_id = map_entry["map_id"]
    node_id = node_entry["node_id"]
    print(f"  Target: {node_id}  (map: {map_id})")
    print(f"  Old pose: x={node_entry['pose'].get('x'):.3f}, y={node_entry['pose'].get('y'):.3f}")

    # 2. Get current SLAM pose
    print(f"  Reading current pose from SLAM...")
    try:
        current_pose = get_current_pose(timeout_s=args.timeout)
    except RuntimeError as e:
        print(f"\n  ERROR: {e}", file=sys.stderr)
        print(f"  Prerequisites:", file=sys.stderr)
        print(f"    1. SLAM must be running: bash scripts/start_go2w_slam_stack.sh", file=sys.stderr)
        print(f"    2. PCD must be loaded + localized: build_multi_pcd.sh relocate", file=sys.stderr)
        print(f"    3. Robot should be at the desired waypoint position", file=sys.stderr)
        sys.exit(1)

    new_pose_dict = pose_to_dict(current_pose)
    print(f"  New pose: x={new_pose_dict['x']:.4f}, y={new_pose_dict['y']:.4f}, yaw={new_pose_dict['yaw']:.4f}")

    if args.dry_run:
        print(f"  --dry-run: not saving")
        return

    # 3. Update node
    node_entry["pose"] = new_pose_dict

    # Update tags
    tags = node_entry.get("tags", [])
    if "live_calibrated" not in tags:
        tags.append("live_calibrated")
    # Remove placeholder tags if present
    for stale in ["needs_calibration", "needs_standing_verification"]:
        if stale in tags:
            tags.remove(stale)
    node_entry["tags"] = tags

    # 4. Save
    save_registry(reg)

    # 5. Print summary
    dist_x = new_pose_dict["x"] - node_entry.get("_old_x", new_pose_dict["x"])
    dist_y = new_pose_dict["y"] - node_entry.get("_old_y", new_pose_dict["y"])
    print(f"\n  ✅ {node_id}: pose updated in {map_id}")
    print(f"     Position: ({new_pose_dict['x']:.3f}, {new_pose_dict['y']:.3f})")
    print(f"     Yaw: {new_pose_dict['yaw']:.4f} rad ({new_pose_dict['yaw']*57.3:.1f}°)")
    print(f"     Tags: {tags}")

    # Print copy-paste command for next node
    next_suggestions = [
        n["node_id"] for n in map_entry.get("topology_nodes", [])
        if "live_calibrated" not in n.get("tags", [])
    ]
    if next_suggestions:
        print(f"\n  Remaining nodes in {map_id}:")
        for ns in next_suggestions:
            print(f"    python3 scripts/snapshot_waypoint.py {ns}")


if __name__ == "__main__":
    main()
