#!/usr/bin/env python3
"""
snapshot_waypoint.py — Create a topology node at current SLAM pose with full attributes.

Usage:
  python3 scripts/snapshot_waypoint.py --map <map_id> --type <node_type> \\
      --name "中文名称" [--node-id <id>] [--aliases "a,b"] \\
      [--person <name>] [--area <area>] [--landmark <landmark>] \\
      [--connects "A,B"] [--tags "t1,t2"]

Node types:
  attributed       — person station, named location (needs --person or --area)
  corridor_endpoint — passage waypoint (needs --connects)
  rotation_point   — spot with turn space

Examples:
  # Attribute a person's station
  python3 scripts/snapshot_waypoint.py --map map_701 --type attributed \
      --name "尹思园工位" --person "尹思园" --area "701右侧" \
      --aliases "yin_siyuan,尹思园"

  # Corridor endpoint
  python3 scripts/snapshot_waypoint.py --map map_701 --type corridor_endpoint \
      --name "701中心走廊口" --connects "701办公区,外走廊" \
      --node-id corridor_701_center

  # Simple entrance
  python3 scripts/snapshot_waypoint.py --map map_terrace_wc --type corridor_endpoint \
      --name "露台入口" --connects "走廊,露台" --aliases "terrace,露台"

If --node-id is omitted, it's auto-generated from --name (lowercase, spaces→underscores).
"""

from __future__ import annotations

import argparse
import json
import os
import re
import sys
import time
from pathlib import Path
from typing import Optional

REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = REPO_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

REGISTRY_PATH = REPO_ROOT / "configs" / "maps" / "go2w_multi_map_registry_v2.json"


def name_to_node_id(name: str) -> str:
    """Auto-generate node_id from Chinese/English name."""
    # If name is already in English/node-id format, just clean it
    if re.match(r'^[a-z][a-z0-9_]*$', name):
        return name
    # Try to extract English aliases from the name (common pattern: "English中文")
    # Otherwise transliterate: keep ASCII, lowercase, replace spaces with _
    ascii_part = re.sub(r'[^a-zA-Z0-9 _-]', '', name).strip().lower()
    if ascii_part and len(ascii_part) > 3:
        return re.sub(r'[ _-]+', '_', ascii_part)
    # Fallback: use a hash of the name
    import hashlib
    h = hashlib.md5(name.encode()).hexdigest()[:8]
    return f"node_{h}"


def get_current_pose(timeout_s: int = 10) -> dict:
    """Read current SLAM pose via Gateway get_world_state."""
    import subprocess
    import threading
    import queue

    client = str(REPO_ROOT / "robot" / "slam_gateway_refactor" / "build" / "slam_llm_command_client")
    if not os.path.exists(client):
        client = "robot/slam_gateway_refactor/build/slam_llm_command_client"

    p = subprocess.Popen(
        [client, "eth0"],
        stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
        text=True, bufsize=1, cwd=str(REPO_ROOT),
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

    threading.Thread(target=reader, daemon=True).start()
    time.sleep(1.5)

    p.stdin.write(json.dumps({"action": "get_world_state", "request_id": "snap"}) + "\n")
    p.stdin.flush()

    deadline = time.time() + timeout_s
    while time.time() < deadline:
        try:
            d = q.get(timeout=1)
            if "world_state" in d:
                pose = d["world_state"].get("current_pose", {}).get("pose", {})
                if abs(pose.get("x", 0)) > 0.001 or abs(pose.get("y", 0)) > 0.001:
                    p.stdin.close()
                    p.terminate()
                    return pose
        except queue.Empty:
            pass

    p.stdin.close()
    p.terminate()
    raise RuntimeError(
        "Failed to get valid pose from Gateway. Is SLAM localized?\n"
        "Run: bash scripts/build_multi_pcd.sh relocate"
    )


def load_registry() -> dict:
    with open(REGISTRY_PATH) as f:
        return json.load(f)


def save_registry(reg: dict):
    with open(REGISTRY_PATH, "w") as f:
        json.dump(reg, f, indent=2, ensure_ascii=False)


def build_node(args, pose: dict) -> dict:
    """Build a complete topology node entry from CLI args + SLAM pose."""
    node_id = args.node_id or name_to_node_id(args.name)
    aliases = [a.strip() for a in (args.aliases or "").split(",") if a.strip()]
    # Always include the name itself as an alias
    if args.name not in aliases:
        aliases.insert(0, args.name)

    tags = ["real_site", "live_calibrated"]
    if args.tags:
        tags.extend(t.strip() for t in args.tags.split(",") if t.strip())

    attributes = {}
    if args.type == "attributed":
        if args.person:
            attributes["person"] = args.person
        if args.area:
            attributes["area"] = args.area
        if args.landmark:
            attributes["landmark"] = args.landmark
        point_category = "person_station" if args.person else "named_location"

    elif args.type == "corridor_endpoint":
        connects = [c.strip() for c in (args.connects or "").split(",") if c.strip()]
        attributes["connects"] = connects
        point_category = "passage_waypoint"

    elif args.type == "rotation_point":
        point_category = "rotation_spot"

    else:
        print(f"ERROR: unknown node_type '{args.type}'", file=sys.stderr)
        sys.exit(1)

    node = {
        "node_id": node_id,
        "name": args.name,
        "node_type": args.type,
        "point_category": point_category,
        "aliases": aliases,
        "tags": tags,
        "attributes": attributes,
        "pose": {
            "x": float(pose.get("x", 0.0)),
            "y": float(pose.get("y", 0.0)),
            "z": float(pose.get("z", 0.0)),
            "yaw": float(pose.get("yaw", 0.0)),
            "q_x": float(pose.get("q_x", 0.0)),
            "q_y": float(pose.get("q_y", 0.0)),
            "q_z": float(pose.get("q_z", 0.0)),
            "q_w": float(pose.get("q_w", 1.0)),
            "name": node_id,
            "speed": float(args.speed),
            "mode": int(args.mode),
        },
        "description": args.description or f"{args.name}。标定于 {time.strftime('%Y-%m-%d %H:%M')}。",
    }
    return node


def main():
    parser = argparse.ArgumentParser(
        description="Create a topology node at current SLAM pose with full attributes."
    )
    parser.add_argument("--map", required=True, dest="map_id",
                        help="Target map_id (e.g. map_701, map_701_left, map_terrace_wc)")
    parser.add_argument("--type", required=True, dest="type",
                        choices=["attributed", "corridor_endpoint", "rotation_point"],
                        help="Node type")
    parser.add_argument("--name", required=True,
                        help="Human-readable name (Chinese OK, e.g. '尹思园工位')")
    parser.add_argument("--node-id", dest="node_id",
                        help="Machine ID (auto-generated from --name if omitted)")
    parser.add_argument("--aliases", default="",
                        help="Comma-separated aliases (e.g. 'yin_siyuan,尹思园')")

    # Attributed node fields
    parser.add_argument("--person", help="Person name (for attributed nodes)")
    parser.add_argument("--area", help="Area description")
    parser.add_argument("--landmark", help="Landmark description")

    # Corridor endpoint fields
    parser.add_argument("--connects", help="Comma-separated connected areas")

    # Common
    parser.add_argument("--tags", default="",
                        help="Extra comma-separated tags")
    parser.add_argument("--description", help="Custom description")
    parser.add_argument("--speed", type=float, default=0.5, help="Nav speed (default 0.5)")
    parser.add_argument("--mode", type=int, default=0, help="Nav mode (default 0)")

    # Runtime
    parser.add_argument("--dry-run", action="store_true", help="Print but don't save")
    parser.add_argument("--timeout", type=int, default=10, help="Gateway timeout seconds")
    args = parser.parse_args()

    # Load registry
    reg = load_registry()

    # Find target map
    map_entry = None
    for m in reg["maps"]:
        if m["map_id"] == args.map_id:
            map_entry = m
            break
    if map_entry is None:
        print(f"ERROR: map '{args.map_id}' not found in registry.", file=sys.stderr)
        sys.exit(1)

    node_id = args.node_id or name_to_node_id(args.name)

    # Check for duplicate
    for n in map_entry.get("topology_nodes", []):
        if n["node_id"] == node_id:
            print(f"WARNING: node_id '{node_id}' already exists in {args.map_id}.", file=sys.stderr)
            print(f"  Use --node-id to specify a different ID, or delete the old one first.", file=sys.stderr)
            sys.exit(1)

    # Get current pose
    print(f"  Map:    {args.map_id}")
    print(f"  Node:   {node_id} ({args.name})")
    print(f"  Type:   {args.type}")
    print(f"  Reading current SLAM pose...")
    try:
        pose = get_current_pose(timeout_s=args.timeout)
    except RuntimeError as e:
        print(f"\n  ERROR: {e}", file=sys.stderr)
        sys.exit(1)

    print(f"  Pose:   x={pose.get('x',0):.4f}, y={pose.get('y',0):.4f}, yaw={pose.get('yaw',0):.4f}")

    if args.dry_run:
        node = build_node(args, pose)
        print(f"\n  --dry-run: would create:\n{json.dumps(node, indent=2, ensure_ascii=False)}")
        return

    # Build and insert node
    node = build_node(args, pose)
    if "topology_nodes" not in map_entry:
        map_entry["topology_nodes"] = []
    map_entry["topology_nodes"].append(node)

    # Save
    save_registry(reg)

    # Summary
    print(f"\n  ✅ Created: {node_id} in {args.map_id}")
    print(f"     Position: ({pose.get('x',0):.3f}, {pose.get('y',0):.3f}) yaw={pose.get('yaw',0):.4f} rad")
    print(f"     Aliases: {node['aliases']}")
    print(f"     Tags: {node['tags']}")
    print(f"\n  Copyable command for re-snap:")
    alias_str = ",".join(node['aliases'][:3])
    type_args = ""
    if args.type == "attributed" and args.person:
        type_args += f" --person '{args.person}'"
    if args.area:
        type_args += f" --area '{args.area}'"
    if args.type == "corridor_endpoint" and args.connects:
        type_args += f" --connects '{args.connects}'"
    print(f"  python3 scripts/snapshot_waypoint.py --map {args.map_id} --type {args.type} "
          f"--name '{args.name}' --node-id {node_id} --aliases '{alias_str}'{type_args}")


if __name__ == "__main__":
    main()
