#!/usr/bin/env python3
"""
snapshot_waypoint.py — Interactive waypoint creation at current SLAM pose.

Workflow:
  1. Read current SLAM pose (Gateway get_world_state)
  2. Choose map/area
  3. Choose node type(s) — can be multiple
  4. Enter Chinese name
  5. Enter type-specific attributes
  6. Review → confirm → save to V2 registry

No placeholders. Every node is created fresh at snapshot time.

Usage:
  python3 scripts/snapshot_waypoint.py              # mark a waypoint
  python3 scripts/snapshot_waypoint.py --transition # record transition anchor + switch PCD
"""

from __future__ import annotations

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

# ── Map definitions ──
MAPS = [
    {"id": "map_701", "label": "701中心区域"},
    {"id": "map_701_left", "label": "701左侧"},
    {"id": "map_terrace_wc", "label": "露台区域"},
]

# ── Node type definitions ──
NODE_TYPES = [
    {
        "id": "attributed",
        "label": "个人节点",
        "desc": "人物工位/办公室/固定位置",
        "point_category": "person_station",
        "attrs": ["person", "area", "landmark"],
        "attr_labels": {"person": "人物姓名", "area": "所属区域", "landmark": "地标描述"},
    },
    {
        "id": "corridor_endpoint",
        "label": "过道中心",
        "desc": "走廊口/通道节点",
        "point_category": "passage_waypoint",
        "attrs": ["connects"],
        "attr_labels": {"connects": "连接区域（逗号分隔，如: 701办公区,外走廊）"},
    },
    {
        "id": "rotation_point",
        "label": "旋转点",
        "desc": "有旋转空间的点",
        "point_category": "rotation_spot",
        "attrs": ["rotation_space"],
        "attr_labels": {"rotation_space": "旋转空间描述（如: 可360°, 仅左转90°）"},
    },
]


def getch(prompt: str = "") -> str:
    """Get single character input."""
    if prompt:
        print(prompt, end="", flush=True)
    try:
        import msvcrt
        return msvcrt.getch().decode().lower()
    except ImportError:
        import tty, termios
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            return sys.stdin.read(1).lower()
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)


def input_str(prompt: str, default: str = "") -> str:
    """Get string input with default."""
    if default:
        result = input(f"{prompt} [{default}]: ").strip()
        return result if result else default
    return input(f"{prompt}: ").strip()


def get_current_pose(timeout_s: int = 10) -> dict:
    """Read current SLAM pose via Gateway get_world_state."""
    import subprocess
    import threading
    import queue

    client = str(REPO_ROOT / "robot" / "slam_gateway_refactor" / "build" / "slam_llm_command_client")
    if not os.path.exists(client):
        client = "robot/slam_gateway_refactor/build/slam_llm_command_client"

    print("  连接 Gateway...")
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
                p.stdin.close()
                p.terminate()
                return pose
        except queue.Empty:
            pass

    p.stdin.close()
    p.terminate()
    raise RuntimeError(
        "\n  ❌ 无法读取 SLAM 位姿。\n"
        "  SLAM 是否已重定位？运行: bash scripts/build_multi_pcd.sh relocate"
    )


def load_registry() -> dict:
    with open(REGISTRY_PATH) as f:
        return json.load(f)


def save_registry(reg: dict):
    with open(REGISTRY_PATH, "w") as f:
        json.dump(reg, f, indent=2, ensure_ascii=False)


def generate_node_id(name: str) -> str:
    """Generate English node_id from Chinese name."""
    # Common translations
    TRANSLATIONS = {
        "尹思园": "yin_siyuan",
        "赵波": "zhao_bo",
        "聂国力": "nie_guoli",
        "陈家宇": "chen_jiayu",
        "工位": "station",
        "办公": "office",
        "办公室": "office",
        "走廊": "corridor",
        "过道": "corridor",
        "入口": "entrance",
        "出口": "exit",
        "露台": "terrace",
        "厕所": "wc",
        "中心": "center",
        "旋转": "rotation",
    }

    # Try to build meaningful English ID
    parts = []
    for char in name:
        if char in TRANSLATIONS:
            parts.append(TRANSLATIONS[char])
        elif char in "，。！？、；：""''（）【】《》 \t\n\r":
            continue  # skip punctuation
        elif '\u4e00' <= char <= '\u9fff':
            continue  # skip untranslated Chinese

    if not parts:
        # Fallback: use pinyin-style from first chars
        import hashlib
        return "wp_" + hashlib.md5(name.encode()).hexdigest()[:6]

    return "_".join(parts)


def node_id_exists(reg: dict, node_id: str) -> bool:
    """Check if node_id already exists anywhere in registry."""
    for m in reg.get("maps", []):
        for n in m.get("topology_nodes", []):
            if n["node_id"] == node_id:
                return True
    return False


def build_node(
    map_id: str,
    name: str,
    types: list[dict],
    attrs: dict,
    pose: dict,
) -> dict:
    """Build a complete topology node entry."""
    node_id_base = generate_node_id(name)

    # Merge attributes from all selected types
    point_categories = []
    all_attrs = {}
    for t in types:
        point_categories.append(t["point_category"])
        for k in t["attrs"]:
            if k in attrs and attrs[k]:
                all_attrs[k] = attrs[k]

    # Generate unique node_id
    node_id = node_id_base
    reg = load_registry()
    counter = 1
    while node_id_exists(reg, node_id):
        counter += 1
        node_id = f"{node_id_base}_{counter}"

    # Aliases: Chinese name + generated ID
    aliases = [name, node_id_base]
    if "person" in all_attrs:
        aliases.append(all_attrs["person"])

    # Tags
    tags = ["real_site", "live_calibrated"]

    # Multi-type: primary type is first selected
    primary_type = types[0]["id"] if types else "attributed"
    primary_category = point_categories[0] if point_categories else "named_location"

    # If multiple types, note in attributes
    if len(types) > 1:
        all_attrs["secondary_types"] = [t["id"] for t in types[1:]]

    node = {
        "node_id": node_id,
        "name": name,
        "node_type": primary_type,
        "point_category": primary_category,
        "aliases": list(dict.fromkeys(aliases)),  # deduplicate
        "tags": tags,
        "attributes": all_attrs,
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
            "speed": 0.5,
            "mode": 0,
        },
        "description": f"{name}。标定于 {time.strftime('%Y-%m-%d %H:%M')}。",
    }
    return node


def review_node(node: dict, map_label: str):
    """Print review and confirm save."""
    print()
    print("  ╔══════════════════════════════════════════╗")
    print("  ║           标 点 审 阅                    ║")
    print("  ╚══════════════════════════════════════════╝")
    print(f"  所属区域:  {map_label}")
    print(f"  节点类型:  {node['node_type']}")
    if "secondary_types" in node.get("attributes", {}):
        print(f"  附加类型:  {', '.join(node['attributes']['secondary_types'])}")
    print(f"  节点 ID:   {node['node_id']}")
    print(f"  中文名:    {node['name']}")
    print(f"  别名:      {', '.join(node['aliases'])}")
    print(f"  位姿:      x={node['pose']['x']:.4f}, y={node['pose']['y']:.4f}, yaw={node['pose']['yaw']:.4f} rad")
    attrs = {k: v for k, v in node.get("attributes", {}).items() if k != "secondary_types"}
    if attrs:
        print(f"  属性:      {json.dumps(attrs, ensure_ascii=False)}")
    print()


def do_transition():
    """Record transition anchor and optionally switch PCD."""
    print()
    print("  ╔══════════════════════════════════════════╗")
    print("  ║     PCD 切换 — 录过渡锚点              ║")
    print("  ╚══════════════════════════════════════════╝")
    print()
    print("  前提: 已重定位到当前 PCD，机器人停在过渡点位置。")
    print()

    # Step 1: Read pose
    print("  [1/4] 读取当前 SLAM 位姿...")
    try:
        pose = get_current_pose()
    except RuntimeError as e:
        print(e, file=sys.stderr)
        sys.exit(1)
    print(f"  ✅ 位姿: x={pose.get('x',0):.4f}, y={pose.get('y',0):.4f}, yaw={pose.get('yaw',0):.4f} rad")
    print()

    # Step 2: Which map are we currently in?
    print("  [2/4] 当前所在 PCD:")
    for i, m in enumerate(MAPS, 1):
        print(f"    {i}) {m['label']}  ({m['id']})")
    while True:
        choice = input_str("  当前地图", "1")
        try:
            idx = int(choice) - 1
            if 0 <= idx < len(MAPS):
                from_map = MAPS[idx]
                break
        except ValueError:
            pass

    # Step 3: Target map
    other_maps = [m for m in MAPS if m["id"] != from_map["id"]]
    print(f"\n  [3/4] 要切换到哪个 PCD:")
    for i, m in enumerate(other_maps, 1):
        print(f"    {i}) {m['label']}  ({m['id']})")
    while True:
        choice = input_str("  目标地图", "1")
        try:
            idx = int(choice) - 1
            if 0 <= idx < len(other_maps):
                to_map = other_maps[idx]
                break
        except ValueError:
            pass
    print(f"  ✅ {from_map['label']} → {to_map['label']}")
    print()

    # Step 4: Record and switch
    print("  [4/4] 记录过渡锚点 & 切换...")
    reg = load_registry()

    # Find from_map entry
    from_entry = None
    to_entry = None
    gw_entry = None
    for m in reg["maps"]:
        if m["map_id"] == from_map["id"]:
            from_entry = m
        if m["map_id"] == to_map["id"]:
            to_entry = m
        if m["map_id"] == "go2w_real_site":
            gw_entry = m

    if not from_entry or not to_entry:
        print("  ❌ 注册表错误：找不到地图条目", file=sys.stderr)
        sys.exit(1)

    # Generate transition anchor ID
    trans_id = f"transition_{from_map['id'].replace('map_','')}_to_{to_map['id'].replace('map_','')}"
    reverse_id = to_entry.get("mapping_origin_anchor_id", f"mapping_origin_{to_map['id']}")

    # Build transition anchor entry
    transition_anchor = {
        "anchor_id": trans_id,
        "name": f"{from_map['label']}→{to_map['label']}过渡",
        "pose": {
            "x": float(pose.get("x", 0.0)),
            "y": float(pose.get("y", 0.0)),
            "z": float(pose.get("z", 0.0)),
            "yaw": float(pose.get("yaw", 0.0)),
            "q_x": float(pose.get("q_x", 0.0)),
            "q_y": float(pose.get("q_y", 0.0)),
            "q_z": float(pose.get("q_z", 0.0)),
            "q_w": float(pose.get("q_w", 1.0)),
            "name": "transition",
            "speed": 0.0,
            "mode": 0,
        },
        "connects_to": to_map["id"],
        "reverse_anchor": reverse_id,
        "note": f"标定于 {time.strftime('%Y-%m-%d %H:%M')}。从 {from_map['id']} 帧记录。",
    }

    # Add/update transition anchor in from_map
    if "transition_anchors" not in from_entry:
        from_entry["transition_anchors"] = []
    # Replace existing if same anchor_id
    from_entry["transition_anchors"] = [
        t for t in from_entry.get("transition_anchors", [])
        if t["anchor_id"] != trans_id
    ]
    from_entry["transition_anchors"].append(transition_anchor)

    # Update go2w_real_site: pcd_path → target PCD, sync anchors
    if gw_entry:
        gw_entry["pcd_path"] = to_entry["pcd_path"]
        # Merge target map's relocalization_anchors into gw
        existing_ids = {a["anchor_id"] for a in gw_entry.get("relocalization_anchors", [])}
        for a in to_entry.get("relocalization_anchors", []):
            if a["anchor_id"] not in existing_ids:
                gw_entry["relocalization_anchors"].append(a)
                existing_ids.add(a["anchor_id"])

    save_registry(reg)

    # Log
    log_path = REPO_ROOT / "artifacts" / "snapshot_log.jsonl"
    log_path.parent.mkdir(parents=True, exist_ok=True)
    with open(log_path, "a") as f:
        f.write(json.dumps({
            "timestamp": time.strftime("%Y-%m-%dT%H:%M:%S"),
            "type": "transition",
            "transition_id": trans_id,
            "from_map": from_map["id"],
            "to_map": to_map["id"],
            "pose": transition_anchor["pose"],
        }, ensure_ascii=False) + "\n")

    print(f"  ✅ 过渡锚点已保存: {trans_id}")
    print(f"     from: {from_map['label']}  ({from_map['id']})")
    print(f"     to:   {to_map['label']}  ({to_map['id']})")
    print(f"     位置: ({pose.get('x',0):.4f}, {pose.get('y',0):.4f})")
    print()

    # Offer to switch now
    print("  ╔══════════════════════════════════════════╗")
    print("  ║  现在切换到目标 PCD？                  ║")
    print("  ║  需要: SLAM 运行 + 机器人不动          ║")
    print("  ╚══════════════════════════════════════════╝")
    print()
    print(f"  切换命令:")
    print(f"    bash scripts/build_multi_pcd.sh relocate --anchor {reverse_id}")
    print()
    switch_now = input_str("  立即执行切换？", "Y").strip().lower()
    if switch_now in ("y", "yes", ""):
        print()
        print("  执行重定位...")
        import subprocess
        o = subprocess.run(
            ["bash", str(REPO_ROOT / "scripts" / "build_multi_pcd.sh"), "relocate", "--anchor", reverse_id],
            cwd=str(REPO_ROOT),
        )
        if o.returncode == 0:
            print(f"\n  ✅ 已切换到 {to_map['label']}！")
            print(f"  现在可以标 {to_map['label']} 的节点:")
            print(f"    python3 scripts/snapshot_waypoint.py")
        else:
            print(f"\n  ⚠ 切换失败 (exit={o.returncode})。手动重试:")
            print(f"    bash scripts/build_multi_pcd.sh relocate --anchor {reverse_id}")
    else:
        print(f"\n  稍后手动切换:")
        print(f"    1. 确保机器人在过渡点不动")
        print(f"    2. bash scripts/build_multi_pcd.sh relocate --anchor {reverse_id}")

    print()


def main():
    import argparse
    parser = argparse.ArgumentParser(description="GO2W topology waypoint tool")
    parser.add_argument("--transition", action="store_true",
                        help="Record transition anchor and switch PCD")
    args, _ = parser.parse_known_args()

    if args.transition:
        do_transition()
        return
    print()
    print("  ╔══════════════════════════════════════════╗")
    print("  ║     GO2W 拓扑标点工具                   ║")
    print("  ╚══════════════════════════════════════════╝")
    print()

    # ── Step 1: Read current pose ──
    print("  [1/5] 读取当前 SLAM 位姿...")
    try:
        pose = get_current_pose()
    except RuntimeError as e:
        print(e, file=sys.stderr)
        sys.exit(1)
    print(f"  ✅ 位姿: x={pose.get('x',0):.4f}, y={pose.get('y',0):.4f}, yaw={pose.get('yaw',0):.4f} rad ({pose.get('yaw',0)*57.3:.1f}°)")
    print()

    # ── Step 2: Choose map ──
    print("  [2/5] 选择所属区域:")
    for i, m in enumerate(MAPS, 1):
        print(f"    {i}) {m['label']}  ({m['id']})")
    while True:
        choice = input_str("  选择", "1")
        try:
            idx = int(choice) - 1
            if 0 <= idx < len(MAPS):
                selected_map = MAPS[idx]
                break
        except ValueError:
            pass
        print(f"  请输入 1-{len(MAPS)}")
    print(f"  ✅ {selected_map['label']}")
    print()

    # ── Step 3: Choose node type(s) ──
    print("  [3/5] 选择节点类型（可多选，如: 1,2）:")
    for i, t in enumerate(NODE_TYPES, 1):
        print(f"    {i}) {t['label']}  — {t['desc']}")
    while True:
        choice = input_str("  选择（逗号分隔）", "1")
        try:
            indices = [int(c.strip()) - 1 for c in choice.split(",")]
            selected_types = []
            for idx in indices:
                if 0 <= idx < len(NODE_TYPES):
                    selected_types.append(NODE_TYPES[idx])
            if selected_types:
                break
        except ValueError:
            pass
        print(f"  请输入 1-{len(NODE_TYPES)}，多选用逗号分隔")
    type_labels = ", ".join(t["label"] for t in selected_types)
    print(f"  ✅ {type_labels}")
    print()

    # ── Step 4: Name ──
    print("  [4/5] 节点命名:")
    node_id_hint = generate_node_id("")
    while True:
        name = input_str("  中文名称").strip()
        if name:
            break
        print("  名称不能为空")
    suggested_id = generate_node_id(name)
    print(f"  (自动生成 node_id: {suggested_id})")
    custom_id = input_str("  node_id（回车确认自动生成）", "").strip()
    if not custom_id:
        custom_id = suggested_id
    print(f"  ✅ 名称: {name}  |  ID: {custom_id}")
    print()

    # ── Step 5: Attributes ──
    print("  [5/5] 填写属性:")
    attrs = {}
    seen_attrs = set()
    for t in selected_types:
        for attr_key in t["attrs"]:
            if attr_key in seen_attrs:
                continue
            seen_attrs.add(attr_key)
            label = t["attr_labels"].get(attr_key, attr_key)
            val = input_str(f"  {label}", "").strip()
            if val:
                attrs[attr_key] = val

    # ── Build node ──
    node = build_node(selected_map["id"], name, selected_types, attrs, pose)
    # Override auto-generated node_id with user's choice
    node["node_id"] = custom_id
    node["pose"]["name"] = custom_id

    # ── Review ──
    review_node(node, selected_map["label"])

    while True:
        confirm = input_str("  保存到注册表？", "Y").strip().lower()
        if confirm in ("y", "yes", ""):
            break
        elif confirm in ("n", "no"):
            print("  ❌ 已取消。")
            return
        print("  请输入 Y/n")

    # ── Save ──
    reg = load_registry()
    for m in reg["maps"]:
        if m["map_id"] == selected_map["id"]:
            if "topology_nodes" not in m:
                m["topology_nodes"] = []
            m["topology_nodes"].append(node)
            break
    save_registry(reg)

    # Append to snapshot log for audit trail
    log_path = REPO_ROOT / "artifacts" / "snapshot_log.jsonl"
    log_path.parent.mkdir(parents=True, exist_ok=True)
    log_entry = {
        "timestamp": time.strftime("%Y-%m-%dT%H:%M:%S"),
        "node_id": custom_id,
        "map_id": selected_map["id"],
        "name": name,
        "types": [t["id"] for t in selected_types],
        "pose": node["pose"],
    }
    with open(log_path, "a") as f:
        f.write(json.dumps(log_entry, ensure_ascii=False) + "\n")

    print(f"\n  ✅ 已保存: {custom_id} → {selected_map['label']}")
    print(f"     位姿: ({node['pose']['x']:.3f}, {node['pose']['y']:.3f}) yaw={node['pose']['yaw']:.4f}")
    print(f"     类型: {type_labels}")
    print()

    # Print remaining nodes in this map for reference
    map_nodes = [n["node_id"] for m in reg["maps"] if m["map_id"] == selected_map["id"]
                 for n in m.get("topology_nodes", [])]
    print(f"  {selected_map['label']} 已标节点 ({len(map_nodes)}):")
    for nid in map_nodes:
        print(f"    - {nid}")
    print()

    # Suggest next
    print("  继续标下一个点？直接运行:")
    print("    python3 scripts/snapshot_waypoint.py")
    print()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\n  已取消。")
        sys.exit(0)
