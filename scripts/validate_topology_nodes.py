"""validate_topology_nodes.py — check topology node integrity in go2w_multi_map_registry_v2.json

Checks:
  [1] Every node has pcd_owner field
  [2] pcd_owner maps to an existing map_id in the registry
  [3] Edge from/to node_ids exist in topology_nodes
  [4] Required fields present on every node
  [5] Non-placeholder nodes must not have pose (0,0,0) on all axes
  [6] go2w_real_site nodes must not have pcd_owner == 'go2w_real_site'
  [7] Placeholder nodes have the right tags
"""
import json
import sys
import os

REGISTRY_PATH = os.environ.get(
    "REGISTRY_PATH",
    r"E:\GO2W_0\configs\maps\go2w_multi_map_registry_v2.json"
)

REQUIRED_FIELDS = [
    "node_id", "name", "node_type", "point_category",
    "pcd_owner", "aliases", "tags", "attributes", "pose", "description"
]

POSE_KEYS = {"x", "y", "z", "yaw", "q_x", "q_y", "q_z", "q_w", "name", "speed", "mode"}


def load_registry(path):
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def validate(data):
    errors = []
    warnings = []
    map_ids = {m["map_id"] for m in data["maps"]}
    all_nodes = []  # (map_id, node_dict)

    for map_entry in data["maps"]:
        mid = map_entry["map_id"]
        nodes = map_entry.get("topology_nodes", [])
        edges = map_entry.get("topology_edges", [])

        # Collect node IDs for this map
        node_ids = {n["node_id"] for n in nodes}

        for node in nodes:
            all_nodes.append((mid, node))

            # [1] pcd_owner present?
            if "pcd_owner" not in node:
                errors.append(f"[map={mid}] node '{node.get('node_id','?')}': missing pcd_owner")
            else:
                pcd = node["pcd_owner"]
                # [2] pcd_owner valid?
                if pcd not in map_ids:
                    errors.append(f"[map={mid}] node '{node['node_id']}': pcd_owner '{pcd}' not in registry maps {sorted(map_ids)}")
                # [6] go2w_real_site wrapper check
                if pcd == "go2w_real_site":
                    errors.append(f"[map={mid}] node '{node['node_id']}': pcd_owner is 'go2w_real_site' — wrapper maps cannot own nodes")

            # [4] required fields
            for field in REQUIRED_FIELDS:
                if field not in node:
                    errors.append(f"[map={mid}] node '{node.get('node_id','?')}': missing required field '{field}'")

            # Check pose sub-fields
            if "pose" in node:
                pose = node["pose"]
                for pk in POSE_KEYS:
                    if pk not in pose:
                        errors.append(f"[map={mid}] node '{node['node_id']}': pose missing key '{pk}'")

            # [7] placeholder tag consistency
            tags = node.get("tags", [])
            is_placeholder = "placeholder" in tags
            if is_placeholder and "needs_calibration" not in tags:
                warnings.append(f"[map={mid}] node '{node['node_id']}': tagged 'placeholder' but missing 'needs_calibration'")
            if not is_placeholder and "needs_calibration" in tags:
                warnings.append(f"[map={mid}] node '{node['node_id']}': has 'needs_calibration' but not 'placeholder'")

            # [5] non-placeholder pose all-zero check
            if not is_placeholder and "pose" in node:
                pose = node["pose"]
                if pose.get("x") == 0.0 and pose.get("y") == 0.0 and pose.get("z") == 0.0:
                    errors.append(f"[map={mid}] node '{node['node_id']}': non-placeholder has all-zero pose — needs real coordinates")

        # [3] Edge references
        for edge in edges:
            for endpoint in ("from", "to"):
                ref = edge.get(endpoint)
                if ref is None:
                    errors.append(f"[map={mid}] edge missing '{endpoint}' key: {edge}")
                elif ref not in node_ids:
                    errors.append(f"[map={mid}] edge '{endpoint}': '{ref}' not found in topology_nodes (existing: {sorted(node_ids)})")

    # Summary
    print(f"Registry: {REGISTRY_PATH}")
    print(f"Maps: {len(data['maps'])}  |  Total topology nodes: {len(all_nodes)}")
    print(f"Map IDs: {sorted(map_ids)}")
    print()

    if errors:
        print(f"❌ ERRORS: {len(errors)}")
        for e in errors:
            print(f"  • {e}")
    else:
        print("✅ No errors.")

    if warnings:
        print(f"\n⚠️  WARNINGS: {len(warnings)}")
        for w in warnings:
            print(f"  • {w}")

    print()
    # Per-map node counts
    for map_entry in data["maps"]:
        mid = map_entry["map_id"]
        nodes = map_entry.get("topology_nodes", [])
        edges = map_entry.get("topology_edges", [])
        n_placeholder = sum(1 for n in nodes if "placeholder" in n.get("tags", []))
        print(f"  {mid}: {len(nodes)} nodes ({n_placeholder} placeholder), {len(edges)} edges")

    return len(errors) == 0


def main():
    if not os.path.exists(REGISTRY_PATH):
        print(f"❌ Registry not found: {REGISTRY_PATH}")
        sys.exit(1)

    data = load_registry(REGISTRY_PATH)
    ok = validate(data)
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
