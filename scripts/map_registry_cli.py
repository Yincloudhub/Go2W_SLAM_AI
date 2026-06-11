from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any

REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = REPO_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from edge_autonomy.map_registry import MapRegistry, MapRegistryError  # noqa: E402


DEFAULT_REGISTRY = REPO_ROOT / "configs" / "maps" / "go2w_map_registry.example.json"


def print_json(data: dict[str, Any], *, pretty: bool) -> None:
    if pretty:
        print(json.dumps(data, ensure_ascii=False, indent=2))
    else:
        print(json.dumps(data, ensure_ascii=False, separators=(",", ":")))


def load_registry(path: str | Path) -> MapRegistry:
    return MapRegistry.from_file(path)


def command_list(args: argparse.Namespace) -> int:
    registry = load_registry(args.registry)
    print(f"default_map_id: {registry.default_map_id}")
    for profile in registry.maps:
        print(f"- {profile.map_id}: {profile.name}")
        print(f"  pcd: {profile.pcd_path}")
        print(f"  topology: {profile.topology_path}")
        print(f"  mapping origin anchor: {profile.mapping_origin_anchor_id or '(unset)'}")
        print(f"  anchors: {', '.join(a.anchor_id for a in profile.relocalization_anchors) or '(none)'}")
        print(
            "  archived anchors: "
            f"{', '.join(a.anchor_id for a in profile.archived_relocalization_anchors) or '(none)'}"
        )
        print(f"  nodes: {', '.join(n.node_id for n in profile.topology_nodes) or '(none)'}")
    return 0


def command_show(args: argparse.Namespace) -> int:
    registry = load_registry(args.registry)
    profile = registry.get_map(args.map_id)
    print_json(
        {
            "map_id": profile.map_id,
            "name": profile.name,
            "status": profile.status,
            "pcd_path": profile.pcd_path,
            "topology_path": profile.topology_path,
            "mapping_origin_anchor_id": profile.mapping_origin_anchor_id,
            "frame_id": profile.frame_id,
            "description": profile.description,
            "rviz_topics": profile.rviz_topics,
            "pcd_statistics": profile.pcd_statistics,
            "relocalization_anchors": [
                {
                    "anchor_id": a.anchor_id,
                    "name": a.name,
                    "status": a.status,
                    "allowed_radius_m": a.allowed_radius_m,
                    "allowed_yaw_error_deg": a.allowed_yaw_error_deg,
                    "pose": a.pose.to_unitree_json(name=a.anchor_id),
                    "description": a.description,
                }
                for a in profile.relocalization_anchors
            ],
            "archived_relocalization_anchors": [
                {
                    "anchor_id": a.anchor_id,
                    "name": a.name,
                    "status": a.status,
                    "allowed_radius_m": a.allowed_radius_m,
                    "allowed_yaw_error_deg": a.allowed_yaw_error_deg,
                    "pose": a.pose.to_unitree_json(name=a.anchor_id),
                    "description": a.description,
                }
                for a in profile.archived_relocalization_anchors
            ],
            "topology_nodes": [
                {
                    "node_id": n.node_id,
                    "name": n.name,
                    "node_type": n.node_type,
                    "anchor_id": n.anchor_id,
                    "aliases": list(n.aliases),
                    "tags": list(n.tags),
                    "pose": n.pose.to_unitree_json(name=n.node_id),
                    "description": n.description,
                }
                for n in profile.topology_nodes
            ],
            "topology_edges": [
                {
                    "from": e.from_node,
                    "to": e.to_node,
                    "bidirectional": e.bidirectional,
                    "expected_distance_m": e.expected_distance_m,
                    "description": e.description,
                }
                for e in profile.topology_edges
            ],
        },
        pretty=True,
    )
    return 0


def command_relocate(args: argparse.Namespace) -> int:
    registry = load_registry(args.registry)
    command = registry.get_map(args.map_id).relocate_command(args.anchor_id)
    print_json(command, pretty=args.pretty)
    return 0


def command_navigate_node(args: argparse.Namespace) -> int:
    registry = load_registry(args.registry)
    command = registry.get_map(args.map_id).navigate_to_node_command(args.node_id, speed=args.speed, mode=args.mode)
    print_json(command, pretty=args.pretty)
    return 0


def command_export_topology(args: argparse.Namespace) -> int:
    registry = load_registry(args.registry)
    topology = registry.get_map(args.map_id).to_unitree_topology_json()
    text = json.dumps(topology, ensure_ascii=False, indent=2)
    if args.output:
        Path(args.output).write_text(text + "\n", encoding="utf-8")
    else:
        print(text)
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="GO2W map registry and topology command helper.")
    parser.add_argument("--registry", default=str(DEFAULT_REGISTRY), help="Path to map registry JSON.")
    subparsers = parser.add_subparsers(dest="command", required=True)

    list_parser = subparsers.add_parser("list", help="List configured maps, anchors, and topology nodes.")
    list_parser.set_defaults(func=command_list)

    show_parser = subparsers.add_parser("show", help="Show one map profile.")
    show_parser.add_argument("--map-id", required=True)
    show_parser.set_defaults(func=command_show)

    relocate_parser = subparsers.add_parser("relocate", help="Emit one JSON line for slam_llm_command_client relocation.")
    relocate_parser.add_argument("--map-id", required=True)
    relocate_parser.add_argument("--anchor-id", required=True)
    relocate_parser.add_argument("--pretty", action="store_true")
    relocate_parser.set_defaults(func=command_relocate)

    nav_parser = subparsers.add_parser("navigate-node", help="Emit one JSON line for navigating to a semantic topology node.")
    nav_parser.add_argument("--map-id", required=True)
    nav_parser.add_argument("--node-id", required=True)
    nav_parser.add_argument("--speed", type=float, help="Override node speed in m/s. Use 0.35 for normal indoor tests after safety validation.")
    nav_parser.add_argument("--mode", type=int, help="Override Unitree navigation mode. Current project default uses mode=0.")
    nav_parser.add_argument("--pretty", action="store_true")
    nav_parser.set_defaults(func=command_navigate_node)

    export_parser = subparsers.add_parser("export-topology", help="Export topology nodes in Unitree topology_points.json shape.")
    export_parser.add_argument("--map-id", required=True)
    export_parser.add_argument("--output", help="Optional output path. Stdout is used if omitted.")
    export_parser.set_defaults(func=command_export_topology)
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    try:
        return args.func(args)
    except MapRegistryError as exc:
        print(f"map registry error: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
