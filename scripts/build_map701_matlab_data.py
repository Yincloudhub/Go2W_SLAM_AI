from __future__ import annotations

import argparse
import hashlib
import json
import math
import re
from pathlib import Path
from typing import Any


DEFAULT_MARKDOWN = Path(r"C:\Users\c\Desktop\map_701_waypoints.md")
DEFAULT_BASE = Path(r"C:\Users\c\Desktop\map_701_matlab_visualization")
DEFAULT_REGISTRY = (
    DEFAULT_BASE
    / "robot_pull"
    / "home__unitree__Go2W_SLAM_AI__configs__maps__go2w_multi_map_registry_v2.json"
)
DEFAULT_PCD = DEFAULT_BASE / "robot_pull" / "home__unitree__maps__staging__map_701.pcd"
DEFAULT_META = DEFAULT_BASE / "map_701_pcd_topdown_meta.json"
DEFAULT_OUTPUT = DEFAULT_BASE / "map_701_matlab_data.json"
DEFAULT_FLOORPLAN_PNG = DEFAULT_BASE / "map_701_office_floorplan_background.png"
DEFAULT_FLOORPLAN_META = DEFAULT_BASE / "map_701_office_floorplan_meta.json"
DEFAULT_MANUAL_BOUNDARIES = Path(r"C:\Users\c\Desktop\map_701_manual_boundaries.json")


def _read_json_block(markdown: str) -> list[dict[str, Any]]:
    blocks = re.findall(r"```json\s*(.*?)\s*```", markdown, flags=re.S)
    if not blocks:
        raise ValueError("no fenced json block found in waypoint markdown")
    parsed = json.loads(blocks[-1])
    if not isinstance(parsed, list):
        raise ValueError("waypoint json block must be a list")
    return parsed


def _split_types(value: Any) -> list[str]:
    if isinstance(value, list):
        raw = value
    else:
        raw = str(value or "").split(",")
    return [str(item).strip() for item in raw if str(item).strip()]


def _map_by_id(items: list[dict[str, Any]], key: str) -> dict[str, dict[str, Any]]:
    result: dict[str, dict[str, Any]] = {}
    for item in items:
        item_id = str(item.get(key) or "")
        if item_id:
            result[item_id] = item
    return result


def _pose_from_registry(item: dict[str, Any] | None) -> dict[str, float] | None:
    if not item:
        return None
    pose = item.get("pose", {})
    if not isinstance(pose, dict):
        return None
    return {
        "x": float(pose.get("x", 0.0)),
        "y": float(pose.get("y", 0.0)),
        "z": float(pose.get("z", 0.0)),
        "yaw": float(pose.get("yaw", 0.0)),
    }


def _pose_from_doc(item: dict[str, Any]) -> dict[str, float]:
    return {
        "x": float(item.get("x", 0.0)),
        "y": float(item.get("y", 0.0)),
        "z": float(item.get("z", 0.0)),
        "yaw": float(item.get("yaw", 0.0)),
    }


def _yaw_deg(yaw: float) -> float:
    return yaw * 180.0 / math.pi


def _transition_connects_to(anchor_id: str) -> str:
    if anchor_id.endswith("_701_left"):
        return "map_701_left"
    if anchor_id.endswith("_terrace"):
        return "map_terrace_wc"
    return ""


def _build_node(
    doc_item: dict[str, Any],
    registry_node: dict[str, Any] | None,
    transition_anchor: dict[str, Any] | None,
) -> dict[str, Any]:
    node_id = str(doc_item["id"])
    types = _split_types(doc_item.get("type"))
    pose = _pose_from_registry(registry_node)
    if pose is None:
        pose = _pose_from_registry(transition_anchor)
    if pose is None:
        pose = _pose_from_doc(doc_item)

    attrs: dict[str, Any] = {}
    aliases: list[str] = []
    tags: list[str] = ["real_site", "live_calibrated"]
    point_category = "pcd_transition" if "transition" in types else ""
    description = ""
    if registry_node:
        attrs = dict(registry_node.get("attributes", {}))
        aliases = list(registry_node.get("aliases", []))
        tags = list(registry_node.get("tags", tags))
        point_category = str(registry_node.get("point_category") or point_category)
        description = str(registry_node.get("description") or "")
    if transition_anchor:
        attrs.update(
            {
                "connects_to": transition_anchor.get("connects_to")
                or _transition_connects_to(node_id),
                "reverse_anchor": transition_anchor.get("reverse_anchor", ""),
                "note": transition_anchor.get("note", ""),
            }
        )
        aliases = [node_id, str(doc_item.get("name") or "")]
        tags = ["real_site", "live_calibrated", "pcd_transition"]
        description = str(transition_anchor.get("note") or "")

    name = str(doc_item.get("name") or (registry_node or {}).get("name") or node_id)
    if not aliases:
        aliases = [name, node_id]

    return {
        "node_id": node_id,
        "name": name,
        "types": types,
        "primary_type": types[0] if types else "",
        "point_category": point_category,
        "aliases": aliases,
        "tags": tags,
        "attributes": attrs,
        "description": description,
        "x": pose["x"],
        "y": pose["y"],
        "z": pose["z"],
        "yaw": pose["yaw"],
        "yaw_deg": _yaw_deg(pose["yaw"]),
        "speed": 0.5,
        "mode": 0,
        "source": "desktop_markdown_with_registry_precision",
    }


def _flatten_transition_anchor(anchor: dict[str, Any]) -> dict[str, Any]:
    pose = _pose_from_registry(anchor) or {"x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0}
    yaw = float(pose.get("yaw", 0.0))
    return {
        "anchor_id": anchor.get("anchor_id", ""),
        "name": anchor.get("name", ""),
        "connects_to": anchor.get("connects_to", ""),
        "reverse_anchor": anchor.get("reverse_anchor", ""),
        "note": anchor.get("note", ""),
        "x": float(pose.get("x", 0.0)),
        "y": float(pose.get("y", 0.0)),
        "z": float(pose.get("z", 0.0)),
        "yaw": yaw,
        "yaw_deg": _yaw_deg(yaw),
    }


def build_model(args: argparse.Namespace) -> dict[str, Any]:
    markdown_path = Path(args.markdown)
    registry_path = Path(args.registry)
    meta_path = Path(args.meta)
    pcd_path = Path(args.pcd)

    doc_nodes = _read_json_block(markdown_path.read_text(encoding="utf-8"))
    registry = json.loads(registry_path.read_text(encoding="utf-8"))
    meta = json.loads(meta_path.read_text(encoding="utf-8"))
    floorplan_meta_path = Path(args.floorplan_meta)
    floorplan_meta = (
        json.loads(floorplan_meta_path.read_text(encoding="utf-8"))
        if floorplan_meta_path.exists()
        else {}
    )
    manual_path = Path(args.manual_boundaries)
    manual_annotations = (
        json.loads(manual_path.read_text(encoding="utf-8"))
        if manual_path.exists()
        else {"version": 1, "map_id": "map_701", "frame_id": "map", "features": []}
    )

    map701 = next(m for m in registry["maps"] if m.get("map_id") == "map_701")
    registry_nodes = _map_by_id(map701.get("topology_nodes", []), "node_id")
    transition_anchors = _map_by_id(map701.get("transition_anchors", []), "anchor_id")

    nodes = [
        _build_node(
            item,
            registry_nodes.get(str(item.get("id") or "")),
            transition_anchors.get(str(item.get("id") or "")),
        )
        for item in doc_nodes
    ]

    connector_edges = [
        {
            "from": "transition_701_to_701_left",
            "to": "wp_b738c7",
            "label": "to map_701_left",
            "style": "pcd_switch",
        },
        {
            "from": "transition_701_to_terrace",
            "to": "wp_e643bc",
            "label": "to map_terrace_wc",
            "style": "pcd_switch",
        },
    ]
    connector_edges = [
        edge
        for edge in connector_edges
        if any(n["node_id"] == edge["from"] for n in nodes)
        and any(n["node_id"] == edge["to"] for n in nodes)
    ]

    return {
        "map_id": "map_701",
        "map_name": map701.get("name", "map_701"),
        "frame_id": map701.get("frame_id", "map"),
        "build_date": map701.get("build_date", ""),
        "pcd_path": map701.get("pcd_path", ""),
        "mapping_origin_anchor_id": map701.get("mapping_origin_anchor_id", ""),
        "calibration_window": "2026-06-17 15:15-15:27",
        "nodes": nodes,
        "transition_anchors": [
            _flatten_transition_anchor(anchor)
            for anchor in transition_anchors.values()
        ],
        "connector_edges": connector_edges,
        "regions": [
            {
                "id": "room_701_right_office",
                "label": "\u0037\u0030\u0031\u53f3\u4fa7\u529e\u516c\u533a",
                "x_min": -2.3,
                "x_max": 1.9,
                "y_min": -5.5,
                "y_max": 0.2,
                "color": [0.22, 0.42, 0.86],
            },
            {
                "id": "outside_corridor",
                "label": "\u95e8\u5916\u8d70\u5eca",
                "x_min": -6.1,
                "x_max": -4.9,
                "y_min": -0.5,
                "y_max": 0.8,
                "color": [0.14, 0.58, 0.36],
            },
        ],
        "pcd_topdown_png": str(Path(args.pcd_png)),
        "pcd_topdown_meta": meta,
        "floorplan_background_png": str(Path(args.floorplan_png)),
        "floorplan_meta": floorplan_meta,
        "manual_annotations_path": str(manual_path),
        "manual_annotations": manual_annotations,
        "style": {
            "type_colors": {
                "attributed": [0.12, 0.38, 0.78],
                "corridor_endpoint": [0.10, 0.55, 0.32],
                "rotation_point": [0.93, 0.48, 0.13],
                "transition": [0.55, 0.20, 0.74],
            },
            "type_labels": {
                "attributed": "attributed / named place",
                "corridor_endpoint": "corridor endpoint",
                "rotation_point": "rotation point",
                "transition": "PCD transition point",
            },
        },
        "source": {
            "local_markdown": str(markdown_path),
            "remote_host": "unitree@192.168.3.17",
            "remote_registry": "/home/unitree/Go2W_SLAM_AI/configs/maps/go2w_multi_map_registry_v2.json",
            "remote_real_site_symlink": (
                "configs/maps/go2w_real_site_map_registry.json -> "
                "go2w_multi_map_registry_v2.json"
            ),
            "remote_pcd": "/home/unitree/maps/staging/map_701.pcd",
            "pulled_registry": str(registry_path),
            "pulled_pcd": str(pcd_path),
            "pcd_sha256": hashlib.sha256(pcd_path.read_bytes()).hexdigest(),
            "generated_at": "2026-06-17T15:45:00+08:00",
            "source_note": "Remote files were read/pulled only; no remote file was modified.",
        },
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--markdown", default=str(DEFAULT_MARKDOWN))
    parser.add_argument("--registry", default=str(DEFAULT_REGISTRY))
    parser.add_argument("--pcd", default=str(DEFAULT_PCD))
    parser.add_argument("--meta", default=str(DEFAULT_META))
    parser.add_argument("--pcd-png", default=str(DEFAULT_BASE / "map_701_pcd_topdown.png"))
    parser.add_argument("--floorplan-png", default=str(DEFAULT_FLOORPLAN_PNG))
    parser.add_argument("--floorplan-meta", default=str(DEFAULT_FLOORPLAN_META))
    parser.add_argument("--manual-boundaries", default=str(DEFAULT_MANUAL_BOUNDARIES))
    parser.add_argument("--output", default=str(DEFAULT_OUTPUT))
    args = parser.parse_args()

    model = build_model(args)
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(model, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(f"wrote {output}")
    print(f"nodes={len(model['nodes'])} connector_edges={len(model['connector_edges'])}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
