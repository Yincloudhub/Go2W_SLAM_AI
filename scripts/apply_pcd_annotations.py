from __future__ import annotations

import argparse
import base64
import json
from pathlib import Path
from typing import Any


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_REGISTRY = REPO_ROOT / "configs" / "maps" / "go2w_real_site_map_registry.json"
DEFAULT_CALIBRATION_DIR = REPO_ROOT / "artifacts" / "real_site_pcd"
DEFAULT_MAP_ID = "go2w_real_site"
PROTECTED_TAGS = {"live_calibrated", "live_verified", "ui_verified"}


def zh(text: str) -> str:
    return text.encode("ascii").decode("unicode_escape")


NODE_SPECS: dict[str, dict[str, Any]] = {
    zh(r"\u5c39\u601d\u56ed\u5de5\u4f4d"): {
        "node_id": "yin_siyuan_station",
        "node_type": "annotated_waypoint",
        "aliases": [zh(r"\u5c39\u601d\u56ed\u5de5\u4f4d"), zh(r"\u5c39\u601d\u56ed"), zh(r"\u5c39\u601d\u56ed\u4f4d\u7f6e"), "yin_siyuan_station"],
        "tags": ["real_site", "pcd_annotated", "workstation", "needs_calibration"],
        "description": "annotated on test.pcd; needs live calibration",
    },
    zh(r"\u9648\u5609\u745c\u5de5\u4f4d"): {
        "node_id": "chen_jiayu_station",
        "node_type": "annotated_waypoint",
        "aliases": [zh(r"\u9648\u5609\u745c\u5de5\u4f4d"), zh(r"\u9648\u5609\u745c"), zh(r"\u9648\u5609\u745c\u4f4d\u7f6e"), "chen_jiayu_station"],
        "tags": ["real_site", "pcd_annotated", "workstation", "needs_calibration"],
        "description": "annotated on test.pcd; needs live calibration",
    },
    zh(r"\u6768\u4e66\u6d0b\u5de5\u4f4d"): {
        "node_id": "yang_shuyang_station",
        "node_type": "annotated_waypoint",
        "aliases": [zh(r"\u6768\u4e66\u6d0b\u5de5\u4f4d"), zh(r"\u6768\u4e66\u6d0b"), zh(r"\u6768\u4e66\u6d0b\u4f4d\u7f6e"), "yang_shuyang_station"],
        "tags": ["real_site", "pcd_annotated", "workstation", "needs_calibration"],
        "description": "annotated on test.pcd; needs live calibration",
    },
    zh(r"\u8d75\u535a\u529e\u516c\u5ba4\u95e8\u53e3"): {
        "node_id": "zhao_bo_office_front",
        "node_type": "annotated_waypoint",
        "aliases": [
            zh(r"\u8d75\u535a\u529e\u516c\u5ba4\u95e8\u53e3"),
            zh(r"\u8d75\u535a\u8001\u5e08\u529e\u516c\u5ba4\u95e8\u53e3"),
            zh(r"\u8d75\u535a\u529e\u516c\u5ba4"),
            zh(r"\u8d75\u535a\u95e8\u53e3"),
            zh(r"\u8d75\u535a"),
            zh(r"\u8d75\u535a\u8001\u5e08"),
            "zhao_bo_office_front",
        ],
        "tags": ["real_site", "pcd_annotated", "office", "photo_required", "needs_calibration"],
        "description": "annotated on test.pcd; photo required; needs live calibration",
    },
    zh(r"701\u95e8\u5916\u8d70\u5eca"): {
        "node_id": "room_701_corridor",
        "node_type": "annotated_waypoint",
        "aliases": [zh(r"701\u95e8\u5916\u8d70\u5eca"), zh(r"701\u95e8\u53e3"), zh(r"701\u8d70\u5eca"), "room_701_corridor"],
        "tags": ["real_site", "pcd_annotated", "701", "corridor", "photo_required", "needs_calibration"],
        "description": "annotated on test.pcd; photo required; needs live calibration",
    },
}


def load_annotations(args: argparse.Namespace) -> list[dict[str, Any]]:
    if args.annotations_b64:
        text = base64.b64decode(args.annotations_b64).decode("utf-8")
    elif args.annotations_json:
        text = Path(args.annotations_json).read_text(encoding="utf-8")
    else:
        raise SystemExit("provide --annotations-json or --annotations-b64")
    data = json.loads(text)
    if not isinstance(data, list):
        raise SystemExit("annotation payload must be a JSON array")
    return [item for item in data if isinstance(item, dict)]


def normalized_pose(raw_pose: dict[str, Any], *, default_speed: float, default_mode: int | None) -> dict[str, Any]:
    pose = {
        "x": float(raw_pose["x"]),
        "y": float(raw_pose["y"]),
        "z": float(raw_pose.get("z", 0.0)),
        "q_x": float(raw_pose.get("q_x", 0.0)),
        "q_y": float(raw_pose.get("q_y", 0.0)),
        "q_z": float(raw_pose.get("q_z", 0.0)),
        "q_w": float(raw_pose.get("q_w", 1.0)),
        "speed": float(raw_pose.get("speed", default_speed)),
    }
    pose["mode"] = int(raw_pose.get("mode", default_mode if default_mode is not None else 0))
    return pose


def load_calibrated_node_ids(calibration_dir: Path) -> set[str]:
    calibrated: set[str] = set()
    if not calibration_dir.is_dir():
        return calibrated
    for path in calibration_dir.glob("*_calibration_*.json"):
        try:
            record = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            continue
        node_id = str(record.get("node_id", "")).strip() if isinstance(record, dict) else ""
        confirmed_pose = record.get("confirmed_pose") if isinstance(record, dict) else None
        if node_id and isinstance(confirmed_pose, dict):
            calibrated.add(node_id)
    return calibrated


def protected_node_reason(node: dict[str, Any], calibrated_node_ids: set[str]) -> str | None:
    node_id = str(node.get("node_id", "")).strip()
    if node_id in calibrated_node_ids:
        return "confirmed calibration artifact exists"
    tags = node.get("tags")
    if isinstance(tags, list):
        protected = sorted(PROTECTED_TAGS.intersection(str(tag) for tag in tags))
        if protected:
            return f"protected tags: {', '.join(protected)}"
    if isinstance(node.get("calibration"), dict) or isinstance(node.get("calibration_observation"), dict):
        return "calibration metadata exists"
    return None


def apply_annotations(
    registry: dict[str, Any],
    annotations: list[dict[str, Any]],
    args: argparse.Namespace,
) -> dict[str, list[dict[str, Any]]]:
    target_map = None
    for item in registry.get("maps", []):
        if item.get("map_id") == args.map_id:
            target_map = item
            break
    if target_map is None:
        raise SystemExit(f"map_id not found: {args.map_id}")

    nodes = target_map.setdefault("topology_nodes", [])
    by_id = {node.get("node_id"): node for node in nodes if isinstance(node, dict)}
    result: dict[str, list[dict[str, Any]]] = {"updated": [], "skipped": []}
    seen_ids: set[str] = set()
    calibration_dir = Path(getattr(args, "calibration_dir", DEFAULT_CALIBRATION_DIR))
    calibrated_node_ids = load_calibrated_node_ids(calibration_dir)
    allow_overwrite_verified = bool(getattr(args, "allow_overwrite_verified", False))

    for annotation in annotations:
        name = str(annotation.get("name", "")).strip()
        if not name or name not in NODE_SPECS:
            continue
        spec = NODE_SPECS[name]
        node_id = spec["node_id"]
        if node_id in seen_ids:
            continue
        seen_ids.add(node_id)

        pose = normalized_pose(annotation.get("pose", {}), default_speed=args.default_speed, default_mode=args.mode)
        node = by_id.get(node_id)
        if node is None:
            node = {"node_id": node_id}
            nodes.append(node)
            by_id[node_id] = node
        elif not allow_overwrite_verified:
            reason = protected_node_reason(node, calibrated_node_ids)
            if reason:
                result["skipped"].append(
                    {
                        "node_id": node_id,
                        "name": name,
                        "reason": reason,
                    }
                )
                continue

        node.update(
            {
                "node_id": node_id,
                "name": name,
                "node_type": spec["node_type"],
                "aliases": spec["aliases"],
                "tags": spec["tags"],
                "pose": pose,
                "description": spec["description"],
            }
        )
        result["updated"].append({"node_id": node_id, "name": name, "pose": pose})

    return result


def main() -> int:
    parser = argparse.ArgumentParser(description="Apply UTF-8 PCD annotation JSON to the GO2W real-site map registry.")
    parser.add_argument("--registry", default=str(DEFAULT_REGISTRY))
    parser.add_argument("--map-id", default=DEFAULT_MAP_ID)
    parser.add_argument("--annotations-json", default="")
    parser.add_argument("--annotations-b64", default="")
    parser.add_argument("--default-speed", type=float, default=0.3)
    parser.add_argument("--mode", type=int, default=0)
    parser.add_argument("--calibration-dir", default=str(DEFAULT_CALIBRATION_DIR))
    parser.add_argument(
        "--allow-overwrite-verified",
        action="store_true",
        help="Explicitly allow PCD annotations to replace calibrated or verified nodes.",
    )
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()

    registry_path = Path(args.registry)
    registry = json.loads(registry_path.read_text(encoding="utf-8"))
    annotations = load_annotations(args)
    result = apply_annotations(registry, annotations, args)
    if not result["updated"] and not result["skipped"]:
        raise SystemExit("no known annotations matched; check Chinese names and UTF-8 encoding")

    if not args.dry_run:
        registry_path.write_text(json.dumps(registry, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({**result, "dry_run": bool(args.dry_run)}, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
