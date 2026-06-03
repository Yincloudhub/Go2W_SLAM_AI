#!/usr/bin/env python3
from __future__ import annotations

import argparse
import hashlib
import json
import os
import sys
from pathlib import Path
from typing import Any


REAL_SITE_REGISTRY = Path("configs/maps/go2w_real_site_map_registry.json")
EXPECTED_MAP_ID = "go2w_real_site"
EXPECTED_PCD_PATH = "/home/unitree/test.pcd"
EXPECTED_TOPOLOGY_PATH = "/home/unitree/topology_points.json"


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def load_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def find_map(registry: dict[str, Any], map_id: str) -> dict[str, Any] | None:
    maps = registry.get("maps", [])
    if not isinstance(maps, list):
        return None
    for item in maps:
        if isinstance(item, dict) and item.get("map_id") == map_id:
            return item
    return None


def describe_registry(path: Path) -> dict[str, Any]:
    registry = load_json(path)
    map_profile = find_map(registry, EXPECTED_MAP_ID)
    if map_profile is None:
        raise ValueError(f"{EXPECTED_MAP_ID} missing from {path}")
    return {
        "path": str(path),
        "resolved_path": str(path.resolve()),
        "is_symlink": path.is_symlink(),
        "sha256": sha256(path),
        "default_map_id": registry.get("default_map_id"),
        "map_id": map_profile.get("map_id"),
        "status": map_profile.get("status"),
        "pcd_path": map_profile.get("pcd_path"),
        "topology_path": map_profile.get("topology_path"),
        "topology_nodes": len(map_profile.get("topology_nodes", [])),
        "relocalization_anchors": len(map_profile.get("relocalization_anchors", [])),
    }


def default_active_repo() -> Path:
    configured = os.environ.get("GO2W_ACTIVE_REPO")
    if configured:
        return Path(configured)
    robot_repo = Path("/home/unitree/Go2W_SLAM_AI")
    if robot_repo.exists():
        return robot_repo
    return Path(__file__).resolve().parents[1]


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Check GO2W runtime repo and map registry ownership.")
    parser.add_argument("--active-repo", default=str(default_active_repo()))
    parser.add_argument("--legacy-repo", default=os.environ.get("GO2W_LEGACY_REPO", "/home/unitree/go2w_slam_agent"))
    parser.add_argument("--json", action="store_true", help="Print machine-readable JSON.")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    active_repo = Path(args.active_repo).expanduser()
    legacy_repo = Path(args.legacy_repo).expanduser()
    active_registry = active_repo / REAL_SITE_REGISTRY
    legacy_registry = legacy_repo / REAL_SITE_REGISTRY
    errors: list[str] = []
    warnings: list[str] = []

    if not active_registry.exists():
        errors.append(f"active registry missing: {active_registry}")
        result = {"active_repo": str(active_repo), "errors": errors, "warnings": warnings}
        print(json.dumps(result, ensure_ascii=False, indent=2))
        return 2

    active = describe_registry(active_registry)
    if active["default_map_id"] != EXPECTED_MAP_ID:
        errors.append(f"active default_map_id is {active['default_map_id']!r}")
    if active["pcd_path"] != EXPECTED_PCD_PATH:
        errors.append(f"active pcd_path is {active['pcd_path']!r}")
    if active["topology_path"] != EXPECTED_TOPOLOGY_PATH:
        errors.append(f"active topology_path is {active['topology_path']!r}")

    legacy: dict[str, Any] | None = None
    if legacy_repo.exists():
        if not legacy_registry.exists():
            errors.append(f"legacy repo exists but registry is missing: {legacy_registry}")
        else:
            legacy = describe_registry(legacy_registry)
            if legacy["resolved_path"] != active["resolved_path"]:
                errors.append(
                    "legacy real-site registry is not the same physical file as active registry: "
                    f"{legacy_registry} -> {legacy['resolved_path']}"
                )
            if not legacy["is_symlink"]:
                warnings.append(f"legacy registry is not a symlink: {legacy_registry}")

    result = {
        "active_repo": str(active_repo),
        "legacy_repo": str(legacy_repo),
        "active_registry": active,
        "legacy_registry": legacy,
        "errors": errors,
        "warnings": warnings,
    }
    if args.json:
        print(json.dumps(result, ensure_ascii=False, indent=2))
    else:
        print(f"active_repo={active_repo}")
        print(f"active_registry={active_registry}")
        print(f"active_registry_sha256={active['sha256']}")
        print(
            "active_map="
            f"{active['map_id']} status={active['status']} "
            f"pcd={active['pcd_path']} topology={active['topology_path']} "
            f"nodes={active['topology_nodes']} anchors={active['relocalization_anchors']}"
        )
        if legacy is not None:
            print(f"legacy_repo={legacy_repo}")
            print(f"legacy_registry_resolved={legacy['resolved_path']}")
            print(f"legacy_registry_symlink={legacy['is_symlink']}")
        for warning in warnings:
            print(f"warning: {warning}", file=sys.stderr)
        for error in errors:
            print(f"error: {error}", file=sys.stderr)
    return 0 if not errors else 2


if __name__ == "__main__":
    raise SystemExit(main())
