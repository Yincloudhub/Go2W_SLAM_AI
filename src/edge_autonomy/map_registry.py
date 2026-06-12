from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any


class MapRegistryError(ValueError):
    pass


def _finite_float(data: dict[str, Any], key: str, *, required: bool, default: float = 0.0) -> float:
    if required and key not in data:
        raise MapRegistryError(f"pose is missing required field '{key}'")
    value = data.get(key, default)
    if isinstance(value, bool):
        raise MapRegistryError(f"pose field '{key}' must be a finite number")
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise MapRegistryError(f"pose field '{key}' must be a finite number") from exc
    if not math.isfinite(number):
        raise MapRegistryError(f"pose field '{key}' must be a finite number")
    return number


def yaw_to_quaternion(yaw: float) -> tuple[float, float, float, float]:
    half = yaw / 2.0
    return 0.0, 0.0, math.sin(half), math.cos(half)


def quaternion_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


@dataclass(frozen=True)
class UnitreePose:
    x: float
    y: float
    z: float = 0.0
    q_x: float = 0.0
    q_y: float = 0.0
    q_z: float = 0.0
    q_w: float = 1.0
    yaw: float | None = None
    name: str = ""
    speed: float = 0.5
    mode: int = 0

    @classmethod
    def from_dict(cls, data: dict[str, Any], *, default_name: str = "") -> "UnitreePose":
        if not isinstance(data, dict):
            raise MapRegistryError("pose must be an object")
        x = _finite_float(data, "x", required=True)
        y = _finite_float(data, "y", required=True)
        z = _finite_float(data, "z", required=False)
        if "yaw" in data and not {"q_x", "q_y", "q_z", "q_w"}.issubset(data):
            yaw = _finite_float(data, "yaw", required=True)
            q_x, q_y, q_z, q_w = yaw_to_quaternion(yaw)
        else:
            q_x = _finite_float(data, "q_x", required=False)
            q_y = _finite_float(data, "q_y", required=False)
            q_z = _finite_float(data, "q_z", required=False)
            q_w = _finite_float(data, "q_w", required=False, default=1.0)
            quaternion_norm = math.sqrt(q_x * q_x + q_y * q_y + q_z * q_z + q_w * q_w)
            if quaternion_norm < 0.5 or quaternion_norm > 1.5:
                raise MapRegistryError("pose quaternion norm must be in [0.5, 1.5]")
            yaw = (
                _finite_float(data, "yaw", required=True)
                if "yaw" in data
                else quaternion_to_yaw(q_x, q_y, q_z, q_w)
            )

        speed = _finite_float(data, "speed", required=False, default=0.5)
        mode_value = data.get("mode", 0)
        if isinstance(mode_value, bool):
            raise MapRegistryError("pose field 'mode' must be 0 or 1")
        try:
            mode = int(mode_value)
        except (TypeError, ValueError) as exc:
            raise MapRegistryError("pose field 'mode' must be 0 or 1") from exc
        if mode not in {0, 1}:
            raise MapRegistryError("pose field 'mode' must be 0 or 1")
        return cls(
            name=str(data.get("name", default_name)),
            x=x,
            y=y,
            z=z,
            q_x=q_x,
            q_y=q_y,
            q_z=q_z,
            q_w=q_w,
            yaw=yaw,
            speed=speed,
            mode=mode,
        )

    def to_unitree_json(self, *, name: str | None = None, speed: float | None = None, mode: int | None = None) -> dict[str, Any]:
        return {
            "name": name if name is not None else self.name,
            "x": self.x,
            "y": self.y,
            "z": self.z,
            "q_x": self.q_x,
            "q_y": self.q_y,
            "q_z": self.q_z,
            "q_w": self.q_w,
            "yaw": self.yaw if self.yaw is not None else quaternion_to_yaw(self.q_x, self.q_y, self.q_z, self.q_w),
            "speed": speed if speed is not None else self.speed,
            "mode": mode if mode is not None else self.mode,
        }


@dataclass(frozen=True)
class RelocalizationAnchor:
    anchor_id: str
    name: str
    pose: UnitreePose
    allowed_radius_m: float = 1.5
    allowed_yaw_error_deg: float = 30.0
    status: str = "candidate"
    description: str = ""

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "RelocalizationAnchor":
        anchor_id = str(data["anchor_id"])
        return cls(
            anchor_id=anchor_id,
            name=str(data.get("name", anchor_id)),
            pose=UnitreePose.from_dict(data.get("pose", {}), default_name=anchor_id),
            allowed_radius_m=float(data.get("allowed_radius_m", 1.5)),
            allowed_yaw_error_deg=float(data.get("allowed_yaw_error_deg", 30.0)),
            status=str(data.get("status", "candidate")),
            description=str(data.get("description", "")),
        )

    @property
    def verified(self) -> bool:
        status = self.status.strip().lower()
        return status == "verified" or status.startswith("verified_")


@dataclass(frozen=True)
class TopologyNode:
    node_id: str
    name: str
    pose: UnitreePose
    node_type: str = "waypoint"
    anchor_id: str | None = None
    aliases: tuple[str, ...] = field(default_factory=tuple)
    tags: tuple[str, ...] = field(default_factory=tuple)
    description: str = ""

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "TopologyNode":
        node_id = str(data["node_id"])
        return cls(
            node_id=node_id,
            name=str(data.get("name", node_id)),
            pose=UnitreePose.from_dict(data.get("pose", {}), default_name=node_id),
            node_type=str(data.get("node_type", "waypoint")),
            anchor_id=str(data["anchor_id"]) if data.get("anchor_id") else None,
            aliases=tuple(str(v) for v in data.get("aliases", [])),
            tags=tuple(str(v) for v in data.get("tags", [])),
            description=str(data.get("description", "")),
        )

    def to_unitree_waypoint(self) -> dict[str, Any]:
        return self.pose.to_unitree_json(name=self.node_id)


@dataclass(frozen=True)
class TopologyEdge:
    from_node: str
    to_node: str
    bidirectional: bool = True
    expected_distance_m: float | None = None
    distance_verified: bool = False
    description: str = ""

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "TopologyEdge":
        return cls(
            from_node=str(data["from"]),
            to_node=str(data["to"]),
            bidirectional=bool(data.get("bidirectional", True)),
            expected_distance_m=float(data["expected_distance_m"]) if data.get("expected_distance_m") is not None else None,
            distance_verified=bool(data.get("distance_verified", False)),
            description=str(data.get("description", "")),
        )


@dataclass(frozen=True)
class MapProfile:
    map_id: str
    name: str
    pcd_path: str
    topology_path: str
    mapping_origin_anchor_id: str = ""
    frame_id: str = "map"
    status: str = "candidate"
    description: str = ""
    rviz_topics: dict[str, str] = field(default_factory=dict)
    pcd_statistics: dict[str, Any] = field(default_factory=dict)
    relocalization_anchors: tuple[RelocalizationAnchor, ...] = field(default_factory=tuple)
    archived_relocalization_anchors: tuple[RelocalizationAnchor, ...] = field(default_factory=tuple)
    topology_nodes: tuple[TopologyNode, ...] = field(default_factory=tuple)
    topology_edges: tuple[TopologyEdge, ...] = field(default_factory=tuple)

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "MapProfile":
        map_id = str(data["map_id"])
        profile = cls(
            map_id=map_id,
            name=str(data.get("name", map_id)),
            pcd_path=str(data["pcd_path"]),
            topology_path=str(data.get("topology_path", f"/home/unitree/maps/{map_id}.topology.json")),
            mapping_origin_anchor_id=str(data.get("mapping_origin_anchor_id", "")),
            frame_id=str(data.get("frame_id", "map")),
            status=str(data.get("status", "candidate")),
            description=str(data.get("description", "")),
            rviz_topics=dict(data.get("rviz_topics", {})),
            pcd_statistics=dict(data.get("pcd_statistics", {})),
            relocalization_anchors=tuple(RelocalizationAnchor.from_dict(v) for v in data.get("relocalization_anchors", [])),
            archived_relocalization_anchors=tuple(
                RelocalizationAnchor.from_dict(v)
                for v in data.get("archived_relocalization_anchors", [])
            ),
            topology_nodes=tuple(TopologyNode.from_dict(v) for v in data.get("topology_nodes", [])),
            topology_edges=tuple(TopologyEdge.from_dict(v) for v in data.get("topology_edges", [])),
        )
        active_ids = [anchor.anchor_id for anchor in profile.relocalization_anchors]
        archived_ids = [anchor.anchor_id for anchor in profile.archived_relocalization_anchors]
        if len(active_ids) != len(set(active_ids)):
            raise MapRegistryError(f"map '{map_id}' has duplicate active relocalization anchor ids")
        if len(archived_ids) != len(set(archived_ids)):
            raise MapRegistryError(f"map '{map_id}' has duplicate archived relocalization anchor ids")
        overlap = sorted(set(active_ids) & set(archived_ids))
        if overlap:
            raise MapRegistryError(
                f"map '{map_id}' has anchors present in both active and archive: {', '.join(overlap)}"
            )
        if profile.status == "real" and not profile.mapping_origin_anchor_id:
            raise MapRegistryError(
                f"real map '{map_id}' must define mapping_origin_anchor_id"
            )
        if (
            profile.mapping_origin_anchor_id
            and profile.mapping_origin_anchor_id not in set(active_ids)
        ):
            raise MapRegistryError(
                f"map '{map_id}' mapping origin anchor "
                f"'{profile.mapping_origin_anchor_id}' is not active"
            )
        topology_ids = [node.node_id for node in profile.topology_nodes]
        if len(topology_ids) != len(set(topology_ids)):
            raise MapRegistryError(f"map '{map_id}' has duplicate topology node ids")
        semantic_terms: dict[str, str] = {}
        for node in profile.topology_nodes:
            for value in (node.node_id, node.name, *node.aliases):
                term = value.strip().casefold()
                if not term:
                    continue
                previous = semantic_terms.get(term)
                if previous is not None and previous != node.node_id:
                    raise MapRegistryError(
                        f"map '{map_id}' has ambiguous topology term '{value}' "
                        f"for nodes '{previous}' and '{node.node_id}'"
                    )
                semantic_terms[term] = node.node_id
        return profile

    def get_anchor(self, anchor_id: str) -> RelocalizationAnchor:
        for anchor in self.relocalization_anchors:
            if anchor.anchor_id == anchor_id:
                return anchor
        raise MapRegistryError(f"unknown anchor_id '{anchor_id}' for map '{self.map_id}'")

    def get_node(self, node_id_or_alias: str) -> TopologyNode:
        for node in self.topology_nodes:
            if node.node_id == node_id_or_alias or node.name == node_id_or_alias or node_id_or_alias in node.aliases:
                return node
        raise MapRegistryError(f"unknown node '{node_id_or_alias}' for map '{self.map_id}'")

    def relocate_command(self, anchor_id: str) -> dict[str, Any]:
        anchor = self.get_anchor(anchor_id)
        if not anchor.verified:
            raise MapRegistryError(
                f"relocalization anchor '{anchor.anchor_id}' is not verified: {anchor.status}"
            )
        return {
            "action": "relocate",
            "operator_ack": True,
            "map_id": self.map_id,
            "map_path": self.pcd_path,
            "anchor_id": anchor.anchor_id,
            "initial_pose": anchor.pose.to_unitree_json(name=anchor.anchor_id, speed=0.0, mode=0),
        }

    def navigate_to_node_command(self, node_id_or_alias: str, *, speed: float | None = None, mode: int | None = None) -> dict[str, Any]:
        node = self.get_node(node_id_or_alias)
        return {
            "action": "navigate_to_pose",
            "map_id": self.map_id,
            "map_path": self.pcd_path,
            "target_node": node.node_id,
            "target_pose": node.pose.to_unitree_json(name=node.node_id, speed=speed, mode=mode),
        }

    def to_unitree_topology_json(self) -> dict[str, Any]:
        return {
            "version": 1,
            "map_id": self.map_id,
            "pcd_path": self.pcd_path,
            "waypoints": [node.to_unitree_waypoint() for node in self.topology_nodes],
        }


@dataclass(frozen=True)
class MapRegistry:
    version: int
    default_map_id: str
    maps: tuple[MapProfile, ...]
    robot: dict[str, Any] = field(default_factory=dict)

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "MapRegistry":
        maps = tuple(MapProfile.from_dict(v) for v in data.get("maps", []))
        if not maps:
            raise MapRegistryError("registry must contain at least one map")
        default_map_id = str(data.get("default_map_id", maps[0].map_id))
        registry = cls(
            version=int(data.get("version", 1)),
            default_map_id=default_map_id,
            maps=maps,
            robot=dict(data.get("robot", {})),
        )
        registry.get_map(default_map_id)
        return registry

    @classmethod
    def from_file(cls, path: str | Path) -> "MapRegistry":
        with Path(path).open("r", encoding="utf-8") as f:
            return cls.from_dict(json.load(f))

    def get_map(self, map_id: str | None = None) -> MapProfile:
        target_id = map_id or self.default_map_id
        for profile in self.maps:
            if profile.map_id == target_id:
                return profile
        raise MapRegistryError(f"unknown map_id '{target_id}'")

    def map_ids(self) -> list[str]:
        return [m.map_id for m in self.maps]
