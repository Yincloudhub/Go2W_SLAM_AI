from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any


class MapRegistryError(ValueError):
    pass


def yaw_to_quaternion(yaw: float) -> tuple[float, float, float, float]:
    half = yaw / 2.0
    return 0.0, 0.0, math.sin(half), math.cos(half)


def quaternion_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


@dataclass(frozen=True)
class UnitreePose:
    """Pose shape used by Unitree SLAM navigation JSON."""
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
        if "yaw" in data and not {"q_x", "q_y", "q_z", "q_w"}.issubset(data):
            q_x, q_y, q_z, q_w = yaw_to_quaternion(float(data["yaw"]))
        else:
            q_x = float(data.get("q_x", 0.0))
            q_y = float(data.get("q_y", 0.0))
            q_z = float(data.get("q_z", 0.0))
            q_w = float(data.get("q_w", 1.0))

        yaw = float(data["yaw"]) if "yaw" in data else quaternion_to_yaw(q_x, q_y, q_z, q_w)
        return cls(
            name=str(data.get("name", default_name)),
            x=float(data.get("x", 0.0)),
            y=float(data.get("y", 0.0)),
            z=float(data.get("z", 0.0)),
            q_x=q_x,
            q_y=q_y,
            q_z=q_z,
            q_w=q_w,
            yaw=yaw,
            speed=float(data.get("speed", 0.5)),
            mode=int(data.get("mode", 0)),
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
    description: str = ""

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "TopologyEdge":
        return cls(
            from_node=str(data["from"]),
            to_node=str(data["to"]),
            bidirectional=bool(data.get("bidirectional", True)),
            expected_distance_m=float(data["expected_distance_m"]) if data.get("expected_distance_m") is not None else None,
            description=str(data.get("description", "")),
        )


@dataclass(frozen=True)
class MapProfile:
    """One PCD map plus semantic nodes and relocalization anchors."""
    map_id: str
    name: str
    pcd_path: str
    topology_path: str
    frame_id: str = "map"
    status: str = "candidate"
    description: str = ""
    rviz_topics: dict[str, str] = field(default_factory=dict)
    pcd_statistics: dict[str, Any] = field(default_factory=dict)
    relocalization_anchors: tuple[RelocalizationAnchor, ...] = field(default_factory=tuple)
    topology_nodes: tuple[TopologyNode, ...] = field(default_factory=tuple)
    topology_edges: tuple[TopologyEdge, ...] = field(default_factory=tuple)

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "MapProfile":
        map_id = str(data["map_id"])
        return cls(
            map_id=map_id,
            name=str(data.get("name", map_id)),
            pcd_path=str(data["pcd_path"]),
            topology_path=str(data.get("topology_path", f"/home/unitree/maps/{map_id}.topology.json")),
            frame_id=str(data.get("frame_id", "map")),
            status=str(data.get("status", "candidate")),
            description=str(data.get("description", "")),
            rviz_topics=dict(data.get("rviz_topics", {})),
            pcd_statistics=dict(data.get("pcd_statistics", {})),
            relocalization_anchors=tuple(RelocalizationAnchor.from_dict(v) for v in data.get("relocalization_anchors", [])),
            topology_nodes=tuple(TopologyNode.from_dict(v) for v in data.get("topology_nodes", [])),
            topology_edges=tuple(TopologyEdge.from_dict(v) for v in data.get("topology_edges", [])),
        )

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
        return {
            "action": "relocate",
            "map_id": self.map_id,
            "map_path": self.pcd_path,
            "anchor_id": anchor.anchor_id,
            "initial_pose": anchor.pose.to_unitree_json(name=anchor.anchor_id, speed=0.0, mode=0),
        }

    def navigate_to_node_command(self, node_id_or_alias: str, *, speed: float | None = None, mode: int | None = None) -> dict[str, Any]:
        """Return a validated high-level command for slam_llm_command_client."""
        node = self.get_node(node_id_or_alias)
        return {
            "action": "navigate_to_pose",
            "map_id": self.map_id,
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
