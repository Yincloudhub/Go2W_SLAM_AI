from __future__ import annotations

import argparse
import json
import tempfile
import time
import unittest
from pathlib import Path
from unittest.mock import patch

from scripts import go2w_supervised_acceptance as acceptance


def registry(root: Path, *, anchor_status: str = "verified_startup_relocalization_20260601") -> Path:
    path = root / "registry.json"
    path.write_text(
        json.dumps(
            {
                "default_map_id": "go2w_real_site",
                "maps": [
                    {
                        "map_id": "go2w_real_site",
                        "name": "real",
                        "status": "real",
                        "pcd_path": "/home/unitree/test.pcd",
                        "topology_path": "/home/unitree/topology_points.json",
                        "mapping_origin_anchor_id": "mapping_origin",
                        "relocalization_anchors": [
                            {
                                "anchor_id": "mapping_origin",
                                "status": anchor_status,
                                "allowed_radius_m": 1.5,
                                "allowed_yaw_error_deg": 30.0,
                                "pose": {"x": 0.0, "y": 0.0, "yaw": 0.0},
                            }
                        ],
                        "topology_nodes": [
                            {
                                "node_id": "safe_target",
                                "name": "Safe",
                                "tags": ["live_verified"],
                                "pose": {"x": 1.0, "y": 0.0, "yaw": 0.0},
                            },
                            {
                                "node_id": "blocked_target",
                                "name": "Blocked",
                                "tags": ["needs_calibration"],
                                "pose": {"x": 2.0, "y": 0.0, "yaw": 0.0},
                            },
                        ],
                        "topology_edges": [],
                    }
                ],
            }
        ),
        encoding="utf-8",
    )
    return path


def write_lidar_summary(root: Path, *, calibrated: bool = True) -> Path:
    path = root / "lidar.json"
    path.write_text(
        json.dumps(
            {
                "source": "lidar_pointcloud",
                "timestamp_ms": int(time.time() * 1000),
                "stale": not calibrated,
                "stale_reasons": [] if calibrated else ["uncalibrated_xt16_geometry"],
                "recommended_action": "normal",
                "blocked_directions": [],
                "front_clearance_m": 2.0,
                "left_clearance_m": 2.0,
                "right_clearance_m": 2.0,
                "rear_clearance_m": 2.0,
                "summary": {
                    "calibrated": calibrated,
                    "calibration_id": "test-calibration" if calibrated else None,
                },
            }
        ),
        encoding="utf-8",
    )
    return path


def localized_world(*, timestamp_ms: int = 1000, navigation_allowed: bool = True) -> dict:
    return {
        "accepted": True,
        "action": "get_world_state",
        "world_state": {
            "timestamp_ms": timestamp_ms,
            "current_pose": {
                "timestamp_ms": timestamp_ms,
                "map_id": "test",
                "map_path": "/home/unitree/test.pcd",
                "pose": {"x": 0.1, "y": 0.0, "yaw": 0.05},
            },
            "localization": {
                "status": "localized",
                "pose_age_ms": 100,
                "confidence": 0.9,
                "map_id": "test",
                "map_path": "/home/unitree/test.pcd",
            },
            "slam_health": {
                "status": "ok",
                "slam_alive": True,
                "localization_alive": True,
            },
            "local_obstacle": {
                "source": "lidar_pointcloud",
                "stale": False,
                "age_ms": 100,
                "recommended_action": "normal",
                "front_clearance_m": 2.0,
                "left_clearance_m": 2.0,
                "right_clearance_m": 2.0,
                "front_confidence": 1.0,
                "left_confidence": 1.0,
                "right_confidence": 1.0,
            },
            "safety": {
                "allow_navigation": navigation_allowed,
                "reason": "ok" if navigation_allowed else "blocked",
                "recommended_mode": "normal",
            },
            "navigation": {"state": "idle", "target_node": "", "is_arrived": False},
        },
    }


def args(path: Path, **overrides) -> argparse.Namespace:
    values = {
        "registry": str(path),
        "map_id": "go2w_real_site",
        "anchor": "mapping_origin",
        "target": "",
        "confirm_relocation": "",
        "samples": 2,
        "interval_s": 0.0,
        "relocation_settle_s": 0.0,
        "max_pose_age_ms": 500.0,
        "max_obstacle_age_ms": 1500.0,
        "nav_speed_mps": 0.1,
        "gateway_client": "gateway",
        "network_interface": "eth0",
        "timeout_s": 1,
        "gateway_startup_wait_s": 0.0,
        "lidar_summary": str(path.parent / "lidar.json"),
        "llm_model": str(path.parent / "missing.gguf"),
        "llm_ask_script": str(path.parent / "missing.sh"),
    }
    values.update(overrides)
    return argparse.Namespace(**values)


class FakePersistentGatewaySession:
    def __init__(self, samples: list[dict]) -> None:
        self.samples = list(samples)
        self.commands: list[dict] = []

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, traceback):
        return None

    def command(self, command: dict) -> dict:
        self.commands.append(command)
        return self.samples.pop(0)


class SupervisedAcceptanceTests(unittest.TestCase):
    @staticmethod
    def runtime_ready() -> dict:
        return {
            "ready": True,
            "processes": {"xt16_driver": True, "unitree_slam": True},
            "errors": {},
        }

    def test_relocation_requires_exact_confirmation(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            config = args(registry(Path(temp)))
            with patch.object(acceptance, "runtime_process_status", return_value=self.runtime_ready()):
                with patch.object(
                    acceptance,
                    "world_state",
                    return_value={"accepted": True, "world_state": {"localization": {"status": "not_started"}}},
                ):
                    code, payload = acceptance.relocate_stage(config)
            self.assertEqual(code, 2)
            self.assertFalse(payload["accepted"])
            self.assertFalse(payload["motion_commands_sent"])

    def test_unverified_anchor_is_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            config = args(registry(Path(temp), anchor_status="candidate"), confirm_relocation="mapping_origin")
            with patch.object(acceptance, "runtime_process_status", return_value=self.runtime_ready()):
                code, payload = acceptance.relocate_stage(config)
            self.assertEqual(code, 3)
            self.assertIn("not verified", payload["reason"])

    def test_confirmed_relocation_runs_continuous_localization_check(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            config = args(registry(Path(temp)), confirm_relocation="mapping_origin")
            not_started = {"accepted": True, "world_state": {"localization": {"status": "not_started"}}}
            samples = [localized_world(timestamp_ms=1000), localized_world(timestamp_ms=2000)]
            session = FakePersistentGatewaySession(samples)
            with patch.object(acceptance, "runtime_process_status", return_value=self.runtime_ready()):
                with patch.object(acceptance, "world_state", return_value=not_started):
                    with patch.object(
                        acceptance,
                        "run_gateway_command",
                        return_value={"accepted": True},
                    ) as gateway:
                        with patch.object(
                            acceptance,
                            "PersistentGatewaySession",
                            return_value=session,
                        ):
                            code, payload = acceptance.relocate_stage(config)
            self.assertEqual(code, 0)
            self.assertTrue(payload["localization_verified"])
            self.assertFalse(payload["motion_commands_sent"])
            self.assertEqual(gateway.call_args.args[0]["action"], "relocate")
            self.assertEqual(
                session.commands,
                [{"action": "get_world_state"}, {"action": "get_world_state"}],
            )

    def test_prepare_navigation_never_sends_motion(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            write_lidar_summary(root)
            config = args(registry(root), target="safe_target")
            with patch.object(acceptance, "runtime_process_status", return_value=self.runtime_ready()):
                with patch.object(acceptance, "world_state", return_value=localized_world()):
                    code, payload = acceptance.prepare_navigation_stage(config)
            self.assertEqual(code, 0)
            self.assertTrue(payload["ready"])
            self.assertFalse(payload["motion_commands_sent"])
            self.assertIn("--nav-speed-mps 0.10", payload["supervised_motion_command"])
            self.assertIn("--no-auto-relocate", payload["supervised_motion_command"])

    def test_prepare_navigation_does_not_require_robot_to_remain_near_relocation_anchor(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            write_lidar_summary(root)
            state = localized_world()
            state["world_state"]["current_pose"]["pose"] = {
                "x": 8.0,
                "y": -4.0,
                "yaw": 1.2,
            }
            config = args(registry(root), target="safe_target", anchor="")
            with patch.object(acceptance, "runtime_process_status", return_value=self.runtime_ready()):
                with patch.object(acceptance, "world_state", return_value=state):
                    code, payload = acceptance.prepare_navigation_stage(config)
            self.assertEqual(code, 0)
            self.assertTrue(payload["ready"])
            self.assertNotIn("anchor_check", payload)

    def test_status_lists_origin_and_relocation_choices_without_default_command(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            path = registry(root)
            config = args(path, anchor="")
            with patch.object(acceptance, "runtime_process_status", return_value=self.runtime_ready()):
                with patch.object(acceptance, "world_state", return_value=localized_world()):
                    code, payload = acceptance.status_stage(config)
            self.assertEqual(code, 0)
            self.assertEqual(payload["build_map_origin"], "mapping_origin")
            self.assertEqual(
                [item["anchor_id"] for item in payload["active_relocalization_anchors"]],
                ["mapping_origin"],
            )
            self.assertTrue(payload["active_relocalization_anchors"][0]["is_build_map_origin"])
            self.assertIsNone(payload["next_command"])

    def test_relocation_requires_explicit_anchor(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            config = args(registry(Path(temp)), anchor="")
            with patch.object(acceptance, "runtime_process_status", return_value=self.runtime_ready()):
                code, payload = acceptance.relocate_stage(config)
            self.assertEqual(code, 2)
            self.assertIn("--anchor is required", payload["reason"])

    def test_localization_verification_requires_explicit_anchor(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            config = args(registry(Path(temp)), anchor="")
            code, payload = acceptance.verify_localization_stage(config)
            self.assertEqual(code, 2)
            self.assertIn("--anchor is required", payload["reason"])

    def test_prepare_navigation_rejects_unverified_target(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            write_lidar_summary(root)
            config = args(registry(root), target="blocked_target")
            with patch.object(acceptance, "runtime_process_status", return_value=self.runtime_ready()):
                with patch.object(acceptance, "world_state", return_value=localized_world()):
                    code, payload = acceptance.prepare_navigation_stage(config)
            self.assertEqual(code, 6)
            self.assertFalse(payload["ready"])
            self.assertIn("needs_calibration", payload["target"]["blocking_tags"])
            self.assertIsNone(payload["supervised_motion_command"])

    def test_unverified_prefix_does_not_count_as_verified(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            config = args(
                registry(Path(temp), anchor_status="unverified_startup_relocalization"),
                confirm_relocation="mapping_origin",
            )
            with patch.object(acceptance, "runtime_process_status", return_value=self.runtime_ready()):
                code, payload = acceptance.relocate_stage(config)
            self.assertEqual(code, 3)
            self.assertFalse(payload["accepted"])

    def test_missing_pose_timestamps_fail_continuous_verification(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            config = args(registry(Path(temp)))
            samples = [localized_world(timestamp_ms=1000), localized_world(timestamp_ms=2000)]
            for sample in samples:
                del sample["world_state"]["current_pose"]["timestamp_ms"]
            profile = acceptance.load_profile(config)
            anchor = profile.get_anchor("mapping_origin")
            verified, details = acceptance.verify_samples(
                config,
                profile,
                anchor,
                get_world=lambda: samples.pop(0),
            )
            self.assertFalse(verified)
            self.assertTrue(all(not item["timestamp_advanced"] for item in details))

    def test_relocation_verification_requires_pose_newer_than_gateway_response(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            config = args(registry(Path(temp)))
            samples = [
                localized_world(timestamp_ms=1000),
                localized_world(timestamp_ms=2000),
            ]
            profile = acceptance.load_profile(config)
            anchor = profile.get_anchor("mapping_origin")

            verified, details = acceptance.verify_samples(
                config,
                profile,
                anchor,
                get_world=lambda: samples.pop(0),
                minimum_timestamp_ms=1500,
            )

            self.assertFalse(verified)
            self.assertFalse(details[0]["timestamp_advanced"])
            self.assertTrue(details[1]["timestamp_advanced"])

    def test_uncalibrated_lidar_blocks_navigation_command(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            write_lidar_summary(root, calibrated=False)
            config = args(registry(root), target="safe_target")
            with patch.object(acceptance, "runtime_process_status", return_value=self.runtime_ready()):
                with patch.object(acceptance, "world_state", return_value=localized_world()):
                    code, payload = acceptance.prepare_navigation_stage(config)
            self.assertEqual(code, 6)
            self.assertFalse(payload["ready"])
            self.assertIn("not calibrated", payload["reason"])
            self.assertIsNone(payload["supervised_motion_command"])

    def test_wrong_active_map_blocks_navigation_command(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            write_lidar_summary(root)
            state = localized_world()
            state["world_state"]["current_pose"]["map_id"] = "wrong"
            state["world_state"]["current_pose"]["map_path"] = "/home/unitree/wrong.pcd"
            config = args(registry(root), target="safe_target")
            with patch.object(acceptance, "runtime_process_status", return_value=self.runtime_ready()):
                with patch.object(acceptance, "world_state", return_value=state):
                    code, payload = acceptance.prepare_navigation_stage(config)
            self.assertEqual(code, 6)
            self.assertFalse(payload["map_identity"]["matches"])
            self.assertIsNone(payload["supervised_motion_command"])

    def test_matching_map_id_cannot_override_wrong_reported_map_path(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            write_lidar_summary(root)
            state = localized_world()
            state["world_state"]["current_pose"]["map_id"] = "test"
            state["world_state"]["current_pose"]["map_path"] = "/home/unitree/wrong.pcd"
            state["world_state"]["localization"]["map_path"] = "/home/unitree/wrong.pcd"
            config = args(registry(root), target="safe_target")
            with patch.object(acceptance, "runtime_process_status", return_value=self.runtime_ready()):
                with patch.object(acceptance, "world_state", return_value=state):
                    code, payload = acceptance.prepare_navigation_stage(config)
            self.assertEqual(code, 6)
            self.assertFalse(payload["map_identity"]["matches"])
            self.assertIn("path does not match", payload["map_identity"]["reason"])
            self.assertIsNone(payload["supervised_motion_command"])

    def test_conflicting_pose_and_localization_map_paths_fail_closed(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            write_lidar_summary(root)
            state = localized_world()
            state["world_state"]["localization"]["map_path"] = "/home/unitree/wrong.pcd"
            config = args(registry(root), target="safe_target")
            with patch.object(acceptance, "runtime_process_status", return_value=self.runtime_ready()):
                with patch.object(acceptance, "world_state", return_value=state):
                    code, payload = acceptance.prepare_navigation_stage(config)
            self.assertEqual(code, 6)
            self.assertFalse(payload["map_identity"]["matches"])
            self.assertIsNone(payload["supervised_motion_command"])

    def test_correct_map_path_cannot_override_conflicting_map_id(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            write_lidar_summary(root)
            state = localized_world()
            state["world_state"]["localization"]["map_id"] = "wrong"
            config = args(registry(root), target="safe_target")
            with patch.object(acceptance, "runtime_process_status", return_value=self.runtime_ready()):
                with patch.object(acceptance, "world_state", return_value=state):
                    code, payload = acceptance.prepare_navigation_stage(config)
            self.assertEqual(code, 6)
            self.assertFalse(payload["map_identity"]["matches"])
            self.assertEqual(
                payload["map_identity"]["conflicting_map_ids"],
                {"localization": "wrong"},
            )
            self.assertIsNone(payload["supervised_motion_command"])

    def test_matching_map_id_without_any_map_path_fails_closed(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            write_lidar_summary(root)
            state = localized_world()
            state["world_state"]["current_pose"]["map_path"] = ""
            state["world_state"]["localization"]["map_path"] = ""
            config = args(registry(root), target="safe_target")
            with patch.object(acceptance, "runtime_process_status", return_value=self.runtime_ready()):
                with patch.object(acceptance, "world_state", return_value=state):
                    code, payload = acceptance.prepare_navigation_stage(config)
            self.assertEqual(code, 6)
            self.assertFalse(payload["map_identity"]["matches"])
            self.assertIn("path was not reported", payload["map_identity"]["reason"])
            self.assertIsNone(payload["supervised_motion_command"])

    def test_relocation_verification_rejects_wrong_actual_map(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            config = args(registry(Path(temp)), confirm_relocation="mapping_origin")
            not_started = {"accepted": True, "world_state": {"localization": {"status": "not_started"}}}
            samples = [localized_world(timestamp_ms=1000), localized_world(timestamp_ms=2000)]
            for sample in samples:
                sample["world_state"]["current_pose"]["map_path"] = "/home/unitree/wrong.pcd"
            session = FakePersistentGatewaySession(samples)
            with patch.object(acceptance, "runtime_process_status", return_value=self.runtime_ready()):
                with patch.object(acceptance, "world_state", return_value=not_started):
                    with patch.object(
                        acceptance,
                        "run_gateway_command",
                        return_value={"accepted": True},
                    ):
                        with patch.object(
                            acceptance,
                            "PersistentGatewaySession",
                            return_value=session,
                        ):
                            code, payload = acceptance.relocate_stage(config)
            self.assertEqual(code, 5)
            self.assertFalse(payload["localization_verified"])
            self.assertTrue(all(not sample["map_identity"]["matches"] for sample in payload["samples"]))

    def test_verify_localization_rejects_wrong_actual_map(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            config = args(registry(Path(temp)))
            samples = [localized_world(timestamp_ms=1000), localized_world(timestamp_ms=2000)]
            for sample in samples:
                sample["world_state"]["localization"]["map_id"] = "wrong"
            session = FakePersistentGatewaySession(samples)
            with patch.object(
                acceptance,
                "PersistentGatewaySession",
                return_value=session,
            ):
                code, payload = acceptance.verify_localization_stage(config)
            self.assertEqual(code, 5)
            self.assertFalse(payload["localization_verified"])
            self.assertTrue(all(not sample["map_identity"]["matches"] for sample in payload["samples"]))

    def test_rejected_gateway_response_never_prepares_navigation(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            write_lidar_summary(root)
            state = localized_world()
            state["accepted"] = False
            state["reason"] = "rejected"
            config = args(registry(root), target="safe_target")
            with patch.object(acceptance, "runtime_process_status", return_value=self.runtime_ready()):
                with patch.object(acceptance, "world_state", return_value=state):
                    code, payload = acceptance.prepare_navigation_stage(config)
            self.assertEqual(code, 6)
            self.assertFalse(payload["ready"])
            self.assertIsNone(payload["supervised_motion_command"])


if __name__ == "__main__":
    unittest.main()
