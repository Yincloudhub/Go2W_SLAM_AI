import json
import subprocess
import tempfile
import unittest
from pathlib import Path
from unittest.mock import MagicMock, patch

from edge_autonomy.chassis_controller import (
    ChassisController,
    GatewayConfig,
    PersistentGatewaySession,
    gateway_allows_navigation,
    run_gateway_command,
)


class ChassisControllerTests(unittest.TestCase):
    def test_resolve_node_prefers_first_target_in_command_order(self) -> None:
        registry = {
            "version": 1,
            "default_map_id": "site",
            "maps": [
                {
                    "map_id": "site",
                    "pcd_path": "/tmp/site.pcd",
                    "topology_nodes": [
                        {"node_id": "station", "name": "station", "aliases": ["station"], "pose": {"x": 0, "y": 0}},
                        {"node_id": "room701", "name": "room701", "aliases": ["room701"], "pose": {"x": 1, "y": 0}},
                    ],
                }
            ],
        }
        with tempfile.TemporaryDirectory() as tmp:
            registry_path = Path(tmp) / "registry.json"
            registry_path.write_text(json.dumps(registry), encoding="utf-8")
            controller = ChassisController(
                registry_path=registry_path,
                map_id="site",
                gateway=GatewayConfig(client_path="/bin/false"),
            )

            result = controller.resolve_node("go room701 then return station")

        self.assertTrue(result["matched"])
        self.assertTrue(result["multi_target"])
        self.assertEqual(result["selected"]["node_id"], "room701")
        self.assertEqual([item["node_id"] for item in result["matches"]], ["room701", "station"])

    def test_preflight_uses_gateway_authority_without_recomputing_sensor_policy(self) -> None:
        allowed, reason = gateway_allows_navigation(
            {
                "accepted": True,
                "world_state": {
                    "safety": {"allow_navigation": True, "reason": "ok"},
                    "slam_health": {"status": "ok", "slam_alive": True, "localization_alive": True},
                    "localization": {"status": "localized", "confidence": 0.9, "pose_age_ms": 100},
                    "current_pose": {"pose": {"x": 0.0, "y": 0.0, "yaw": 0.0}},
                    "local_obstacle": {
                        "source": "manual_stub",
                        "stale": True,
                        "age_ms": -1,
                        "confidence": 0.0,
                        "front_confidence": 0.0,
                        "left_confidence": 0.0,
                        "right_confidence": 0.0,
                        "front_clearance_m": 6.0,
                        "left_clearance_m": 6.0,
                        "right_clearance_m": 6.0,
                    },
                }
            }
        )

        self.assertTrue(allowed, reason)

    def test_gateway_timeout_kills_child_and_fails_deterministically(self) -> None:
        process = MagicMock()
        process.communicate.side_effect = [
            subprocess.TimeoutExpired(cmd=["gateway"], timeout=1),
            ("", ""),
        ]
        with patch("edge_autonomy.chassis_controller.subprocess.Popen", return_value=process):
            with self.assertRaisesRegex(RuntimeError, "timed out after 1s"):
                run_gateway_command(
                    {"action": "get_world_state"},
                    GatewayConfig(client_path="gateway", timeout_s=1),
                )
        process.kill.assert_called_once_with()

    def test_topology_node_cannot_be_used_as_relocalization_anchor(self) -> None:
        registry = {
            "version": 1,
            "default_map_id": "site",
            "maps": [
                {
                    "map_id": "site",
                    "pcd_path": "/tmp/site.pcd",
                    "topology_nodes": [
                        {"node_id": "initial_point", "pose": {"x": 1.0, "y": 2.0}}
                    ],
                    "relocalization_anchors": [],
                }
            ],
        }
        with tempfile.TemporaryDirectory() as tmp:
            registry_path = Path(tmp) / "registry.json"
            registry_path.write_text(json.dumps(registry), encoding="utf-8")
            controller = ChassisController(
                registry_path=registry_path,
                map_id="site",
                gateway=GatewayConfig(client_path="gateway"),
            )
            with patch("edge_autonomy.chassis_controller.run_gateway_command") as gateway:
                result = controller.relocate_to_anchor(
                    "initial_point",
                    map_path_fallback="/tmp/fallback.pcd",
                )

        self.assertFalse(result["accepted"])
        self.assertIn("unknown anchor_id", result["reason"])
        gateway.assert_not_called()

    def test_relocation_uses_registry_command_contract(self) -> None:
        registry = {
            "version": 1,
            "default_map_id": "site",
            "maps": [
                {
                    "map_id": "site",
                    "name": "site",
                    "pcd_path": "/tmp/site.pcd",
                    "relocalization_anchors": [
                        {
                            "anchor_id": "mapping_origin",
                            "status": "verified",
                            "pose": {
                                "x": 0.0,
                                "y": 0.0,
                                "z": 0.0,
                                "q_x": 0.0,
                                "q_y": 0.0,
                                "q_z": 0.0,
                                "q_w": 1.0,
                            },
                        }
                    ],
                }
            ],
        }
        with tempfile.TemporaryDirectory() as tmp:
            registry_path = Path(tmp) / "registry.json"
            registry_path.write_text(json.dumps(registry), encoding="utf-8")
            controller = ChassisController(
                registry_path=registry_path,
                map_id="site",
                gateway=GatewayConfig(client_path="gateway"),
            )
            with patch(
                "edge_autonomy.chassis_controller.run_gateway_command",
                return_value={"accepted": True},
            ) as gateway:
                result = controller.relocate_to_anchor(
                    "mapping_origin",
                    map_path_fallback="/tmp/fallback.pcd",
                )

        self.assertTrue(result["accepted"])
        command = gateway.call_args.args[0]
        self.assertEqual(command["anchor_id"], "mapping_origin")
        self.assertEqual(command["initial_pose"]["name"], "mapping_origin")
        self.assertEqual(command["initial_pose"]["speed"], 0.0)
        self.assertEqual(command["initial_pose"]["mode"], 0)

    def test_persistent_session_matches_response_request_id(self) -> None:
        class FakeStdin:
            def __init__(self):
                self.payload = ""

            def write(self, payload):
                self.payload += payload

            def flush(self):
                return None

        class FakeProcess:
            def __init__(self):
                self.stdin = FakeStdin()

            def poll(self):
                return None

        import queue

        session = PersistentGatewaySession.__new__(PersistentGatewaySession)
        session.timeout_s = 1
        session.process = FakeProcess()
        session.messages = queue.Queue()
        session.session_token = "token"
        session.lease_timeout_ms = 2000
        session.navigation_session = True
        session._request_sequence = 0
        session.active = False
        session.async_events = []
        session.messages.put(
            {
                "accepted": False,
                "action": "get_world_state",
                "request_id": "stale-request",
            }
        )
        session.messages.put(
            {
                "accepted": True,
                "action": "get_world_state",
                "request_id": "session-request-1",
            }
        )

        result = session.command({"action": "get_world_state"})

        self.assertTrue(result["accepted"])
        self.assertEqual(result["request_id"], "session-request-1")
        self.assertEqual(session.async_events[0]["request_id"], "stale-request")

    def test_read_only_persistent_session_omits_navigation_token(self) -> None:
        class FakeStdin:
            def __init__(self):
                self.payload = ""

            def write(self, payload):
                self.payload += payload

            def flush(self):
                return None

        class FakeProcess:
            def __init__(self):
                self.stdin = FakeStdin()

            def poll(self):
                return None

        import queue

        session = PersistentGatewaySession.__new__(PersistentGatewaySession)
        session.timeout_s = 1
        session.process = FakeProcess()
        session.messages = queue.Queue()
        session.session_token = ""
        session.lease_timeout_ms = 0
        session.navigation_session = False
        session._request_sequence = 0
        session.active = False
        session.async_events = []
        session.messages.put(
            {
                "accepted": True,
                "action": "get_world_state",
                "request_id": "session-request-1",
            }
        )

        result = session.command({"action": "get_world_state"})
        payload = json.loads(session.process.stdin.payload)

        self.assertTrue(result["accepted"])
        self.assertNotIn("navigation_session_token", payload)
        self.assertEqual(payload["request_id"], "session-request-1")

    def test_read_only_persistent_session_rejects_heartbeat_locally(self) -> None:
        session = PersistentGatewaySession.__new__(PersistentGatewaySession)
        session.navigation_session = False

        with self.assertRaisesRegex(RuntimeError, "does not support"):
            session.heartbeat()

    def test_rejected_pause_keeps_navigation_session_active(self) -> None:
        class FakeStdin:
            def write(self, payload):
                return len(payload)

            def flush(self):
                return None

        class FakeProcess:
            stdin = FakeStdin()

            def poll(self):
                return None

        import queue

        session = PersistentGatewaySession.__new__(PersistentGatewaySession)
        session.timeout_s = 1
        session.process = FakeProcess()
        session.messages = queue.Queue()
        session.session_token = "token"
        session.navigation_session = True
        session._request_sequence = 0
        session.active = True
        session.async_events = []
        session.messages.put(
            {
                "accepted": False,
                "action": "pause_navigation",
                "request_id": "session-request-1",
            }
        )

        result = session.command({"action": "pause_navigation"})

        self.assertFalse(result["accepted"])
        self.assertTrue(session.active)

    def test_persistent_session_reports_unready_reason_immediately(self) -> None:
        import queue

        session = PersistentGatewaySession.__new__(PersistentGatewaySession)
        session.messages = queue.Queue()
        session.async_events = []
        session.messages.put(
            {
                "type": "navigation_session_unready",
                "reason": "fresh_localization_with_map_identity_required",
            }
        )

        with self.assertRaisesRegex(
            RuntimeError,
            "fresh_localization_with_map_identity_required",
        ):
            session._wait_for(lambda value: False, timeout_s=1)


if __name__ == "__main__":
    unittest.main()
