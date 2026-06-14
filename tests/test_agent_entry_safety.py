import base64
import contextlib
import io
import inspect
import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from scripts import go2w_agent_entry
from scripts import run_robot_closed_loop


REGISTRY_PATH = Path(__file__).resolve().parents[1] / "configs" / "maps" / "go2w_real_site_map_registry.json"


class AgentEntrySafetyTests(unittest.TestCase):
    def test_closed_loop_has_no_direct_slam_command_execution_fallback(self) -> None:
        source = inspect.getsource(run_robot_closed_loop.main)

        self.assertNotIn("execution_result = run_gateway_command(", source)
        self.assertIn("build_mission_decision(", source)
        self.assertIn('{"execute_queue", "dry_run_queue"}', source)
        self.assertIn("communication.reserve_execution(", source)
        self.assertIn("event_sink=journal_queue_event", source)

    def test_closed_loop_rejects_execute_with_skipped_gateway_check(self) -> None:
        stdout = io.StringIO()
        with contextlib.redirect_stdout(stdout):
            rc = run_robot_closed_loop.main(
                [
                    "--command",
                    "wp_1",
                    "--registry",
                    str(REGISTRY_PATH),
                    "--map-id",
                    "go2w_real_site",
                    "--map-path",
                    "/home/unitree/test.pcd",
                    "--no-live-snapshot",
                    "--execute",
                    "--skip-gateway-check",
                ]
            )

        payload = json.loads(stdout.getvalue())
        self.assertEqual(rc, 2)
        self.assertFalse(payload["gateway"]["allowed"])
        self.assertIn("only allowed for dry-runs", payload["execution"]["blocked_reason"])

    def test_agent_entry_rejects_execute_with_skipped_gateway_check(self) -> None:
        command_b64 = base64.b64encode("去赵博办公室门口".encode("utf-8")).decode("ascii")
        stdout = io.StringIO()
        with contextlib.redirect_stdout(stdout):
            rc = go2w_agent_entry.main(["--go-b64", command_b64, "--execute", "--skip-gateway-check"])

        payload = json.loads(stdout.getvalue())
        self.assertEqual(rc, 2)
        self.assertIn("only allowed for dry-runs", payload["error"])

    def test_dry_run_go_can_skip_gateway_preflight_cleanly(self) -> None:
        command_b64 = base64.b64encode("去赵博办公室门口".encode("utf-8")).decode("ascii")
        original = go2w_agent_entry.run_closed_loop

        def fake_run_closed_loop(args, command):
            return {
                "returncode": 0,
                "result": {
                    "result": {
                        "planner": {},
                        "execution": {"executed": False, "blocked_reason": "dry run"},
                    }
                },
                "stderr": "",
            }

        go2w_agent_entry.run_closed_loop = fake_run_closed_loop
        try:
            with tempfile.TemporaryDirectory() as tmp:
                stdout = io.StringIO()
                with contextlib.redirect_stdout(stdout):
                    rc = go2w_agent_entry.main(
                        [
                            "--go-b64",
                            command_b64,
                            "--dry-run",
                            "--skip-gateway-check",
                            "--full-output",
                            "--log-dir",
                            tmp,
                        ]
                    )
        finally:
            go2w_agent_entry.run_closed_loop = original

        payload = json.loads(stdout.getvalue())
        preflight = next(step for step in payload["steps"] if step["step"] == "go_preflight")
        self.assertEqual(rc, 0)
        self.assertFalse(preflight["checked"])
        self.assertIsNone(preflight["allowed"])
        self.assertNotIn("go_dry_run_preflight_not_enforced", [step["step"] for step in payload["steps"]])

    def test_explicit_relocation_requires_matching_anchor_confirmation(self) -> None:
        stdout = io.StringIO()
        with patch.object(go2w_agent_entry, "relocate_to_anchor") as relocate:
            with contextlib.redirect_stdout(stdout):
                rc = go2w_agent_entry.main(
                    [
                        "--relocate",
                        "--relocation-anchor",
                        "mapping_origin",
                    ]
                )

        payload = json.loads(stdout.getvalue())
        self.assertEqual(rc, 2)
        self.assertFalse(payload["steps"][-1]["result"]["accepted"])
        relocate.assert_not_called()

    def test_explicit_relocation_requires_anchor_selection(self) -> None:
        stdout = io.StringIO()
        with patch.object(go2w_agent_entry, "relocate_to_anchor") as relocate:
            with contextlib.redirect_stdout(stdout):
                rc = go2w_agent_entry.main(["--relocate"])

        payload = json.loads(stdout.getvalue())
        self.assertEqual(rc, 2)
        self.assertIn("--relocation-anchor is required", payload["steps"][-1]["result"]["reason"])
        relocate.assert_not_called()

    def test_live_node_calibration_records_protected_provenance(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            registry_path = Path(tmp) / "registry.json"
            registry_path.write_text(
                json.dumps(
                    {
                        "maps": [
                            {
                                "map_id": "go2w_real_site",
                                "topology_nodes": [
                                    {
                                        "node_id": "wp_a",
                                        "name": "A",
                                        "tags": ["real_site", "needs_calibration"],
                                        "pose": {"x": 0.0, "y": 0.0, "speed": 0.2, "mode": 0},
                                    }
                                ],
                            }
                        ]
                    }
                ),
                encoding="utf-8",
            )
            args = type("Args", (), {"registry": str(registry_path), "map_id": "go2w_real_site"})()
            world_state = {
                "world_state": {
                    "current_pose": {
                        "pose": {
                            "x": 1.2,
                            "y": -0.3,
                            "z": 0.01,
                            "q_x": 0.0,
                            "q_y": 0.0,
                            "q_z": 0.1,
                            "q_w": 0.995,
                        }
                    }
                }
            }

            with patch.object(go2w_agent_entry, "get_world_state", return_value=world_state):
                result = go2w_agent_entry.calibrate_node_from_current_pose(args, "wp_a")

            updated = json.loads(registry_path.read_text(encoding="utf-8"))["maps"][0]["topology_nodes"][0]

        self.assertTrue(result["updated"])
        self.assertNotIn("needs_calibration", updated["tags"])
        self.assertIn("live_calibrated", updated["tags"])
        self.assertEqual(updated["calibration"]["source"], "gateway_world_state.current_pose")
        self.assertEqual(updated["calibration"]["method"], "cli_calibrate_node_current_pose")
        self.assertEqual(updated["calibration"]["previous_pose"]["x"], 0.0)


if __name__ == "__main__":
    unittest.main()
