import base64
import contextlib
import io
import json
import tempfile
import unittest
from pathlib import Path

from scripts import go2w_agent_entry
from scripts import run_robot_closed_loop


REGISTRY_PATH = Path(__file__).resolve().parents[1] / "configs" / "maps" / "go2w_real_site_map_registry.json"


class AgentEntrySafetyTests(unittest.TestCase):
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


if __name__ == "__main__":
    unittest.main()
