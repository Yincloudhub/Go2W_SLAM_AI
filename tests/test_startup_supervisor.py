import unittest

from scripts.go2w_startup_supervisor import build_startup_plan, run_step, startup_summary


class StartupSupervisorTests(unittest.TestCase):
    def test_plan_contains_only_non_motion_steps(self) -> None:
        plan = build_startup_plan(
            start_slam_script="scripts/start_go2w_slam_stack.sh",
            gateway_client="/tmp/slam_llm_command_client",
            network_interface="eth0",
        )

        self.assertEqual([step.name for step in plan], ["slam_stack", "gateway_world_state_probe"])
        self.assertTrue(all(not step.starts_motion for step in plan))

    def test_dry_run_does_not_execute_commands(self) -> None:
        step = build_startup_plan(
            start_slam_script="missing.sh",
            gateway_client="missing_client",
            network_interface="eth0",
        )[0]

        record = run_step(step, dry_run=True)
        summary = startup_summary([record])

        self.assertTrue(record["dry_run"])
        self.assertIsNone(record["returncode"])
        self.assertTrue(summary["ok"])
        self.assertFalse(summary["motion_commands_sent"])


if __name__ == "__main__":
    unittest.main()
