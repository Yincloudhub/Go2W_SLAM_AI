import json
import time
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

    def test_summary_separates_relocalization_from_navigation_readiness(self) -> None:
        gateway_response = {
            "accepted": True,
            "action": "get_world_state",
            "world_state": {
                "safety": {
                    "allow_navigation": False,
                    "reason": "localization_not_valid",
                },
                "slam_health": {"status": "failed", "slam_alive": False},
                "localization": {"status": "not_started", "pose_age_ms": -1},
                "current_pose": {"pose": {"x": 0.0, "y": 0.0}},
                "local_obstacle": {
                    "source": "lidar_pointcloud",
                    "stale": True,
                    "age_ms": 100,
                },
            },
        }
        records = [
            {"name": "slam_stack", "required": True, "ok": True, "dry_run": False, "starts_motion": False},
            {
                "name": "gateway_world_state_probe",
                "required": False,
                "ok": True,
                "returncode": 0,
                "stdout": "ready\n" + json.dumps(gateway_response),
                "dry_run": False,
                "starts_motion": False,
            },
        ]
        summary = startup_summary(
            records,
            lidar_summary={
                "source": "lidar_pointcloud",
                "timestamp_ms": int(time.time() * 1000),
                "stale": True,
                "stale_reasons": ["uncalibrated_xt16_geometry"],
                "summary": {"calibrated": False},
            },
        )

        self.assertTrue(summary["startup_ok"])
        self.assertTrue(summary["readiness"]["relocalization_ready"])
        self.assertFalse(summary["readiness"]["navigation_ready"])
        self.assertFalse(summary["readiness"]["perception_ready"])
        self.assertEqual(summary["readiness"]["next_action"], "request relocalization from a verified anchor")


if __name__ == "__main__":
    unittest.main()
