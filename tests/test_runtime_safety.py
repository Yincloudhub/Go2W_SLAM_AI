import unittest

from edge_autonomy.safety import SafetySupervisor, SupervisorAction
from edge_autonomy.slam_state import LocalObstacleSummary, LocalizationState, SlamHealth


class RuntimeSafetyTests(unittest.TestCase):
    def setUp(self) -> None:
        self.supervisor = SafetySupervisor()
        self.health = SlamHealth(
            timestamp_ms=1,
            slam_alive=True,
            lidar_alive=True,
            imu_alive=True,
            odom_alive=True,
            localization_alive=True,
            last_pose_age_ms=20,
            status="ok",
        )
        self.localization = LocalizationState(timestamp_ms=1, map_id="701", status="localized", confidence=0.9, pose_age_ms=20)
        self.clear_obstacle = LocalObstacleSummary(timestamp_ms=1, source="lidar_pointcloud", front_clearance_m=3.0, confidence=0.9, stale=False)

    def test_lost_localization_pauses(self) -> None:
        decision = self.supervisor.evaluate_runtime(
            self.health,
            LocalizationState(timestamp_ms=1, map_id="701", status="lost", confidence=0.0, pose_age_ms=3000),
            self.clear_obstacle,
        )

        self.assertEqual(decision.action, SupervisorAction.PAUSE)
        self.assertTrue(decision.requires_human_ack)

    def test_front_obstacle_emergency_stops(self) -> None:
        decision = self.supervisor.evaluate_runtime(
            self.health,
            self.localization,
            LocalObstacleSummary(timestamp_ms=1, source="lidar_pointcloud", front_clearance_m=0.5, confidence=0.9, stale=False),
        )

        self.assertEqual(decision.action, SupervisorAction.EMERGENCY_STOP)

    def test_degraded_state_pauses(self) -> None:
        decision = self.supervisor.evaluate_runtime(
            SlamHealth(
                timestamp_ms=1,
                slam_alive=True,
                lidar_alive=True,
                imu_alive=True,
                odom_alive=True,
                localization_alive=True,
                last_pose_age_ms=1200,
                status="degraded",
            ),
            self.localization,
            self.clear_obstacle,
        )

        self.assertEqual(decision.action, SupervisorAction.PAUSE)

    def test_clear_runtime_state_passes(self) -> None:
        decision = self.supervisor.evaluate_runtime(self.health, self.localization, self.clear_obstacle)

        self.assertEqual(decision.action, SupervisorAction.PASS_THROUGH)

    def test_manual_clearance_stub_is_never_motion_safe(self) -> None:
        decision = self.supervisor.evaluate_runtime(
            self.health,
            self.localization,
            LocalObstacleSummary(timestamp_ms=1, front_clearance_m=6.0),
        )

        self.assertEqual(decision.action, SupervisorAction.PAUSE)
        self.assertTrue(decision.requires_human_ack)


if __name__ == "__main__":
    unittest.main()
