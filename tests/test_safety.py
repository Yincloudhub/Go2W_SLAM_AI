import unittest

from edge_autonomy.models import Pose2D, RiskEvent, RobotState, SemanticObject, WorldState
from edge_autonomy.safety import LinkQuality, SafetySupervisor, SupervisorAction
from edge_autonomy.slam_adapter import NavigationFeedback, NavigationStatus


class SafetySupervisorTests(unittest.TestCase):
    def setUp(self) -> None:
        self.supervisor = SafetySupervisor()
        self.safe_world = WorldState(
            timestamp_ms=1,
            frame_id="map",
            robot=RobotState(pose=Pose2D(0.0, 0.0, 0.0), mode="autonomy", battery_percent=90.0),
            objects=[],
            risk_events=[],
            task_phase="navigate",
        )
        self.safe_feedback = NavigationFeedback(status=NavigationStatus.NAVIGATING, pose=Pose2D(0.0, 0.0, 0.0), distance_to_goal_m=2.0)
        self.good_link = LinkQuality(bandwidth_kbps=2048.0, latency_ms=80.0, packet_loss_ratio=0.0)

    def test_critical_risk_triggers_emergency_stop(self) -> None:
        world = WorldState(
            timestamp_ms=1,
            frame_id="map",
            robot=self.safe_world.robot,
            objects=[],
            risk_events=[RiskEvent(event_type="cliff", severity="critical", description="cliff detected", distance_m=0.5)],
            task_phase="navigate",
        )

        decision = self.supervisor.evaluate(world, self.safe_feedback, self.good_link)

        self.assertEqual(decision.action, SupervisorAction.EMERGENCY_STOP)

    def test_weak_network_triggers_slow_down(self) -> None:
        weak_link = LinkQuality(bandwidth_kbps=64.0, latency_ms=700.0, packet_loss_ratio=0.05)

        decision = self.supervisor.evaluate(self.safe_world, self.safe_feedback, weak_link)

        self.assertEqual(decision.action, SupervisorAction.SLOW_DOWN)
        self.assertLess(decision.speed_limit_scale, 1.0)

    def test_blocked_object_requests_replan(self) -> None:
        world = WorldState(
            timestamp_ms=1,
            frame_id="map",
            robot=self.safe_world.robot,
            objects=[
                SemanticObject(
                    object_id="box-1",
                    category="box",
                    pose=Pose2D(0.9, 0.0, 0.0),
                    distance_m=0.9,
                    traversable=False,
                )
            ],
            risk_events=[],
            task_phase="navigate",
        )

        decision = self.supervisor.evaluate(world, self.safe_feedback, self.good_link)

        self.assertEqual(decision.action, SupervisorAction.REQUEST_REPLAN)

    def test_failed_navigation_triggers_emergency_stop(self) -> None:
        failed_feedback = NavigationFeedback(status=NavigationStatus.FAILED, pose=Pose2D(0.0, 0.0, 0.0), distance_to_goal_m=None)

        decision = self.supervisor.evaluate(self.safe_world, failed_feedback, self.good_link)

        self.assertEqual(decision.action, SupervisorAction.EMERGENCY_STOP)

    def test_safe_state_passes_through(self) -> None:
        decision = self.supervisor.evaluate(self.safe_world, self.safe_feedback, self.good_link)

        self.assertEqual(decision.action, SupervisorAction.PASS_THROUGH)


if __name__ == "__main__":
    unittest.main()
