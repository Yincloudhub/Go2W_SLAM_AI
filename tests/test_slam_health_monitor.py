import unittest

from edge_autonomy.models import Pose2D
from edge_autonomy.slam_health_monitor import SlamHealthMonitor
from edge_autonomy.slam_state import CurrentPose


class SlamHealthMonitorTests(unittest.TestCase):
    def test_no_pose_means_not_started_and_failed_health(self) -> None:
        monitor = SlamHealthMonitor(map_id="701")

        localization = monitor.get_localization_state(timestamp_ms=1000)
        health = monitor.get_slam_health(timestamp_ms=1000)

        self.assertEqual(localization.status, "not_started")
        self.assertEqual(localization.confidence, 0.0)
        self.assertEqual(health.status, "failed")
        self.assertFalse(health.slam_alive)

    def test_pose_age_drives_localization_state(self) -> None:
        monitor = SlamHealthMonitor(map_id="701")
        monitor.observe_pose(CurrentPose(timestamp_ms=1000, map_id="701", frame_id="map", pose=Pose2D(0.0, 0.0, 0.0)))

        self.assertEqual(monitor.get_localization_state(timestamp_ms=1200).status, "localized")
        self.assertEqual(monitor.get_localization_state(timestamp_ms=1800).status, "degraded")

        lost = monitor.get_localization_state(timestamp_ms=4000)
        self.assertEqual(lost.status, "lost")
        self.assertGreater(lost.lost_duration_ms, 0)

    def test_pose_age_drives_slam_health(self) -> None:
        monitor = SlamHealthMonitor(map_id="701")
        monitor.observe_pose(CurrentPose(timestamp_ms=1000, map_id="701", frame_id="map", pose=Pose2D(0.0, 0.0, 0.0)))

        self.assertEqual(monitor.get_slam_health(timestamp_ms=1200).status, "ok")
        self.assertEqual(monitor.get_slam_health(timestamp_ms=1800).status, "degraded")
        self.assertEqual(monitor.get_slam_health(timestamp_ms=7000).status, "failed")


if __name__ == "__main__":
    unittest.main()
