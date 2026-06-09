import unittest

from edge_autonomy.perception_fusion import DepthCameraSummary, depth_summary_is_usable, fuse_local_obstacle_summary
from edge_autonomy.slam_state import LocalObstacleSummary


class PerceptionFusionTests(unittest.TestCase):
    def test_valid_depth_can_only_reduce_clearance(self) -> None:
        lidar = LocalObstacleSummary(timestamp_ms=1000, source="lidar_pointcloud", front_clearance_m=2.4, left_clearance_m=2.0, right_clearance_m=2.0, confidence=0.9, stale=False)
        depth = DepthCameraSummary(timestamp_ms=1050, front_clearance_m=0.7, left_clearance_m=0.2, right_clearance_m=0.3, confidence=0.8)

        fused = fuse_local_obstacle_summary(lidar, depth, now_ms=1100)

        self.assertEqual(fused.front_clearance_m, 0.7)
        self.assertEqual(fused.left_clearance_m, 2.0)
        self.assertEqual(fused.right_clearance_m, 2.0)
        self.assertIn("front", fused.blocked_directions)
        self.assertNotIn("left", fused.blocked_directions)
        self.assertNotIn("right", fused.blocked_directions)
        self.assertEqual(fused.recommended_action, "pause")
        self.assertEqual(fused.source, "lidar_pointcloud+stereo_depth")

    def test_depth_never_relaxes_lidar_safety(self) -> None:
        lidar = LocalObstacleSummary(
            timestamp_ms=1000,
            source="lidar_pointcloud",
            front_clearance_m=0.6,
            left_clearance_m=2.0,
            right_clearance_m=2.0,
            body_clearance_m={"front": 1.2},
            low_hazard_clearance_m={"front": 0.6},
            low_hazard_directions=["front"],
            recommended_action="pause",
            confidence=0.9,
            stale=False,
        )
        depth = DepthCameraSummary(timestamp_ms=1050, front_clearance_m=3.0, confidence=0.9)

        fused = fuse_local_obstacle_summary(lidar, depth, now_ms=1100)

        self.assertEqual(fused.front_clearance_m, 0.6)
        self.assertEqual(fused.recommended_action, "pause")
        self.assertEqual(fused.body_clearance_m, {"front": 1.2})
        self.assertEqual(fused.low_hazard_clearance_m, {"front": 0.6})
        self.assertEqual(fused.low_hazard_directions, ["front"])

    def test_stale_depth_is_ignored(self) -> None:
        lidar = LocalObstacleSummary(timestamp_ms=1000, source="lidar_pointcloud", front_clearance_m=2.4, confidence=0.9, stale=False)
        depth = DepthCameraSummary(timestamp_ms=1050, front_clearance_m=0.4, confidence=0.9, stale=True)

        fused = fuse_local_obstacle_summary(lidar, depth, now_ms=1100)

        self.assertEqual(fused, lidar)

    def test_old_depth_is_ignored(self) -> None:
        depth = DepthCameraSummary(timestamp_ms=1000, front_clearance_m=0.4, confidence=0.9)

        self.assertFalse(depth_summary_is_usable(depth, now_ms=1700, max_age_ms=500))

    def test_low_confidence_depth_is_ignored(self) -> None:
        lidar = LocalObstacleSummary(timestamp_ms=1000, source="lidar_pointcloud", front_clearance_m=2.4, confidence=0.9, stale=False)
        depth = DepthCameraSummary(timestamp_ms=1050, front_clearance_m=0.4, confidence=0.2)

        fused = fuse_local_obstacle_summary(lidar, depth, now_ms=1100)

        self.assertEqual(fused, lidar)


if __name__ == "__main__":
    unittest.main()
