import unittest

from edge_autonomy.lidar_geometry import ManualLidarGeometryPerception


class ManualLidarGeometryPerceptionTests(unittest.TestCase):
    def test_front_obstacle_requests_pause(self) -> None:
        perception = ManualLidarGeometryPerception()

        summary = perception.set_clearance(0.5, 2.0, 2.0, 2.0, timestamp_ms=1)

        self.assertIn("front", summary.blocked_directions)
        self.assertEqual(summary.recommended_action, "pause")

    def test_near_side_obstacle_requests_slow_mode(self) -> None:
        perception = ManualLidarGeometryPerception()

        summary = perception.set_clearance(2.0, 0.5, 2.0, 2.0, timestamp_ms=1)

        self.assertEqual(summary.blocked_directions, [])
        self.assertEqual(summary.recommended_action, "go_slow")

    def test_extremely_close_side_obstacle_requests_pause(self) -> None:
        perception = ManualLidarGeometryPerception()

        summary = perception.set_clearance(2.0, 0.1, 2.0, 2.0, timestamp_ms=1)

        self.assertEqual(summary.blocked_directions, ["left"])
        self.assertEqual(summary.recommended_action, "pause")

    def test_clear_area_is_normal(self) -> None:
        perception = ManualLidarGeometryPerception()

        summary = perception.set_clearance(3.0, 2.0, 2.0, 2.0, timestamp_ms=1)

        self.assertEqual(summary.blocked_directions, [])
        self.assertEqual(summary.recommended_action, "normal")


if __name__ == "__main__":
    unittest.main()
