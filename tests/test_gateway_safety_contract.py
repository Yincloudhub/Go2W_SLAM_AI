import unittest

from scripts.run_robot_closed_loop import gateway_allows_navigation


def world_state(*, source="stereo_depth", stale=False, age_ms=100, front=2.0, left=2.0, right=2.0):
    return {
        "world_state": {
            "safety": {"allow_navigation": True, "reason": "ok"},
            "slam_health": {"status": "ok"},
            "localization": {"status": "localized"},
            "local_obstacle": {
                "source": source,
                "stale": stale,
                "age_ms": age_ms,
                "confidence": 0.8,
                "front_confidence": 0.8,
                "left_confidence": 0.8,
                "right_confidence": 0.8,
                "front_clearance_m": front,
                "left_clearance_m": left,
                "right_clearance_m": right,
            },
        }
    }


class GatewaySafetyContractTests(unittest.TestCase):
    def test_fresh_sensor_backed_clearance_allows_navigation(self):
        allowed, reason = gateway_allows_navigation(world_state())

        self.assertTrue(allowed, reason)

    def test_manual_stub_is_blocked(self):
        allowed, reason = gateway_allows_navigation(world_state(source="manual_stub"))

        self.assertFalse(allowed)
        self.assertIn("not sensor backed", reason)

    def test_stale_summary_is_blocked(self):
        allowed, reason = gateway_allows_navigation(world_state(stale=True))

        self.assertFalse(allowed)
        self.assertIn("stale", reason)

    def test_side_obstacle_is_blocked(self):
        allowed, reason = gateway_allows_navigation(world_state(right=0.6))

        self.assertFalse(allowed)
        self.assertIn("right clearance", reason)


if __name__ == "__main__":
    unittest.main()
