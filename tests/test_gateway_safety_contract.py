import unittest

from edge_autonomy.gateway_safety import gateway_allows_navigation


def world_state(
    *,
    source="stereo_depth",
    stale=False,
    age_ms=100,
    front=2.0,
    left=2.0,
    right=2.0,
    localization_status="localized",
    pose_age_ms=100,
):
    return {
        "world_state": {
            "safety": {"allow_navigation": True, "reason": "ok"},
            "slam_health": {"status": "ok", "slam_alive": True, "localization_alive": True},
            "localization": {"status": localization_status, "confidence": 0.9, "pose_age_ms": pose_age_ms},
            "current_pose": {"pose": {"x": 0.0, "y": 0.0, "yaw": 0.0}},
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

    def test_manual_stub_does_not_override_gateway_safety(self):
        allowed, reason = gateway_allows_navigation(world_state(source="manual_stub"))

        self.assertTrue(allowed, reason)
        self.assertIn("gateway allows navigation", reason)

    def test_stale_trusted_summary_fails_closed(self):
        allowed, reason = gateway_allows_navigation(world_state(stale=True))

        self.assertFalse(allowed)
        self.assertIn("stale", reason)

    def test_adapter_fresh_summary_blocks_even_when_older_than_default_period(self):
        allowed, reason = gateway_allows_navigation(world_state(age_ms=2500, right=0.6))

        self.assertFalse(allowed)
        self.assertIn("right clearance", reason)

    def test_side_obstacle_is_blocked(self):
        allowed, reason = gateway_allows_navigation(world_state(right=0.6))

        self.assertFalse(allowed)
        self.assertIn("right clearance", reason)

    def test_missing_safety_decision_is_blocked(self):
        state = world_state()
        del state["world_state"]["safety"]

        allowed, reason = gateway_allows_navigation(state)

        self.assertFalse(allowed)
        self.assertIn("missing safety", reason)

    def test_missing_localization_is_blocked(self):
        state = world_state()
        del state["world_state"]["localization"]

        allowed, reason = gateway_allows_navigation(state)

        self.assertFalse(allowed)
        self.assertIn("missing localization", reason)

    def test_stale_localization_pose_is_blocked(self):
        allowed, reason = gateway_allows_navigation(world_state(pose_age_ms=2500))

        self.assertFalse(allowed)
        self.assertIn("pose is stale", reason)

    def test_degraded_localization_is_allowed_when_gateway_safety_allows(self):
        allowed, reason = gateway_allows_navigation(world_state(localization_status="degraded", pose_age_ms=1500))

        self.assertTrue(allowed, reason)
        self.assertIn("gateway allows navigation", reason)


if __name__ == "__main__":
    unittest.main()
