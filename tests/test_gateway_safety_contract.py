import unittest

from edge_autonomy.gateway_safety import gateway_allows_navigation


def world_state(*, allowed=True, reason="ok", mode="normal"):
    return {
        "accepted": True,
        "world_state": {
            "safety": {
                "allow_navigation": allowed,
                "reason": reason,
                "recommended_mode": mode,
            },
            "slam_health": {"status": "ok", "slam_alive": True, "localization_alive": True},
            "localization": {"status": "localized", "confidence": 0.9, "pose_age_ms": 100},
            "current_pose": {"pose": {"x": 0.0, "y": 0.0, "yaw": 0.0}},
            "local_obstacle": {
                "source": "lidar_pointcloud",
                "calibration_verified": True,
                "calibration_id": "test-calibration",
                "stale": False,
                "age_ms": 100,
                "confidence": 0.8,
                "front_confidence": 0.8,
                "left_confidence": 0.8,
                "right_confidence": 0.8,
                "front_clearance_m": 2.0,
                "left_clearance_m": 2.0,
                "right_clearance_m": 2.0,
            },
        }
    }


class GatewaySafetyContractTests(unittest.TestCase):
    def test_gateway_allow_decision_is_returned(self):
        allowed, reason = gateway_allows_navigation(world_state())

        self.assertTrue(allowed, reason)
        self.assertEqual(reason, "ok")

    def test_gateway_block_decision_is_returned(self):
        allowed, reason = gateway_allows_navigation(
            world_state(allowed=False, reason="local_obstacle_not_fresh", mode="hold")
        )

        self.assertFalse(allowed)
        self.assertIn("local_obstacle_not_fresh", reason)

    def test_non_normal_gateway_mode_is_preserved(self):
        allowed, reason = gateway_allows_navigation(world_state(mode="slow"))

        self.assertTrue(allowed)
        self.assertIn("slow", reason)

    def test_rejected_gateway_response_fails_closed(self):
        state = world_state()
        state["accepted"] = False
        state["reason"] = "rejected"

        allowed, reason = gateway_allows_navigation(state)

        self.assertFalse(allowed)
        self.assertIn("not accepted", reason)

    def test_missing_safety_decision_is_blocked(self):
        state = world_state()
        del state["world_state"]["safety"]

        allowed, reason = gateway_allows_navigation(state)

        self.assertFalse(allowed)
        self.assertIn("missing safety", reason)

    def test_host_does_not_recompute_gateway_sensor_policy(self):
        state = world_state()
        del state["world_state"]["localization"]
        state["world_state"]["local_obstacle"]["source"] = "manual_stub"
        state["world_state"]["local_obstacle"]["stale"] = True

        allowed, reason = gateway_allows_navigation(state)

        self.assertTrue(allowed, reason)


if __name__ == "__main__":
    unittest.main()
