import unittest

from edge_autonomy.service_supervision import ConsecutiveFailureRecovery


class ConsecutiveFailureRecoveryTests(unittest.TestCase):
    def test_recovery_requires_consecutive_failures(self) -> None:
        recovery = ConsecutiveFailureRecovery(failure_threshold=3, cooldown_ms=30_000)

        self.assertEqual(
            recovery.observe(healthy=False, now_ms=1_000).action,
            "wait_for_threshold",
        )
        self.assertEqual(
            recovery.observe(healthy=False, now_ms=2_000).action,
            "wait_for_threshold",
        )
        self.assertEqual(
            recovery.observe(healthy=False, now_ms=3_000).action,
            "recover",
        )

    def test_healthy_sample_resets_failure_count(self) -> None:
        recovery = ConsecutiveFailureRecovery(failure_threshold=2, cooldown_ms=0)

        recovery.observe(healthy=False, now_ms=1_000)
        healthy = recovery.observe(healthy=True, now_ms=2_000)
        next_failure = recovery.observe(healthy=False, now_ms=3_000)

        self.assertEqual(healthy.action, "healthy")
        self.assertEqual(next_failure.action, "wait_for_threshold")
        self.assertEqual(next_failure.consecutive_failures, 1)

    def test_cooldown_prevents_restart_loop(self) -> None:
        recovery = ConsecutiveFailureRecovery(failure_threshold=1, cooldown_ms=30_000)

        self.assertEqual(recovery.observe(healthy=False, now_ms=1_000).action, "recover")
        cooldown = recovery.observe(healthy=False, now_ms=2_000)

        self.assertEqual(cooldown.action, "cooldown")
        self.assertEqual(cooldown.retry_after_ms, 29_000)


if __name__ == "__main__":
    unittest.main()
