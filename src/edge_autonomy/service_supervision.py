from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class RecoveryDecision:
    action: str
    consecutive_failures: int
    retry_after_ms: int = 0


class ConsecutiveFailureRecovery:
    """Convert noisy health samples into bounded recovery decisions."""

    def __init__(self, *, failure_threshold: int, cooldown_ms: int) -> None:
        if failure_threshold <= 0:
            raise ValueError("failure_threshold must be positive")
        if cooldown_ms < 0:
            raise ValueError("cooldown_ms must be non-negative")
        self.failure_threshold = int(failure_threshold)
        self.cooldown_ms = int(cooldown_ms)
        self.consecutive_failures = 0
        self.last_recovery_ms: int | None = None

    def observe(self, *, healthy: bool, now_ms: int) -> RecoveryDecision:
        if now_ms < 0:
            raise ValueError("now_ms must be non-negative")
        if healthy:
            self.consecutive_failures = 0
            return RecoveryDecision("healthy", 0)

        self.consecutive_failures += 1
        if self.consecutive_failures < self.failure_threshold:
            return RecoveryDecision("wait_for_threshold", self.consecutive_failures)

        if self.last_recovery_ms is not None:
            elapsed_ms = max(0, now_ms - self.last_recovery_ms)
            if elapsed_ms < self.cooldown_ms:
                return RecoveryDecision(
                    "cooldown",
                    self.consecutive_failures,
                    self.cooldown_ms - elapsed_ms,
                )

        self.last_recovery_ms = now_ms
        self.consecutive_failures = 0
        return RecoveryDecision("recover", 0)
