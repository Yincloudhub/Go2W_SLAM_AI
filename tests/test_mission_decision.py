import json
import unittest
from pathlib import Path

from jsonschema import Draft202012Validator

from edge_autonomy.gateway_safety import gateway_allows_navigation
from edge_autonomy.mission_decision import (
    build_mission_decision,
    build_recovery_decision,
)


REPO_ROOT = Path(__file__).resolve().parents[1]
DECISION_SCHEMA = json.loads(
    (REPO_ROOT / "schemas" / "mission_decision_v1.schema.json").read_text(encoding="utf-8")
)
DECISION_VALIDATOR = Draft202012Validator(DECISION_SCHEMA)


def navigation_queue() -> dict:
    return {
        "queue_id": "queue_test",
        "mode": "sequential",
        "status": "planned",
        "source": "semantic_topology",
        "targets": ["wp_1"],
        "steps": [
            {
                "task_id": "task_1",
                "action": "navigate",
                "status": "pending",
                "target_node": "wp_1",
            }
        ],
        "communication_policy": {
            "mode": "normal",
            "send": ["task_state", "navigation_feedback", "world_state_summary"],
            "drop": [],
        },
    }


def gateway_state(reason: str, *, allowed: bool) -> dict:
    return {
        "accepted": True,
        "world_state": {
            "timestamp_ms": 123,
            "localization": {"pose_age_ms": 200},
            "local_obstacle": {"age_ms": 300},
            "safety": {
                "allow_navigation": allowed,
                "reason": reason,
                "recommended_mode": "normal" if allowed else "hold",
            },
        },
    }


class MissionDecisionTests(unittest.TestCase):
    def build(self, state: dict, *, execute: bool = True, registry_allowed: bool = True) -> dict:
        allowed, reason = gateway_allows_navigation(state)
        decision = build_mission_decision(
            navigation_queue(),
            execute_requested=execute,
            registry_allowed=registry_allowed,
            registry_reason="map identity matches" if registry_allowed else "map_id mismatch",
            topology_allowed=True,
            topology_reason="topology target allows navigation",
            gateway_checked=True,
            gateway_allowed=allowed,
            gateway_reason=reason,
            gateway_state=state,
            timestamp_ms=123,
        )
        DECISION_VALIDATOR.validate(decision)
        return decision

    def test_all_gates_pass_to_python_supervisor(self) -> None:
        decision = self.build(gateway_state("ok", allowed=True))

        self.assertEqual(decision["decision"], "execute_queue")
        self.assertTrue(decision["motion_allowed"])
        self.assertEqual(decision["execution_owner"], "python_persistent_supervisor")
        self.assertFalse(decision["llm_direct_motion"])
        self.assertTrue(decision["gateway_final_authority"])

    def test_dry_run_remains_available_when_gateway_is_offline(self) -> None:
        state = {"accepted": False, "reason": "gateway_preflight_error", "error": "connection refused"}
        decision = self.build(state, execute=False)

        self.assertEqual(decision["decision"], "dry_run_queue")
        self.assertFalse(decision["motion_allowed"])
        self.assertEqual(decision["preflight"]["gateway"]["gateway_reason"], "gateway_preflight_error")

    def test_fault_matrix_fails_closed_with_gateway_reason(self) -> None:
        cases = {
            "lost_localization": "localization_not_valid",
            "stale_sensor": "local_obstacle_not_fresh",
        }
        for label, gateway_reason in cases.items():
            with self.subTest(label=label):
                decision = self.build(gateway_state(gateway_reason, allowed=False))
                self.assertEqual(decision["decision"], "hold")
                self.assertEqual(decision["reason_code"], "gateway_blocked")
                self.assertIn(gateway_reason, decision["reason"])
                self.assertEqual(
                    decision["preflight"]["gateway"]["safety"]["reason"],
                    gateway_reason,
                )

    def test_map_mismatch_is_rejected_before_execution(self) -> None:
        decision = self.build(
            gateway_state("ok", allowed=True),
            registry_allowed=False,
        )

        self.assertEqual(decision["decision"], "reject")
        self.assertEqual(decision["reason_code"], "registry_blocked")
        self.assertIn("map_id mismatch", decision["reason"])

    def test_network_loss_fails_closed_for_real_execution(self) -> None:
        state = {"accepted": False, "reason": "gateway_preflight_error", "error": "connection refused"}
        decision = self.build(state)

        self.assertEqual(decision["decision"], "hold")
        self.assertFalse(decision["motion_allowed"])
        self.assertIn("gateway_preflight_error", decision["reason"])

    def test_invalid_queue_is_rejected(self) -> None:
        queue = navigation_queue()
        queue["steps"] = []
        decision = build_mission_decision(
            queue,
            execute_requested=True,
            registry_allowed=True,
            registry_reason="ok",
            topology_allowed=True,
            topology_reason="ok",
            gateway_checked=True,
            gateway_allowed=True,
            gateway_reason="ok",
            timestamp_ms=123,
        )

        self.assertEqual(decision["decision"], "reject")
        self.assertEqual(decision["reason_code"], "invalid_task_queue")
        self.assertFalse(decision["motion_allowed"])

    def test_recovery_decision_bounds_llm_reposition(self) -> None:
        analysis = {
            "required": True,
            "candidates": [
                {"direction": "left", "distance_m": 0.5},
            ],
        }
        accepted = build_recovery_decision(
            analysis,
            {
                "action": "reposition",
                "direction": "left",
                "distance_m": 0.3,
                "confidence": 0.9,
                "reason": "left has usable translation clearance",
                "source": "local_llm",
            },
            timestamp_ms=123,
        )
        rejected = build_recovery_decision(
            analysis,
            {
                "action": "reposition",
                "direction": "right",
                "distance_m": 0.3,
                "confidence": 0.9,
                "reason": "move right",
                "source": "local_llm",
            },
            timestamp_ms=124,
        )

        self.assertEqual(accepted["decision"], "execute_reposition")
        self.assertEqual(
            accepted["authorized_command"]["direction"],
            "left",
        )
        self.assertFalse(accepted["llm_direct_motion"])
        self.assertFalse(rejected["accepted"])
        self.assertEqual(
            rejected["reason_code"],
            "reposition_direction_not_in_candidates",
        )


if __name__ == "__main__":
    unittest.main()
