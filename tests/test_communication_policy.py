from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path

from jsonschema import Draft202012Validator

from edge_autonomy.communication_policy import (
    AppendOnlyJournal,
    CommunicationPolicyExecutor,
    JournalCorruptionError,
)


REPO_ROOT = Path(__file__).resolve().parents[1]
JOURNAL_SCHEMA = json.loads(
    (REPO_ROOT / "schemas" / "communication_journal_v1.schema.json").read_text(
        encoding="utf-8"
    )
)
JOURNAL_VALIDATOR = Draft202012Validator(JOURNAL_SCHEMA)


def communication_policy() -> dict:
    return {
        "mode": "semantic_only",
        "send": [
            "task_state",
            "mission_decision",
            "execution_state",
            "navigation_feedback",
        ],
        "drop": ["raw_video", "dense_pointcloud", "full_log", "high_rate_images"],
        "reason": "test",
    }


def navigation_queue(queue_id: str = "queue_remote_1") -> dict:
    return {
        "queue_id": queue_id,
        "mode": "sequential",
        "status": "planned",
        "source": "scripted",
        "targets": ["wp_1"],
        "steps": [
            {
                "task_id": "task_1",
                "action": "navigate",
                "status": "pending",
                "target_node": "wp_1",
            }
        ],
        "communication_policy": communication_policy(),
    }


class CommunicationPolicyTests(unittest.TestCase):
    def make_executor(
        self,
        root: str,
        *,
        link_state: str = "normal",
    ) -> CommunicationPolicyExecutor:
        return CommunicationPolicyExecutor(
            AppendOnlyJournal(
                Path(root) / "communication.jsonl",
                source_id="go2w_robot_test",
            ),
            communication_policy=communication_policy(),
            link_state=link_state,
        )

    def test_restart_recovers_monotonic_sequence_and_pending_events(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            first = self.make_executor(temp)
            event_1 = first.record_event(
                "task_state",
                {"status": "planned"},
                queue_id="queue_1",
                timestamp_ms=100,
            )
            event_2 = first.record_event(
                "mission_decision",
                {"decision": "dry_run_queue"},
                queue_id="queue_1",
                timestamp_ms=200,
            )

            restarted = self.make_executor(temp)
            recovered = restarted.journal.recover()
            event_3 = restarted.record_event(
                "execution_state",
                {"status": "completed", "dry_run": True},
                queue_id="queue_1",
                timestamp_ms=300,
            )

            self.assertEqual(event_1["sequence"], 1)
            self.assertEqual(event_2["sequence"], 2)
            self.assertEqual(recovered["last_sequence"], 2)
            self.assertEqual([item["sequence"] for item in recovered["pending"]], [1, 2])
            self.assertEqual(event_3["sequence"], 3)
            status = restarted.status()
            self.assertEqual(status["recovered_queue_count"], 1)
            self.assertFalse(
                status["recovered_queues"]["queue_1"]["automatic_resume_allowed"]
            )
            for record in restarted.journal.records():
                JOURNAL_VALIDATOR.validate(record)

    def test_ack_is_append_only_and_removes_only_acknowledged_prefix(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            executor = self.make_executor(temp)
            for index in range(3):
                executor.record_event(
                    "task_state",
                    {"index": index},
                    queue_id="queue_1",
                )

            executor.acknowledge(2)
            restarted = self.make_executor(temp)
            status = restarted.status()
            pending = restarted.pending_events()

            self.assertEqual(status["last_sequence"], 3)
            self.assertEqual(status["last_ack_sequence"], 2)
            self.assertEqual([event["sequence"] for event in pending], [3])
            with self.assertRaisesRegex(ValueError, "rollback"):
                restarted.acknowledge(1)

    def test_disconnected_never_calls_transport_and_recovered_replays_missing_only(
        self,
    ) -> None:
        with tempfile.TemporaryDirectory() as temp:
            disconnected = self.make_executor(temp, link_state="disconnected")
            disconnected.record_event(
                "task_state",
                {"status": "running"},
                queue_id="queue_1",
            )
            disconnected.record_event(
                "execution_state",
                {"status": "completed"},
                queue_id="queue_1",
            )
            calls: list[int] = []

            result = disconnected.drain(
                lambda event: calls.append(event["sequence"]) or event["sequence"]
            )

            self.assertEqual(result["attempted"], 0)
            self.assertEqual(calls, [])
            recovered = self.make_executor(temp, link_state="recovered")
            replayed: list[int] = []
            replay_result = recovered.drain(
                lambda event: replayed.append(event["sequence"])
                or {"ack_sequence": event["sequence"]}
            )

            self.assertEqual(replayed, [1, 2])
            self.assertEqual(replay_result["acknowledged"], 2)
            self.assertEqual(recovered.pending_events(), [])
            self.assertEqual(recovered.drain(lambda event: 0)["attempted"], 0)

    def test_missing_ack_keeps_event_pending_for_later_replay(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            executor = self.make_executor(temp, link_state="weak")
            executor.record_event(
                "task_state",
                {"status": "running"},
                queue_id="queue_1",
            )

            result = executor.drain(lambda event: None)

            self.assertEqual(result["stopped_reason"], "ack_missing_or_behind")
            self.assertEqual([event["sequence"] for event in executor.pending_events()], [1])

    def test_cumulative_ack_skips_already_covered_events_in_same_batch(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            executor = self.make_executor(temp, link_state="recovered")
            for index in range(3):
                executor.record_event(
                    "task_state",
                    {"index": index},
                    queue_id="queue_1",
                )
            sent: list[int] = []

            result = executor.drain(
                lambda event: sent.append(event["sequence"])
                or {"ack_sequence": 3}
            )

            self.assertEqual(sent, [1])
            self.assertEqual(result["acknowledged"], 1)
            self.assertEqual(executor.pending_events(), [])

    def test_replay_payload_strips_gateway_commands_and_raw_artifacts(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            executor = self.make_executor(temp)
            event = executor.record_event(
                "execution_state",
                {
                    "status": "completed",
                    "queue_execution": {
                        "events": [
                            {
                                "status": "ok",
                                "slam_command": {
                                    "action": "navigate_to_pose",
                                    "target_pose": {"x": 1.0, "y": 2.0},
                                },
                                "result": {"image_bytes": 1234},
                            }
                        ]
                    },
                },
                queue_id="queue_1",
            )

            serialized = json.dumps(event["payload"], ensure_ascii=False)
            self.assertNotIn("slam_command", serialized)
            self.assertNotIn("target_pose", serialized)
            self.assertNotIn("image_bytes", serialized)
            self.assertIn("completed", serialized)

    def test_execution_claim_survives_restart_and_blocks_duplicate_execution(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            executor = self.make_executor(temp)
            first = executor.reserve_execution(
                queue_id="queue_1",
                decision_id="decision_1",
            )
            restarted = self.make_executor(temp)
            second = restarted.reserve_execution(
                queue_id="queue_1",
                decision_id="decision_2",
            )

            self.assertTrue(first["claimed"])
            self.assertFalse(second["claimed"])
            self.assertIn("already", second["reason"])
            self.assertFalse(first["event"]["execution_directive"])
            self.assertNotIn("slam_command", first["event"]["payload"])

    def test_remote_task_routes_only_to_mission_decision_and_deduplicates(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            executor = self.make_executor(temp)
            message = {
                "schema_version": 1,
                "message_id": "remote_message_1",
                "source_id": "operator_cloud",
                "message_type": "task_proposal",
                "task_queue": navigation_queue(),
            }

            first = executor.route_remote_message(message)
            second = executor.route_remote_message(message)

            self.assertTrue(first["accepted"])
            self.assertEqual(first["route"], "mission_decision_engine")
            self.assertTrue(first["requires_mission_decision"])
            self.assertFalse(first["gateway_invoked"])
            self.assertFalse(first["motion_allowed"])
            self.assertFalse(second["accepted"])
            self.assertTrue(second["duplicate"])

    def test_remote_direct_gateway_or_pose_command_is_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            executor = self.make_executor(temp)
            direct_gateway = {
                "message_id": "remote_message_1",
                "message_type": "gateway_command",
                "gateway_command": {"action": "navigate"},
            }
            queue_with_pose = navigation_queue()
            queue_with_pose["steps"][0]["target_pose"] = {"x": 1.0, "y": 2.0}
            hidden_direct_command = {
                "message_id": "remote_message_2",
                "message_type": "task_proposal",
                "task_queue": queue_with_pose,
            }

            rejected_gateway = executor.route_remote_message(direct_gateway)
            rejected_pose = executor.route_remote_message(hidden_direct_command)

            self.assertFalse(rejected_gateway["accepted"])
            self.assertFalse(rejected_gateway["gateway_invoked"])
            self.assertFalse(rejected_pose["accepted"])
            self.assertIn("target_pose", rejected_pose["reason"])

    def test_remote_task_queue_rejects_unknown_step_fields(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            executor = self.make_executor(temp)
            queue = navigation_queue()
            queue["steps"][0]["shell_command"] = "ignored today but unsafe later"

            result = executor.route_remote_message(
                {
                    "message_id": "remote_message_unknown_field",
                    "message_type": "task_proposal",
                    "task_queue": queue,
                }
            )

            self.assertFalse(result["accepted"])
            self.assertIn("unsupported fields", result["reason"])

    def test_corrupt_or_truncated_journal_is_not_silently_replayed(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / "communication.jsonl"
            path.write_text('{"schema":"go2w_communication_journal_v1"', encoding="utf-8")
            journal = AppendOnlyJournal(path, source_id="go2w_robot_test")

            with self.assertRaises(JournalCorruptionError):
                journal.recover()

    def test_existing_lock_file_does_not_block_restart(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / "communication.jsonl"
            lock_path = path.with_name(path.name + ".lock")
            lock_path.write_bytes(b"0")
            executor = self.make_executor(temp)

            event = executor.record_event(
                "task_state",
                {"status": "planned"},
                queue_id="queue_1",
            )

            self.assertEqual(event["sequence"], 1)


if __name__ == "__main__":
    unittest.main()
