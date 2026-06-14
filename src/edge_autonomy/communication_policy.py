from __future__ import annotations

import copy
import json
import os
import time
from pathlib import Path
from typing import Any, Callable

from .task_queue import validate_communication_policy, validate_task_queue


JOURNAL_SCHEMA = "go2w_communication_journal_v1"
JOURNAL_SCHEMA_VERSION = 1
LINK_STATES = {"normal", "weak", "disconnected", "recovered"}
CORE_EVENT_TYPES = {
    "task_state",
    "mission_decision",
    "execution_claim",
    "execution_state",
    "navigation_feedback",
    "risk_event",
    "keyframe_index",
    "world_state_summary",
    "remote_task_proposal",
}
TERMINAL_EXECUTION_STATES = {"completed", "failed", "blocked", "cancelled"}
FORBIDDEN_REMOTE_KEYS = {
    "api_id",
    "angular_velocity",
    "gateway_command",
    "linear_velocity",
    "motion_command",
    "operator_ack",
    "slam_command",
    "target_pose",
    "unitree_api",
    "unitree_command",
    "velocity",
}
REPLAY_STRIP_KEYS = FORBIDDEN_REMOTE_KEYS | {
    "dense_pointcloud",
    "full_log",
    "high_rate_images",
    "image_bytes",
    "raw_video",
}
REMOTE_QUEUE_KEYS = {
    "communication_policy",
    "mode",
    "queue_id",
    "source",
    "status",
    "steps",
    "targets",
    "user_reply",
    "weak_link_payload",
}
REMOTE_STEP_KEYS = {
    "action",
    "condition",
    "message",
    "requires_preflight",
    "semantic_reason",
    "status",
    "step_id",
    "target_name",
    "target_node",
    "task_id",
}


class JournalCorruptionError(ValueError):
    pass


def _non_empty_string(value: Any) -> bool:
    return isinstance(value, str) and bool(value.strip())


def _json_clone(value: Any) -> Any:
    return json.loads(json.dumps(value, ensure_ascii=False))


def _contains_forbidden_remote_key(value: Any) -> str | None:
    if isinstance(value, dict):
        for key, child in value.items():
            if str(key) in FORBIDDEN_REMOTE_KEYS:
                return str(key)
            found = _contains_forbidden_remote_key(child)
            if found:
                return found
    elif isinstance(value, list):
        for child in value:
            found = _contains_forbidden_remote_key(child)
            if found:
                return found
    return None


def _sanitize_replay_payload(value: Any) -> Any:
    if isinstance(value, dict):
        return {
            str(key): _sanitize_replay_payload(child)
            for key, child in value.items()
            if str(key) not in REPLAY_STRIP_KEYS
        }
    if isinstance(value, list):
        return [_sanitize_replay_payload(child) for child in value]
    return _json_clone(value)


def _validate_remote_task_queue_shape(task_queue: dict[str, Any]) -> None:
    unknown_queue_keys = sorted(set(task_queue) - REMOTE_QUEUE_KEYS)
    if unknown_queue_keys:
        raise ValueError(
            "remote TaskQueue contains unsupported fields: "
            + ", ".join(unknown_queue_keys)
        )
    for index, step in enumerate(task_queue.get("steps", [])):
        if not isinstance(step, dict):
            continue
        unknown_step_keys = sorted(set(step) - REMOTE_STEP_KEYS)
        if unknown_step_keys:
            raise ValueError(
                f"remote TaskQueue step {index} contains unsupported fields: "
                + ", ".join(unknown_step_keys)
            )


class AppendOnlyJournal:
    """Durable JSONL journal with monotonic event sequences and append-only acks."""

    def __init__(
        self,
        path: str | Path,
        *,
        source_id: str,
        lock_timeout_s: float = 2.0,
    ) -> None:
        if not _non_empty_string(source_id):
            raise ValueError("source_id must be a non-empty string")
        self.path = Path(path)
        self.source_id = source_id.strip()
        self.lock_path = self.path.with_name(self.path.name + ".lock")
        self.lock_timeout_s = max(0.1, float(lock_timeout_s))

    def _acquire_lock(self) -> Any:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        deadline = time.monotonic() + self.lock_timeout_s
        while True:
            handle = self.lock_path.open("a+b")
            if self.lock_path.stat().st_size == 0:
                handle.write(b"0")
                handle.flush()
            handle.seek(0)
            try:
                if os.name == "nt":
                    import msvcrt

                    msvcrt.locking(handle.fileno(), msvcrt.LK_NBLCK, 1)
                else:
                    import fcntl

                    fcntl.flock(handle.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
                return handle
            except OSError:
                handle.close()
                if time.monotonic() >= deadline:
                    raise TimeoutError(f"timed out waiting for journal lock: {self.lock_path}")
                time.sleep(0.01)

    def _release_lock(self, handle: Any) -> None:
        try:
            handle.seek(0)
            if os.name == "nt":
                import msvcrt

                msvcrt.locking(handle.fileno(), msvcrt.LK_UNLCK, 1)
            else:
                import fcntl

                fcntl.flock(handle.fileno(), fcntl.LOCK_UN)
        finally:
            handle.close()

    def _append_record(self, record: dict[str, Any]) -> None:
        encoded = (
            json.dumps(record, ensure_ascii=False, separators=(",", ":")) + "\n"
        ).encode("utf-8")
        descriptor = os.open(
            self.path,
            os.O_CREAT | os.O_APPEND | os.O_WRONLY,
            0o600,
        )
        try:
            offset = 0
            while offset < len(encoded):
                written = os.write(descriptor, encoded[offset:])
                if written <= 0:
                    raise OSError(
                        f"short journal write stopped at {offset} of {len(encoded)} bytes"
                    )
                offset += written
            os.fsync(descriptor)
        finally:
            os.close(descriptor)

    def _read_records_unlocked(self) -> list[dict[str, Any]]:
        if not self.path.exists():
            return []
        records: list[dict[str, Any]] = []
        expected_sequence = 1
        last_ack_sequence = 0
        with self.path.open("r", encoding="utf-8") as handle:
            for line_number, raw_line in enumerate(handle, start=1):
                line = raw_line.strip()
                if not line:
                    continue
                try:
                    record = json.loads(line)
                except json.JSONDecodeError as exc:
                    raise JournalCorruptionError(
                        f"invalid JSON at journal line {line_number}: {exc.msg}"
                    ) from exc
                if not isinstance(record, dict):
                    raise JournalCorruptionError(
                        f"journal line {line_number} must be an object"
                    )
                if record.get("schema") != JOURNAL_SCHEMA:
                    raise JournalCorruptionError(
                        f"journal line {line_number} has an invalid schema"
                    )
                if record.get("schema_version") != JOURNAL_SCHEMA_VERSION:
                    raise JournalCorruptionError(
                        f"journal line {line_number} has an invalid schema_version"
                    )
                if record.get("source_id") != self.source_id:
                    raise JournalCorruptionError(
                        f"journal line {line_number} source_id does not match {self.source_id!r}"
                    )
                record_type = record.get("record_type")
                if record_type == "event":
                    sequence = record.get("sequence")
                    if sequence != expected_sequence:
                        raise JournalCorruptionError(
                            f"journal line {line_number} expected sequence "
                            f"{expected_sequence}, got {sequence!r}"
                        )
                    if record.get("event_id") != f"{self.source_id}:{sequence}":
                        raise JournalCorruptionError(
                            f"journal line {line_number} has an invalid event_id"
                        )
                    if record.get("event_type") not in CORE_EVENT_TYPES:
                        raise JournalCorruptionError(
                            f"journal line {line_number} has an invalid event_type"
                        )
                    expected_sequence += 1
                elif record_type == "ack":
                    ack_sequence = record.get("ack_sequence")
                    if not isinstance(ack_sequence, int) or ack_sequence < last_ack_sequence:
                        raise JournalCorruptionError(
                            f"journal line {line_number} has a non-monotonic ack_sequence"
                        )
                    if ack_sequence >= expected_sequence:
                        raise JournalCorruptionError(
                            f"journal line {line_number} acknowledges an unknown sequence"
                        )
                    last_ack_sequence = ack_sequence
                else:
                    raise JournalCorruptionError(
                        f"journal line {line_number} has an invalid record_type"
                    )
                records.append(record)
        return records

    def records(self) -> list[dict[str, Any]]:
        handle = self._acquire_lock()
        try:
            return self._read_records_unlocked()
        finally:
            self._release_lock(handle)

    @staticmethod
    def _state_from_records(records: list[dict[str, Any]]) -> dict[str, Any]:
        events = [record for record in records if record.get("record_type") == "event"]
        ack_records = [record for record in records if record.get("record_type") == "ack"]
        last_sequence = int(events[-1]["sequence"]) if events else 0
        last_ack_sequence = int(ack_records[-1]["ack_sequence"]) if ack_records else 0
        pending = [
            copy.deepcopy(record)
            for record in events
            if int(record["sequence"]) > last_ack_sequence
        ]
        queue_states: dict[str, dict[str, Any]] = {}
        remote_message_ids: set[str] = set()
        for event in events:
            queue_id = event.get("queue_id")
            if _non_empty_string(queue_id):
                state = queue_states.setdefault(
                    str(queue_id),
                    {
                        "queue_id": str(queue_id),
                        "task_state": None,
                        "mission_decision": None,
                        "execution_claim": None,
                        "execution_state": None,
                    },
                )
                event_type = str(event.get("event_type"))
                if event_type in state:
                    state[event_type] = copy.deepcopy(event.get("payload"))
            message_id = event.get("message_id")
            if _non_empty_string(message_id):
                remote_message_ids.add(str(message_id))
        return {
            "last_sequence": last_sequence,
            "last_ack_sequence": last_ack_sequence,
            "pending": pending,
            "queue_states": queue_states,
            "remote_message_ids": remote_message_ids,
        }

    def recover(self) -> dict[str, Any]:
        state = self._state_from_records(self.records())
        return {
            **state,
            "path": str(self.path),
            "source_id": self.source_id,
        }

    def append_event(
        self,
        event_type: str,
        payload: dict[str, Any],
        *,
        queue_id: str | None = None,
        message_id: str | None = None,
        timestamp_ms: int | None = None,
    ) -> dict[str, Any]:
        if event_type not in CORE_EVENT_TYPES:
            raise ValueError(f"unsupported event_type: {event_type!r}")
        if not isinstance(payload, dict):
            raise ValueError("payload must be an object")
        descriptor = self._acquire_lock()
        try:
            state = self._state_from_records(self._read_records_unlocked())
            sequence = int(state["last_sequence"]) + 1
            record = {
                "schema_version": JOURNAL_SCHEMA_VERSION,
                "schema": JOURNAL_SCHEMA,
                "record_type": "event",
                "source_id": self.source_id,
                "event_id": f"{self.source_id}:{sequence}",
                "sequence": sequence,
                "timestamp_ms": int(
                    timestamp_ms if timestamp_ms is not None else time.time() * 1000
                ),
                "event_type": event_type,
                "queue_id": queue_id if _non_empty_string(queue_id) else None,
                "message_id": message_id if _non_empty_string(message_id) else None,
                "replay_safe": True,
                "execution_directive": False,
                "payload": _json_clone(payload),
            }
            self._append_record(record)
            return copy.deepcopy(record)
        finally:
            self._release_lock(descriptor)

    def acknowledge(
        self,
        ack_sequence: int,
        *,
        timestamp_ms: int | None = None,
    ) -> dict[str, Any]:
        if not isinstance(ack_sequence, int) or isinstance(ack_sequence, bool):
            raise ValueError("ack_sequence must be an integer")
        descriptor = self._acquire_lock()
        try:
            state = self._state_from_records(self._read_records_unlocked())
            last_sequence = int(state["last_sequence"])
            last_ack_sequence = int(state["last_ack_sequence"])
            if ack_sequence < last_ack_sequence:
                raise ValueError(
                    f"ack_sequence rollback: current={last_ack_sequence}, requested={ack_sequence}"
                )
            if ack_sequence > last_sequence:
                raise ValueError(
                    f"ack_sequence {ack_sequence} exceeds last_sequence {last_sequence}"
                )
            if ack_sequence == last_ack_sequence:
                return {
                    "schema_version": JOURNAL_SCHEMA_VERSION,
                    "schema": JOURNAL_SCHEMA,
                    "record_type": "ack",
                    "source_id": self.source_id,
                    "ack_sequence": ack_sequence,
                    "timestamp_ms": int(
                        timestamp_ms
                        if timestamp_ms is not None
                        else time.time() * 1000
                    ),
                    "unchanged": True,
                }
            record = {
                "schema_version": JOURNAL_SCHEMA_VERSION,
                "schema": JOURNAL_SCHEMA,
                "record_type": "ack",
                "source_id": self.source_id,
                "ack_sequence": ack_sequence,
                "timestamp_ms": int(
                    timestamp_ms if timestamp_ms is not None else time.time() * 1000
                ),
            }
            self._append_record(record)
            return copy.deepcopy(record)
        finally:
            self._release_lock(descriptor)

    def claim_execution(
        self,
        *,
        queue_id: str,
        decision_id: str,
        timestamp_ms: int | None = None,
    ) -> dict[str, Any]:
        if not _non_empty_string(queue_id):
            raise ValueError("queue_id must be a non-empty string")
        if not _non_empty_string(decision_id):
            raise ValueError("decision_id must be a non-empty string")
        descriptor = self._acquire_lock()
        try:
            state = self._state_from_records(self._read_records_unlocked())
            queue_state = state["queue_states"].get(queue_id, {})
            existing_claim = queue_state.get("execution_claim")
            existing_execution = queue_state.get("execution_state")
            execution_blocks = bool(
                isinstance(existing_execution, dict)
                and not existing_execution.get("dry_run")
            )
            if existing_claim is not None or execution_blocks:
                return {
                    "claimed": False,
                    "queue_id": queue_id,
                    "reason": "queue already has an execution claim or state",
                    "existing_claim": copy.deepcopy(existing_claim),
                    "existing_execution": copy.deepcopy(existing_execution),
                }
            sequence = int(state["last_sequence"]) + 1
            payload = {
                "queue_id": queue_id,
                "decision_id": decision_id,
                "state": "reserved",
                "resume_policy": "manual_reconciliation_required",
            }
            record = {
                "schema_version": JOURNAL_SCHEMA_VERSION,
                "schema": JOURNAL_SCHEMA,
                "record_type": "event",
                "source_id": self.source_id,
                "event_id": f"{self.source_id}:{sequence}",
                "sequence": sequence,
                "timestamp_ms": int(
                    timestamp_ms if timestamp_ms is not None else time.time() * 1000
                ),
                "event_type": "execution_claim",
                "queue_id": queue_id,
                "message_id": None,
                "replay_safe": True,
                "execution_directive": False,
                "payload": payload,
            }
            self._append_record(record)
            return {"claimed": True, "queue_id": queue_id, "event": copy.deepcopy(record)}
        finally:
            self._release_lock(descriptor)

    def append_remote_proposal_once(
        self,
        *,
        message_id: str,
        queue_id: str,
        payload: dict[str, Any],
        timestamp_ms: int | None = None,
    ) -> dict[str, Any]:
        descriptor = self._acquire_lock()
        try:
            state = self._state_from_records(self._read_records_unlocked())
            if message_id in state["remote_message_ids"]:
                return {
                    "accepted": False,
                    "duplicate": True,
                    "reason": "remote message_id already journaled",
                }
            queue_state = state["queue_states"].get(queue_id, {})
            if queue_state.get("execution_claim") is not None:
                return {
                    "accepted": False,
                    "duplicate": True,
                    "reason": "queue already has an execution claim",
                }
            execution_state = queue_state.get("execution_state")
            if (
                isinstance(execution_state, dict)
                and not execution_state.get("dry_run")
                and execution_state.get("status") in TERMINAL_EXECUTION_STATES
            ):
                return {
                    "accepted": False,
                    "duplicate": True,
                    "reason": "queue already has a terminal execution state",
                }
            sequence = int(state["last_sequence"]) + 1
            record = {
                "schema_version": JOURNAL_SCHEMA_VERSION,
                "schema": JOURNAL_SCHEMA,
                "record_type": "event",
                "source_id": self.source_id,
                "event_id": f"{self.source_id}:{sequence}",
                "sequence": sequence,
                "timestamp_ms": int(
                    timestamp_ms if timestamp_ms is not None else time.time() * 1000
                ),
                "event_type": "remote_task_proposal",
                "queue_id": queue_id,
                "message_id": message_id,
                "replay_safe": True,
                "execution_directive": False,
                "payload": _json_clone(payload),
            }
            self._append_record(record)
            return {"accepted": True, "duplicate": False, "event": copy.deepcopy(record)}
        finally:
            self._release_lock(descriptor)


class CommunicationPolicyExecutor:
    """Journal-first communication policy that never owns robot execution."""

    def __init__(
        self,
        journal: AppendOnlyJournal,
        *,
        communication_policy: dict[str, Any],
        link_state: str = "normal",
    ) -> None:
        validate_communication_policy(communication_policy)
        if link_state not in LINK_STATES:
            raise ValueError(f"invalid link_state: {link_state!r}")
        self.journal = journal
        self.communication_policy = _json_clone(communication_policy)
        self.link_state = link_state

    def set_link_state(self, link_state: str) -> None:
        if link_state not in LINK_STATES:
            raise ValueError(f"invalid link_state: {link_state!r}")
        self.link_state = link_state

    def record_event(
        self,
        event_type: str,
        payload: dict[str, Any],
        *,
        queue_id: str | None = None,
        message_id: str | None = None,
        timestamp_ms: int | None = None,
    ) -> dict[str, Any]:
        return self.journal.append_event(
            event_type,
            _sanitize_replay_payload(payload),
            queue_id=queue_id,
            message_id=message_id,
            timestamp_ms=timestamp_ms,
        )

    def acknowledge(self, ack_sequence: int) -> dict[str, Any]:
        return self.journal.acknowledge(ack_sequence)

    def reserve_execution(self, *, queue_id: str, decision_id: str) -> dict[str, Any]:
        return self.journal.claim_execution(
            queue_id=queue_id,
            decision_id=decision_id,
        )

    def pending_events(self, *, limit: int | None = None) -> list[dict[str, Any]]:
        pending = self.journal.recover()["pending"]
        if limit is None:
            return pending
        return pending[: max(0, int(limit))]

    def replay_batch(self, *, limit: int = 100) -> list[dict[str, Any]]:
        if self.link_state == "disconnected":
            return []
        return self.pending_events(limit=limit)

    def drain(
        self,
        transport: Callable[[dict[str, Any]], int | dict[str, Any] | None],
        *,
        limit: int = 100,
    ) -> dict[str, Any]:
        if self.link_state == "disconnected":
            return {
                "link_state": self.link_state,
                "attempted": 0,
                "acknowledged": 0,
                "stopped_reason": "disconnected",
            }
        attempted = 0
        acknowledged = 0
        stopped_reason = "empty"
        acknowledged_through = int(self.journal.recover()["last_ack_sequence"])
        for event in self.replay_batch(limit=limit):
            if int(event["sequence"]) <= acknowledged_through:
                continue
            attempted += 1
            try:
                response = transport(copy.deepcopy(event))
            except Exception as exc:
                stopped_reason = f"transport_error:{exc}"
                break
            if isinstance(response, dict):
                ack_sequence = response.get("ack_sequence")
            else:
                ack_sequence = response
            if not isinstance(ack_sequence, int) or ack_sequence < int(event["sequence"]):
                stopped_reason = "ack_missing_or_behind"
                break
            self.acknowledge(ack_sequence)
            acknowledged_through = ack_sequence
            acknowledged += 1
            stopped_reason = "batch_complete"
        return {
            "link_state": self.link_state,
            "attempted": attempted,
            "acknowledged": acknowledged,
            "stopped_reason": stopped_reason,
        }

    def route_remote_message(self, message: dict[str, Any]) -> dict[str, Any]:
        if not isinstance(message, dict):
            return {
                "accepted": False,
                "reason": "remote message must be an object",
                "gateway_invoked": False,
                "motion_allowed": False,
            }
        message_type = message.get("message_type")
        if message_type != "task_proposal":
            return {
                "accepted": False,
                "reason": "only task_proposal messages may enter the local decision path",
                "gateway_invoked": False,
                "motion_allowed": False,
            }
        forbidden_key = _contains_forbidden_remote_key(message)
        if forbidden_key:
            return {
                "accepted": False,
                "reason": f"remote message contains forbidden execution field: {forbidden_key}",
                "gateway_invoked": False,
                "motion_allowed": False,
            }
        message_id = message.get("message_id")
        if not _non_empty_string(message_id):
            return {
                "accepted": False,
                "reason": "remote task_proposal requires message_id",
                "gateway_invoked": False,
                "motion_allowed": False,
            }
        task_queue = message.get("task_queue")
        try:
            validate_task_queue(task_queue)
            _validate_remote_task_queue_shape(task_queue)
        except (TypeError, ValueError) as exc:
            return {
                "accepted": False,
                "reason": f"invalid remote TaskQueue: {exc}",
                "gateway_invoked": False,
                "motion_allowed": False,
            }
        queue_id = str(task_queue["queue_id"])
        journal_result = self.journal.append_remote_proposal_once(
            message_id=str(message_id),
            queue_id=queue_id,
            payload={
                "message_id": str(message_id),
                "source_id": message.get("source_id"),
                "task_queue": task_queue,
                "route": "mission_decision_engine",
            },
        )
        if not journal_result.get("accepted"):
            return {
                "accepted": False,
                "duplicate": bool(journal_result.get("duplicate")),
                "reason": journal_result.get("reason"),
                "gateway_invoked": False,
                "motion_allowed": False,
            }
        return {
            "accepted": True,
            "duplicate": False,
            "route": "mission_decision_engine",
            "requires_mission_decision": True,
            "gateway_invoked": False,
            "motion_allowed": False,
            "task_queue": _json_clone(task_queue),
            "journal_event": journal_result.get("event"),
        }

    def status(self) -> dict[str, Any]:
        recovered = self.journal.recover()
        pending = recovered["pending"]
        recovered_queues: dict[str, dict[str, Any]] = {}
        for queue_id, queue_state in recovered["queue_states"].items():
            task_state = queue_state.get("task_state")
            mission_decision = queue_state.get("mission_decision")
            execution_claim = queue_state.get("execution_claim")
            execution_state = queue_state.get("execution_state")
            recovered_queues[queue_id] = {
                "task_status": (
                    task_state.get("status")
                    if isinstance(task_state, dict)
                    else None
                ),
                "decision": (
                    mission_decision.get("decision")
                    if isinstance(mission_decision, dict)
                    else None
                ),
                "execution_claimed": execution_claim is not None,
                "execution_status": (
                    execution_state.get("status")
                    if isinstance(execution_state, dict)
                    else None
                ),
                "dry_run": (
                    bool(execution_state.get("dry_run"))
                    if isinstance(execution_state, dict)
                    else False
                ),
                "automatic_resume_allowed": False,
            }
        return {
            "schema_version": 1,
            "schema": "go2w_communication_policy_status_v1",
            "link_state": self.link_state,
            "policy": _json_clone(self.communication_policy),
            "journal_path": str(self.journal.path),
            "source_id": self.journal.source_id,
            "last_sequence": recovered["last_sequence"],
            "last_ack_sequence": recovered["last_ack_sequence"],
            "pending_count": len(pending),
            "pending_first_sequence": pending[0]["sequence"] if pending else None,
            "pending_last_sequence": pending[-1]["sequence"] if pending else None,
            "replay_contains_execution_directives": any(
                event.get("execution_directive") is not False for event in pending
            ),
            "recovered_queue_count": len(recovered_queues),
            "recovered_queues": recovered_queues,
            "recovery_policy": "restore_state_without_automatic_motion_replay",
        }
