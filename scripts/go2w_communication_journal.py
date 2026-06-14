#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import os
import sys
from pathlib import Path
from typing import Any


REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = REPO_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

try:
    sys.stdout.reconfigure(encoding="utf-8")
    sys.stderr.reconfigure(encoding="utf-8")
except (AttributeError, ValueError):
    pass

from edge_autonomy.communication_policy import (  # noqa: E402
    AppendOnlyJournal,
    CommunicationPolicyExecutor,
)


DEFAULT_JOURNAL = (
    REPO_ROOT / "artifacts" / "communication" / "communication_journal_v1.jsonl"
)
DEFAULT_POLICY = {
    "mode": "semantic_only",
    "send": [
        "task_state",
        "mission_decision",
        "execution_state",
        "navigation_feedback",
        "risk_events",
        "keyframe",
        "world_state_summary",
    ],
    "drop": ["raw_video", "dense_pointcloud", "full_log", "high_rate_images"],
    "reason": "journal CLI defaults to bounded semantic synchronization",
}


def load_json(value: str, path: str) -> dict[str, Any]:
    if path:
        loaded = json.loads(Path(path).read_text(encoding="utf-8"))
    elif value:
        loaded = json.loads(value)
    else:
        loaded = {}
    if not isinstance(loaded, dict):
        raise ValueError("JSON payload must be an object")
    return loaded


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Inspect and exercise the GO2W append-only communication journal."
    )
    parser.add_argument(
        "action",
        choices=["status", "append", "ack", "replay", "route-remote"],
    )
    parser.add_argument(
        "--journal",
        default=os.environ.get("GO2W_COMMUNICATION_JOURNAL", str(DEFAULT_JOURNAL)),
    )
    parser.add_argument(
        "--source-id",
        default=os.environ.get("GO2W_COMMUNICATION_SOURCE_ID", "go2w_robot"),
    )
    parser.add_argument(
        "--link-state",
        choices=["normal", "weak", "disconnected", "recovered"],
        default=os.environ.get("GO2W_LINK_STATE", "normal"),
    )
    parser.add_argument("--event-type", default="task_state")
    parser.add_argument("--queue-id", default="")
    parser.add_argument("--message-id", default="")
    parser.add_argument("--payload-json", default="")
    parser.add_argument("--payload-file", default="")
    parser.add_argument("--ack-sequence", type=int)
    parser.add_argument("--limit", type=int, default=100)
    parser.add_argument("--pretty", action="store_true")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    executor = CommunicationPolicyExecutor(
        AppendOnlyJournal(args.journal, source_id=args.source_id),
        communication_policy=DEFAULT_POLICY,
        link_state=args.link_state,
    )

    if args.action == "status":
        output: dict[str, Any] = executor.status()
    elif args.action == "append":
        payload = load_json(args.payload_json, args.payload_file)
        output = executor.record_event(
            args.event_type,
            payload,
            queue_id=args.queue_id or None,
            message_id=args.message_id or None,
        )
    elif args.action == "ack":
        if args.ack_sequence is None:
            raise SystemExit("--ack-sequence is required for ack")
        output = executor.acknowledge(args.ack_sequence)
    elif args.action == "replay":
        output = {
            "status": executor.status(),
            "events": executor.replay_batch(limit=args.limit),
        }
    else:
        output = executor.route_remote_message(
            load_json(args.payload_json, args.payload_file)
        )

    if args.pretty:
        print(json.dumps(output, ensure_ascii=False, indent=2))
    else:
        print(json.dumps(output, ensure_ascii=False, separators=(",", ":")))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
