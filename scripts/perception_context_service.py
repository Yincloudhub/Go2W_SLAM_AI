#!/usr/bin/env python3
"""Publish the single bounded GO2W PerceptionContext v1 artifact."""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = REPO_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from edge_autonomy.perception_context import (  # noqa: E402
    SensorSequenceTracker,
    build_live_perception_context,
    load_perception_context_file,
    write_perception_context_file,
)


def publish_once(args: argparse.Namespace, tracker: SensorSequenceTracker) -> dict:
    context = build_live_perception_context(
        Path(args.repo_root).resolve(),
        sequence_tracker=tracker,
        ti_producer_instance_id=args.ti_producer_instance_id or None,
    )
    write_perception_context_file(Path(args.output), context)
    return context


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo-root", default=str(REPO_ROOT))
    parser.add_argument(
        "--output",
        default=str(REPO_ROOT / "artifacts" / "perception_context_v1.json"),
    )
    parser.add_argument("--interval-ms", type=int, default=250)
    parser.add_argument("--ti-producer-instance-id", default="")
    parser.add_argument("--once", action="store_true")
    parser.add_argument("--check", action="store_true")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    output = Path(args.output)
    if args.check:
        context = load_perception_context_file(output)
        if context is None:
            print(f"perception_context=unavailable_or_stale path={output}")
            return 1
        print(
            "perception_context=fresh"
            f" context_id={context['context_id']}"
            f" generated_at_ms={context['generated_at_ms']}"
            f" sources={len(context['sources'])}"
            f" degraded={len(context['degraded_capabilities'])}"
        )
        return 0

    tracker = SensorSequenceTracker()
    interval_s = max(0.05, int(args.interval_ms) / 1000.0)
    while True:
        publish_once(args, tracker)
        if args.once:
            return 0
        time.sleep(interval_s)


if __name__ == "__main__":
    raise SystemExit(main())
