from __future__ import annotations

import argparse
import base64
import json
import shlex
import sys
from typing import Any


try:
    sys.stdout.reconfigure(encoding="utf-8")
    sys.stderr.reconfigure(encoding="utf-8")
except (AttributeError, ValueError):
    pass


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Encode a UTF-8 GO2W command for safe shell/SSH transport.")
    parser.add_argument("text", nargs="*", help="Command text. If omitted, stdin is used.")
    parser.add_argument("--mode", choices=["go", "command", "say"], default="go")
    parser.add_argument("--execute", action="store_true", help="Append --execute to the example command for go/command modes.")
    parser.add_argument("--full-output", action="store_true", help="Append --full-output instead of --brief for go mode.")
    parser.add_argument("--pretty", action="store_true")
    return parser


def shell_join(parts: list[str]) -> str:
    return " ".join(shlex.quote(part) for part in parts)


def build_examples(text: str, encoded: str, *, mode: str, execute: bool, full_output: bool) -> dict[str, Any]:
    flag = {"go": "--go-b64", "command": "--command-b64", "say": "--say-b64"}[mode]
    agent_args = ["scripts/go2w_agent_entry.py", flag, encoded]
    if mode in {"go", "command"}:
        agent_args.append("--execute" if execute else "--dry-run")
        agent_args.append("--full-output" if full_output else "--brief")
    linux_parts = ["PYTHONPATH=src", "python3", *agent_args]
    powershell_parts = ["python", *agent_args]
    return {
        "text": text,
        "mode": mode,
        "base64": encoded,
        "flag": flag,
        "linux_command": shell_join(linux_parts),
        "powershell_command": "$env:PYTHONPATH='src'; " + " ".join(powershell_parts),
    }


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    text = " ".join(args.text).strip()
    if not text:
        text = sys.stdin.read().strip()
    if not text:
        raise SystemExit("missing text to encode")
    encoded = base64.b64encode(text.encode("utf-8")).decode("ascii")
    payload = build_examples(text, encoded, mode=args.mode, execute=args.execute, full_output=args.full_output)
    if args.pretty:
        print(json.dumps(payload, ensure_ascii=False, indent=2))
    else:
        print(json.dumps(payload, ensure_ascii=False, separators=(",", ":")))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
