#!/usr/bin/env python3
"""Unified GO2W LLM agent CLI.

This entrypoint lets the robot choose a cloud or local LLM provider for a
plain message. It returns a chat reply plus a read-only mission preview. It
does not execute chassis motion.
"""

from __future__ import annotations

import argparse
import base64
import json
import os
import sys
from pathlib import Path
from typing import Any, List, Optional


REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = REPO_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))


try:
    sys.stdout.reconfigure(encoding="utf-8")
    sys.stderr.reconfigure(encoding="utf-8")
except (AttributeError, ValueError):
    pass


from edge_autonomy.mission_agent import (  # noqa: E402
    DEFAULT_LOCAL_ASK_SCRIPT,
    DEFAULT_LOCAL_MODEL,
    handle_agent_message,
)


DEFAULT_REGISTRY = str(REPO_ROOT / "configs" / "maps" / "go2w_multi_map_registry_v2.json")
DEFAULT_MAP_ID = "map_701"


def decode_message(args: argparse.Namespace) -> str:
    if args.message_b64:
        return base64.b64decode(args.message_b64).decode("utf-8")
    return args.message or ""


def print_json(value: Any, *, pretty: bool) -> None:
    if pretty:
        print(json.dumps(value, ensure_ascii=False, indent=2))
    else:
        print(json.dumps(value, ensure_ascii=False, separators=(",", ":")))


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="GO2W cloud/local LLM message agent.")
    parser.add_argument("--provider", choices=["cloud", "local"], default=os.environ.get("GO2W_LLM_PROVIDER", "cloud"))
    parser.add_argument("--message", default="", help="UTF-8 message text.")
    parser.add_argument("--message-b64", default="", help="UTF-8 base64 encoded message.")
    parser.add_argument("--registry", default=os.environ.get("GO2W_MAP_REGISTRY", DEFAULT_REGISTRY))
    parser.add_argument("--map-id", default=os.environ.get("GO2W_MAP_ID", DEFAULT_MAP_ID))
    parser.add_argument("--current-node", default=os.environ.get("GO2W_CURRENT_NODE", ""))
    parser.add_argument("--cloud-base-url", default=os.environ.get("GO2W_CLOUD_LLM_BASE_URL", ""))
    parser.add_argument("--cloud-model", default=os.environ.get("GO2W_CLOUD_LLM_MODEL", ""))
    parser.add_argument(
        "--cloud-api-key-env",
        default="GO2W_CLOUD_LLM_API_KEY",
        help="Environment variable name that stores the cloud LLM API key.",
    )
    parser.add_argument(
        "--cloud-api-key-file",
        default=os.environ.get("GO2W_CLOUD_LLM_API_KEY_FILE", ""),
        help="Optional file containing the cloud LLM API key.",
    )
    parser.add_argument("--local-ask-script", default=os.environ.get("GO2W_LOCAL_ASK_SCRIPT", DEFAULT_LOCAL_ASK_SCRIPT))
    parser.add_argument("--local-model", default=os.environ.get("GO2W_LOCAL_MODEL", DEFAULT_LOCAL_MODEL))
    parser.add_argument("--timeout-s", type=int, default=int(os.environ.get("GO2W_LLM_TIMEOUT_S", "45")))
    parser.add_argument("--max-tokens", type=int, default=int(os.environ.get("GO2W_LLM_MAX_TOKENS", "512")))
    parser.add_argument("--pretty", action="store_true")
    return parser


def main(argv: Optional[List[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    message = decode_message(args)
    if not message:
        print_json({"ok": False, "error": "missing message", "execution_enabled": False}, pretty=args.pretty)
        return 2
    if args.cloud_api_key_file:
        os.environ["GO2W_CLOUD_LLM_API_KEY_FILE"] = args.cloud_api_key_file
    cloud_key = os.environ.get(args.cloud_api_key_env, "")
    result = handle_agent_message(
        message,
        provider=args.provider,
        registry_path=args.registry,
        map_id=args.map_id,
        current_node=args.current_node,
        cloud_base_url=args.cloud_base_url,
        cloud_api_key=cloud_key,
        cloud_model=args.cloud_model,
        local_ask_script=args.local_ask_script,
        local_model=args.local_model,
        timeout_s=args.timeout_s,
        max_tokens=args.max_tokens,
    )
    print_json(result, pretty=args.pretty)
    return 0 if result.get("ok") else 1


if __name__ == "__main__":
    raise SystemExit(main())
