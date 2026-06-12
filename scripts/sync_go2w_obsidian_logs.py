#!/usr/bin/env python3
"""Mirror repository GO2W field logs into the active Obsidian vault."""

from __future__ import annotations

import argparse
import shutil
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SOURCE = REPO_ROOT / "docs" / "obsidian_go2w_logs"
DEFAULT_VAULT = Path("E:/codexprofile/obsidian_vault")
VAULT_LOG_DIR = Path("\u673a\u5668\u72d7") / "\u73b0\u573a\u65e5\u5fd7"


def sync_logs(source: Path, vault: Path) -> list[Path]:
    marker = vault / ".obsidian"
    if not marker.is_dir():
        raise FileNotFoundError(f"Obsidian vault marker is missing: {marker}")
    if not source.is_dir():
        raise FileNotFoundError(f"GO2W log source is missing: {source}")

    destination_dir = vault / VAULT_LOG_DIR
    destination_dir.mkdir(parents=True, exist_ok=True)
    written: list[Path] = []
    for source_path in sorted(source.glob("*.md")):
        destination = destination_dir / source_path.name
        shutil.copyfile(source_path, destination)
        written.append(destination)
    return written


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, default=DEFAULT_SOURCE)
    parser.add_argument("--vault", type=Path, default=DEFAULT_VAULT)
    return parser


def main() -> int:
    args = build_parser().parse_args()
    written = sync_logs(args.source.resolve(), args.vault.resolve())
    for path in written:
        print(f"updated={path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
