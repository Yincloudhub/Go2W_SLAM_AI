from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any


MOJIBAKE_TERMS = ("\u951f", "\u9471", "\u9419", "\u95c2", "\u62f7", "\u5b21", "\u704f", "\u7ad9\u6e74\u5730", "\ufffd")
PLACEHOLDERS = {"new target", "wp_new", "\u65b0\u76ee\u6807\u70b9"}


def walk(value: Any, path: str = "$") -> list[tuple[str, str]]:
    findings: list[tuple[str, str]] = []
    if isinstance(value, dict):
        for key, item in value.items():
            findings.extend(walk(item, f"{path}.{key}"))
    elif isinstance(value, list):
        for index, item in enumerate(value):
            findings.extend(walk(item, f"{path}[{index}]"))
    elif isinstance(value, str):
        stripped = value.strip()
        if stripped and set(stripped) <= {"?"}:
            findings.append((path, "all question marks"))
        elif "??" in value:
            findings.append((path, "contains repeated question marks"))
        elif any(term in value for term in MOJIBAKE_TERMS):
            findings.append((path, "possible mojibake or replacement character"))
        elif any("\u0700" <= ch <= "\u07ff" for ch in value):
            findings.append((path, "contains suspicious non-CJK unicode"))
        elif stripped in PLACEHOLDERS and ".topology_nodes[" in path:
            findings.append((path, "placeholder text left in topology registry"))
    return findings


def main() -> int:
    parser = argparse.ArgumentParser(description="Scan GO2W JSON files for likely waypoint text corruption.")
    parser.add_argument("json_file")
    args = parser.parse_args()

    path = Path(args.json_file)
    data = json.loads(path.read_text(encoding="utf-8"))
    findings = walk(data)
    if findings:
        print(f"Potential text corruption in {path}:")
        for item_path, reason in findings:
            print(f"- {item_path}: {reason}")
        return 1
    print(f"OK: no obvious waypoint text corruption in {path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
