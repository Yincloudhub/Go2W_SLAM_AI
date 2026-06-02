#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Sync curated GO2W project notes into the active local Obsidian vault."""

from __future__ import annotations

import argparse
import os
import shutil
from pathlib import Path
from typing import Iterable


START = "<!-- GO2W_FIELD_SYNC_START -->"
END = "<!-- GO2W_FIELD_SYNC_END -->"
DEFAULT_VAULT = Path("E:/codexprofile/obsidian_vault")


COPIES = (
    (
        Path("docs/GO2W_现场录点与晚间测试操作手册_2026-06-02.md"),
        Path("12-现场录点与晚间测试操作手册.md"),
    ),
    (
        Path("docs/multimodal_edge_autonomous_robot_plan.md"),
        Path("13-多模态边缘自主机器狗系统规划.md"),
    ),
    (
        Path("docs/edge_perception_node_contract.md"),
        Path("14-TI雷达NX边缘感知节点契约.md"),
    ),
    (
        Path("docs/go2w_near_field_collision_incident_2026-06-01.md"),
        Path("15-近场碰撞事故与恢复验证.md"),
    ),
)


INDEX_BLOCK = f"""{START}
## 2026-06-02 现场录点交接

- [[12-现场录点与晚间测试操作手册]]
- [[13-多模态边缘自主机器狗系统规划]]
- [[14-TI雷达NX边缘感知节点契约]]
- [[15-近场碰撞事故与恢复验证]]

当前操作边界：

- 机器人上电后先充电和复查，建议电量达到 `35%` 以上再做站立录点或真实运动。
- 沿用现有地图录点时不要开启建图。
- 真实运动必须满足 `loc=true`、`map=true`、`safety=ok` 和 D435 深度摘要新鲜。
- TI 雷达 / NX 当前只保留可选摘要接口，默认 `semantic_only`，不会授权运动。
{END}
"""


def upsert_block(text: str, block: str = INDEX_BLOCK) -> str:
    if START in text and END in text:
        before = text.split(START, 1)[0].rstrip()
        after = text.split(END, 1)[1].lstrip()
        return before + "\n\n" + block.rstrip() + "\n\n" + after
    return text.rstrip() + "\n\n" + block.rstrip() + "\n"


def update_frontmatter_date(text: str, updated: str = "2026-06-02") -> str:
    lines = text.splitlines()
    if not lines or lines[0] != "---":
        return text
    for index in range(1, min(len(lines), 30)):
        if lines[index].startswith("updated:"):
            lines[index] = f"updated: {updated}"
            return "\n".join(lines) + ("\n" if text.endswith("\n") else "")
        if lines[index] == "---":
            break
    return text


def copy_files(repo_root: Path, overview_dir: Path, copies: Iterable[tuple[Path, Path]] = COPIES) -> list[Path]:
    written: list[Path] = []
    overview_dir.mkdir(parents=True, exist_ok=True)
    for source_relative, destination_relative in copies:
        source = repo_root / source_relative
        if not source.is_file():
            raise FileNotFoundError(source)
        destination = overview_dir / destination_relative
        shutil.copyfile(source, destination)
        written.append(destination)
    return written


def sync_docs(repo_root: Path, vault: Path, desktop_copy: Path | None = None) -> list[Path]:
    if not (vault / ".obsidian").is_dir():
        raise FileNotFoundError(f"Obsidian vault marker is missing: {vault / '.obsidian'}")

    overview_dir = vault / "机器狗" / "GO2W边缘自治项目总览"
    written = copy_files(repo_root, overview_dir)

    index = overview_dir / "00-项目总览与阅读路径.md"
    if not index.is_file():
        raise FileNotFoundError(index)
    index_text = index.read_text(encoding="utf-8")
    next_text = update_frontmatter_date(upsert_block(index_text))
    index.write_text(next_text, encoding="utf-8")
    written.append(index)

    if desktop_copy is not None:
        desktop_copy.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(repo_root / COPIES[0][0], desktop_copy)
        written.append(desktop_copy)
    return written


def make_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--vault", default=os.environ.get("GO2W_OBSIDIAN_VAULT", str(DEFAULT_VAULT)))
    parser.add_argument("--repo-root", default=str(Path(__file__).resolve().parents[1]))
    parser.add_argument("--desktop-copy", default="")
    return parser


def main() -> int:
    args = make_parser().parse_args()
    desktop_copy = Path(args.desktop_copy).expanduser() if args.desktop_copy else None
    written = sync_docs(
        repo_root=Path(args.repo_root).expanduser().resolve(),
        vault=Path(args.vault).expanduser().resolve(),
        desktop_copy=desktop_copy,
    )
    for path in written:
        print(f"updated={path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
