#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Sync curated GO2W project notes between the repo and the active Obsidian vault."""

from __future__ import annotations

import argparse
import os
import re
import shutil
from pathlib import Path
from typing import Iterable


START = "<!-- GO2W_FIELD_SYNC_START -->"
END = "<!-- GO2W_FIELD_SYNC_END -->"
DEFAULT_VAULT = Path("E:/codexprofile/obsidian_vault")
DEFAULT_EXPORT_DIR = Path("docs/obsidian_go2w_overview")


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
    (
        Path("docs/go2w_competition_edge_autonomy_handoff_20260612.md"),
        Path("16-比赛边缘自治架构与新Session交接.md"),
    ),
    (
        Path("docs/stereo_depth_camera_integration.md"),
        Path("17-D435与DeepYOLO统一感知服务.md"),
    ),
    (
        Path("docs/go2w_runtime_operations_runbook.md"),
        Path("18-GO2W运行操作手册.md"),
    ),
    (
        Path("docs/perception_context_v1.md"),
        Path("19-PerceptionContext-v1统一感知契约.md"),
    ),
)


INDEX_BLOCK = f"""{START}
## 2026-06-12 比赛边缘自治交接

- [[12-现场录点与晚间测试操作手册]]
- [[13-多模态边缘自主机器狗系统规划]]
- [[14-TI雷达NX边缘感知节点契约]]
- [[15-近场碰撞事故与恢复验证]]
- [[16-比赛边缘自治架构与新Session交接]]
- [[17-D435与DeepYOLO统一感知服务]]
- [[18-GO2W运行操作手册]]
- [[19-PerceptionContext-v1统一感知契约]]

当前操作边界：

- 比赛唯一真实执行链为 `TaskQueue -> MissionDecisionEngine -> SLAM Gateway -> Unitree SDK`。
- Gateway 是最终运动权威，LLM 不得生成底层速度、任意坐标或 Unitree API ID。
- P0-1 已完成：D435 深度与 DeepYOLO 由单一采集 owner 驱动，旧入口只做兼容转发。
- P0-2 schema/loaders 已完成：旧 artifact 无生产者证据时必须 offline。
- TI 雷达 / NX 先以 `semantic_only` 接入，不直接授权运动。
- 弱网只影响远程同步，本地感知、任务状态机和 Gateway 不等待网络。
{END}
"""


def upsert_block(text: str, block: str = INDEX_BLOCK) -> str:
    if START in text and END in text:
        before = text.split(START, 1)[0].rstrip()
        after = text.split(END, 1)[1].lstrip()
        return before + "\n\n" + block.rstrip() + "\n\n" + after
    return text.rstrip() + "\n\n" + block.rstrip() + "\n"


def update_frontmatter_date(text: str, updated: str = "2026-06-13") -> str:
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


def sanitize_export_text(text: str) -> str:
    text = re.sub(
        r"(?i)(GO2W_SSH_PASSWORD\s*=\s*['\"])(?!<)[^'\"]+(['\"])",
        r"\1<现场密码>\2",
        text,
    )
    text = re.sub(
        r"(?i)(RADAR_STATION_SUDO_PASSWORD\s*=\s*['\"])(?!<)[^'\"]+(['\"])",
        r"\1<sudo_password>\2",
        text,
    )
    text = re.sub(
        r"(?i)(--password\s+)(?!<)\S+",
        r"\1<现场密码>",
        text,
    )
    return text.rstrip() + "\n"


def export_overview(overview_dir: Path, export_dir: Path) -> list[Path]:
    export_dir.mkdir(parents=True, exist_ok=True)
    for stale in export_dir.glob("*.md"):
        stale.unlink()

    written: list[Path] = []
    for source in sorted(overview_dir.glob("*.md")):
        destination = export_dir / source.name
        text = source.read_text(encoding="utf-8")
        destination.write_text(sanitize_export_text(text), encoding="utf-8")
        written.append(destination)
    return written


def sync_docs(
    repo_root: Path,
    vault: Path,
    desktop_copy: Path | None = None,
    *,
    export_dir: Path | None = None,
) -> list[Path]:
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

    if export_dir is not None:
        written.extend(export_overview(overview_dir, export_dir))

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
    parser.add_argument(
        "--export-overview",
        action="store_true",
        help="export only the GO2W overview Markdown notes into the repository",
    )
    return parser


def main() -> int:
    args = make_parser().parse_args()
    desktop_copy = Path(args.desktop_copy).expanduser() if args.desktop_copy else None
    repo_root = Path(args.repo_root).expanduser().resolve()
    written = sync_docs(
        repo_root=repo_root,
        vault=Path(args.vault).expanduser().resolve(),
        desktop_copy=desktop_copy,
        export_dir=repo_root / DEFAULT_EXPORT_DIR if args.export_overview else None,
    )
    for path in written:
        print(f"updated={path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
