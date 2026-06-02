from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

from scripts.sync_go2w_obsidian_docs import END, START, export_overview, sync_docs


class SyncGo2wObsidianDocsTests(unittest.TestCase):
    def test_sync_docs_copies_curated_notes_and_updates_index_idempotently(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            repo = root / "repo"
            vault = root / "vault"
            overview = vault / "机器狗" / "GO2W边缘自治项目总览"
            desktop = root / "desktop" / "GO2W_Field_Runbook.md"
            (vault / ".obsidian").mkdir(parents=True)
            overview.mkdir(parents=True)
            (overview / "00-项目总览与阅读路径.md").write_text(
                "---\nupdated: 2026-05-24\n---\n\n# GO2W\n",
                encoding="utf-8",
            )

            sources = (
                "docs/GO2W_现场录点与晚间测试操作手册_2026-06-02.md",
                "docs/multimodal_edge_autonomous_robot_plan.md",
                "docs/edge_perception_node_contract.md",
                "docs/go2w_near_field_collision_incident_2026-06-01.md",
            )
            for index, source in enumerate(sources):
                path = repo / source
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_text(f"source-{index}\n", encoding="utf-8")

            sync_docs(repo, vault, desktop)
            sync_docs(repo, vault, desktop)

            index_text = (overview / "00-项目总览与阅读路径.md").read_text(encoding="utf-8")
            self.assertEqual(index_text.count(START), 1)
            self.assertEqual(index_text.count(END), 1)
            self.assertIn("updated: 2026-06-02", index_text)
            self.assertEqual(desktop.read_text(encoding="utf-8"), "source-0\n")
            self.assertEqual(
                (overview / "12-现场录点与晚间测试操作手册.md").read_text(encoding="utf-8"),
                "source-0\n",
            )

    def test_export_overview_only_copies_markdown_and_replaces_credentials(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            overview = root / "overview"
            export = root / "export"
            overview.mkdir()
            export.mkdir()
            (overview / "00-总览.md").write_text(
                "$env:GO2W_SSH_PASSWORD='robot-password'\n"
                "RADAR_STATION_SUDO_PASSWORD='radar-password'\n"
                "python snapshot.py --password 123\n",
                encoding="utf-8",
            )
            (overview / "backup.zip").write_bytes(b"not-for-git")
            (export / "stale.md").write_text("stale\n", encoding="utf-8")

            written = export_overview(overview, export)

            self.assertEqual([path.name for path in written], ["00-总览.md"])
            self.assertFalse((export / "stale.md").exists())
            self.assertFalse((export / "backup.zip").exists())
            self.assertEqual(
                (export / "00-总览.md").read_text(encoding="utf-8"),
                "$env:GO2W_SSH_PASSWORD='<现场密码>'\n"
                "RADAR_STATION_SUDO_PASSWORD='<sudo_password>'\n"
                "python snapshot.py --password <现场密码>\n",
            )


if __name__ == "__main__":
    unittest.main()
