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
                "docs/go2w_competition_edge_autonomy_handoff_20260612.md",
                "docs/stereo_depth_camera_integration.md",
                "docs/go2w_runtime_operations_runbook.md",
                "docs/perception_context_v1.md",
                "docs/mission_decision_v1.md",
                "docs/go2w_current_system_status_20260613.md",
                "docs/obsidian_go2w_logs/2026-06-12-GO2W定位安全闭环日志.md",
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
            self.assertIn("updated: 2026-06-13", index_text)
            self.assertIn("[[17-D435与DeepYOLO统一感知服务]]", index_text)
            self.assertIn("[[18-GO2W运行操作手册]]", index_text)
            self.assertIn("[[19-PerceptionContext-v1统一感知契约]]", index_text)
            self.assertIn("[[20-MissionDecision-v1与唯一执行链]]", index_text)
            self.assertIn("[[21-GO2W当前系统状态与实操]]", index_text)
            self.assertIn("P0-1 已完成", index_text)
            self.assertEqual(desktop.read_text(encoding="utf-8"), "source-0\n")
            self.assertEqual(
                (overview / "12-现场录点与晚间测试操作手册.md").read_text(encoding="utf-8"),
                "source-0\n",
            )
            self.assertEqual(
                (overview / "16-比赛边缘自治架构与新Session交接.md").read_text(encoding="utf-8"),
                "source-4\n",
            )
            self.assertEqual(
                (overview / "17-D435与DeepYOLO统一感知服务.md").read_text(encoding="utf-8"),
                "source-5\n",
            )
            self.assertEqual(
                (overview / "18-GO2W运行操作手册.md").read_text(encoding="utf-8"),
                "source-6\n",
            )
            self.assertEqual(
                (overview / "19-PerceptionContext-v1统一感知契约.md").read_text(encoding="utf-8"),
                "source-7\n",
            )
            self.assertEqual(
                (overview / "20-MissionDecision-v1与唯一执行链.md").read_text(encoding="utf-8"),
                "source-8\n",
            )
            self.assertEqual(
                (overview / "21-GO2W当前系统状态与实操.md").read_text(encoding="utf-8"),
                "source-9\n",
            )
            self.assertEqual(
                (vault / "机器狗" / "现场日志" / "2026-06-12-GO2W定位安全闭环日志.md").read_text(encoding="utf-8"),
                "source-10\n",
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
