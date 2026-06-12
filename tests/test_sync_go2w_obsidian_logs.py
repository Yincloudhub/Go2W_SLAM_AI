from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

from scripts.sync_go2w_obsidian_logs import sync_logs


class SyncGo2wObsidianLogsTests(unittest.TestCase):
    def test_sync_copies_markdown_logs_without_deleting_existing_notes(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            source = root / "source"
            vault = root / "vault"
            destination = vault / "机器狗" / "现场日志"
            source.mkdir()
            (vault / ".obsidian").mkdir(parents=True)
            destination.mkdir(parents=True)
            (source / "2026-06-12-log.md").write_text("# log\n", encoding="utf-8")
            (source / "ignore.txt").write_text("ignore\n", encoding="utf-8")
            (destination / "old.md").write_text("# old\n", encoding="utf-8")

            written = sync_logs(source, vault)

            self.assertEqual(written, [destination / "2026-06-12-log.md"])
            self.assertEqual(written[0].read_text(encoding="utf-8"), "# log\n")
            self.assertTrue((destination / "old.md").is_file())
            self.assertFalse((destination / "ignore.txt").exists())


if __name__ == "__main__":
    unittest.main()
