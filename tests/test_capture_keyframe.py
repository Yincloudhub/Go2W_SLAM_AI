from __future__ import annotations

import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
SCRIPT = REPO_ROOT / "scripts" / "capture_keyframe.py"


class CaptureKeyframeScriptTests(unittest.TestCase):
    def run_script(self, *args: str) -> tuple[int, dict]:
        completed = subprocess.run(
            [sys.executable, str(SCRIPT), *args],
            cwd=REPO_ROOT,
            text=True,
            encoding="utf-8",
            capture_output=True,
            timeout=15,
        )
        self.assertTrue(completed.stdout.strip(), completed.stderr)
        return completed.returncode, json.loads(completed.stdout.splitlines()[-1])

    def test_source_image_capture_writes_image_and_sidecar(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            source = root / "source.jpg"
            source.write_bytes(b"fake-jpeg")
            code, payload = self.run_script(
                "--target-node",
                "node_a",
                "--target-name",
                "Node A",
                "--run-id",
                "test_run",
                "--output-dir",
                str(root / "keyframes"),
                "--source-image",
                str(source),
            )

            self.assertEqual(code, 0, payload)
            self.assertTrue(payload["captured"])
            self.assertEqual(payload["target_node"], "node_a")
            self.assertEqual(payload["run_id"], "test_run")
            self.assertEqual(payload["image_bytes"], len(b"fake-jpeg"))
            self.assertTrue(Path(payload["image_path"]).exists())
            sidecar = Path(payload["sidecar_path"])
            self.assertTrue(sidecar.exists())
            sidecar_payload = json.loads(sidecar.read_text(encoding="utf-8"))
            self.assertTrue(sidecar_payload["captured"])

    def test_dry_run_writes_audit_sidecar_without_capture(self):
        with tempfile.TemporaryDirectory() as tmp:
            code, payload = self.run_script(
                "--target-node",
                "node_a",
                "--run-id",
                "dry_run",
                "--output-dir",
                str(Path(tmp) / "keyframes"),
                "--dry-run",
            )

            self.assertEqual(code, 0, payload)
            self.assertFalse(payload["captured"])
            self.assertEqual(payload["reason"], "dry_run")
            self.assertTrue(Path(payload["sidecar_path"]).exists())


if __name__ == "__main__":
    unittest.main()
