import tempfile
import unittest
from pathlib import Path

from scripts.capture_go2w_field_acceptance import (
    build_parser,
    extract_last_json,
    load_json,
    selected_map,
    sha256_file,
)


class FieldAcceptanceCaptureTests(unittest.TestCase):
    def test_extract_last_json_ignores_prefix(self) -> None:
        value = extract_last_json('ready\n{"accepted":true}\n{"stage":"status"}\n')
        self.assertEqual(value, {"stage": "status"})

    def test_load_json_and_select_map(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / "registry.json"
            path.write_text(
                '{"maps":[{"map_id":"site","pcd_path":"/tmp/site.pcd"}]}',
                encoding="utf-8",
            )
            registry = load_json(path)
            self.assertEqual(selected_map(registry, "site")["pcd_path"], "/tmp/site.pcd")

    def test_missing_map_is_none(self) -> None:
        self.assertIsNone(selected_map({"maps": []}, "missing"))

    def test_sha256_file_is_stable(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / "value.txt"
            path.write_bytes(b"go2w\n")
            self.assertEqual(
                sha256_file(path),
                "ae66bd4984420eb25821ff200b3e01e86f67614d1b2f12cf812741b2e23c5e54",
            )

    def test_verify_anchor_is_explicit_and_optional(self) -> None:
        parser = build_parser()
        base = parser.parse_args(["--test-id", "baseline"])
        verified = parser.parse_args(
            ["--test-id", "origin", "--verify-anchor", "mapping_origin"]
        )
        self.assertEqual(base.verify_anchor, "")
        self.assertEqual(verified.verify_anchor, "mapping_origin")


if __name__ == "__main__":
    unittest.main()
