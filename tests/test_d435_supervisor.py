import json
import tempfile
import unittest
from pathlib import Path

from scripts.go2w_d435_supervisor import diagnose_summary


class D435SupervisorTests(unittest.TestCase):
    def test_diagnosis_exposes_online_owner_and_freshness_evidence(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / "summary.json"
            path.write_text(
                json.dumps(
                    {
                        "timestamp_ms": 9_900,
                        "status": "fresh",
                        "stale": False,
                        "frame_sequence": 42,
                        "owner": {
                            "pid": 123,
                            "running": True,
                            "status": "owned",
                            "process_start_ticks": 456,
                        },
                        "capture": {"status": "fresh", "frame_sequence": 42},
                    }
                ),
                encoding="utf-8",
            )

            diagnosis = diagnose_summary(path, now_ms=10_000)

        self.assertTrue(diagnosis["available"])
        self.assertEqual(diagnosis["age_ms"], 100)
        self.assertEqual(diagnosis["owner"]["pid"], 123)
        self.assertTrue(diagnosis["owner"]["running"])
        self.assertEqual(diagnosis["capture"]["status"], "fresh")

    def test_diagnosis_reports_missing_summary(self) -> None:
        diagnosis = diagnose_summary(Path("missing-d435-summary.json"), now_ms=10_000)

        self.assertFalse(diagnosis["available"])
        self.assertEqual(diagnosis["reason"], "summary_missing")


if __name__ == "__main__":
    unittest.main()
