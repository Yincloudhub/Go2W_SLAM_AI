from __future__ import annotations

import copy
import json
import unittest
from pathlib import Path

from scripts.xt16_calibration_guard import PARAMETER_ENV
from scripts.xt16_supervised_release_guard import validate_record


REPO_ROOT = Path(__file__).resolve().parents[1]
RELEASE_RECORD = REPO_ROOT / "configs" / "perception" / "xt16_supervised_release.json"


class Xt16SupervisedReleaseGuardTests(unittest.TestCase):
    def setUp(self) -> None:
        self.record = json.loads(RELEASE_RECORD.read_text(encoding="utf-8"))
        self.environment = {
            env_name: str(self.record["parameters"][key])
            for key, env_name in PARAMETER_ENV.items()
        }

    def test_checked_in_release_matches_runtime_parameters(self) -> None:
        ok, reason, release_id, max_speed_mps = validate_record(self.record, self.environment)

        self.assertTrue(ok, reason)
        self.assertEqual(release_id, "xt16-engineering-20260614")
        self.assertEqual(max_speed_mps, 0.1)

    def test_release_cannot_claim_formal_calibration(self) -> None:
        record = copy.deepcopy(self.record)
        record["formal_calibration"] = True

        ok, reason, _, _ = validate_record(record, self.environment)

        self.assertFalse(ok)
        self.assertIn("must not claim formal calibration", reason)

    def test_release_cannot_raise_speed_limit(self) -> None:
        record = copy.deepcopy(self.record)
        record["max_speed_mps"] = 0.2

        ok, reason, _, _ = validate_record(record, self.environment)

        self.assertFalse(ok)
        self.assertIn("exceeds 0.1", reason)

    def test_release_cannot_relax_front_departure_hard_stop(self) -> None:
        record = copy.deepcopy(self.record)
        record["hard_stop_m"]["front_departure"] = 0.5

        ok, reason, _, _ = validate_record(record, self.environment)

        self.assertFalse(ok)
        self.assertIn("changes hard stop: front_departure", reason)

    def test_release_cannot_change_side_advisory_contract(self) -> None:
        record = copy.deepcopy(self.record)
        record["advisory_clearance_m"]["side"] = 0.1

        ok, reason, _, _ = validate_record(record, self.environment)

        self.assertFalse(ok)
        self.assertIn("changes advisory clearance: side", reason)

    def test_release_requires_exact_runtime_parameters(self) -> None:
        environment = dict(self.environment)
        environment[PARAMETER_ENV["footprint_half_width_m"]] = "0.45"

        ok, reason, _, _ = validate_record(self.record, environment)

        self.assertFalse(ok)
        self.assertIn("footprint_half_width_m", reason)


if __name__ == "__main__":
    unittest.main()
