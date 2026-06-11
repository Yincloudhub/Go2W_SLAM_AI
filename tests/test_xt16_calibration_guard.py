from __future__ import annotations

import copy
import json
import unittest
from pathlib import Path

from scripts.xt16_calibration_guard import PARAMETER_ENV, validate_record


REPO_ROOT = Path(__file__).resolve().parents[1]
PENDING_RECORD = REPO_ROOT / "configs" / "perception" / "xt16_geometry_calibration.json"


class Xt16CalibrationGuardTests(unittest.TestCase):
    def setUp(self) -> None:
        self.record = json.loads(PENDING_RECORD.read_text(encoding="utf-8"))
        self.environment = {
            env_name: str(self.record["parameters"][key])
            for key, env_name in PARAMETER_ENV.items()
        }

    def verified_record(self) -> dict:
        record = copy.deepcopy(self.record)
        record["status"] = "verified"
        record["calibration_id"] = "xt16-field-20260609"
        return record

    def test_pending_record_cannot_enable_calibrated_mode(self) -> None:
        ok, reason, calibration_id = validate_record(self.record, self.environment)

        self.assertFalse(ok)
        self.assertIn("not verified", reason)
        self.assertEqual(calibration_id, "")

    def test_verified_record_requires_exact_runtime_parameters(self) -> None:
        environment = dict(self.environment)
        environment[PARAMETER_ENV["footprint_half_width_m"]] = "0.45"

        ok, reason, _ = validate_record(self.verified_record(), environment)

        self.assertFalse(ok)
        self.assertIn("footprint_half_width_m", reason)

    def test_matching_verified_record_returns_calibration_id(self) -> None:
        ok, reason, calibration_id = validate_record(self.verified_record(), self.environment)

        self.assertTrue(ok, reason)
        self.assertEqual(calibration_id, "xt16-field-20260609")


if __name__ == "__main__":
    unittest.main()
