# GO2W Field Acceptance Record - 2026-06-09

## Active Session

- Session: `20260609_restart_origin_then_forward_1p4m`
- Robot restarted and physically returned to the mapping origin.
- SLAM, XT16 geometry sidecar, and forward depth sidecar were started.
- `mapping_origin` relocation passed 5/5 consecutive localization samples.
- Relocation does not command chassis motion.

## Valid Measurements

Baseline:

- Artifact: `artifacts/supervised_acceptance/20260609_145933_restart_origin_tables_both_sides_baseline.json`
- Pose: `x=-0.2843`, `y=-0.1247`, `yaw=-0.0671 rad`
- Physical scene: tables on both sides.
- XT16 clearance: front `4.129 m`, left `0.612 m`, right `0.034 m`.
- Forward depth center sector: `4.151 m`.

After manual movement:

- Artifact: `artifacts/supervised_acceptance/20260609_150238_after_forward_1p4m_right_0p05m_tables_both_sides.json`
- Pose: `x=1.1320`, `y=-0.3074`, `yaw=-0.1220 rad`
- Operator confirmed terrain mode caused sliding and actual forward travel was about `1.4 m`.
- SLAM displacement: `1.428 m`; body-frame estimate: forward `1.425 m`, right `0.087 m`.
- XT16 front change: `4.129 -> 2.696 m` (`1.433 m`).
- Forward depth center-sector change: `4.151 -> 2.773 m` (`1.378 m`).

The three independent forward measurements agree. No SLAM scale defect is
indicated by this session.

## Sensor Semantics

- XT16 is the 360-degree source for robot-centric front, left, right, rear,
  body-height, and low-hazard clearances.
- The depth camera is forward-facing. Its `left` and `right` fields are image
  sectors inside the forward field of view, not robot-side clearances.
- Stereo depth may only reduce the fused front clearance.
- Stereo-only data cannot authorize navigation.

## XT16 Calibration Status

XT16 remains explicitly uncalibrated and navigation remains fail-closed.

At the second position, scanning footprint half-widths from `0.40` to `0.55 m`
kept the nearest right return at about `0.72 m` from the robot center. This
indicates a stable scene return, likely table geometry, rather than a return
from the robot body. Because the robot also moved forward `1.4 m`, the two
positions do not isolate lateral error against the same table edge.

Do not set `GO2W_XT16_GEOMETRY_CALIBRATED=1` until measured body-edge-to-table
distances are compared with XT16 left/right clearances in one stationary pose.

## Invalid Historical Samples

The earlier `right_box_before`, `after_forward_0p8m_right_clear`, and warmed
right-clear captures are not part of this session and must not be used as its
calibration baseline. The authoritative runtime validity ledger is:

`artifacts/supervised_acceptance/capture_validity.json`

## Current Safety State

- SLAM map identity: `/home/unitree/test.pcd`
- Localization: healthy after verified relocation
- XT16: live but `calibrated=false`
- Navigation: blocked by design
- LLM: may plan, explain, and request clarification, but cannot bypass map,
  localization, target-verification, perception, or gateway safety gates
