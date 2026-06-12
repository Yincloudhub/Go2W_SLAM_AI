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
distances are compared with XT16 left/right clearances in at least three
stationary scenes.

The runtime now additionally requires
`configs/perception/xt16_geometry_calibration.json` to have status `verified`,
a non-empty calibration ID, and parameters identical to the running sidecar.
The checked-in record is intentionally `pending_field_measurement`.

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

## 2026-06-11 Closure Check

The current pose stream was independently checked against the map after the
latest manual movement. The live pose remained consistent with the map, so the
earlier distance disagreement is classified as physical movement/terrain-mode
sliding or odometry estimation, not a relocalization jump.

The execution chain now uses a persistent C++ session:

1. bind the session to the live SLAM map identity;
2. load one fixed topology registry snapshot;
3. resolve the named target to the registry's canonical pose;
4. require fresh localization and trusted calibrated XT16 geometry;
5. start a 500 ms heartbeat with a 2 s lease before submitting navigation;
6. monitor map identity, localization, and safety every 100 ms;
7. use an independent pause client if supervision is lost.

Live no-motion acceptance confirmed that `initial_point` reaches the final
safety gate but is rejected because the checked-in XT16 calibration record is
still `pending_field_measurement`. This is the first intended blocker. After
calibration, repeat the check in a scene whose measured side clearances also
pass the runtime thresholds before any supervised motion acceptance.

## 2026-06-11 Anchor Model Cleanup

- `mapping_origin_anchor_id=mapping_origin` records the unique build-map origin.
- The registry architecture permits multiple active verified relocalization
  anchors on the same PCD. At this checkpoint, `mapping_origin` is the only one
  that has completed field verification.
- The failed historical `initial_point` relocation observation and all other
  candidates were moved to `archived_relocalization_anchors`.
- The navigation topology node `initial_point` remains unchanged and cannot be
  used as a relocation pose.
- Automatic relocation from `--current-node` and raw `--init-x/--init-y`
  relocation were removed.
- Python, Web, the C++ operator panel, and the C++ gateway now reject archived,
  unverified, mismatched, or unconfirmed relocation requests.
- Consecutive localization verification now uses one persistent gateway client
  instead of spawning a new DDS subscriber for every sample.
- A later 2026-06-11 attempt incorrectly treated the robot's stated "initial
  point" as the build-map origin. The stationary pose diverged from
  `mapping_origin` by approximately 3.61 m, 5.62 m, then 7.10 m before pose
  output stopped. This attempt is invalid as anchor verification and no
  navigation command was issued.
- `mapping_origin` is no longer an implicit recovery default. The operator must
  select the active verified anchor that matches the robot's actual physical
  pose.
- Navigation preflight no longer requires the current pose to remain inside the
  selected relocation anchor envelope. Anchor distance/yaw checks apply only to
  relocation verification; navigation uses fresh SLAM/map identity and live
  safety gates.

## 2026-06-12 Restart and Relocation Investigation

- The robot was restarted and placed at the operator-reported build-map
  origin. XT16, Unitree SLAM, XT16 geometry, and forward stereo depth were
  restarted.
- The live scene was close to the successful June 9 baseline: front about
  `4.209 m`, left `0.570 m`, right `0.027 m`; forward depth center about
  `4.514 m`.
- The first explicit `mapping_origin` relocation failed with Unitree error
  `509` and ICP score `0.0325202`, slightly above the configured `0.03`
  threshold.
- One controlled retry returned accepted at the request layer, but Unitree SLAM
  did not publish a usable `/slam_info` or relocation odometry stream.
  Read-only status therefore remained `debug_map` / `not_started`.
- No navigation or chassis motion command was sent. The unlocalized state and
  pending XT16 calibration continued to block navigation.
- Investigation found a diagnostic defect: consecutive verification previously
  opened a persistent navigation session, whose startup correctly requires
  fresh localization. That prevented the verifier from observing the
  transition from `not_started` to `localized`. The verifier now uses a
  persistent read-only world-state session that can observe this transition
  and rejects every non-`get_world_state` action.

Current conclusion: the deterministic safety boundary behaved correctly. The
remaining field issue is repeatable physical alignment / vendor ICP and pose
publication, not a permitted unsafe navigation path.

## 2026-06-12 Closure Hardening

- Navigation now fails closed as soon as localization or SLAM health becomes
  `degraded`; the previous conservative-mode allowance was removed because the
  submitted Unitree goal speed was not actually reduced.
- Automatic pause uses up to three attempts. A rejected pause no longer clears
  the Python session's active state, so disconnect handling can try again.
- C++ OperatorPanel real commands now reuse the Python persistent supervised
  executor. Direct C++ queue navigation is explicitly disabled until it owns
  the same lease, request-correlation, and runtime monitoring contract.
- OperatorPanel relocation now runs the supervised relocation stage and reports
  success only after consecutive localization samples pass.
- Real map profiles must declare one `mapping_origin_anchor_id`.
- Historical topology edge distances are not exposed to the LLM unless an edge
  is explicitly marked `distance_verified=true`.
