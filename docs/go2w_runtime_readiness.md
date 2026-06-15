# GO2W Runtime Readiness

## One-command startup

Run on the robot:

```bash
cd /home/unitree/Go2W_SLAM_AI
bash scripts/start_go2w_runtime_stack.sh
```

This command checks or starts XT16 PTP, the XT16 driver, XT16 geometry,
Unitree SLAM, the unified D435 owner, PerceptionContext, and the structured
gateway probe. It waits for a real cold start and gives the gateway subscriber
time to initialize. It never sends relocation, navigation, mapping, or chassis
motion commands.

For the explicitly supervised field acceptance path:

```bash
bash scripts/start_go2w_runtime_stack.sh --supervised-engineering-release --json
```

This flag uses `configs/perception/xt16_supervised_release.json`. The record is
not a formal measured calibration and does not change
`xt16_geometry_calibration.json`. It requires an on-site operator and emergency
stop, caps navigation at `0.1 m/s`, and retains every stale, confidence,
missing-ROI, and close-obstacle hard stop.

## Readiness layers

The startup summary separates states that must not be collapsed into one flag:

1. `services_ready`: startup scripts and the gateway query path are working.
2. `relocalization_ready`: the gateway can accept an explicitly confirmed
   relocation request. Local obstacle clearance is not required because
   relocation initializes SLAM and does not command chassis motion.
3. `execution_arming_ready`: localization has a fresh valid pose. Enabling the
   operator execution mode does not mean a navigation command is safe.
4. `perception_ready`: the trusted XT16 summary is fresh and either formally
   calibrated or covered by the explicit supervised engineering release.
5. `navigation_ready`: SLAM, localization, gateway safety, and the current
   navigation obstacle checks all allow movement.
6. `llm_ready`: a local or HTTP LLM is available. This is diagnostic only and
   never grants motion permission.

## Safety boundary

The LLM may resolve ambiguous language, select registered topology targets, and
produce a validated task queue. It cannot bypass these deterministic checks:

```text
operator confirmation
  -> persistent C++ navigation session bound to current SLAM map
  -> fixed registry snapshot and canonical topology target
  -> fresh localization
  -> gateway safety
  -> fresh trusted XT16 summary
  -> queue preflight
  -> Unitree navigation API
  -> 500 ms heartbeat / 2 s lease
  -> 100 ms map, localization, and safety monitoring
  -> independent pause on disconnect, timeout, map change, or unsafe state
```

An uncalibrated or stale trusted XT16 summary fails closed for navigation
unless the explicit supervised engineering release is active. Manual
relocation remains available so localization can be recovered.

The supervised release uses `semantic_mobility_v5`. Registered targets pass
directly to Unitree `mode=0` navigation and its native obstacle avoidance.
GO2W does not approximate the robot's rotational swept footprint from a fixed
left/right clearance threshold. Only after native navigation reports failure
or sustained no progress can fresh XT16 geometry produce bounded forward,
backward, left, and right recovery candidates. The local LLM selects a
candidate from that set; `MissionDecisionEngine` validates the direction and
distance, and the Gateway executes it with
`SportClient::Move`, zero yaw rate, a 0.10 m/s speed cap, and direction-specific
clearance reserves. Each step is capped at 0.50 m, stops, refreshes world state,
and returns to the LLM before another reposition or mapped navigation decision.

This internal recovery loop does not expose general relative motion. A D435
front hard stop is also required to persist across two distinct fresh frames
when XT16 reports a clear front corridor. During supervised native navigation,
ordinary local proximity remains an advisory to avoid overriding Unitree's
planner; health, localization, sensor validity, lease loss, and explicit
emergency-stop conditions remain hard stops.

Build-map origin, relocation, and navigation use separate registry roles:

- `mapping_origin_anchor_id` identifies exactly one build-map coordinate origin.
- `relocalization_anchors` may contain multiple active verified poses from the
  same PCD. The build-map origin may also be used as one relocation anchor.
- `initial_point` is a navigation topology node only.
- failed and candidate relocation observations are retained under
  `archived_relocalization_anchors`; they are visible for audit but cannot emit
  or authorize a relocation command.

The current real-site registry has only one verified active relocation anchor,
`mapping_origin`; additional PCD locations become selectable only after field
verification. The C++ gateway is the final relocation authorization boundary. It requires
operator acknowledgement and exact agreement among `map_id`, `map_path`,
`anchor_id`, and the active verified anchor pose in the fixed startup registry
snapshot. Raw coordinates, archived anchors, and topology-node poses are
rejected.

The C++ gateway loads the deployed map registry once when the persistent
session starts. The request names a topology node, carries a pose assertion
for diagnostics, and may request a lower speed, but executable coordinates,
orientation, mode, and maximum speed come from the registry snapshot. Every
request/response pair has a `request_id`; late responses and asynchronous
lease events cannot be mistaken for the next command.

There is one physical map in this runtime: `/home/unitree/test.pcd`. Unitree
SLAM reports its backend name as `test`; the registry uses the logical profile
ID `go2w_real_site`. Both identifiers are bound to the same PCD path, and
navigation fails if that path changes.

The depth camera is forward-facing. Its `left_clearance_m` and
`right_clearance_m` values are left/right thirds of the forward image, not the
robot's true lateral clearances. Stereo depth may only tighten the front
clearance. XT16 remains authoritative for left, right, rear, body, and
low-hazard geometry, and stereo-only data cannot authorize navigation.

## Read-only acceptance

```bash
python3 scripts/go2w_startup_supervisor.py --run --json
```

To make a script fail unless supervised navigation is fully ready:

```bash
python3 scripts/go2w_startup_supervisor.py --run --require-navigation-ready
```

The output must always report:

```text
motion_commands_sent=false
```

until a separate, explicitly confirmed topology navigation command is issued.

## Guided supervised acceptance

The shortest operator entry point exposes no navigation execution action:

```bash
cd /home/unitree/Go2W_SLAM_AI
bash scripts/go2w_accept.sh status
bash scripts/go2w_accept.sh relocate SELECTED_ANCHOR confirm
bash scripts/go2w_accept.sh verify SELECTED_ANCHOR
bash scripts/go2w_accept.sh check TARGET_NODE
bash scripts/go2w_accept.sh snapshot TEST_ID [SELECTED_ANCHOR]
```

`check` only prints a separately reviewable motion command after all gates
pass. The wrapper never executes navigation.

`snapshot` is also read-only. It records the Git revision and dirty state,
registry and PCD hashes, XT16 calibration identity, compact sensor summaries,
processes, runtime status, operator measurements, and optionally five
consecutive localization samples for the selected anchor.

The underlying acceptance helper remains available for JSON output and advanced
diagnostics:

```bash
python3 scripts/go2w_supervised_acceptance.py --stage status --json
```

The tool has four explicit stages:

1. `status`: read-only readiness and world-state check.
2. `relocate`: only a verified registry anchor is accepted, and the exact
   anchor ID must also be supplied through `--confirm-relocation`. Relocation
   initializes SLAM coordinates and does not command chassis motion.
3. `verify-localization`: reuses one persistent read-only world-state gateway
   subscriber across all samples and checks pose age, SLAM health, monotonic
   timestamps, anchor radius, and yaw tolerance. This diagnostic session does
   not require localization to be healthy before it starts and cannot issue a
   motion or SLAM mutation command.
4. `prepare-navigation`: validates the target and live navigation gate, then
   prints a 0.1 m/s supervised command. It never executes navigation itself
   and does not require the robot to remain near its previous relocation anchor.

`prepare-navigation` fails closed unless all of these conditions hold:

- `xt16_driver` and `unitree_slam` are running;
- localization and pose timestamps are fresh and advancing;
- the active SLAM `map_path` matches the registry PCD;
- the target has the positive `live_verified` tag and no blocking tag;
- XT16 geometry either reports `calibrated=true` with a verified repository
  calibration ID, or reports an explicit guarded supervised release;
- the selected calibration or release record parameters exactly match the
  running sidecar;
- the trusted obstacle summary is fresh, confident, and clear;
- the gateway's own safety decision allows navigation.

The positive checks are deliberate. Missing map identity, calibration metadata,
timestamps, target verification, or sensor provenance are blockers rather than
defaults.

Example relocation after the operator physically confirms the robot is at the
verified build-map origin:

```bash
python3 scripts/go2w_supervised_acceptance.py \
  --stage relocate \
  --anchor mapping_origin \
  --confirm-relocation mapping_origin
```

Prepare, but do not execute, one short navigation target:

```bash
python3 scripts/go2w_supervised_acceptance.py \
  --stage prepare-navigation \
  --target TARGET_NODE
```

The relocation anchor is only an initialization and verification reference.
Once localization is healthy, normal movement away from that anchor must not
invalidate navigation. Navigation is gated by fresh SLAM localization, exact
map identity, target verification, calibrated perception, and live safety.

Only run the printed motion command while the robot remains in sight and the
operator has immediate access to the emergency stop.

## XT16 calibration provenance

The runtime calibration record is:

```text
configs/perception/xt16_geometry_calibration.json
```

`GO2W_XT16_GEOMETRY_CALIBRATED=1` is not sufficient by itself. Calibrated mode
is rejected unless the record status is exactly `verified`, it contains a
non-empty calibration ID, and every recorded geometry parameter matches the
sidecar's effective runtime parameter. Free-form extra geometry arguments are
not allowed in calibrated mode.

The checked-in record remains `pending_field_measurement`. Promote it only
after all five stationary scenes have physical body-edge-to-obstacle
measurements recorded and compared with XT16 output. Each scene must contain
25 unique point-cloud timestamps, remain within the configured stability and
effective-age limits, and have no stale reason other than the expected
`uncalibrated_xt16_geometry` marker.

The separate supervised engineering record is:

```text
configs/perception/xt16_supervised_release.json
```

`GO2W_XT16_SUPERVISED_RELEASE=1` is insufficient by itself. The release guard
requires exact runtime geometry parameters, operator-presence and emergency
stop declarations, a maximum speed no greater than `0.1 m/s`, immutable
engineering evidence hashes, Unitree obstacle-avoidance mode `0`, and the
unchanged `0.80 m` forward-departure hard stop.

## Live no-motion acceptance - 2026-06-11

Robot-side verification after rebuilding:

- gateway C++ smoke tests: `3/3` passed;
- host C++ smoke tests: `6/6` passed;
- Python tests at that checkpoint: `202/202` passed;
- active SLAM identity: `test` at `/home/unitree/test.pcd`;
- localization: `localized`, fresh pose;
- short-lived navigation client: rejected with
  `persistent_navigation_session_required`;
- unsupervised resume: rejected with
  `resume_requires_new_supervised_navigation_session`;
- persistent session: bound to the active map and fixed registry;
- verified `initial_point` request: rejected by `local_obstacle_not_fresh`
  because XT16 calibration is still pending.

No navigation goal or chassis command was accepted during this acceptance.

The count above is historical evidence, not the current suite total. Use the
latest field record and current test output for the active revision.

## Live no-motion closure verification - 2026-06-12

- gateway C++ smoke tests: `3/3` passed on the robot;
- host C++ smoke tests: `6/6` passed on the robot;
- Python tests: `229/229` passed locally and on the robot;
- shell syntax checks passed on the robot;
- persistent world-state session returned live read-only state and rejected
  `pause_navigation` with `world_state_session_is_read_only`;
- repeated localization verification returned five explicit `not_started`
  samples rather than failing during session startup;
- no navigation or chassis motion command was sent.

The remaining field blocker is physical/vendor relocation convergence and pose
publication. XT16 calibration also remains pending, so navigation remains
blocked even after localization recovers.

Startup now distinguishes two cases:

- XT16 has a process but no pointcloud: the startup script performs one
  controlled driver restart, tries reliable and best-effort QoS, then fails if
  the pointcloud is still silent.
- Unitree SLAM has no `/slam_info` before relocation: this is not treated as a
  stale process because the topic may legitimately start only after relocation.
  The required gateway probe and subsequent five-sample localization check
  determine readiness.

If an automatic navigation pause is rejected, the persistent gateway session
keeps a `pause_pending` fault and retries every second while the session is
alive. A failed pause is never reported as a completed stop.
