# GO2W Runtime Readiness

## One-command startup

Run on the robot:

```bash
cd /home/unitree/Go2W_SLAM_AI
bash scripts/start_go2w_runtime_stack.sh
```

This command starts/checks the XT16 driver, XT16 geometry sidecar, Unitree
SLAM, and the structured gateway probe. It never sends relocation, navigation,
mapping, or chassis motion commands.

## Readiness layers

The startup summary separates states that must not be collapsed into one flag:

1. `services_ready`: startup scripts and the gateway query path are working.
2. `relocalization_ready`: the gateway can accept an explicitly confirmed
   relocation request. Local obstacle clearance is not required because
   relocation initializes SLAM and does not command chassis motion.
3. `execution_arming_ready`: localization has a fresh valid pose. Enabling the
   operator execution mode does not mean a navigation command is safe.
4. `perception_ready`: the trusted XT16 summary is fresh and calibrated.
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

An uncalibrated or stale trusted XT16 summary fails closed for navigation.
Manual relocation remains available so localization can be recovered.

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
bash scripts/go2w_accept.sh relocate mapping_origin
bash scripts/go2w_accept.sh verify mapping_origin
bash scripts/go2w_accept.sh check TARGET_NODE
```

`check` only prints a separately reviewable motion command after all gates
pass. The wrapper never executes navigation.

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
3. `verify-localization`: samples localization repeatedly and checks pose age,
   SLAM health, monotonic timestamps, anchor radius, and yaw tolerance.
4. `prepare-navigation`: validates the target and live navigation gate, then
   prints a 0.1 m/s supervised command. It never executes navigation itself.

`prepare-navigation` fails closed unless all of these conditions hold:

- `xt16_driver` and `unitree_slam` are running;
- localization and pose timestamps are fresh and advancing;
- the active SLAM `map_path` matches the registry PCD;
- the relocation anchor has an explicit `verified`/`verified_*` status;
- the target has the positive `live_verified` tag and no blocking tag;
- XT16 geometry explicitly reports `calibrated=true`;
- XT16 reports a non-empty calibration ID from a verified repository
  calibration record whose parameters exactly match the running sidecar;
- the trusted obstacle summary is fresh, confident, and clear;
- the gateway's own safety decision allows navigation.

The positive checks are deliberate. Missing map identity, calibration metadata,
timestamps, target verification, or sensor provenance are blockers rather than
defaults.

Example relocation after the operator physically confirms the robot is at the
verified mapping origin:

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
  --anchor mapping_origin \
  --target TARGET_NODE
```

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
after at least three stationary scenes have physical body-edge-to-obstacle
measurements recorded and compared with XT16 output.

## Live no-motion acceptance - 2026-06-11

Robot-side verification after rebuilding:

- gateway C++ smoke tests: `3/3` passed;
- host C++ smoke tests: `6/6` passed;
- Python tests: `202/202` passed;
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
