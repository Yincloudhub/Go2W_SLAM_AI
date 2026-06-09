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
  -> registered topology target
  -> fresh localization
  -> gateway safety
  -> fresh trusted XT16 summary
  -> queue preflight
  -> runtime watchdog marker
  -> Unitree navigation API
  -> runtime safety polling
  -> accepted pause on arrival, rejection, timeout, or stale state
```

An uncalibrated or stale trusted XT16 summary fails closed for navigation.
Manual relocation remains available so localization can be recovered.

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

Use the dedicated acceptance helper instead of remembering individual gateway
commands:

```bash
cd /home/unitree/Go2W_SLAM_AI
python3 scripts/go2w_supervised_acceptance.py --stage status
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
