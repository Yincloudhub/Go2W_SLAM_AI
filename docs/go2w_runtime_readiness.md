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
