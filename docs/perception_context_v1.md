# GO2W PerceptionContext v1 Contract

更新：2026-06-13

## Scope

`SensorEnvelope v1` is the only normalized sensor input to
`PerceptionContext v1`. It carries compact semantics only. Raw point clouds,
continuous video, radar ADC, and other high-rate streams remain inside their
sensor producer.

The first P0-2 commit defines:

- `schemas/sensor_envelope_v1.schema.json`
- `schemas/perception_context_v1.schema.json`
- `src/edge_autonomy/perception_context.py`
- adapters for XT16 geometry, D435 depth, D435 YOLO, and TI/NX summaries
- reserved offline envelopes for future IMU and odometry motion summaries

WorldState, C++ LLM, and UI migration are separate commits. Until that
migration is complete, this module must not be described as the active motion
control path.

## SensorEnvelope v1

Every source contains:

```text
source_id
source_kind
timestamp_ms
received_ms
sequence
age_ms
stale_ms
frame_id
status
confidence
calibration_status
calibration_id
producer
status_reasons
payload
```

The only source statuses are:

```text
fresh | stale | offline | invalid | uncalibrated
```

`sequence` remains nullable for offline/invalid reservations, but every
`fresh` envelope must carry a native non-negative sequence. The XT16 producer
now publishes its written-summary count, D435 publishes the RGBD frameset
sequence, and the TI/NX v1 bridge contract requires its own sequence. Adapters
must not invent a missing counter. `SensorSequenceTracker` rejects rollback
within one stable producer instance.

Calibration status is independent of transport freshness:

```text
verified | pending | not_required | invalid | unknown
```

XT16 requires verified project calibration before its envelope can be
`fresh`. D435 currently reports `unknown` project calibration because the
unified artifact has no project calibration identity. TI/NX remains
`semantic_only`; `safety_wired` is always false in the generic adapter.

## Fail-Closed Status Rules

Status precedence is:

1. `invalid`: malformed JSON, wrong identity/schema, invalid types, future
   timestamp, or D435 sequence rollback.
2. `offline`: artifact missing/unreadable or required producer evidence absent.
3. `uncalibrated`: XT16 producer is online and timely but project calibration
   is not verified.
4. `stale`: producer is online but declares stale/unhealthy or exceeds its
   stale budget.
5. `fresh`: all source-specific checks pass.

File existence alone never proves online status:

- XT16 requires its PID file and a matching
  `xt16_lidar_geometry_summary.py` process.
- D435 requires the embedded owner PID, owner state, expected process, and a
  matching live process.
- TI/NX requires explicit bridge-online evidence from its transport manager.
  The current repository has no TI/NX bridge supervisor, so the default is
  `offline`.

Every fresh source also requires `producer_instance_id`. XT16 and D435 use
boot ID plus PID plus process start ticks when the local `/proc` identity is
available; TI/NX uses the transport session identity supplied by its bridge
manager.

Default stale budgets are:

```text
XT16 geometry: 1000 ms
D435 depth:    1000 ms
D435 YOLO:     3000 ms
TI/NX summary: 3000 ms
```

XT16 effective age includes the producer's reported sensor latency. Device
clock timestamps such as RealSense sensor time are diagnostic fields and are
not compared directly with host epoch time.

## PerceptionContext v1

The context contains bounded semantic sections:

```text
robot_motion
local_geometry
visual_objects
radar_tracks
risk_events
sources
degraded_capabilities
policy
```

`local_geometry.primary` can only come from fresh, verified XT16 geometry.
Fresh D435 depth may appear only in `local_geometry.forward_supplements`; it
cannot replace the four-direction primary source.

Every derived object or event carries `source_id` and `timestamp_ms`.
Only `fresh` sources contribute to derived semantic sections. All sources,
including offline reservations, remain visible in `sources` for diagnostics.
The builder recomputes effective age at `generated_at_ms`, so an envelope
cannot stay fresh merely because it was fresh when first loaded.
Consumers also reject the entire context when
`current_time_ms - generated_at_ms > stale_ms`; an old context artifact cannot
keep previously fresh sources alive.

The policy is fixed:

```text
motion_authority = slam_gateway
llm_direct_motion = false
raw_sensor_streams_allowed = false
execution_chain =
  task_queue
  -> mission_decision_engine
  -> slam_gateway
  -> unitree_sdk
```

The context is an input contract, not a second execution path. Gateway remains
the final motion authority.

## WorldState Integration

Python and C++ WorldState reducers now accept only one
`perception_context`. They reject stale, malformed, or policy-inconsistent
contexts. The old free-form `perception_summaries` input and its missing
timestamp fallback have been removed.

`WorldState.perception_summaries` remains temporarily as a read-only
compatibility projection of `PerceptionContext.sources`; it is never assembled
from separate artifacts. `detected_objects` is projected from
`visual_objects`.

## Next Integration Boundary

The next commit must make Python Planner, C++ LLM, UI, and runtime logs receive
the same current context instance. They must not reopen XT16, D435, or TI/NX
artifacts independently.
