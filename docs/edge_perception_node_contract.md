# GO2W External Edge Perception Node Contract

## Purpose

An external NX can host TI radar processing without becoming a hard dependency
of the robot runtime. The NX owns raw radar ADC, point clouds, tracking, and
vendor SDK details. The robot receives only one bounded latest-value summary.

```text
TI radar -> external NX -> compact summary -> GO2W C++ operator core
```

The initial integration is `semantic_only`: the summary may enter the UI and
LLM context, but it cannot authorize motion or relax an existing safety block.

## Local artifact

The first transport boundary is:

```text
artifacts/edge_perception_summary.json
```

The NX bridge should atomically replace this file. A later HTTP, MQTT, DDS, or
ROS2 transport may update the same local artifact without changing C++ policy
code.

Use this example:

```text
configs/perception/edge_perception_summary.example.json
```

Required fields:

```text
schema_version = 1
node_id
sensor_type
source
timestamp_ms
sequence
clock_domain = unix_epoch_ms
health.status
policy.mode
```

Recommended TI radar observation fields:

```text
track_id
type
range_m
azimuth_deg
radial_velocity_mps
confidence
```

## Safety boundary

The C++ loader exposes:

```text
available
fresh
eligible_for_llm
safety_candidate
safety_wired
```

`safety_wired` remains `false` in the generic loader. Promoting TI radar to a
SafetyGate source requires an explicit adapter, calibration, static obstacle
tests, timestamp validation, and failure-mode tests. New sensor data may
increase caution; it must never silently relax XT16 or D435 blocking decisions.

## Resource boundary

- Keep raw ADC, dense radar point clouds, and long histories on the NX.
- Publish a bounded latest-value summary at 1-5 Hz.
- Send event summaries immediately when a tracked object or risk state changes.
- Keep each summary below 256 KiB.
- Keep `observations` and `events` bounded; the C++ loader truncates each to 32.
- Mark stale or unhealthy data explicitly instead of replaying old detections.

## PerceptionContext v1 Adapter

P0-2 normalizes this artifact as one `SensorEnvelope v1` source with:

```text
source_id = ti_nx:<node_id>
source_kind = radar_semantics
producer = nx_edge_bridge
default stale budget = 3000 ms
```

The current repository does not contain a TI/NX bridge supervisor or PID
contract. The adapter therefore defaults to `offline` unless the transport
manager supplies explicit bridge-online evidence. Artifact existence, a stored
`health.status=ok`, or a recent timestamp alone is insufficient.

The v1 bridge must publish a native monotonically increasing sequence and
declare `clock_domain=unix_epoch_ms`. The local transport manager must also
supply a stable producer session identity. Missing sequence, unknown clock
domain, or missing session evidence prevents a fresh envelope; the adapter
must not invent any of them.

TI/NX remains `semantic_only`. Tracks and events may enter the bounded
PerceptionContext and LLM input when fresh, but `safety_wired` remains false
and the source cannot authorize motion or relax XT16/D435 caution.
