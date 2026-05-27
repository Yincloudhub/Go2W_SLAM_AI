# GO2W C++ runtime consolidation

## Goal

The runtime path should converge on one C++ operator core. Python remains in the
repo, but its role should be explicit: offline data, map tooling, evaluation,
dataset generation, and temporary LLM fallback.

## Runtime ownership

```text
robot/slam_gateway_refactor
  -> Unitree SDK / SLAM gateway adapter

cpp/
  -> operator panel
  -> WorldState v1
  -> OperatorDisplayState
  -> SafetyGate
  -> SemanticRouter
  -> TaskQueue validator
  -> QueueExecutor

scripts/
  -> one-command launch wrappers
  -> robot sync/build helpers
  -> offline inspection tools

src/edge_autonomy/
  -> Python prototype/reference implementation
  -> offline eval and data generation support
  -> LLM fallback until C++ LLM HTTP client is ready
```

## Rules for new features

- If it affects live robot execution, status display, SafetyGate, task queues,
  or arrival feedback, implement the runtime owner in `cpp/` first.
- If it is a sensor plugin, expose only a bounded semantic summary to C++:
  `source`, `confidence`, `stale`, `latency_ms`, `timestamp_ms`, and `summary`.
- Do not put raw video, dense point clouds, raw radar ADC, or long logs on the
  LLM/UI boundary.
- Do not add another live process unless it has a health check, stale-data
  policy, and a non-blocking failure mode.
- Python can mirror the C++ contract for tests and offline work, but it should
  not become the primary live state owner again.

## Current consolidation step

This step adds C++ equivalents for the previous Python state contract:

- `cpp/include/go2w/world_state_v1.hpp`
- `cpp/src/world_state_v1.cpp`
- `cpp/src/world_state_v1_smoke_test.cpp`

The C++ operator panel now formats status through:

```text
gateway world_state
  -> buildWorldStateV1()
  -> buildOperatorDisplayState()
  -> formatOperatorDisplayLine()
```

The Python `world_state_v1.py`, `operator_display.py`, and `runtime_log.py`
remain as prototype parity and offline test tools.

The robot-side UI launcher now defaults to `scripts/start_go2w_slam_stack.sh`.
Startup therefore runs as:

```text
run_go2w_operator_ui.sh
  -> go2w_operator_panel --ensure-slam-on-start
      -> start_go2w_slam_stack.sh
      -> C++ GatewayClient get_world_state
```

The older Python startup supervisor is kept for diagnostics, but it is no
longer on the default live UI path.

## Robot-side non-motion verification

2026-05-27 verified on `unitree@192.168.123.18` without sending motion commands:

```text
cd /home/unitree/go2w_slam_agent
bash -n scripts/run_go2w_operator_ui.sh
cd cpp
cmake -S . -B build
cmake --build build -j2
./build/go2w_world_state_v1_smoke_test
```

Result:

- `go2w_plan_executor_core` built.
- `go2w_operator_panel` built.
- `go2w_plan_executor_dry_run` built.
- `go2w_world_state_v1_smoke_test` built and passed.
- No navigation, relocation, mapping, stop-SLAM, or chassis motion command was
  sent.

## Next consolidation targets

1. Move startup supervision into a C++ or systemd-managed launcher on the robot,
   keeping the existing shell wrapper only as a compatibility entry.
2. Replace Python LLM fallback with a C++ HTTP client to a local llama.cpp or
   OpenAI-compatible service.
3. Move runtime log writing from Python into `QueueExecutor`.
4. Add C++ sensor-summary adapters for stereo depth and TI radar/NX, with stale
   data ignored rather than blocking the robot loop.
5. Keep Python-only files under an explicit `offline/prototype` boundary after
   the C++ path is stable.
