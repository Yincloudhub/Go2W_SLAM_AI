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
- `cpp/include/go2w/llm_http_client.hpp`
- `cpp/src/llm_http_client.cpp`
- `cpp/src/llm_http_client_smoke_test.cpp`

The C++ operator panel now formats status through:

```text
gateway world_state
  -> buildWorldStateV1()
  -> buildOperatorDisplayState()
  -> formatOperatorDisplayLine()
```

The Python `world_state_v1.py`, `operator_display.py`, and `runtime_log.py`
remain as prototype parity and offline test tools.

The operator panel can now call an optional OpenAI-compatible local HTTP LLM
service directly from C++. The LLM output is constrained to registered
`node_id` targets:

```json
{"reply":"short operator reply","targets":["registered_node_id"],"capture_keyframe":false}
```

C++ then routes those `node_id` values through `SemanticRouter`, `TaskQueue`,
`SafetyGate`, and `QueueExecutor`. The model is not allowed to output raw
coordinates, speeds, Unitree API ids, or direct motion commands.

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

The C++ operator panel now owns the first set of operator workflow commands:

```text
/mapping start confirm
/mapping end confirm [map_path]
/topology preview NAME
/topology add NAME confirm
/rviz2 start confirm
```

The important boundary is deliberate: UI startup checks SLAM/LiDAR/gateway
health, while mapping, topology writes, and RViz2 launch remain explicit
operator actions. `topology preview` is read-only; `topology add` goes through
the gateway action `add_current_pose_waypoint`, which requires operator ack and
fresh localization before writing the live pose. The freshness rule is based on
pose age rather than UI refresh rate: `localized/degraded/tracking` and
`pose_age_ms<=2000`.

## Robot-side non-motion verification

2026-05-27 verified on `unitree@192.168.123.18` without sending motion commands:

```text
cd /home/unitree/go2w_slam_agent
bash -n scripts/run_go2w_operator_ui.sh
cd cpp
cmake -S . -B build
cmake --build build -j2
./build/go2w_world_state_v1_smoke_test
./build/go2w_llm_http_client_smoke_test
```

Result:

- `go2w_plan_executor_core` built.
- `go2w_operator_panel` built.
- `go2w_plan_executor_dry_run` built.
- `go2w_world_state_v1_smoke_test` built and passed.
- `go2w_llm_http_client_smoke_test` built and passed.
- No navigation, relocation, mapping, stop-SLAM, or chassis motion command was
  sent.

2026-05-29 verified the workflow-command extension on `unitree@192.168.123.18`
without sending motion or state-changing SLAM commands:

```text
cd /home/unitree/go2w_slam_agent
bash -n scripts/run_go2w_operator_ui.sh
bash -n scripts/start_go2w_rviz2.sh

cd /home/unitree/slam_gateway_refactor
bash scripts/build_on_go2.sh

cd /home/unitree/go2w_slam_agent/cpp
cmake -S . -B build
cmake --build build -j2
./build/go2w_world_state_v1_smoke_test
./build/go2w_llm_http_client_smoke_test
```

Result:

- `slam_gateway_core`, `slam_keyboard_client`, and `slam_llm_command_client`
  built.
- `go2w_plan_executor_core`, `go2w_operator_panel`,
  `go2w_plan_executor_dry_run`, and both smoke tests built.
- Both smoke tests passed.
- `/mapping`, `/topology add`, `/rviz2 start`, navigation, relocation,
  stop-SLAM, and chassis motion commands were not executed.

## Next consolidation targets

1. Move startup supervision into a C++ or systemd-managed launcher on the robot,
   keeping the existing shell wrapper only as a compatibility entry.
2. Add a thin browser/Qt operator surface over the C++ core: task queue,
   world-state screen, LLM/operator feedback display, confirmation dialogs,
   and optional RViz2 launch status.
3. Move runtime log writing from Python into `QueueExecutor`.
4. Add C++ sensor-summary adapters for stereo depth and TI radar/NX, with stale
   data ignored rather than blocking the robot loop.
5. Keep Python-only files under an explicit `offline/prototype` boundary after
   the C++ path is stable.
