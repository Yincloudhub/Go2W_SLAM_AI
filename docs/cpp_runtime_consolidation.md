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

The browser UI is now a thin shell around that C++ panel:

```text
run_go2w_operator_web.sh
  -> go2w_operator_web.py
      -> short-lived go2w_operator_panel stdin sessions
          -> C++ GatewayClient / SemanticRouter / SafetyGate / QueueExecutor
```

The web layer owns only browser/session state: dry-run vs execute toggle,
weak-link display mode, current-node hint, and a short command history. It does
not subscribe to raw ROS2 topics, run dense perception, or duplicate the
robot-facing control policy.

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

2026-05-29 verified the thin browser UI on `unitree@192.168.123.18` without
sending motion or state-changing SLAM commands:

```text
cd /home/unitree/go2w_slam_agent
bash -n scripts/run_go2w_operator_web.sh
python3 -m py_compile scripts/go2w_operator_web.py
python3 scripts/go2w_operator_web.py --self-test
curl http://127.0.0.1:8765/api/state
curl http://127.0.0.1:8765/api/status
```

Result:

- Web UI process started on `127.0.0.1:8765` with PID recorded in
  `artifacts/operator_web/operator_web.pid`.
- `/api/state` returned dry-run session state.
- `/api/status` returned a C++ panel summary with `phase=idle`,
  `motion=false`, and current safety status `slam_health_failed`.
- No `/start-slam`, `/mapping`, `/topology add`, `/rviz2 start`, navigation,
  relocation, stop-SLAM, or chassis motion command was executed.

2026-05-31 prone-safe stabilization notes:

- Local `127.0.0.1:8765` browser access was restored through
  `scripts/go2w_web_tunnel.py`, forwarding to the robot-side Web UI over SSH.
- `scripts/start_go2w_slam_stack.sh` now preflights the Unitree SLAM log
  directory and prints the exact `sudo mkdir/chown` repair command if the
  root-owned install path is not writable. It also waits through a short
  stability window and returns non-zero when `unitree_slam` exits early or logs
  fatal startup errors such as `lidar ysn check failed`.
- On the robot, `unitree_slam` originally exited because
  `/unitree/module/unitree_slam/bin/logs/slam_server` was missing/not writable.
  Creating the log directory and assigning it to `unitree` allowed
  `unitree_slam` to pass log initialization.
- A second blocker remains before point recording: `unitree_slam` exits with
  `mid360 lidar ysn check failed`, reporting current ysn `47PGO440010032` and
  current IP `192.168.123.20`. The config file
  `/unitree/module/unitree_slam/config/slam_interfaces_server_config/param.yaml`
  contains those values under the `Go2`/`Go2_W` sections, so the next repair
  should verify the binary's selected robot/lidar profile instead of blindly
  editing official SLAM config.
- The head-mounted LiDAR path should be treated as the XT16 path in this
  project. `xt16_driver` can connect to the LiDAR at `192.168.123.20` and load
  correction data. The official Unitree `Go2`/`Go2_W` SLAM profile was restored
  to its pre-test value after a failed A/B check, so the next fix should compare
  the known-good launch method/environment before changing vendor configs.
  `xt16_driver` and `unitree_slam` were started for diagnostics only. Mapping,
  relocation, navigation, topology writing, RViz2 launch, stop-SLAM, and chassis
  motion commands were not executed.
- Follow-up evidence after comparing the robot history: the old workspace log
  `/home/unitree/go2w_slam/go2w_edge_autonomy/logs/slam_stack/unitree_slam.log`
  recorded `xt16 lidar ysn check success!` on 2026-05-26, but
  `/unitree/module/unitree_slam/bin/unitree_slam`,
  `/unitree/module/unitree_slam/bin/xt16_driver`, and the related SLAM libs and
  configs all have 2026-05-29 20:08 mtimes. The current restored `param.yaml`
  is only known to match the 2026-05-31 14:59 pre-test backup, not necessarily
  the 2026-05-26 known-good state. Treat this as a module-version/config drift
  investigation before changing the vendor profile again.
- `scripts/start_go2w_slam_stack.sh` now prints the `unitree_slam` binary,
  `xt16_driver` binary, and `slam_interfaces_server_config/param.yaml` mtimes
  and sha256 hashes, plus the B2/B2_W/Go2/Go2_W lidar profile summary, before
  starting anything. This makes future UI one-click startup failures easier to
  separate into config drift, binary drift, or runtime health.
- `go2w_operator_panel` now propagates the last command's exit code, so the Web
  UI reports `/start-slam` as `accepted=false` when the SLAM startup script
  detects the current fatal condition.
- `scripts/go2w_operator_web.py` now bounds display refresh load with a
  short `/api/status` cache (`GO2W_STATUS_CACHE_MS`, default 1500 ms) and a
  panel subprocess mutex. Automatic browser refresh no longer spawns a new C++
  panel session every request; manual refresh can still force a fresh read with
  `/api/status?force=1`.
- RealSense D435I was detected and short depth captures were converted into
  `artifacts/stereo_depth_summary.json`. The low-rate loop mode keeps the
  camera pipeline open and atomically replaces the summary file; the latest
  prone-safe 3-sample test produced about 135-141 ms latency after warm-up and
  confidence around 0.52 in the front ROI after warm-up. The exact center pixel
  can still be invalid while prone, so center-distance UI feedback and ROI
  safety fusion should be shown as separate values instead of collapsing them
  into one "valid/invalid" score.

## Next consolidation targets

1. Move startup supervision into a C++ or systemd-managed launcher on the robot,
   keeping the existing shell wrapper only as a compatibility entry.
2. Replace the browser UI's short-lived process bridge with a persistent C++
   local IPC/HTTP adapter after the panel contract is stable.
3. Move runtime log writing from Python into `QueueExecutor`.
4. Add C++ sensor-summary adapters for stereo depth and TI radar/NX, with stale
   data ignored rather than blocking the robot loop.
5. Keep Python-only files under an explicit `offline/prototype` boundary after
   the C++ path is stable.
