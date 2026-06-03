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
cd /home/unitree/Go2W_SLAM_AI
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
cd /home/unitree/Go2W_SLAM_AI
bash -n scripts/run_go2w_operator_ui.sh
bash -n scripts/start_go2w_rviz2.sh

cd /home/unitree/slam_gateway_refactor
bash scripts/build_on_go2.sh

cd /home/unitree/Go2W_SLAM_AI/cpp
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
cd /home/unitree/Go2W_SLAM_AI
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

2026-05-31 Unitree SLAM drift evidence:

- The current `/unitree` tree was overwritten by Unitree's upgrade service, not
  by this git repository. `/etc/systemd/system/unitree-upgrade.service` runs
  `/usr/bin/python3 /upgradePythonServer/server.py`; the `upgrade/run` path
  unzips an uploaded package, executes `rm -rf /unitree`, copies `unitree/` to
  `/`, then runs `/unitree/services/install.sh`.
- The uploaded package still present on the robot is
  `/upgradePythonServer/uploads/733c1dd4422147f184be6cda92db5b4f.zip`
  (`sha256=f682f8d289f5bd6ad0a744153a07d33c297d45cd1c08d3bae5b85f9fd9c4e0d5`).
  `/upgradePythonServer/recover/backup.zip` has the same hash, so "recover"
  would reapply the same current package rather than restore the 2026-05-26
  known-good SLAM version.
- The package contents match the current failing runtime exactly:
  `unitree_slam` hash
  `1d06d424b3e942931b44cab24e7542c1430038c4310debafeb2b89d72e39e88c`,
  `xt16_driver` hash
  `7d260a8311899e3d7c6606893d2995b8d424f9575020a864d8a4cdf5140afa97`, and
  `slam_interfaces_server_config/param.yaml` hash
  `727316783aa91fa14782a98767bdc31cdb6e20a1f026f8f8b44e3ed81233e0dd`.
- The preserved 2026-05-26 log proves an older runtime reached
  `xt16 lidar ysn check success!` and started `slam_operate`. The old
  `/unitree/module/graph_pid_ws`, `/unitreebk`, old `unitree_slam`, old
  `xt16_driver`, and old zip were not found in the robot filesystem, local
  `artifacts/recovered*` snapshots, `F:\browser`, or `E:\codexprofile`.
  The recovered local snapshots only include `/home/unitree/unitree/Odometer_service`.
- Next recovery path: locate the pre-2026-05-29 `/unitree` package from another
  robot, a disk image, the machine that uploaded the zip (previous auth logs
  point to `192.168.123.162` around the upgrade window), or vendor support. Do
  not rely on `unitree-upgrade` recover for this issue.

2026-05-31 relocation status after serial-number repair:

- After correcting the XT16 serial profile and rebooting the robot, the
  prone-safe `scripts/start_go2w_slam_stack.sh` path started `xt16_driver` and
  `unitree_slam` successfully. The log showed `xt16 lidar ysn check success!`
  and `server started. name:slam_operate`.
- A `relocate` command for `/home/unitree/test.pcd` at the `mapping_origin`
  anchor was accepted by `slam_operate` (`apiId:1804`) and returned
  `Successfully started relocation.` The SLAM log recorded `ICP Score:0.011119`.
- Runtime verification should read `/slam_info` and
  `/unitree/slam_relocation/odom`. The older `/uslam/localization/odom` path had
  no publisher in this run, while `/unitree/slam_relocation/odom`,
  `/unitree/slam_relocation/global_map`, `/unitree/slam_lidar/points`, and
  `/slam_info` did have publishers and produced live samples.
- The short-lived `slam_llm_command_client` needs a small DDS warm-up window
  before stdin commands. Immediate `get_world_state` can report
  `localization=not_started`, but delaying stdin by about 1.5 s yielded
  `localization=localized`, `slam_health=ok`, and a live pose. The C++ operator
  path now carries `gateway_startup_wait_s` into `GatewayClient` so UI status
  polls do not race DDS discovery.

2026-05-31 prone workstation recovery hardening:

- The C++ panel now exposes `/relocate ANCHOR_ID confirm`. This sends only a
  SLAM relocalization command built from the registry anchor pose and does not
  command chassis motion.
- The browser UI adds a thin `Relocalize` quick control that delegates to the
  same C++ panel command instead of duplicating robot-facing policy in Python.
- `QueueExecutor` now passes `gateway_startup_wait_s` through every gateway
  call, so execution-time world-state polling uses the same DDS warm-up policy
  as panel status polling.
- Targets tagged `needs_calibration` or `needs_standing_verification` are now
  visible in the planned task queue and are blocked before real motion when
  execution mode is enabled. Dry-run still prints the route and the guard so
  the operator can inspect what would have happened.
- The current `chen_jiayu_station` anchor was sampled while the robot was
  prone near the workstation. A relocation attempt from that anchor returned
  Unitree `errorCode=509` with low ICP match (`ICP Score:0.082658`), while the
  earlier `mapping_origin` relocation succeeded (`ICP Score:0.011119`). Treat
  this as a standing-verification/recovery workflow issue, not as a navigation
  target that should be executed.

2026-06-01 restart relocalization check:

- After a reboot/restart, `xt16_driver` and `unitree_slam` must be running
  before localization can succeed. If they are not running, the gateway
  relocation path returns `status_code=3104` and world state stays
  `localization=not_started`.
- Running `scripts/start_go2w_slam_stack.sh` restarted the XT16 LiDAR driver and
  Unitree SLAM without sending motion commands.
- Registry anchor `initial_point` failed startup relocalization on this run:
  Unitree returned `errorCode=509` with `ICP Score:0.0372128`.
- Direct relocalization at the map zero pose succeeded:
  `x=0,y=0,z=0,q=(0,0,0,1),yaw=0`, `ICP Score:0.0165342`, and
  `Successfully started relocation.`
- The registry now includes `mapping_origin` as the verified startup
  relocalization anchor. The browser Relocalize control defaults to
  `mapping_origin`.
- Current restart-safe sequence is:

```text
/start-slam
/relocate mapping_origin confirm
/status
```

Expected status after success: `loc=true`, `health=ok/localized`,
`safety=ok`. This is still SLAM-only; it does not imply navigation is enabled.

2026-06-01 standard workstation prone-pose refresh:

- With SLAM localized and the robot still prone at the standard workstation
  position, the operator status reported target `陈嘉瑜工位(0.16m)`,
  `motion=false`, `safety=ok`, and pose `x=-1.11, y=-0.73, yaw=1.52`.
- A gateway world-state sample then retained the precise raw prone observation:
  `x=-1.1047908068, y=-0.6664881110, z=0.0445867404,
  yaw=1.5275480013`, including its original quaternion.
- Registry node and candidate relocalization anchor `chen_jiayu_station` were
  refreshed with precise planar navigation pose and tagged
  `standard_prone_pose`. The raw prone sample is stored separately as
  `calibration_observation`. Keep `needs_standing_verification` before enabling
  real navigation to this target; the current calibration verifies
  localization and operator display while prone.

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
