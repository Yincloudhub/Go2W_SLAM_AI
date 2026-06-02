# GO2W C++ plan executor

This folder contains a SDK-free C++ dry-run executor for `LocalLlmPlan`.

It is intentionally separate from `artifacts/slam_gateway_refactor_source`:

- `cpp/` can be compiled on a normal Linux/NX machine without Unitree SDK.
- `artifacts/slam_gateway_refactor_source` is the Unitree SDK-facing client.

Build on Linux/NX:

```bash
cd /path/to/GO2W_0/cpp
cmake -S . -B build
cmake --build build -j
```

Run dry-run:

```bash
./build/go2w_plan_executor_dry_run \
  --registry ../configs/maps/go2w_map_registry.example.json \
  --plan /tmp/local_llm_plan.json
```

Or pipe a plan through stdin:

```bash
cat /tmp/local_llm_plan.json | ./build/go2w_plan_executor_dry_run \
  --registry ../configs/maps/go2w_map_registry.example.json
```

This executable does not move the robot. It validates the plan and prints the
`navigate_to_pose` command plus the planned `resume -> navigate -> wait -> pause`
sequence. The same validation logic can later be moved into the Unitree SDK
client before `slam_llm_command_client` sends real commands.

## Operator panel prototype

Build also creates `go2w_operator_panel`, a C++ operator-facing terminal panel.
It is the first step toward a Qt/RViz2-style UI:

- reads `world_state` through `slam_llm_command_client`;
- uses a C++ `GatewayClient` with fork/pipe instead of shell temp files for
  gateway reads, navigation commands, and pause commands;
- prints Chinese semantic status instead of raw JSON;
- supports weak-link compact status with `/weak on`;
- accepts Chinese LLM commands and forwards them through the UTF-8 base64
  `go2w_agent_entry.py --go-b64 ... --human` path;
- defaults to dry-run and only executes movement after `/execute on`.
- routes known topology commands in C++ first; Python/LLM is now only the
  fallback when no registered semantic point matches.
- builds `WorldState v1`, `OperatorDisplayState`, and bounded runtime-log records
  in C++ so the hot operator path no longer depends on the Python prototype data
  model.
- can call an optional local OpenAI-compatible HTTP LLM service from C++. The
  model is only allowed to select registered `node_id` values; C++ still builds
  the task queue and runs SafetyGate/QueueExecutor.

Run on the robot/NX:

```bash
cd ~/go2w_slam_agent/cpp
cmake -S . -B build
cmake --build build -j

./build/go2w_operator_panel --repo-root ~/go2w_slam_agent --current-node yin_siyuan_station
```

One-command robot-side launch, including LiDAR driver and SLAM startup/check:

```bash
cd ~/go2w_slam_agent
./scripts/run_go2w_operator_ui.sh
```

Browser UI launch:

```bash
cd ~/go2w_slam_agent
./scripts/run_go2w_operator_web.sh
```

The web UI defaults to `127.0.0.1:8765` for safer SSH/MobaXterm tunneling. It
does not own robot control logic; every status refresh and button action is
delegated back to short-lived `go2w_operator_panel` sessions. Set
`GO2W_WEB_HOST=0.0.0.0` only when direct LAN access is needed.

The current UI is a robot-side terminal panel. Open it from MobaXterm by SSHing
to the robot/NX and running the launcher above; X11 forwarding is not required.
A later Qt/RViz2 panel can reuse the same operator-core boundary, but the safer
field path today is to keep gateway, LiDAR, SLAM, and the terminal UI on the
robot/NX.

The launcher defaults to `scripts/start_go2w_slam_stack.sh` and lets the C++
operator panel perform the gateway world-state check immediately after startup.
This keeps the live path as `shell launcher -> C++ panel -> C++ gateway client`;
the Python startup supervisor remains available only as a diagnostic helper.

Optional C++ LLM HTTP fallback:

```bash
GO2W_LLM_HTTP_URL=http://127.0.0.1:8080/v1/chat/completions \
GO2W_LLM_HTTP_MODEL=local \
./scripts/run_go2w_operator_ui.sh
```

When enabled, unmatched natural-language commands go to the local HTTP model.
The expected model output is only:

```json
{"reply":"short operator reply","targets":["registered_node_id"],"capture_keyframe":false}
```

The C++ panel rejects empty or unregistered targets and does not accept raw
coordinates or Unitree API ids from the model.

## Optional external edge perception node

An external NX can run vendor-specific TI radar drivers and tracking code, then
publish a bounded latest-value summary to
`artifacts/edge_perception_summary.json`. The C++ operator core and browser UI
load fresh summaries without taking ownership of raw radar streams.

This interface defaults to `semantic_only`: fresh summaries can enrich LLM and
operator context, but they do not authorize movement and cannot relax XT16,
D435, or SLAM safety blocks. Any later SafetyGate integration requires a
separate calibrated adapter and static validation. See
`docs/edge_perception_node_contract.md`.

Watch-only mode:

```bash
./build/go2w_operator_panel --repo-root ~/go2w_slam_agent --watch 0
```

The Qt/RViz2 UI should reuse this command/state boundary instead of directly
embedding model inference inside the visualization layer.

## Runtime ownership rule

New real-time behavior should enter `cpp/` first:

- `world_state_v1.cpp`: low-rate state contract for UI, LLM, and policy code;
- `operator_panel.cpp`: operator input and display shell;
- `queue_executor.cpp`: task queue execution, arrival feedback, and bounded
  event buffers;
- `safety_gate.cpp`: deterministic safety policy;
- `semantic_router.cpp`: topology matching and deterministic task queues.

Python remains useful for offline data generation, map tooling, evaluation,
and LLM fallback, but it should not become the owner of a new hot-loop runtime
feature.

Encoding note: do not pipe Chinese text from Windows PowerShell into this C++
binary. Use MobaXterm/Linux terminal input, ASCII node IDs, or the existing
base64 Python entrypoint when crossing unstable Windows shell boundaries.
