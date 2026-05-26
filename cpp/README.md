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

Run on the robot/NX:

```bash
cd ~/go2w_slam_agent/cpp
cmake -S . -B build
cmake --build build -j

./build/go2w_operator_panel --repo-root ~/go2w_slam_agent --current-node yin_siyuan_station
```

Watch-only mode:

```bash
./build/go2w_operator_panel --repo-root ~/go2w_slam_agent --watch 0
```

The Qt/RViz2 UI should reuse this command/state boundary instead of directly
embedding model inference inside the visualization layer.

Encoding note: do not pipe Chinese text from Windows PowerShell into this C++
binary. Use MobaXterm/Linux terminal input, ASCII node IDs, or the existing
base64 Python entrypoint when crossing unstable Windows shell boundaries.
