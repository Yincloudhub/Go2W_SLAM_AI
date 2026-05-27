# slam_gateway_refactor_v3

This package refactors the original Unitree `keyDemo` / `MySlamService.cpp` style SLAM client into a small external adapter layer.

## Design intent

- Keep the original keyboard input set **unchanged**: `q w a s d f z x` only.
- Provide a separate machine-readable SLAM command input for LLM/task-planner output.
- Keep official `unitree_slam` as the low-level mapping/localization/navigation backend.
- Add phase-1 P0 outputs: current pose, localization state, SLAM health, navigation state, local obstacle summary and safety decision.

## Files

```text
include/slam_gateway/models.hpp
include/slam_gateway/slam_gateway.hpp
include/slam_gateway/topology_manager.hpp
include/slam_gateway/lidar_geometry_perception.hpp
include/slam_gateway/safety_supervisor.hpp
include/slam_gateway/llm_command_processor.hpp
src/slam_gateway.cpp
src/topology_manager.cpp
src/lidar_geometry_perception.cpp
src/safety_supervisor.cpp
src/llm_command_processor.cpp
src/keyboard_main.cpp
src/llm_command_main.cpp
CMakeLists.txt
config/llm_command_examples.jsonl
scripts/build_on_go2.sh
```

## Build

```bash
cd slam_gateway_refactor_v3
./scripts/build_on_go2.sh
```

If Unitree SDK2 is not in the default path:

```bash
./scripts/build_on_go2.sh -DUNITREE_SDK2_ROOT=/home/unitree/unitree_sdk2
```

If your official SDK example uses a different library name, adjust `target_link_libraries` in `CMakeLists.txt` according to the official example.

## Run keyboard client

```bash
./build/slam_keyboard_client eth0
```

The keyboard executable intentionally keeps only the original keys:

| Key | Function |
|---|---|
| `q` | start mapping |
| `w` | end mapping and save `/home/unitree/test.pcd` |
| `a` | relocate using `/home/unitree/test.pcd` |
| `s` | add current pose to waypoint list |
| `d` | execute waypoint list as patrol loop |
| `f` | clear waypoint list |
| `z` | pause navigation |
| `x` | resume navigation |
| other | stop SLAM node |

No new keyboard command is added. Waypoints are auto-saved to `/home/unitree/topology_points.json` when pressing `s`, without adding a new key.

## Run LLM command client

```bash
./build/slam_llm_command_client eth0
```

Then send one JSON command per line through stdin:

```bash
cat config/llm_command_examples.jsonl | ./build/slam_llm_command_client eth0
```

Allowed high-level actions:

- `get_world_state`
- `start_mapping`
- `end_mapping`
- `relocate`
- `navigate_to_pose`
- `pause_navigation`
- `resume_navigation`
- `stop_slam`

Raw API IDs from LLM are rejected. The LLM should output task-level commands, not Unitree API IDs.

The machine-readable path is intentionally stricter than the keyboard path:

- `navigate_to_pose` and `relocate` require finite `x` and `y` values instead of silently defaulting to the map origin.
- `speed` must be in the safe range `(0, 0.8]` for navigation.
- `mode` must be `0` or `1`.
- `start_mapping`, `end_mapping`, and `stop_slam` require `operator_ack=true` or `confirm=true`.

## Example LLM command

```json
{
  "action": "navigate_to_pose",
  "target_pose": {
    "name": "llm_goal_001",
    "x": 2.0,
    "y": 0.0,
    "z": 0.0,
    "q_x": 0.0,
    "q_y": 0.0,
    "q_z": 0.0,
    "q_w": 1.0,
    "speed": 0.5,
    "mode": 0
  }
}
```

## Important notes

1. This code does not modify official `unitree_slam` internals.
2. `LidarGeometryPerception` is a phase-1 conservative stub. Replace it with real `/utlidar/cloud` processing later.
3. The keyboard path and LLM path are intentionally separated:
   - `slam_keyboard_client`: original manual operation.
   - `slam_llm_command_client`: structured command input for LLM/task executor.
4. LLM commands are validated and routed through `SafetySupervisor` before navigation.
5. Navigation obstacle mode follows Unitree SLAM API semantics: `mode=0` means obstacle avoidance, `mode=1` means stop for obstacle. LLM navigation and manually recorded waypoints default to `mode=0`.
6. Short-lived command clients must not stop the SLAM backend when they exit. Use the explicit `stop_slam` action or the keyboard stop path when the backend should really stop.
