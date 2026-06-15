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

## Optional direct keyboard diagnostic client

The keyboard client bypasses registry authorization and is therefore excluded
from the default build and install. Build it only for isolated diagnostics:

```bash
cmake -S . -B build -DBUILD_UNSAFE_KEYBOARD_CLIENT=ON
cmake --build build --target slam_keyboard_client
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

For repeated diagnostics while localization is missing or transitioning, use
the persistent read-only session:

```bash
./build/slam_llm_command_client eth0 --persistent-world-state-session
```

It accepts only `get_world_state`, requires a unique `request_id` for every
query, creates no navigation lease, and rejects all motion or SLAM mutation
actions with `world_state_session_is_read_only`.

Allowed high-level actions:

- `get_world_state`
- `start_mapping`
- `end_mapping`
- `add_current_pose_waypoint`
- `relocate`
- `navigate_to_pose`
- `supervised_reposition`
- `pause_navigation`
- `resume_navigation`
- `stop_slam`

Raw API IDs from LLM are rejected. The LLM should output task-level commands, not Unitree API IDs.

The machine-readable path is intentionally stricter than the keyboard path:

- `navigate_to_pose` requires a persistent supervised navigation session and a registry-authorized topology target.
- `relocate` requires operator confirmation plus an active, verified registry anchor whose map identity and pose exactly match the startup registry snapshot.
- `speed` must be in the safe range `(0, 0.8]` for navigation.
- `mode` must be `0` or `1`.
- `start_mapping`, `end_mapping`, `relocate`, and `stop_slam` require `operator_ack=true` or `confirm=true`.
- `add_current_pose_waypoint` requires `operator_ack=true` or `confirm=true`, and it is rejected unless localization is fresh.

## Example LLM command

```json
{
  "action": "navigate_to_pose",
  "map_id": "go2w_real_site",
  "map_path": "/home/unitree/test.pcd",
  "runtime_watchdog": true,
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
2. `LidarGeometryPerception` treats XT16 as the primary safety source. A present but stale or uncalibrated XT16 summary remains fail-closed and cannot be hidden by D435. A D435 front reading below the hard-stop threshold must be confirmed by two distinct fresh frames before it overrides a clear XT16 front reading. The world state exposes both source readings, confirmation count, and the selected source.
3. The keyboard path and LLM path are intentionally separated:
   - `slam_keyboard_client`: original manual operation.
   - `slam_llm_command_client`: structured command input for LLM/task executor.
4. LLM commands are validated and routed through `SafetySupervisor` before navigation.
5. Navigation obstacle mode follows Unitree SLAM API semantics: `mode=0` means obstacle avoidance, `mode=1` means stop for obstacle. LLM navigation and manually recorded waypoints default to `mode=0`.
6. Short-lived command clients must not stop the SLAM backend when they exit. Use the explicit `stop_slam` action or the keyboard stop path when the backend should really stop.
7. The runtime safety policy is named `safe_guard`. Registered targets pass directly to Unitree `mode=0` navigation and its native obstacle avoidance. GO2W does not approximate the robot's rotational swept footprint from a fixed left/right clearance threshold.
8. `supervised_reposition` is the only internal post-failure recovery action. It is not a general relative-motion interface or a navigation preflight gate. After sustained native-navigation no-progress, the local LLM may select one forward/backward/left/right straight-translation candidate. `MissionDecisionEngine` verifies the choice, and the Gateway uses `SportClient::Move` with zero yaw rate, a `0.50 m` distance cap, a `0.10 m/s` speed cap, and direction-specific XT16 reserves. Recovery is attempted once before the original Unitree target is retried.
9. `navigate_to_pose` requires `map_path`, and it must match the map path reported by the active SLAM localization stream.
10. Stereo depth is forward-facing. Its image-left/image-right sectors may tighten the front view for diagnostics, but they never replace XT16 lateral or rear clearance. Stereo-only data cannot authorize navigation.
11. The default `1 s` obstacle freshness rule remains strict outside supervised release. Unitree GO2 mode-0 pose navigation uses its documented minimum `0.20 m/s`; the separate bounded recovery controller remains capped at `0.10 m/s`. During supervised native navigation, a previously valid XT16 summary may remain usable for at most `1.5 s`; this absorbs a missed 5 Hz update while limiting extra travel during the grace interval. Longer delay still fails closed.
