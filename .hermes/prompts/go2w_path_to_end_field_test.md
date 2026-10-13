# GO2W path-to-end field test guide

Role split:
- Hermes: read each step aloud, wait for the safety officer to confirm before continuing, and stop the test on any stop condition.
- Safety officer: stay beside the robot, keep emergency stop reachable, verify the space is clear, and say STOP immediately if anything looks wrong.
- Codex/remote operator: only run the command for the current approved step.

Current prepared state:
- Robot repo: `/home/unitree/Go2W_SLAM_AI`.
- Active registry path on robot: `configs/maps/go2w_real_site_map_registry.json`, symlinked to v2.
- `go2w_real_site` and `map_701` both expose 8 topology nodes and 7 registry edges.
- Route dry-run is verified: `wp_e643bc -> wp_60497f`, 5 hops, 17.35 m, no Gateway/sport_bridge started.
- SLAM is localized on `map_701`, PTP/XT16/unitree_slam are running.
- Gateway safety is currently false because XT16 supervised release is not active yet.
- `nav_core.py` now blocks motion unless Gateway `safety.allow_navigation` is fresh and true.

Hard stop conditions:
- Safety officer says stop.
- Robot is not standing stably or a person enters the motion area.
- `localization.status` is not `localized`.
- `slam_health.status` is not `ok`.
- `safety.allow_navigation` is not `true`.
- Any command returns `LOST`, `SENSOR_STALE`, `SAFETY_BLOCKED`, `BLOCKED`, `STUCK`, `TIMEOUT`, or `BRIDGE_ERROR`.
- Robot moves in an unexpected direction, does not stop promptly, or pose jumps.

Step 0 - no-motion readiness

Run:
```bash
cd /home/unitree/Go2W_SLAM_AI
bash scripts/go2w_xt16_ptp.sh check
python3 scripts/check_go2w_runtime_layout.py
python3 scripts/waypoint_nav.py wp_60497f --dry-run --from-node wp_e643bc --map-id go2w_real_site --registry configs/maps/go2w_real_site_map_registry.json --graph artifacts/waypoint_graph.json --rebuild-graph
```

Continue only if all pass.

Step 1 - confirm localization

If localization is not clearly `localized / map_701`, run the no-motion relocation:
```bash
cd /home/unitree/Go2W_SLAM_AI
PYTHONIOENCODING=utf-8 python3 scripts/nav_relocate.py map_701 mapping_origin_701
```

Continue only if it reports localized.

Step 2 - enable XT16 supervised release

Only do this after the safety officer confirms operator presence and emergency stop availability.

Run:
```bash
cd /home/unitree/Go2W_SLAM_AI
GO2W_XT16_GEOMETRY_REAR_SELF_MASK_M=0.35 \
GO2W_XT16_GEOMETRY_REAR_SELF_MASK_HALF_WIDTH_M=0.45 \
GO2W_XT16_SUPERVISED_RELEASE=1 \
bash scripts/go2w_xt16_geometry_sidecar.sh restart
```

Then confirm Gateway safety is true before any motion. If safety remains false, stop and do not run motion.

Step 3 - state-only NavigationSession check

This starts Gateway/sport_bridge readers and sends zero velocity on close. It should not move the robot.

Run:
```bash
cd /home/unitree/Go2W_SLAM_AI
PYTHONIOENCODING=utf-8 python3 scripts/nav_core.py
```

Continue only if it repeatedly shows fresh pose, fresh clearance, localized state, and `safety=True`.

Step 4 - posture

Prefer the Unitree controller/app to put the robot into a stable standing or balance stand state. Do not run waypoint motion while the robot is lying down.

Continue only when the safety officer confirms:
- robot is standing stably;
- the path from the current position toward the next waypoint is clear;
- emergency stop is reachable.

Step 5 - first single-hop motion

If current pose is near `wp_60497f`, run the first short hop away from the start:
```bash
cd /home/unitree/Go2W_SLAM_AI
PYTHONIOENCODING=utf-8 python3 scripts/waypoint_nav.py wp_b738c7 --from-node wp_60497f --speed 0.2 --arrival-radius 0.35 --segment-timeout-min 20
```

Stop after the command returns. Do not chain another command automatically.

Step 6 - return hop

Only after Step 5 succeeds and the safety officer confirms the return path is clear:
```bash
cd /home/unitree/Go2W_SLAM_AI
PYTHONIOENCODING=utf-8 python3 scripts/waypoint_nav.py wp_60497f --from-node wp_b738c7 --speed 0.2 --arrival-radius 0.35 --segment-timeout-min 20
```

Step 7 - path-to-end test

Only after two successful single-hop tests and only if the robot is physically near the declared start node:
```bash
cd /home/unitree/Go2W_SLAM_AI
PYTHONIOENCODING=utf-8 python3 scripts/waypoint_nav.py wp_60497f --from-node wp_e643bc --speed 0.2 --arrival-radius 0.35 --segment-timeout-min 20
```

This is the full `wp_e643bc -> wp_60497f` path-to-end route. Do not run it unless the live pose is within the script start tolerance for `wp_e643bc`.

After-test cleanup:
```bash
cd /home/unitree/Go2W_SLAM_AI
printf '{"stop":true}\n' | robot/slam_gateway_refactor/build/sport_bridge eth0
```

