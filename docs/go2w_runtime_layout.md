# GO2W runtime layout

## Source of truth

The robot-side source of truth is:

```text
/home/unitree/Go2W_SLAM_AI
```

Operator UI, web UI, Python checks, C++ builds, the Unitree gateway adapter,
and the real-site registry all live in this repository.

The following old source trees are not runtime modules:

```text
/home/unitree/go2w_slam_agent
/home/unitree/slam_gateway_refactor
```

They must be absent from the active filesystem or stored under
`/home/unitree/_archive/`. Runtime launchers must never point to them.

## Map ownership

There is one real-site semantic registry:

```text
/home/unitree/Go2W_SLAM_AI/configs/maps/go2w_real_site_map_registry.json
```

It owns topology nodes, aliases, relocalization anchors, and bindings to the
two Unitree runtime files:

```text
pcd_path: /home/unitree/test.pcd
topology_path: /home/unitree/topology_points.json
```

Python and C++ read this same registry. They do not maintain separate maps.
Demo registries remain test fixtures and are never selected by the robot
launchers.

## Runtime modules

```text
scripts/start_go2w_runtime_stack.sh
  -> XT16 driver
  -> XT16 geometry sidecar
  -> Unitree SLAM
  -> robot/slam_gateway_refactor/build/slam_llm_command_client

scripts/run_go2w_operator_ui.sh or scripts/run_go2w_operator_web.sh
  -> cpp/build/go2w_operator_panel
      -> SemanticRouter / optional LLM fallback
      -> TaskQueueValidator
      -> SafetyGate
      -> QueueExecutor runtime watchdog
      -> robot/slam_gateway_refactor/build/slam_llm_command_client
          -> SafetySupervisor
          -> Unitree SLAM / SDK services
```

The LLM resolves ambiguous language and proposes registered task-level actions.
It cannot provide raw API IDs, bypass topology validation, grant motion
permission, or override deterministic safety.

## Readiness boundary

- Service startup and gateway probing may succeed before localization.
- Relocalization requires a verified anchor and operator confirmation, but not
  all-around obstacle clearance because it does not move the chassis.
- Enabling execution requires fresh localization.
- Actual navigation additionally requires fresh calibrated XT16 perception,
  gateway safety, queue preflight, and a runtime watchdog.
- Arrival, timeout, stale state, or gateway errors request pause; queue success
  requires pause acceptance.

## Verification

```bash
cd /home/unitree/Go2W_SLAM_AI
python3 scripts/check_go2w_runtime_layout.py
python3 scripts/check_go2w_json_text.py configs/maps/go2w_real_site_map_registry.json
python3 -m pytest -q

cmake -S cpp -B cpp/build -DCMAKE_BUILD_TYPE=Release
cmake --build cpp/build -j$(nproc)
cd cpp/build && ctest --output-on-failure

cd /home/unitree/Go2W_SLAM_AI
cmake -S robot/slam_gateway_refactor -B robot/slam_gateway_refactor/build -DCMAKE_BUILD_TYPE=Release
cmake --build robot/slam_gateway_refactor/build -j$(nproc)
./robot/slam_gateway_refactor/build/safety_supervisor_smoke_test
./robot/slam_gateway_refactor/build/lidar_geometry_perception_smoke_test
```
