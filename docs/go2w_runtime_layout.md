# GO2W runtime layout

## Source of truth

The robot-side source of truth is:

```text
/home/unitree/Go2W_SLAM_AI
```

Run operator UI, web UI, Python checks, and C++ builds from this repo.

`/home/unitree/go2w_slam_agent` is a legacy compatibility copy. It is not a
separate functional module and should not own map data. If it remains on the
robot, its real-site registry should be a symlink to the active repo registry:

```text
/home/unitree/go2w_slam_agent/configs/maps/go2w_real_site_map_registry.json
  -> /home/unitree/Go2W_SLAM_AI/configs/maps/go2w_real_site_map_registry.json
```

## Map ownership

The single real-site semantic registry is:

```text
/home/unitree/Go2W_SLAM_AI/configs/maps/go2w_real_site_map_registry.json
```

That registry owns semantic topology, aliases, relocalization anchors, and the
binding to the live Unitree map files:

```text
pcd_path: /home/unitree/test.pcd
topology_path: /home/unitree/topology_points.json
```

The standalone gateway does not own this semantic registry. It receives concrete
commands with `map_path`, pose, and target values, then calls Unitree SLAM.

## Runtime modules

```text
scripts/run_go2w_operator_ui.sh or scripts/run_go2w_operator_web.sh
  -> cpp/build/go2w_operator_panel
      -> cpp GatewayClient
          -> /home/unitree/slam_gateway_refactor/build/slam_llm_command_client
              -> Unitree SLAM / SDK services
```

Python remains in the repo for map tooling, offline checks, local LLM fallback,
and test parity. New live robot execution policy should be implemented in C++
first, then mirrored in Python only when useful for tests or diagnostics.

## Verification

Run this after robot sync or map changes:

```bash
cd /home/unitree/Go2W_SLAM_AI
python3 scripts/check_go2w_runtime_layout.py
PYTHONPATH=src python3 -m unittest discover -s tests
cmake -S cpp -B cpp/build
cmake --build cpp/build -j$(nproc)
```
