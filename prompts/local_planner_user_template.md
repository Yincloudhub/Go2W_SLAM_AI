# Local Planner User Prompt Template

Fill this template at runtime, then send it to the local Qwen planner with `schemas/local_llm_plan.schema.json`.

```text
User command:
{{ user_command }}

Current task:
{{ task_context_json }}

World state summary:
{{ world_state_json }}

Semantic topology subgraph:
{{ semantic_topology_json }}

Registered tools:
{{ registered_tools_json }}

Safety limits:
{{ safety_limits_json }}

Output one JSON object matching local_llm_plan.schema.json.
Choose conservative behavior when information is missing or risk is high.
```

Minimal example:

```text
User command:
Inspect lab_door. If a person is present, wait. If the network is weak, continue locally and send only semantic updates.

Current task:
{"task_id":"inspect-lab-door-001","phase":"planning"}

World state summary:
{"robot":{"pose":{"x":0.4,"y":1.2,"yaw":0.0},"localized":true,"battery_percent":78},"slam_status":"healthy","link_quality":{"bandwidth_kbps":90,"latency_ms":850,"packet_loss_ratio":0.2},"risk_events":[{"event_type":"human_near_target","severity":"high","node_id":"lab_door","distance_m":0.6}]}

Semantic topology subgraph:
{"current_node":"start_area","target_node":"lab_door","candidate_path":["start_area","corridor_a","lab_door"],"safe_observation_node":"corridor_a","nodes":[{"node_id":"corridor_a","pose":{"x":1.5,"y":0.3,"yaw":0.0}},{"node_id":"lab_door","pose":{"x":3.2,"y":-1.4,"yaw":1.57},"semantic_state":"occupied"}]}

Registered tools:
["set_communication_policy","create_navigation_subgoal","wait_until","capture_keyframe","start_mapless_scout","request_human_confirm","hold_position"]

Safety limits:
{"max_linear_speed_mps":0.4,"max_angular_speed_rps":0.4,"min_person_distance_m":1.5,"max_mapless_distance_m":30}

Output one JSON object matching local_llm_plan.schema.json.
Choose conservative behavior when information is missing or risk is high.
```

