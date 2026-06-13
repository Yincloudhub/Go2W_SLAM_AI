# MissionDecision v1 与唯一执行链

更新：2026-06-13

## 目标

比赛真实执行链固定为：

```text
TaskQueue IR
  -> MissionDecisionEngine
  -> Python persistent supervised executor
  -> SLAM Gateway
  -> Unitree SDK
```

边界：

- Planner、LLM、C++ OperatorPanel 都不能直接向底盘下发运动。
- 所有 Planner 结果必须先归一化为合法 `TaskQueue`。
- `MissionDecisionEngine` 只做确定性裁决，不覆盖 Gateway 的最终运动权威。
- C++ 语义路由和 `QueueExecutor` 是 dry-run/诊断兼容入口；比赛真实导航转交
  Python supervisor。
- `llm_direct_motion=false`，Gateway 拒绝结果只能被解释，不能被 LLM 改写。

## 契约

JSON Schema：

```text
schemas/mission_decision_v1.schema.json
```

实现：

```text
src/edge_autonomy/mission_decision.py
```

决定值：

| decision | 含义 |
|---|---|
| `execute_queue` | 队列、地图、拓扑和 Gateway 预检均通过，可进入 Python supervisor |
| `dry_run_queue` | 仅预演队列，不允许运动 |
| `hold` | Gateway 或计划要求保持当前位置 |
| `await_confirmation` | 队列要求人工确认 |
| `reject` | 队列、地图、拓扑或执行前提无效 |

关键字段：

```text
queue_id / queue_valid / decision / reason_code / reason
execute_requested / motion_allowed
execution_owner=python_persistent_supervisor
gateway_final_authority=true
llm_direct_motion=false
preflight.registry / preflight.topology / preflight.gateway
```

Gateway `DecisionRecord` 保留：

```text
authority / accepted / decision / reason / gateway_reason
policy_version / recommended_mode / motion_direction
sensor_age_ms / timestamp_ms / safety
```

## 故障行为

- 断定位：Gateway 原因进入 `DecisionRecord`，决定为 `hold`。
- 地图错配：决定为 `reject`，不进入执行器。
- 传感器过期：Gateway 原因完整保留，决定为 `hold`。
- Gateway/网络断开：真实执行 fail closed；dry-run 仍可预览队列。
- 最终计划被策略覆盖为 `human_confirm/safe_hold` 时，不得复用覆盖前的旧导航队列。

## 操作

无运动 dry-run：

```bash
cd /home/unitree/Go2W_SLAM_AI
python3 scripts/go2w_agent_entry.py \
  --go-b64 "<UTF-8 base64 command>" \
  --dry-run \
  --skip-gateway-check \
  --full-output
```

检查输出：

```text
planner.task_queue
mission_decision.decision
mission_decision.reason
mission_decision.execution_owner
mission_decision.preflight.gateway
world_state_v1.motion_allowed
operator_display.screen.mission_decision
```

真实执行必须由操作者明确增加 `--execute`，且不得使用
`--skip-gateway-check`。执行前必须确认机器人已站立、现场清空、地图和定位正确、
XT16 主几何源可信，并准备遥控器急停。

## 当前验收状态

本地实现验证：

```text
python_targeted: 76/76 passed
python_full_unittest: 298/298 passed
python_compile: passed
git_diff_check: passed
motion_commands_sent: false
```

本地环境没有可用 CMake 命令；C++ build/CTest 在机器人 fast-forward 后执行。
在机器人无运动验收、三端 commit 一致和服务残留检查完成前，P0-3 不打完成标记。
