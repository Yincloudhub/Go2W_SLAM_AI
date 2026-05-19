# 本地 LLM 微调开发任务交接文档

本文档用于新开一个任务，专门推进本地 Qwen3-4B 策略模型的 prompt 调整、schema 约束、验证器和微调数据开发。

---

## 1. 新任务目标

目标不是更换 LLM，而是把现有 NX 上的 `Qwen3-4B-Q4_K_M.gguf` 调整成可进入机器人闭环的本地策略器。

当前模型位置：

```text
主机：ysy@192.168.33.30
模型：/home/ysy/models/Qwen3-4B-Q4_K_M.gguf
推理：/home/ysy/llama.cpp/build/bin/llama-cli
脚本：/home/ysy/ask_qwen.sh
```

已知性能：

```text
设备：Jetson Orin NX 8GB
速度：约 12 到 14 tok/s
ctx：2048 比较稳
```

第一阶段目标：

```text
不微调，先完成 prompt + json schema + validator。
```

第二阶段目标：

```text
收集失败样本，构造 300 到 800 条 SFT 数据。
```

第三阶段目标：

```text
LoRA / QLoRA 微调后量化部署，回到 NX 做离线评估。
```

---

## 2. 新任务启动 Prompt

把下面这段作为新会话/新任务的开场 prompt。

```text
你现在负责开发 GO2W 项目的本地 LLM 策略器微调与验证链路。

项目背景：
- 机器人是 Unitree Go2_W，边缘端是 Jetson Orin NX。
- 项目目标是弱带宽与定位退化场景下的四足机器人多模式语义自治。
- 运行时主闭环必须 Local-first，不能依赖云端 LLM 主规划。
- 本地 LLM 只做低频策略规划和工具选择，不做底层速度控制。
- 所有会让机器人运动的工具必须经过 SafetySupervisor。

当前 NX 摸底结果：
- 主机：ysy@192.168.33.30
- 系统：Ubuntu 22.04 / JetPack 6.1
- 设备：Jetson Orin NX 8GB
- 模型：/home/ysy/models/Qwen3-4B-Q4_K_M.gguf
- 推理：/home/ysy/llama.cpp/build/bin/llama-cli
- 脚本：/home/ysy/ask_qwen.sh
- 速度：约 12 到 14 tok/s
- llama.cpp 支持 --json-schema 和 --json-schema-file

已有本地文档：
- E:/GO2W_0/docs/local_llm_closed_loop_strategy.md
- E:/GO2W_0/docs/nx_qwen3_llm_readiness_2026-04-24.md
- E:/GO2W_0/schemas/local_llm_plan.schema.json
- E:/GO2W_0/prompts/local_planner_system_prompt.md
- E:/GO2W_0/prompts/local_planner_user_template.md

请完成以下任务：
1. 基于 schemas/local_llm_plan.schema.json 建立离线 JSON 校验器。
2. 设计 PlannerPolicyValidator，拦截格式正确但策略错误的输出。
3. 编写一个本地推理包装脚本设计，默认启用 --temp 0、--top-p 1、--reasoning off、--json-schema-file。
4. 从现有场景构造 50 到 100 条 prompt-only 评估样本。
5. 设计 SFT 数据格式，把 user_command、world_state、semantic_topology、registered_tools 映射到合法 JSON plan。
6. 评估指标包括 JSON 可解析率、工具合法率、危险场景保守率、弱网策略正确率、SLAM 退化处理正确率。
7. 不要改机器人底层控制，不要开放 cmd_vel、raw_slam_operate、shell、关闭安全层等危险工具。

优先交付：
- 可运行的离线评估脚本。
- 50 条初始评估样本。
- 策略校验规则。
- 微调数据 schema。
- 下一步 LoRA/QLoRA 训练路线。
```

---

## 3. 这次发现的关键调整点

### 3.1 必须用 llama.cpp JSON schema

`llama-cli` 支持：

```text
--json-schema SCHEMA
--json-schema-file FILE
```

因此运行时不要只靠 prompt 要求“输出 JSON”，而要使用：

```text
--json-schema-file schemas/local_llm_plan.schema.json
```

### 3.2 ask_qwen.sh 需要包装

原脚本可以保留。建议新增一个机器人策略专用包装脚本：

```text
ask_robot_planner.sh
```

默认参数：

```text
--temp 0
--top-p 1
--reasoning off
--ctx 2048
--max-tokens 256 或 384
--json-schema-file local_llm_plan.schema.json
```

### 3.3 schema 只能管格式，不能管策略正确

测试中出现过：

```text
SLAM 正常且目标在拓扑图中，但模型选择 start_mapless_scout。
```

这类问题必须靠：

```text
PlannerPolicyValidator
SafetySupervisor
微调数据
```

共同解决。

---

## 4. 推荐代码结构

后续建议在仓库里新增：

```text
src/edge_autonomy/local_llm/
  __init__.py
  planner_client.py
  plan_schema.py
  policy_validator.py
  prompt_builder.py
  tool_registry.py
  offline_eval.py

data/local_llm_eval/
  scenarios.jsonl
  expected.jsonl

data/local_llm_sft/
  train.jsonl
  val.jsonl

scripts/
  run_local_planner_eval.ps1
  export_planner_sft_dataset.py
```

说明：

- `planner_client.py`：调用 llama.cpp 或远端 NX 推理。
- `prompt_builder.py`：把 `WorldState`、拓扑图和工具列表拼成 prompt。
- `policy_validator.py`：做策略规则校验。
- `tool_registry.py`：定义允许工具和参数约束。
- `offline_eval.py`：批量跑 50 到 100 个样本。

---

## 5. 初始策略校验规则

建议第一版规则：

| 编号 | 条件 | 不允许 | 推荐替代 |
|---|---|---|---|
| R1 | `slam_status=healthy` 且目标在拓扑图中 | 直接 `start_mapless_scout` | `create_navigation_subgoal` 到安全观察点 |
| R2 | `human_near_target.distance_m < 1.5` | 直接去目标点 | `wait_until` 或去观察点 |
| R3 | `link_quality.bandwidth_kbps < 200` | 继续 raw_video | `semantic_only` |
| R4 | `localized=false` | 远距离 mapped navigation | `safe_hold`、`mapless_scout` 或人工确认 |
| R5 | `battery_percent < 20` 且网络弱 | 继续低优先级任务 | 请求人工确认或返回 |
| R6 | 工具不在白名单 | 执行 | 拒绝并重试 |
| R7 | 速度超过安全阈值 | 执行 | 降速或拒绝 |
| R8 | `requires_human_ack=false` 但策略不确定 | 执行 | 改为 `true` 或请求确认 |

---

## 6. 初始评估场景

至少覆盖：

```text
正常有图巡检
目标点有人
路径上有人
玻璃门关闭
地面湿滑
网络弱但电量足
网络弱且电量低
SLAM 健康
SLAM 退化
无地图前出侦察
Mapless 返回置信度低
工具不可用
用户要求高速
用户要求穿越人群
目标不在拓扑图
```

每类至少 3 到 5 条，先得到 50 条左右。

---

## 7. 微调数据格式

推荐 JSONL，每行一条。

```json
{
  "messages": [
    {
      "role": "system",
      "content": "You are the local strategy planner running on a quadruped robot edge computer. Output exactly one JSON object matching local_llm_plan.schema.json."
    },
    {
      "role": "user",
      "content": "{\"user_command\":\"Inspect lab_door. Wait if a person is present.\",\"world_state\":{\"slam_status\":\"healthy\",\"link_quality\":{\"bandwidth_kbps\":90},\"risk_events\":[{\"event_type\":\"human_near_target\",\"node_id\":\"lab_door\",\"distance_m\":0.6}]},\"semantic_topology\":{\"current_node\":\"start_area\",\"target_node\":\"lab_door\",\"safe_observation_node\":\"corridor_a\"},\"registered_tools\":[\"set_communication_policy\",\"create_navigation_subgoal\",\"wait_until\",\"capture_keyframe\",\"request_human_confirm\",\"hold_position\"]}"
    },
    {
      "role": "assistant",
      "content": "{\"plan_id\":\"inspect-lab-door-001\",\"mode\":\"mapped_navigation\",\"confidence\":0.86,\"reason\":\"SLAM is healthy, but a person is near the target and the link is weak. Use semantic-only communication, move only to the safe observation node, then wait.\",\"steps\":[{\"step_id\":\"switch-low-bandwidth\",\"tool\":\"set_communication_policy\",\"arguments\":{\"mode\":\"semantic_only\",\"send\":[\"task_state\",\"risk_events\",\"keyframe\"],\"drop\":[\"raw_video\",\"dense_pointcloud\"],\"reason\":\"weak link\"}},{\"step_id\":\"go-observation-node\",\"tool\":\"create_navigation_subgoal\",\"arguments\":{\"goal_id\":\"go-corridor-a\",\"target_node\":\"corridor_a\",\"target_pose\":{\"x\":1.5,\"y\":0.3,\"yaw\":0.0},\"constraints\":{\"max_linear_speed_mps\":0.35,\"max_angular_speed_rps\":0.4,\"safety_mode\":\"conservative\"}}},{\"step_id\":\"wait-target-clear\",\"tool\":\"wait_until\",\"arguments\":{\"condition\":{\"type\":\"node_clear\",\"node_id\":\"lab_door\",\"category\":\"person\",\"min_clear_distance_m\":1.5},\"timeout_s\":30}}],\"communication_policy\":{\"mode\":\"semantic_only\",\"send\":[\"task_state\",\"risk_events\",\"keyframe\"],\"drop\":[\"raw_video\",\"dense_pointcloud\"],\"reason\":\"weak link\"},\"requires_human_ack\":false}"
    }
  ]
}
```

---

## 8. 本阶段不要做什么

不要做：

- 不要训练时让模型背具体地图。
- 不要在 NX 8GB 上做主训练。
- 不要让模型输出速度控制。
- 不要开放 shell、cmd_vel、raw_slam_operate。
- 不要先上复杂 DPO/RL。
- 不要把 VLM 接进主策略闭环。

先做：

```text
schema 约束
离线评估
策略 validator
失败样本收集
SFT 小样本
量化部署回测
```

