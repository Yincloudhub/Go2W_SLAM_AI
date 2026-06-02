---
created: 2026-05-15
updated: 2026-05-15
status: draft
type: design
tags:
  - 机器狗/LLM
  - 机器狗/蒸馏
  - 机器狗/微调
  - 机器狗/实时分层
  - 机器狗/工具调用
---

# 本地 LLM 规划、蒸馏与分层机制

## 一句话结论

本地 LLM 不负责实时控制，不替代 SLAM。它负责在低频 `WorldState summary` 上做任务级规划，输出结构化工具调用。

## 运行时分层

```text
实时传感器层
LiDAR / IMU / Odom / Camera / mmWave
10-50 Hz

↓
SLAM Gateway / Perception Adapters
2-20 Hz

↓
SafetySupervisor
5-20 Hz

↓
WorldStateBuilder
1-5 Hz

↓
Event / Summary Gate
事件触发或 0.2-1 Hz

↓
Local LLM Planner
任务级规划
```

原则：

```text
SLAM 实时闭环不经过 LLM。
安全停车不经过 LLM。
局部避障不经过 LLM。
LLM 只看摘要，不看原始点云。
```

## LLM 输入

输入应类似：

```json
{
  "user_command": "去701外面的走廊巡视",
  "slam": {
    "map_id": "701",
    "health": "ok",
    "localization_status": "localized",
    "confidence": 0.86
  },
  "navigation": {
    "state": "idle"
  },
  "local_environment": {
    "front_clearance_m": 2.8,
    "dynamic_target_near_path": false
  },
  "topology": {
    "current_space": "701",
    "available_nodes": ["701_center", "701_door_inside"]
  },
  "safety": {
    "allow_navigation": true,
    "recommended_mode": "conservative"
  }
}
```

## LLM 输出

输出只允许是工具调用：

```json
{
  "intent": "patrol_corridor_outside_701",
  "tools": [
    {
      "name": "create_navigation_subgoal",
      "arguments": {
        "target_node": "701_door_inside",
        "safety_mode": "conservative"
      }
    }
  ]
}
```

执行链路：

```text
LLM output
=> JSON schema validation
=> PlannerPolicyValidator
=> SafetySupervisor
=> Executor
=> SLAM Gateway
=> unitree_slam
```

## 大模型蒸馏

可以用更大模型离线生成训练样本，但目标是窄能力蒸馏：

1. 任务意图理解。
2. SLAM 状态判断。
3. 定位失败时拒绝移动。
4. 障碍物出现时保守策略。
5. 弱网时只传摘要和关键帧。
6. 固定 JSON 工具调用格式。

不指望蒸馏解决：

```text
SLAM 没走通
PCD 没加载
位姿不可信
拓扑点没定义
点云没预处理
```

## 微调速度收益

微调不会显著提升底层 `tokens/s`。

它提升的是：

1. 输出更短。
2. JSON 更稳定。
3. 重试更少。
4. 系统提示词更短。
5. 一次可执行率更高。

准确表述：

```text
微调提升有效任务响应速度和输出稳定性，不是直接提升底层推理速度。
```

## 样本库重点

样本不能只包含“成功导航”，必须包含：

1. `slam_lost` 时拒绝移动。
2. `map_loaded but not localized` 时请求重定位。
3. `front_clearance_m` 太小时暂停。
4. `dynamic_target_near_path` 时限速或等待。
5. 弱网时降低交互频率。
6. 用户危险指令时拒绝执行。
