# 世界状态返回与 LLM 输入模拟闭环

本文档记录当前从 SLAM 状态返回推进到 LLM 规划输入的实现。

当前阶段仍然不接真实大模型，而是先用规则模拟 LLM。目的不是追求智能，而是把协议边界跑通：

```text
机器狗实时状态
↓
WorldState / PlannerInput 摘要
↓
模拟 LLM 输出 LocalLlmPlan
↓
转换成 slam_command
↓
后续再由执行器决定是否发给机器狗
```

---

## 1. 新增文件

```text
src/edge_autonomy/llm_context.py
scripts/simulate_llm_planning.py
tests/test_llm_context.py
```

作用：

| 文件 | 作用 |
|---|---|
| `llm_context.py` | 把 runtime snapshot + map registry 转成 planner input，并模拟 LLM plan |
| `simulate_llm_planning.py` | 现场只读采集状态，模拟用户命令到 LLM 输出 |
| `test_llm_context.py` | 验证回 701入口过道中间、去聂国篱办公室前方、停下、SLAM 异常等策略 |

---

## 2. 怎么运行

只读模拟，不发送运动命令：

```powershell
cd E:\GO2W_0
python .\scripts\simulate_llm_planning.py --command "我现在在701实验室，去国篱师兄门口看看" --host 192.168.123.18 --username unitree --password 123 --map-id test_current_main --map-path /home/unitree/test.pcd --pretty
```

这个脚本会做三件事：

1. 通过 SSH 采集 SLAM/LiDAR 快照。
2. 生成 LLM planner input。
3. 用规则模拟 LLM 输出，并生成一条 `slam_command`。

注意：它不会实际调用 `1102`，不会让机器狗运动。

---

## 3. Planner Input 长什么样

现场测试中，机器狗在 `701入口过道中间` 附近时，输入摘要类似：

```json
{
  "user_command": "我现在在701实验室，去国篱师兄门口看看",
  "world_state_summary": {
    "map": {
      "map_id": "test_current_main",
      "pcd_path": "/home/unitree/test.pcd"
    },
    "robot": {
      "pose": {
        "x": 1.161,
        "y": -0.134,
        "yaw": -0.034
      },
      "nearest_node": {
        "node_id": "701_entrance_hallway_mid",
        "distance_m": 0.015
      },
      "localized": true
    },
    "slam": {
      "health_status": "ok",
      "localization_status": "localized_or_tracking"
    },
    "lidar": {
      "alive": true,
      "cloud_frequency_hz": 15.0,
      "cloud_size": 58234
    },
    "topology": {
      "available_nodes": [
        "701_entrance_hallway_mid",
        "nie_guoli_office_front"
      ]
    },
    "allowed_actions": [
      "hold_position",
      "request_human_confirm",
      "navigate_to_verified_node",
      "pause_navigation"
    ]
  }
}
```

LLM 不看原始点云，不看完整 ROS topic，只看这个压缩状态。

---

## 4. 模拟 LLM 输出

用户说：

```text
我现在在701实验室，去国篱师兄门口看看
```

模拟 LLM 输出：

```json
{
  "mode": "mapped_navigation",
  "confidence": 0.82,
  "reason": "matched user command to topology node nie_guoli_office_front",
  "steps": [
    {
      "tool": "set_communication_policy"
    },
    {
      "tool": "create_navigation_subgoal",
      "arguments": {
        "map_id": "test_current_main",
        "target_node": "nie_guoli_office_front",
        "target_pose": {
          "x": 3.258938789367676,
          "y": -2.2877299785614014,
          "yaw": -1.6144882723819336
        },
        "speed_mps": 0.45,
        "mode": 0,
        "safety_mode": "normal"
      }
    },
    {
      "tool": "wait_until",
      "arguments": {
        "source": "/slam_info",
        "condition": "type == 'ctrl_info' and (data.is_arrived == true or data.stateMachine.state == 'FINISHED')"
      }
    }
  ],
  "requires_human_ack": false
}
```

这和 `schemas/local_llm_plan.schema.json` 的方向一致。

---

## 5. 转成可执行 slam_command

脚本还会生成一条可执行命令，但不自动发送：

```json
{
  "action": "navigate_to_pose",
  "map_id": "test_current_main",
  "target_node": "nie_guoli_office_front",
  "target_pose": {
    "name": "nie_guoli_office_front",
    "x": 3.258938789367676,
    "y": -2.2877299785614014,
    "z": -0.08693132549524307,
    "q_x": 0.011335774324834347,
    "q_y": 0.013992785476148129,
    "q_z": -0.722239077091217,
    "q_w": 0.6914089918136597,
    "yaw": -1.6144882723819336,
    "speed": 0.45,
    "mode": 0
  }
}
```

后续执行器要做的事情是：

```text
检查 SafetySupervisor
检查目标点距离和路线状态
检查当前是否已在目标点附近
通过后才发给 slam_llm_command_client 或底层 API
```

---

## 6. 当前保护规则

已经实现的规则：

| 条件 | 输出 |
|---|---|
| 用户说停下/暂停/别动 | `safe_hold` |
| SLAM 不健康 | `safe_hold` + 需要人工确认 |
| 定位不可靠 | `safe_hold` + 需要人工确认 |
| 命令匹配不到拓扑点 | `human_confirm` |
| 当前距离目标点小于 `0.25m` | `safe_hold`，不重复导航 |
| 命令匹配到拓扑点且状态正常 | `mapped_navigation` |

其中“当前已在目标附近不重复导航”很重要，可以减少到点后反复微调和跺脚。

---

## 7. 下一步

下一步应该做执行器，而不是直接接真实 LLM：

```text
1. planner_input 生成
2. mock_llm_plan 生成
3. SafetySupervisor 审核
4. slam_command 执行
5. 监听 /slam_info ctrl_info
6. 到达后自动 pause/hold
```

等这个闭环稳定后，再把 `simulate_local_llm_plan()` 换成真实本地 LLM 或云端 LLM。
