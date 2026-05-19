# NX 本地 Qwen3 LLM 摸底记录

目标：只读检查 `ysy@192.168.33.30` 这台网线直连 Jetson NX 上的本地 LLM 是否可用于机器人本地策略闭环，并明确后续需要怎么调整。

时间：2026-04-24  
目标主机：`ysy@192.168.33.30`  
登录用户：`ysy`  
检查原则：只读检查，不改远端代码，不写远端文件。

---

## 1. 结论

这台 NX 上的本地 LLM **可以作为第一版本地策略器继续使用**，不需要立刻换模型。

当前可用组件：

```text
模型：/home/ysy/models/Qwen3-4B-Q4_K_M.gguf
推理：/home/ysy/llama.cpp/build/bin/llama-cli
脚本：/home/ysy/ask_qwen.sh
速度：约 12 到 14 tok/s
```

但它还不能直接进入机器人闭环，原因是：

1. 仅靠 prompt 时，输出可能出现 Markdown 代码块或截断。
2. 即使用 JSON schema 强约束，模型仍可能做出不理想的工具选择。
3. 当前没有常驻 LLM 服务，只有命令行脚本。
4. 这台设备是 Orin NX 8GB，不是 16GB，内存预算必须保守。

推荐判断：

```text
先保留 Qwen3-4B-Q4_K_M。
先做 prompt + json schema + rule validator + retry。
再用失败样本做 LoRA / QLoRA 微调。
只有并发压测或策略准确率不过，再切 Qwen2.5-3B。
```

---

## 2. 硬件与系统信息

检查到的关键信息：

```text
主机名：Jetson
系统：Ubuntu 22.04.4 LTS
内核：5.15.148-tegra
JetPack：6.1
L4T：36.4.0
设备：NVIDIA Jetson Orin NX Engineering Reference Developer Kit
模块：NVIDIA Jetson Orin NX 8GB RAM
电源模式：MAXN
CUDA：12.6.68
TensorRT：10.3.0.30
```

内存情况：

```text
总内存：约 7.4 GiB
空闲内存：约 5.3 GiB
Swap：约 3.7 GiB
```

注意：

**这台机器是 Orin NX 8GB。后续不要按 16GB 机器设计本地 LLM 常驻预算。**

---

## 3. 当前 LLM 环境

### 3.1 已有模型

```text
/home/ysy/models/Qwen3-4B-Q4_K_M.gguf
大小：约 2.4 GB
```

### 3.2 已有推理工具

```text
/home/ysy/llama.cpp
/home/ysy/llama.cpp/build/bin/llama-cli
/home/ysy/llama.cpp/build/bin/llama-server
/home/ysy/llama.cpp/build/bin/llama-bench
```

`llama-cli --version` 检查到：

```text
CUDA device：Orin
VRAM：约 7619 MiB
llama.cpp build：b1-8113977
```

### 3.3 Python 环境

系统 Python：

```text
Python 3.10.12
pip 22.0.2
```

当前系统 Python 缺少：

```text
torch
transformers
accelerate
vllm
llama_cpp
ollama
onnxruntime
```

已存在：

```text
tensorrt 10.3.0
```

工程含义：

第一阶段不要从 `transformers` 或 `vllm` 开始。当前最稳路径是继续用 `llama.cpp`。

---

## 4. 当前启动脚本

脚本：

```text
/home/ysy/ask_qwen.sh
```

当前默认参数：

```text
MODEL_PATH=/home/ysy/models/Qwen3-4B-Q4_K_M.gguf
BIN_PATH=/home/ysy/llama.cpp/build/bin/llama-cli
THREADS=6
CTX_SIZE=2048
GPU_LAYERS=99
FLASH_ATTN=1
REASONING_MODE=off
```

当前脚本优点：

1. 已封装模型路径和 `llama-cli`。
2. 默认关闭 reasoning，适合策略 JSON 输出。
3. 会统计 prompt speed、generation speed、耗时。
4. 已使用 GPU offload 和 flash attention。

当前脚本缺口：

1. 没有暴露 `--json-schema-file`。
2. 没有暴露 `--temp`、`--top-p`、`--seed`。
3. 没有严格 JSON 输出校验。
4. 没有自动重试。
5. 输出解析会保留模型可能生成的 Markdown 或不完整 JSON。

推荐后续新增一个包装脚本，而不是直接改原脚本：

```text
/home/ysy/ask_robot_planner.sh
```

它专门用于机器人策略规划，并默认启用：

```text
--temp 0
--top-p 1
--reasoning off
--json-schema-file planner_plan.schema.json
--max-tokens 256 或 384
```

---

## 5. 推理测试结果

### 5.1 普通 prompt 测试

场景：

```text
用户要求去实验室门口巡检。
SLAM 正常。
网络弱。
lab_door 附近 0.6m 有人。
当前在 start_area。
可去 corridor_a 观察点。
```

结果：

```text
模型能运行。
速度约 14 tok/s。
但第一次输出出现 Markdown fence，并且因为 token 限制被截断。
```

问题：

```text
不能只靠 prompt 保证机器人闭环输出。
```

### 5.2 ASCII 严格 prompt 测试

结果：

```text
模型输出了合法 JSON。
速度约 14 tok/s。
```

但仍有问题：

```text
工具字段不稳定。
会输出未注册或不符合我们新接口的 action，例如 inspect。
```

### 5.3 `--json-schema` 约束测试

直接使用 `llama-cli --json-schema` 后，模型可以输出符合 schema 的 JSON。

测试命令使用了：

```text
--temp 0
--top-p 1
--json-schema <planner schema>
```

结果：

```json
{
  "plan_id": "lab_door_inspection",
  "mode": "safe_hold",
  "reason": "Person is near lab_door, semantic_only communication preferred, and safe_observation_node is corridor_a.",
  "requires_human_ack": true,
  "steps": [
    {
      "step_id": "inspect_lab_door",
      "tool": "start_mapless_scout",
      "arguments": {
        "target": "lab_door",
        "mode": "semantic_only"
      }
    },
    {
      "step_id": "wait_for_person",
      "tool": "wait_until",
      "arguments": {
        "condition": "person_present",
        "timeout": 10
      }
    }
  ]
}
```

这个结果说明：

1. JSON 格式可以用 `--json-schema` 解决。
2. 语义策略仍需规则校验和微调。
3. 仅靠 schema 不能保证“SLAM 正常时优先 mapped navigation”的策略正确性。

---

## 6. 当前模型能不能用

可以用，但要限定用途。

适合：

- 本地策略 JSON 输出。
- 用户命令归一化。
- 弱网通信策略选择。
- 有图导航 / Mapless Scout / safe hold 的模式建议。
- 生成可解释 reason。

不适合直接做：

- 运动控制。
- 安全裁决。
- 原始图像理解。
- 原始点云处理。
- 绕过规则直接调用工具。

推荐运行方式：

```text
WorldState + SemanticTopology + UserCommand
  -> Qwen3-4B 本地策略 JSON
  -> JSON schema 校验
  -> rule validator 策略校验
  -> SafetySupervisor 运动审核
  -> ToolExecutor 执行
```

---

## 7. 需要怎么调整

### 7.1 立即调整

第一步：所有策略输出必须使用 llama.cpp 的 schema 约束。

```text
llama-cli --json-schema-file schemas/local_llm_plan.schema.json
```

第二步：推理参数改保守。

```text
--temp 0
--top-p 1
--reasoning off
--ctx 2048
--max-tokens 256 到 384
```

第三步：输出后必须做二次校验。

```text
JSON parse
schema validate
tool whitelist validate
policy rule validate
safety validate
```

### 7.2 策略规则校验

需要补一个 `PlannerPolicyValidator`，用于拦截“格式正确但策略不对”的输出。

典型规则：

| 场景 | 必须拦截 |
|---|---|
| `slam_status=healthy` 且目标在已知拓扑图 | 不应直接选择 `start_mapless_scout` |
| `risk_events` 有 `human_near_target` | 不应直接靠近目标点 |
| `link_quality` 为 weak | 应优先 `set_communication_policy semantic_only` |
| `localized=false` | 不应下发远距离 `create_navigation_subgoal` |
| `battery_percent` 很低 | 应返回/请求人工确认 |
| 工具参数缺少速度、安全模式、目标节点 | 拒绝执行 |

### 7.3 微调方向

微调不是为了学地图，而是为了学：

```text
固定 JSON 结构
工具选择偏好
弱网处理策略
风险场景保守行为
SLAM 退化时切换模式
不调用不存在的工具
```

建议先用现有 prompt 文件作为历史样本来源：

```text
/home/ysy/01_clear_corridor_planner_prompt.txt
/home/ysy/02_person_blocking_path_planner_prompt.txt
/home/ysy/03_closed_glass_door_planner_prompt*.txt
/home/ysy/04_slippery_floor_planner_prompt.txt
/home/ysy/05_weak_network_low_battery_planner_prompt*.txt
```

这些样本需要转换成新的工具计划格式，不能直接照搬旧 schema。

---

## 8. 推荐下一步

第一阶段目标：

```text
不微调，先做 prompt + schema + validator。
```

完成标准：

```text
50 到 100 条离线场景中：
JSON 可解析率 >= 99%
工具名合法率 >= 99%
危险场景保守率 >= 95%
弱网策略正确率 >= 95%
```

第二阶段目标：

```text
收集失败样本，构造 300 到 800 条 SFT 数据。
```

第三阶段目标：

```text
训练 LoRA / QLoRA，量化后回到 NX 上评估。
```

---

## 9. 最终判断

当前 `Qwen3-4B-Q4_K_M.gguf` 在这台 Orin NX 8GB 上是可用的，但必须从“聊天模型调用”调整为“受约束的机器人策略生成器”。

最重要的调整不是换模型，而是：

```text
启用 llama.cpp JSON schema 约束
加入策略规则校验器
加入 SafetySupervisor
收集失败样本做微调
```

