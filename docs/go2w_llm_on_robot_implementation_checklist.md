# GO2W 机器狗本地 LLM 部署实施清单

更新时间：2026-05-19

目标：先在机器狗本地装上 LLM 推理能力，形成“自然语言输入 -> 本地 LLM 规划 -> 安全规则兜底 -> 结构化 SLAM 命令 -> 机器狗执行”的可控闭环。

## 1. 当前机器狗摸底结论

- SSH：`unitree@192.168.3.17`
- 系统：Ubuntu 20.04.5 LTS，aarch64，Jetson/Tegra 内核 `5.10.104-tegra`
- 资源：8 核 ARM，15 GiB 内存，NVMe 根分区约 423 GiB 可用
- 网络：
  - `wlan0`: `192.168.3.17/24`
  - `eth0`: `192.168.123.18/24`
- 已有核心工程：`/home/unitree/slam_gateway_refactor`
- 已有 C++ 可执行文件：
  - `/home/unitree/slam_gateway_refactor/build/slam_keyboard_client`
  - `/home/unitree/slam_gateway_refactor/build/slam_llm_command_client`
- 当前 LLM 运行时准备状态：
  - `/home/unitree/llm_runtime/llama.cpp` 已放入源码
  - CPU 版 `llama-cli` 已编译通过
  - 模型目录 `/home/unitree/models` 已创建

## 2. 现有代码架构

现有 `slam_gateway_refactor` 已经是比较合适的接入基础：

- `slam_keyboard_client`
  - 保留原始键盘控制路径。
  - 不建议把 LLM 逻辑塞进这里。

- `slam_llm_command_client`
  - 当前接受 stdin 中的一行 JSON 命令。
  - 入口文件：`src/llm_command_main.cpp`
  - 命令处理：`src/llm_command_processor.cpp`
  - 已支持动作：
    - `get_world_state`
    - `start_mapping`
    - `end_mapping`
    - `relocate`
    - `navigate_to_pose`
    - `pause_navigation`
    - `resume_navigation`
    - `stop_slam`

- `SlamGateway`
  - 文件：`src/slam_gateway.cpp`
  - 负责 Unitree SDK2 API 调用、订阅 `rt/slam_info`、`rt/slam_key_info`、构造世界状态。

- `SafetySupervisor`
  - 文件：`src/safety_supervisor.cpp`
  - 当前已有基础安全判断：SLAM 失效、定位丢失、前方障碍过近时阻止导航。

- `TopologyManager`
  - 文件：`src/topology_manager.cpp`
  - 当前管理 `/home/unitree/topology_points.json`。

## 3. 需要先处理的代码风险

接 LLM 前，先保证 `slam_gateway_refactor` 可以从源码重新编译。

当前发现一个源码问题：

```cpp
pose.mode = j.value("mode", 0);
```

位置：`/home/unitree/slam_gateway_refactor/src/slam_gateway.cpp`

这里的 `j` 未定义。现有 build 产物能运行不代表当前源码能重编。建议改成：

```cpp
pose.mode = 0;
```

另外，`scripts/build_on_go2.sh` 当前没有执行权限。可以直接：

```bash
cd /home/unitree/slam_gateway_refactor
bash scripts/build_on_go2.sh
```

或者：

```bash
chmod +x scripts/build_on_go2.sh build_on_go2.sh
./scripts/build_on_go2.sh
```

2026-05-27 复查：机器狗端 `/home/unitree/slam_gateway_refactor/src/slam_gateway.cpp` 已经把上述 `j.value` 风险修成 `pose.mode = 0`。新的优先修正点是 `slam_llm_command_client` 的机器可读入口需要更严格的输入校验：

- `navigate_to_pose` 不应在 `target_pose` 缺少 `x/y` 时默认导航到地图原点。
- `relocate` 不应在缺少 `initial_pose` 时默认用零点重定位。
- `speed`、`mode` 应限制在安全范围。
- `start_mapping`、`end_mapping`、`stop_slam` 这类高风险动作应要求显式 `operator_ack=true` 或 `confirm=true`。

本地版本管理目录 `robot/slam_gateway_refactor` 已加入这些防护，后续同步到机器人端后需要重编 `/home/unitree/slam_gateway_refactor`。

## 4. 推理后端选择建议

结论：P0 阶段优先用 `llama.cpp`，后续再评估 TensorRT/Edge-LLM。

### 4.1 llama.cpp

适合现在先用。

优点：

- C/C++ 原生，和现有 `slam_gateway_refactor` 集成路径短。
- GGUF 模型直接可用，适合 Qwen3-4B Q4 这类量化模型。
- CPU 版已经在机器狗上编译通过。
- 后续可以尝试 CUDA 版，但 P0 不依赖 GPU。
- 失败模式简单，便于做超时、JSON 提取、重试、拒绝执行。

缺点：

- CPU 推理速度可能偏慢。
- Jetson CUDA 加速需要单独编译和测试，可能遇到 CUDA/架构兼容问题。

建议用途：

- P0：自然语言 -> JSON plan。
- 输出 token 控制在 512 到 1024。
- 只生成结构化计划，不直接控制电机或 Unitree raw API。

### 4.2 TensorRT-LLM

暂不建议作为第一阶段。

优点：

- NVIDIA GPU 优化路线，性能潜力高。
- 适合后期追求低延迟、高吞吐。

问题：

- Jetson 支持更偏 JetPack 6.1/Orin 路线。
- 当前机器狗是 Ubuntu 20.04、CUDA 11.4，迁移复杂度高。
- 模型转换、engine 构建、版本依赖都比 llama.cpp 重。
- 不适合作为第一版闭环的风险最低路径。

### 4.3 TensorRT Edge-LLM

值得后续关注，但不作为当前 P0。

原因：

- NVIDIA 新的嵌入式/机器人 LLM 路线更贴近 Jetson。
- 但仍需要确认当前机器狗 JetPack/CUDA/硬件是否满足。
- 当前目标是“先闭环”，不是先追求最优推理性能。

### 4.4 Ollama / vLLM / MLC

当前不优先。

- Ollama：部署方便，但作为机器人执行链路的 C++ 可控性不如 llama.cpp。
- vLLM：更偏服务器 GPU，机器狗本地部署不合适。
- MLC：可研究，但集成复杂度高于 llama.cpp。

## 5. P0 推荐目录结构

建议机器狗上使用下面结构：

```text
/home/unitree/
  llm_runtime/
    llama.cpp/
      build/
      build/bin/llama-cli
    scripts/
      ask_qwen.sh
  models/
    Qwen_Qwen3-4B-Instruct-2507-Q4_K_M.gguf
  go2w_llm/
    prompts/
      planner_system.txt
    configs/
      floorplan_demo_v4_map_registry.json
      llm_runtime.json
    logs/
    tmp/
```

不要直接把模型和推理脚本塞进 `slam_gateway_refactor`。`slam_gateway_refactor` 保持为 C++ 执行网关，LLM 运行时单独放。

## 6. 安装 LLM P0 步骤

### 6.1 检查 llama-cli

```bash
/home/unitree/llm_runtime/llama.cpp/build/bin/llama-cli --help | head
```

如果不存在，则重新编译：

```bash
cd /home/unitree/llm_runtime/llama.cpp
cmake -S . -B build \
  -DCMAKE_BUILD_TYPE=Release \
  -DGGML_CUDA=OFF \
  -DGGML_NATIVE=OFF \
  -DLLAMA_BUILD_TESTS=OFF \
  -DLLAMA_BUILD_EXAMPLES=ON
cmake --build build --config Release -j4 --target llama-cli
```

### 6.2 上传模型

本地模型路径：

```text
E:\codexprofile\models\Qwen3-4B-Instruct-2507-GGUF\Qwen_Qwen3-4B-Instruct-2507-Q4_K_M.gguf
```

机器狗目标路径：

```text
/home/unitree/models/Qwen_Qwen3-4B-Instruct-2507-Q4_K_M.gguf
```

上传后校验：

```bash
sha256sum /home/unitree/models/Qwen_Qwen3-4B-Instruct-2507-Q4_K_M.gguf
```

本地已知 SHA256：

```text
2fde00ce69dd4899c70d020845e2638353015bba0fdf161b3eb965f2bca4464e
```

### 6.3 创建 ask_qwen.sh

路径：

```text
/home/unitree/llm_runtime/scripts/ask_qwen.sh
```

建议内容：

```bash
#!/usr/bin/env bash
set -euo pipefail

MODEL_PATH="${MODEL_PATH:-/home/unitree/models/Qwen_Qwen3-4B-Instruct-2507-Q4_K_M.gguf}"
LLAMA_CLI="${LLAMA_CLI:-/home/unitree/llm_runtime/llama.cpp/build/bin/llama-cli}"
MAX_TOKENS="${MAX_TOKENS:-768}"
THREADS="${THREADS:-6}"
CTX_SIZE="${CTX_SIZE:-4096}"
TEMP="${TEMP:-0.1}"

SYSTEM_PROMPT=""
if [[ "${1:-}" == "--system" ]]; then
  SYSTEM_PROMPT="$2"
  shift 2
fi

PROMPT="$*"

exec "$LLAMA_CLI" \
  -m "$MODEL_PATH" \
  -t "$THREADS" \
  -c "$CTX_SIZE" \
  -n "$MAX_TOKENS" \
  --temp "$TEMP" \
  -p "${SYSTEM_PROMPT}

${PROMPT}"
```

赋权：

```bash
chmod +x /home/unitree/llm_runtime/scripts/ask_qwen.sh
```

### 6.4 最小推理测试

```bash
/home/unitree/llm_runtime/scripts/ask_qwen.sh '只输出JSON：{"ok":true}'
```

通过标准：

- 能加载模型。
- 能输出内容。
- 不崩溃。
- 记录首 token 延迟和生成速度。

## 7. 接入架构建议

不要让 LLM 直接调用 `navigate_to_pose`。

推荐分层：

```text
自然语言命令
  -> LLM Planner
  -> JSON plan
  -> Plan Validator / Guardrail
  -> Plan Executor
  -> slam_llm_command_client
  -> SlamGateway
  -> Unitree SLAM API
```

### 7.1 LLM 负责

- 解析用户意图。
- 选择目标点。
- 输出结构化 plan。

### 7.2 本地规则负责

这些不要完全交给 LLM：

- 目标点是否在拓扑表。
- 是否已经在目标附近。
- 低电量是否允许继续任务。
- 弱网策略。
- 是否需要人工确认。
- 是否允许导航。
- 是否补 `wait_until`。

### 7.3 C++ 执行层负责

- 将 plan 转成已有 `slam_llm_command_client` 支持的命令。
- 调用 `get_world_state`。
- 调用 `navigate_to_pose`。
- 监听到达状态。
- 拒绝 raw API ID。
- 日志记录每一步输入/输出。

## 8. 需要补的 C++ 模块

建议在 `slam_gateway_refactor` 中新增：

```text
include/slam_gateway/llm_plan.hpp
include/slam_gateway/llm_plan_validator.hpp
include/slam_gateway/llm_plan_executor.hpp
src/llm_plan_validator.cpp
src/llm_plan_executor.cpp
src/llm_planner_main.cpp
```

其中：

- `llm_plan_validator`
  - 校验 schema。
  - 校验工具名。
  - 校验 target_node 是否存在。
  - 强制弱网通信策略。
  - 强制低电量策略。
  - 强制已到达不重复导航。

- `llm_plan_executor`
  - 执行 `set_communication_policy`。
  - 执行 `create_navigation_subgoal`。
  - 执行 `wait_until`。
  - 执行 `capture_keyframe`。
  - 执行 `hold_position` / `request_human_confirm`。

- `llm_planner_main`
  - 接收自然语言命令。
  - 调用 llama.cpp。
  - 抽取 JSON。
  - 调用 validator。
  - 调用 executor。

## 9. P0 测试清单

### 9.1 推理测试

- 普通中文命令能输出 JSON。
- 输出必须是单个 JSON object。
- 不能输出 markdown。
- 不能输出 raw API ID：`1102`、`1201`、`1202`。

### 9.2 策略测试

- 目标不存在：必须 `human_confirm`。
- 目标不在当前拓扑：禁止 `create_navigation_subgoal`。
- 已在目标附近：必须 `hold_position`。
- 低电量普通任务：必须确认或回充。
- 低电量回充任务：允许导航到 `charging_point`。
- 弱网自动检测：用户不说弱网，只要 `bandwidth_kbps < 200`，必须先 `set_communication_policy`。
- 有图导航：必须 `create_navigation_subgoal -> wait_until`。

### 9.3 真实执行测试

按风险从低到高：

1. 只跑 `get_world_state`。
2. LLM 输出 plan，但不执行，只 dry-run。
3. 执行 `hold_position` / `request_human_confirm`。
4. 执行一个近距离 `navigate_to_pose`。
5. 执行一个拓扑点导航。
6. 执行拍照任务。
7. 执行弱网自动通信策略任务。

## 10. 当前优先级

建议顺序：

1. 上传 GGUF 模型到 `/home/unitree/models`。
2. 跑通 `/home/unitree/llm_runtime/scripts/ask_qwen.sh`。
3. 修复 `slam_gateway_refactor` 当前源码编译问题。
4. 把本地 Python 版 planner 的 guardrail 规则迁移到 C++。
5. 新增 `llm_planner_main`，先 dry-run。
6. dry-run 通过后，再接 `slam_llm_command_client` 实际执行。
7. 最后再考虑 CUDA llama.cpp 或 TensorRT 路线优化速度。

## 11. 后端选择的最终建议

当前阶段：

```text
优先 llama.cpp CPU/GGUF
```

原因：

- 最快闭环。
- 和 C++ 网关最匹配。
- 可控、可调、可离线。
- 失败时容易降级。

下一阶段：

```text
评估 llama.cpp CUDA
```

条件：

- CPU 推理延迟不能接受。
- CUDA 版能在当前 Jetson/CUDA 11.4 上稳定编译。
- 不影响 SLAM 和控制线程实时性。

更后面：

```text
评估 TensorRT Edge-LLM 或 TensorRT-LLM
```

条件：

- 需要明显更高性能。
- 确认 JetPack / CUDA / 硬件满足要求。
- 能接受模型转换和部署复杂度。
