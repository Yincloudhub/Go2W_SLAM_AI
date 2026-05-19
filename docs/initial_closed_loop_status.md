# 机器狗初步闭环验证状态

更新时间：2026-05-19

## 当前结论

目前已经完成“非运动 dry-run 闭环”，但还没有完成“真实 LLM 实时推理闭环”。

原因不是模型文件缺失，而是当前机器狗上的 CPU 版 llama.cpp 跑 Qwen3-4B Q4 太慢：用 `-n 1` 只生成 1 个 token，240 秒仍然超时。

## 已经完成的部分

### 1. Git 与目录隔离

机器狗侧仓库：

```text
/home/unitree/Go2W_SLAM_AI
```

运行时目录：

```text
/home/unitree/llm_runtime
```

模型目录：

```text
/home/unitree/models
```

现有真实执行网关：

```text
/home/unitree/slam_gateway_refactor
```

模型、build、日志和运行产物不进入 git。

### 2. LLM 文件已安装

模型路径：

```text
/home/unitree/models/Qwen_Qwen3-4B-Instruct-2507-Q4_K_M.gguf
```

SHA256 已验证：

```text
2fde00ce69dd4899c70d020845e2638353015bba0fdf161b3eb965f2bca4464e
```

llama.cpp CPU 可执行文件：

```text
/home/unitree/llm_runtime/llama.cpp/build/bin/llama-cli
```

推理 wrapper：

```text
/home/unitree/llm_runtime/scripts/ask_qwen.sh
```

### 3. Dry-run 闭环已通过

在机器狗上执行 mock planner，已经能形成：

```text
用户目标/node_id
  -> LocalLlmPlan
  -> create_navigation_subgoal
  -> wait_until
  -> slam_command: navigate_to_pose
```

示例结果：

```text
mode= mapped_navigation
steps= set_communication_policy, create_navigation_subgoal, wait_until
slam_action= navigate_to_pose
target_node= zhao_bo_office_front
```

这一步不会移动机器狗，只证明上层计划到结构化 SLAM 命令的转换链路可走通。

## 当前阻塞点

### 1. CPU 推理不可用作实时闭环

测试命令等价于：

```bash
timeout 240 /home/unitree/llm_runtime/llama.cpp/build/bin/llama-cli \
  -m /home/unitree/models/Qwen_Qwen3-4B-Instruct-2507-Q4_K_M.gguf \
  -t 6 -c 512 -n 1 --temp 0.1 --no-display-prompt \
  -p 'JSON:'
```

结果：

```text
RC=124
ELAPSED=240s
```

说明 240 秒仍未完成 1 token 输出。当前 CPU 版 4B Q4 不适合直接做实时控制闭环。

### 2. CUDA 版 llama.cpp 还不能直接编译

尝试配置：

```bash
cmake -S . -B build-cuda -DGGML_CUDA=ON
```

失败原因：

```text
CMake 3.18 or higher is required. You are running version 3.16.3
```

机器狗当前系统 CMake 是 3.16.3，当前 llama.cpp CUDA 后端要求至少 3.18。

## 推荐闭环路线

### P0：先做非运动闭环

目标：

```text
自然语言/目标点
  -> planner
  -> LocalLlmPlan
  -> guardrail
  -> slam_command
  -> dry-run 输出
```

不调用真实 `navigate_to_pose`。

这一层已经基本可跑。

### P1：解决本地推理速度

二选一：

1. 升级/安装本地 CMake 3.18+，编译 llama.cpp CUDA 版。
2. 先换 0.5B 或 1.5B GGUF 小模型，在 CPU 上跑通真实 LLM 闭环。

建议优先顺序：

```text
小模型 CPU 闭环 -> llama.cpp CUDA -> 再回到 Qwen3-4B
```

原因：先证明闭环机制，比一开始追求 4B 模型效果更重要。

### P2：接入真实执行网关

在真实运动前必须完成：

1. 修复 `/home/unitree/slam_gateway_refactor` 当前源码重编问题。
2. 把 Python guardrail 迁移到 C++。
3. 增加 C++ plan validator。
4. 增加 C++ plan executor。
5. 先只执行 `get_world_state`。
6. 再执行 `hold_position` / `request_human_confirm`。
7. 最后才执行短距离 `navigate_to_pose`。

## 当前可用命令

安装/更新机器人侧脚本：

```bash
cd /home/unitree/Go2W_SLAM_AI
bash robot/llm_runtime/install_runtime_files.sh
```

运行模型 smoke test：

```bash
/home/unitree/llm_runtime/scripts/smoke_test_llm.sh
```

运行非运动 mock 闭环：

```bash
cd /home/unitree/Go2W_SLAM_AI
PYTHONPATH=src python3 scripts/run_local_llm_planner.py \
  --backend mock \
  --no-live-snapshot \
  --registry configs/maps/go2w_floorplan_v4_map_registry.json \
  --map-id floorplan_demo_v4 \
  --command "zhao_bo_office_front capture photo" \
  --pretty
```

