# 机器狗仓库与现有工程冲突审阅

更新时间：2026-05-19

本文记录 `agent/llm-on-robot` 分支 clone 到机器狗后的目录关系、职责边界和集成风险。

## 当前机器狗目录

```text
/home/unitree/Go2W_SLAM_AI
/home/unitree/slam_gateway_refactor
/home/unitree/llm_runtime
/home/unitree/models
```

## 没有直接路径覆盖冲突

当前这些路径之间没有直接文件覆盖冲突：

- `/home/unitree/Go2W_SLAM_AI`：git 管理的项目 clone。
- `/home/unitree/slam_gateway_refactor`：现有 Unitree SDK2 / SLAM 执行网关。
- `/home/unitree/llm_runtime`：本地 LLM 推理运行时目录。
- `/home/unitree/models`：模型权重目录，必须保持在 git 外部。

## 职责边界

### `/home/unitree/Go2W_SLAM_AI/cpp`

当前 `cpp/` 是 SDK-free 的 dry-run executor。

它负责校验 `LocalLlmPlan` 并打印计划执行序列，但不调用 Unitree SDK，也不会移动机器狗。

因此它不能直接替代 `/home/unitree/slam_gateway_refactor`。

### `/home/unitree/slam_gateway_refactor`

这是当前真实机器人执行网关，负责：

- Unitree SDK2 service 调用。
- `slam_llm_command_client`。
- `slam_keyboard_client`。
- `SlamGateway`。
- 真实导航前的基础安全检查。

LLM 输出必须先经过 plan 校验和本地规则兜底，再进入这个执行层。

### `/home/unitree/llm_runtime`

这个目录只放推理运行时：

- llama.cpp 源码。
- llama.cpp build 产物。
- 已安装的运行脚本。

不要把这些 build 产物提交进 git。

### `/home/unitree/models`

这个目录只放模型权重，例如 GGUF。

当前模型：

```text
/home/unitree/models/Qwen_Qwen3-4B-Instruct-2507-Q4_K_M.gguf
```

模型不进入 git。

## 当前机器狗侧 LLM 状态

已完成：

- llama.cpp 源码位于 `/home/unitree/llm_runtime/llama.cpp`。
- CPU 版 `llama-cli` 已编译完成：

  ```text
  /home/unitree/llm_runtime/llama.cpp/build/bin/llama-cli
  ```

- git clone 已位于：

  ```text
  /home/unitree/Go2W_SLAM_AI
  ```

- 机器狗 SSH key 能以 `ccj-bot` 身份访问 GitHub。
- `ask_qwen.sh` 已安装到：

  ```text
  /home/unitree/llm_runtime/scripts/ask_qwen.sh
  ```

- GGUF 模型已上传到 `/home/unitree/models`。
- 模型 SHA256 已校验通过。

待完成：

- 重新跑一次干净的短推理 smoke test。
- 修复并重编 `/home/unitree/slam_gateway_refactor`。
- 将 plan validator / executor 接入真实 C++ 网关。

## 已知集成风险

### 1. `slam_gateway_refactor` 当前源码可能无法重编

文件：

```text
/home/unitree/slam_gateway_refactor/src/slam_gateway.cpp
```

问题代码：

```cpp
pose.mode = j.value("mode", 0);
```

这里的 `j` 未定义。建议先改成：

```cpp
pose.mode = 0;
```

然后再重新编译。

### 2. LLM 不能直接输出 Unitree raw API ID

禁止让模型直接输出或调用：

```text
1102
1201
1202
```

现有 `LlmCommandProcessor` 已经有 raw API ID 拒绝逻辑，后续 C++ plan executor 也要保留这条边界。

### 3. 弱网策略必须由本地状态触发

弱网不应该靠用户说“弱网”。

应由本地状态判断：

```text
world_state.link_quality.bandwidth_kbps < weak_bandwidth_kbps
```

触发后强制：

```text
set_communication_policy -> create_navigation_subgoal -> wait_until
```

并丢弃：

```text
raw_video
dense_pointcloud
high_rate_images
```

### 4. 模型可能输出不完整 JSON 或内层 JSON

必须先经过 schema 校验。

如果模型只输出了 `communication_policy`、`target_pose` 或其他内层对象，必须拒绝执行。

### 5. `LocalLlmPlan` 到真实 SLAM 命令的转换尚未完成

当前路径：

```text
local_llm_plan -> dry-run executor
```

目标路径：

```text
local_llm_plan -> C++ validator -> C++ executor -> slam_llm_command_client/SlamGateway
```

## 建议下一步

1. 重新做一次短推理 smoke test。
2. 修复 `slam_gateway_refactor` 的重编问题。
3. 把 Python 里的 guardrail 逻辑迁移到 C++。
4. 增加 C++ dry-run 到真实 gateway 的转换层。
5. 先 dry-run，再执行 `get_world_state`，最后再做小范围真实导航。

