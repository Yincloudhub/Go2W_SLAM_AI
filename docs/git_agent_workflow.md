# Git Agent 分支协作规范

本文说明这个仓库后续如何用 agent 分支管理机器狗和 LLM 集成工作。

## 分支约定

- `main`：稳定主线，只放已经确认可复现、可说明的版本。
- `agent/llm-on-robot`：当前 Codex agent 工作分支，用于本地 LLM 规划器、机器狗部署说明、评测数据、guardrail 规则和机器人侧运行脚本。

后续如果有新的并行任务，建议使用：

```text
agent/<任务范围>
```

示例：

```text
agent/cpp-plan-executor
agent/robot-llm-runtime
agent/map-topology-data
```

## 提交内容约定

可以提交：

- 源码。
- schema。
- prompt。
- 配置文件。
- 文档。
- 小规模评测集和 SFT 样本。
- 测试代码。
- 机器人侧安装脚本。

不要提交：

- 模型权重。
- 机器狗恢复包。
- 构建目录。
- 日志。
- 大型评测产物。
- 缓存文件。
- 点云、bag、db3 等运行时数据。

当前 `.gitignore` 已经排除了这些常见运行产物。

## 提交信息

提交信息尽量说明“改了什么”和“为什么改”。

示例：

```text
Add robot-side LLM runtime checklist
Harden local planner guardrails
Add floorplan v4 auto weak-network eval cases
```

后续也可以直接用中文提交信息。

## 推送方式

当前远端：

```text
origin git@github-go2w:Yincloudhub/Go2W_SLAM_AI.git
```

推送当前 agent 分支：

```bash
git push -u origin agent/llm-on-robot
```

机器狗侧使用自己的 SSH key 和 remote alias：

```text
git@github-go2w-robot:Yincloudhub/Go2W_SLAM_AI.git
```

这样可以把 PC 端凭据和机器狗端凭据隔离。

## 合并方式

当 `agent/llm-on-robot` 准备合入 `main` 前，至少需要确认：

- 本地测试通过。
- 机器狗侧 dry-run 已记录。
- 没有提交模型、日志、构建产物和恢复包。
- 部署说明和实际目录状态一致。
- 真实导航前有明确的人工确认和回退方案。

