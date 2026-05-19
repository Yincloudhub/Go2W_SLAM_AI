# GO2W Edge Autonomy

这个仓库把 GO2W 的官方 SLAM / Navigation 当作底层执行底座，不直接从第一步就硬改官方 SLAM 内核。

我们当前的开发目标是先把下面这条链搭起来：

1. `SLAM / Navigation Adapter`
2. `World State / Structured Semantics`
3. `Task Planner`
4. `Safety Supervisor`
5. `Weak-Bandwidth Remote Interaction`

这样做的好处是：

- 官方底层可以继续负责建图、定位、到点导航和短时局部规划。
- 上层的语义、任务、弱网通信和安全规则可以独立演进。
- 就算后面底层从官方 SLAM 切到 `LIO-SAM` 或混合定位方案，上层协议也不用重写。

## 当前仓库包含什么

- `docs/recovered-baseline.md`
  - 从 `D:\go2_backup` 里恢复出的现状、运行痕迹和缺口分析
- `docs/development-roadmap.md`
  - 推荐的分阶段开发路线
- `docs/slam_gateway_flow_and_protocols.md`
  - SLAM Gateway 分层流程图、接口映射和内部协议说明
- `docs/slam_refactor_outputs_and_lidar_perception.md`
  - 面向 SLAM 应用重构的输出协议、实时反馈和 LiDAR 几何感知设计
- `docs/local_llm_closed_loop_strategy.md`
  - 本地 LLM 闭环策略、工具调用、模型选型和微调路线
- `docs/nx_qwen3_llm_readiness_2026-04-24.md`
  - 直连 NX 上 Qwen3-4B 本地推理环境摸底与调整建议
- `docs/local_llm_finetuning_development_brief.md`
  - 新开本地 LLM 微调开发任务时使用的交接说明和启动 prompt
- `docs/local_llm_sft_offline_finetuning_howto.md`
  - SFT 数据构造、LoRA/QLoRA 离线微调和 GGUF 部署手册
- `docs/competition_llm_sample_library_and_finetune_runbook.md`
  - 面向比赛能力的本地 LLM 样本库说明与微调运行步骤
- `scripts/recover_go2_backup.ps1`
  - 从备份中提取关键日志、配置和现有源码
- `schemas/`
  - 结构化语义与导航子目标的跨语言协议
- `src/edge_autonomy/`
  - 最小可用的边缘自治骨架

## 快速开始

先恢复可用资产：

```powershell
powershell -ExecutionPolicy Bypass -File .\scripts\recover_go2_backup.ps1
```

然后先看这几份文档：

- `docs/recovered-baseline.md`
- `docs/development-roadmap.md`
- `docs/slam_gateway_flow_and_protocols.md`
- `docs/slam_refactor_outputs_and_lidar_perception.md`
- `docs/local_llm_closed_loop_strategy.md`
- `docs/nx_qwen3_llm_readiness_2026-04-24.md`
- `docs/local_llm_finetuning_development_brief.md`
- `docs/local_llm_sft_offline_finetuning_howto.md`
- `docs/competition_llm_sample_library_and_finetune_runbook.md`

最后跑一下当前最小测试，确认安全监督与 SLAM Gateway P0 骨架能工作：

```powershell
$env:PYTHONPATH='src'
python -m unittest discover -s tests
```

## 第一原则

在第一阶段，不把“改官方 SLAM 内核”当主线，而是优先做：

- 明确官方底层输入输出
- 固化结构化世界状态协议
- 建立安全监督与高层任务接口
- 再把真实 ROS2 topic / service 接到适配层
