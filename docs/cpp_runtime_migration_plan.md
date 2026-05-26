# GO2W 运行时 C++ 化迁移计划

## 原则

后续开发尽量遵循：

- **机器狗运行时优先 C++**：状态读取、安全检查、语义点匹配、任务队列、导航执行、UI 面板都逐步迁到 C++。
- **Python 保留在离线侧**：训练数据生成、评测、CSV 分析、PCD 标注工具、微调脚本可以继续用 Python。
- **LLM 推理服务化**：不要把模型推理写死在 UI 里。C++ 通过 HTTP/IPC 调用 llama.cpp server 或本地推理服务。
- **安全规则不交给 LLM**：C++ 层必须做目标存在、SLAM 健康、定位状态、低电量、弱网、到点保持等确定性检查。

## 迁移分层

```text
Qt/RViz2 UI 或 C++ TUI
  -> C++ Operator Core
      -> C++ SemanticRouter：中文目标匹配、任务队列、弱网摘要
      -> C++ SafetyGate：SLAM/定位/电量/弱网前置规则
      -> C++ GatewayClient：slam_llm_command_client 或后续直接 SDK
      -> C++ QueueExecutor：导航、等待到点、暂停、拍照事件
      -> LLM Service：llama.cpp/OpenAI-compatible HTTP，仅在语义无法确定时调用
```

## 已迁移

- `cpp/go2w_operator_panel`
  - 世界状态显示
  - 弱网摘要模式
  - LLM/自然语言输入口
  - 默认干跑，显式 `/execute on` 才真实执行
- `cpp/SemanticRouter`
  - 直接读取 `go2w_real_site_map_registry.json`
  - UTF-8 中文别名匹配
  - 多目标顺序队列
  - `photo_required` 和“拍照/看一眼”语义识别
  - 生成 `navigate_to_pose` 命令
- `cpp` 执行路径
  - 拓扑命令匹配成功时不再必须调用 Python
  - C++ 前置检查 `worldAllowsNavigation`
  - C++ 到点监控和自动 `pause_navigation`
  - 未匹配到拓扑点时才 fallback 到 Python/LLM

## 暂时保留 Python 的部分

- `scripts/generate_*`：训练数据和评测数据生成。
- `scripts/run_nx_planner_eval.py`、`export_planner_eval_csv.py`：离线评估。
- `scripts/apply_pcd_annotations.py`、`make_pcd_annotation_page.py`：PCD 标注工作流。
- `src/edge_autonomy/local_llm_planner.py`：作为参考实现和 fallback，等 C++ LLM service 接入后逐步退役。

## 下一步迁移优先级

1. `GatewayClient` 正式 C++ 封装：替换当前通过 shell 调 `slam_llm_command_client` 的方式。
2. `QueueExecutor` 完整化：拍照命令、语音反馈、队列事件日志、失败暂停。
3. `SafetyGate` 完整化：低电量、弱网、已在目标附近、目标不存在、门关闭/人群风险。
4. Qt Widget/RViz2 Panel：把当前 TUI 的状态和输入搬到图形界面。
5. LLM HTTP client：C++ 调本地 llama.cpp server，替代 Python planner fallback。

## 不建议迁移的内容

离线数据、评测和微调不需要强行 C++ 化。它们不在机器人运行闭环里，用 Python 更快、更容易检查，也不会影响现场稳定性。
