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
- `cpp/GatewayClient`
  - C++ 通过 fork/pipe 直接连接 `slam_llm_command_client`
  - 状态读取、导航下发和暂停不再通过 shell 临时文件
  - 保留超时控制和 stderr/stdout 捕获
- `cpp/SafetyGate`
  - 从 `SemanticRouter` 中拆出独立安全门
  - 覆盖 SLAM/定位、无效 pose、低电量、弱网、障碍、风险事件和已在目标附近保持
- `cpp/QueueExecutor`
  - 从 operator panel 中拆出队列执行器
  - 每个导航 step 前重新读取 world_state 并过 SafetyGate
  - 生成 `queue_execution` 事件日志，失败时停止后续 step
  - 区分 SLAM 轮询、UI 刷新、操作员反馈和 LLM 反馈频率
  - 每段导航输出 `operator_feedback`，供 UI 显示“正在去哪/到了哪里/为何阻塞”
  - 输出 `llm_feedback_requests` 和 template `llm_feedback_results`，后续可替换为 C++ LLM HTTP service
- `task_queue` IR
  - 新增 `schemas/task_queue.schema.json`
  - C++ `TaskQueueValidator` 在执行前校验队列结构
  - Python planner / closed-loop fallback 在生成和执行队列前使用同一字段约定

## 现场验证

机器人端已在 `~/go2w_slam_agent/cpp` 编译通过：

```bash
cmake -S . -B build
cmake --build build -j2
```

C++ 路由干跑验证：

```bash
printf 'yin_siyuan_station\n/quit\n' | ./build/go2w_operator_panel --repo-root ~/go2w_slam_agent --current-node initial_point
```

输出显示 `C++语义路由：C++ topology route`，并生成 `cpp_queue`。

注意：不要从 Windows PowerShell 管道直接传中文到机器人端 C++ 面板，中文可能在 PowerShell 源文本阶段变成 `????`。现场应在 MobaXterm/Linux 终端里直接输入中文，或继续用 base64 入口。

## 暂时保留 Python 的部分

- `scripts/generate_*`：训练数据和评测数据生成。
- `scripts/run_nx_planner_eval.py`、`export_planner_eval_csv.py`：离线评估。
- `scripts/apply_pcd_annotations.py`、`make_pcd_annotation_page.py`：PCD 标注工作流。
- `src/edge_autonomy/local_llm_planner.py`：作为参考实现和 fallback，等 C++ LLM service 接入后逐步退役。

## 下一步迁移优先级

1. `QueueExecutor` 完整化：拍照命令配置、语音反馈、事件日志落盘、失败暂停、LLM feedback renderer service 化。
2. `SafetyGate` 策略化：把 allow/deny 扩展为 block/hold/slow/semantic_only/confirm/replan。
3. `task_queue` schema 严格化：从迁移期兼容 `step_id` 收敛到统一 `task_id`。
4. Qt Widget/RViz2 Panel：把当前 TUI 的状态和输入搬到图形界面。
5. LLM HTTP client：C++ 调本地 llama.cpp server，替代 Python planner fallback。

详细评审见 `docs/go2w_closed_loop_simplification_review.md`。

## 不建议迁移的内容

离线数据、评测和微调不需要强行 C++ 化。它们不在机器人运行闭环里，用 Python 更快、更容易检查，也不会影响现场稳定性。
