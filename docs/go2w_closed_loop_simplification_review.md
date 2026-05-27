# GO2W 闭环规划精简与泛化评审

日期：2026-05-27

## 当前判断

现有闭环规划方向是正确的：运行时逻辑正在从 Python 原型收敛到 C++ 主链路，LLM 被放在语义不确定时的服务化 fallback，安全规则由确定性代码兜底，operator panel 只做状态展示和输入入口。

当前已具备的主链路：

```text
operator input
  -> SemanticRouter
  -> task_queue / slam_commands
  -> SafetyGate
  -> QueueExecutor
  -> GatewayClient
  -> slam_llm_command_client
```

这个结构比“LLM 直接出底层命令”更稳，也更容易扩展到其他地图、任务类型和 UI。

## 还不够好的地方

1. `task_queue` 还不是唯一中间表示。
   当前 C++ `SemanticRouter` 会生成队列，Python planner 也会生成类似队列，但两边还没有共享 schema 和统一校验器。后续应把 `task_queue` 定为闭环唯一 IR：无论来自确定性匹配、LLM、脚本还是 UI，都先归一到同一队列，再进入 `QueueExecutor`。

2. `SafetyGate` 还需要策略化。
   现在安全门已覆盖 SLAM、定位、低电量、障碍、弱网、已到点等规则。下一步应把输出从 bool/reason 扩展为稳定 policy：`block`、`hold`、`slow`、`semantic_only`、`confirm`、`replan`，并让 executor 根据 policy 决定是否降速、暂停、要求人工确认或重规划。

3. 运行态仍偏短进程。
   多数状态来自短进程调用 `get_world_state`，导航进度和事件日志还没有统一持久化。现场调试会遇到“新进程看不到上一段任务上下文”的问题。更泛化的做法是引入长驻 operator core 或轻量 state journal。

4. LLM fallback 边界还应更窄。
   已知拓扑点、固定动作、巡检队列、拍照点不需要 LLM。LLM 应主要负责模糊目标解析、任务拆解和自然语言回复；所有坐标、速度、安全策略、执行顺序都由 C++/registry/schema 校验。

5. 动作类型需要插件化。
   现在队列重点是 `navigate` 和 `capture_keyframe`。后续应把动作扩展为小型 action registry，例如 `speak`、`set_light`、`capture_keyframe`、`wait_until`、`ask_confirm`，每个 action 都有参数校验、执行器和 dry-run 输出。

## 精简后的推荐闭环

```text
input text / UI command / scripted task
  -> IntentNormalizer
      - deterministic alias match first
      - LLM only when deterministic confidence is low
  -> TaskQueue IR
      - schema validation
      - target registry binding
      - action expansion
  -> SafetyGate policy
      - preflight before each movement step
      - runtime check during arrival wait
      - policy output, not only allow/deny
  -> QueueExecutor
      - execute one step at a time
      - write event journal
      - stop on first blocking failure
  -> OperatorPanel / UI summary
      - show compact reason trace
      - expose dry-run by default
      - render operator_feedback as a screen-like status area
```

## 运行频率与反馈策略

运行时不要把所有刷新绑定到同一个频率。建议固定三层节奏：

- SLAM 轮询：按 `slam_poll_interval_s` 读取 `world_state`，默认 1 Hz。现场 SLAM 频率更高时也不需要 UI 全量刷新；弱网或 CPU 压力大时可以降到 0.5 Hz。
- UI 刷新：按 `ui_refresh_interval_s` 更新屏幕状态，默认 1 Hz。UI 展示应消费最近一次有效状态，而不是强制每帧重新查网关。
- 操作员/LLM 反馈：按 `operator_feedback_interval_s` 和 `llm_feedback_interval_s` 生成自然语言反馈，默认 5s/8s。多点任务在每段出发、进度、到达、阻塞、超时时都要产生 `operator_feedback`，并尽量产出 `llm_feedback_results` 供 UI 的“显示屏区域”直接展示。

`operator_feedback` 是 UI “显示屏区域”的稳定输入契约：字段包含 `phase`、`severity`、`channel=operator_display`、`llm_surface=true`、`text`、`target_node`、`target_name`、可选 `distance_to_target_m`。`llm_feedback_requests` 记录要交给 LLM 的上下文，`llm_feedback_results` 记录已经生成出来的文本；Python 原型支持 `--llm-feedback-mode template|live|off`，默认 template 保证不拖慢 SLAM 轮询，现场需要真 LLM 回馈时切到 live，失败时保留 deterministic fallback。

为减少内部程序对实时链路的影响，执行器遵循三条工程约束：进度和 queued 阶段默认不调用真实 LLM，`live` 模式也只对阻塞/到达/超时等终态调用真实 LLM；`arrival_samples`、`operator_feedback`、`llm_feedback_*` 都有上限，超过后只保留最新窗口并记录 `dropped_counts`；等待循环按本轮耗时扣减 sleep，并记录 `poll_overruns` 与 `max_loop_elapsed_s`，用于判断 UI/日志/LLM 是否挤占了 SLAM 轮询预算。

意外情况的默认处理：

- 连续网关读取失败达到 `gateway_error_limit`：停止等待、阻塞后续队列、请求人工确认。
- 运行中 SafetyGate 变为不允许导航：停止等待并记录阻断原因。
- 到点超时：停止后续 step，反馈当前目标未确认到达。
- dry-run：仍然生成完整 `operator_feedback` 和队列 JSON，但不访问运动下发路径。
- 事件过多：保留最新窗口，摘要层暴露被丢弃数量，避免长任务把内存和 JSON 序列化时间拖大。

## 下一步优先级

P0：
- 给 `task_queue` 增加 JSON schema 和 C++ validator。（已完成初版）
- 让 Python planner fallback 输出的队列也通过同一个 validator。（已完成初版）
- `QueueExecutor` 写统一 `queue_execution` 事件日志，至少包含 step、preflight、send_result、arrival、blocked_reason。

P1：
- 把 `SafetyGate` 的 policy 输出稳定下来，并给 operator panel 用中文摘要展示。
- 增加 `capture_keyframe` 外部命令配置，保持 dry-run 默认安全。
- 增加 C++ 单元测试：低电量、弱网、已到点、障碍阻断、队列失败停止。
- 把 C++ 侧 template feedback renderer 抽象成可替换的 LLM service client；Python 原型已具备 `llm_feedback_results` 和 live/template/off 三种模式。

P2：
- 接入 C++ LLM HTTP client，逐步替代 Python planner fallback。
- 引入长驻 operator core/state journal，减少短进程状态丢失。
- 把 action registry 泛化，支持更多非导航动作而不改 executor 主循环。

## 泛化原则

- 地图只提供语义拓扑、姿态、标签和限制，不承载执行逻辑。
- LLM 只产出意图和候选队列，不拥有最终执行权。
- SafetyGate 是每个运动步骤的强制前置条件。
- QueueExecutor 是唯一会下发运动命令的运行时组件。
- UI 只展示和输入，不直接拼底层 SLAM 命令。

## 比赛展示与多模态扩展方向

后续整体叙事建议从“伪 6G”收敛为“弱链路下的多模态边缘自主机器狗系统”。弱网不再作为单独通信噱头，而是作为机器人自治能力的压力测试：下行传任务和约束，上行传语义摘要、关键事件、关键帧和安全原因，不上传原始高频点云、连续视频或完整日志。

可选扩展方向包括：

- **巡检任务模板**：多点导航、到达回复、拍照关键帧、异常摘要和结束报告。
- **语音输入/播报**：语音转文本后仍进入同一套 `TaskQueue IR` 和 SafetyGate；播报只消费 `operator_feedback` 和 `llm_feedback_results`。
- **拍照/关键帧**：`capture_keyframe` 从语义事件逐步接入真实相机命令；弱网下只传缩略图、路径、时间戳和摘要。
- **双目深度相机**：作为近距离障碍和视野遮挡增强，只输出 `DepthCameraSummary`，不能放宽 LiDAR/SLAM 的安全阻断。
- **NX + TI 毫米波雷达边缘节点**：作为外部感知节点输出 `RadarDetectionSummary`，用于弱光、烟雾、遮挡或疑似人员/移动目标提示。第一阶段以点云、目标跟踪、占用/人员存在摘要为主，不把完整 SAR 成像作为 P0 承诺。

详细路线见 `docs/multimodal_edge_autonomous_robot_plan.md`。这些能力都应保持可插拔：有硬件时增强感知，没有硬件时主闭环仍可用。
