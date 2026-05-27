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
```

## 下一步优先级

P0：
- 给 `task_queue` 增加 JSON schema 和 C++ validator。
- 让 Python planner fallback 输出的队列也通过同一个 validator。
- `QueueExecutor` 写统一 `queue_execution` 事件日志，至少包含 step、preflight、send_result、arrival、blocked_reason。

P1：
- 把 `SafetyGate` 的 policy 输出稳定下来，并给 operator panel 用中文摘要展示。
- 增加 `capture_keyframe` 外部命令配置，保持 dry-run 默认安全。
- 增加 C++ 单元测试：低电量、弱网、已到点、障碍阻断、队列失败停止。

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
