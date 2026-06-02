---
created: 2026-05-27
updated: 2026-05-27
status: active
type: concept
tags:
  - 机器狗/多模态
  - 机器狗/WorldState
  - 机器狗/LLM
  - 机器狗/研电赛
  - 机器狗/边缘自治
---

# 多模态世界状态与 LLM 分层口径

## 一句话结论

可以把项目理解为：

> 多传感器提供对世界状态的多角度观测，本地 LLM 负责高层语义理解、任务规划建议和人机交互，确定性安全层负责最终执行和兜底。

不建议简单说“LLM 是大脑”。更稳的说法是：

> LLM 类似高层语义中枢，负责理解任务、整合摘要和解释状态；SLAM、SafetyGate、QueueExecutor 和局部感知构成确定性反射与执行系统。

这样既保留“大模型智能”的卖点，也不会被质疑“让 LLM 直接控制机器狗是否安全”。

## 为什么多一个传感器就多一份世界理解

每一种传感器回答的问题不同：

| 来源 | 回答的问题 | 适合输出 |
|---|---|---|
| SLAM / LiDAR | 我在哪里、地图是否可靠、前方是否有障碍 | `CurrentPose`、`SlamHealth`、`LocalObstacleSummary` |
| 双目深度相机 | 近距离空间、视野遮挡、目标附近是否可观察 | `DepthCameraSummary` |
| TI 毫米波雷达 + NX | 弱光、烟雾、遮挡下是否有移动目标或人员候选 | `RadarDetectionSummary` |
| RGB 相机 | 现场可视证据、巡检关键帧 | `PhotoKeyframeSummary` |
| 语音/文本 | 人希望机器人做什么、如何反馈 | `UserIntent`、`operator_feedback` |

这些输入不应以原始大数据形式直接喂给 LLM，而应先压缩成结构化摘要，最后合成统一 `WorldState`。

## 推荐系统表达

```text
多模态传感器
  -> 对世界状态的多角度观测
  -> 统一 WorldState / PerceptionSummary
  -> LLM 理解任务、解释环境、生成候选计划
  -> SafetyGate 校验风险
  -> QueueExecutor 执行
  -> UI / 语音反馈给人
```

核心表达：

> 以多模态感知构建统一世界状态，以本地 LLM 实现语义级任务理解与解释，以确定性安全执行层完成可靠闭环。

## LLM 应该负责什么

LLM 适合负责：

- 理解自然语言任务。
- 把模糊表达映射到候选拓扑点或巡检模板。
- 根据 `WorldState` 生成候选任务队列。
- 把安全层、雷达、双目、SLAM 的结构化状态解释成人能听懂的话。
- 在弱网下生成低频语义反馈，例如“正在去哪里”“为什么暂停”“发现了什么异常”。

LLM 不应该负责：

- 直接输出速度控制。
- 直接调用底层 SLAM 命令。
- 直接越过 SafetyGate。
- 处理原始连续视频、完整点云或雷达 ADC。
- 做实时避障和急停判断。

## 确定性安全层为什么必须独立

比赛答辩时要强调：LLM 是智能交互与语义规划层，不是安全闭环本身。

必须由确定性模块兜底：

| 层 | 作用 |
|---|---|
| `SafetyGate` | 判断是否允许导航、是否需要 hold/slow/block/confirm/replan |
| `QueueExecutor` | 串行执行任务，只允许一个明确 step 进入运动链路 |
| `SLAM Gateway` | 把受控导航命令下发给 Unitree SLAM |
| `PerceptionSummary` | 用低频摘要表达障碍、雷达告警、双目净空 |
| `StateJournal` | 记录当前状态、反馈和可复盘日志 |

这样系统即使 LLM 输出不稳定，也能被规则校验和安全策略拦住。

## 弱网下的意义

弱网场景不是为了证明“有 6G”，而是为了证明机器人可以在通信不稳定时仍然自治：

- 本地继续跑 SLAM、SafetyGate、QueueExecutor。
- UI 不依赖高频视频和完整点云。
- 上行只传语义摘要、关键帧索引、雷达告警、到达事件、安全原因。
- LLM 只做低频解释和任务级规划，不占用实时控制预算。

推荐口径：

> 弱链路下，机器人不再依赖远端连续遥控，而是本地执行任务、安全判断和异常检测；远端只接收语义摘要和关键证据。

## 多模态卖点怎么讲

不要把卖点讲成“传感器越多越强”。更准确的卖点是：

1. **世界状态更完整**：SLAM 知道位置，双目知道近距空间，毫米波知道遮挡/弱光下的运动目标，相机提供可视证据。
2. **语义层更清楚**：LLM 不看原始传感器，而看统一摘要，因此能解释“现在去哪、为什么停、发现了什么”。
3. **安全层更稳**：任何传感器异常都只会影响置信度和策略，不会直接绕过安全链路。
4. **弱网更可用**：远端只看摘要和关键帧，降低带宽压力。
5. **扩展更泛化**：TI 雷达、双目、语音、热成像、气体传感器都可以作为新的 `PerceptionSummary` 插件加入。

## 推荐答辩短句

短版：

> 我们不是让 LLM 直接控制机器狗，而是用多模态传感器构建结构化 WorldState，让本地 LLM 做语义理解和任务解释，最终由确定性安全层执行。

稍长版：

> SLAM、LiDAR、双目、毫米波雷达和相机分别从几何定位、近距空间、遮挡目标和可视证据角度理解环境。系统先把它们压缩成统一的 WorldState，再交给本地 LLM 做任务级理解和自然语言反馈。LLM 只输出候选计划，SafetyGate 和 QueueExecutor 决定能不能执行、怎么执行、何时停止。

## 不建议这样讲

避免这些说法：

```text
LLM 是机器狗大脑，直接控制行动。
传感器原始数据都交给大模型理解。
多接一个传感器就自动更智能。
没有 SLAM 也可以让 LLM 自己判断走多远。
TI 雷达已经完成完整 SAR 成像。
```

应改成：

```text
LLM 是高层语义规划与解释模块。
传感器先变成结构化摘要，再进入 WorldState。
多传感器增强的是状态理解和风险判断。
无 SLAM 时只能降级为短距探测、原地观察或人工确认。
TI 雷达先作为边缘感知节点，输出目标/人员/占用摘要。
```

## 与当前工程的对应关系

当前仓库已经具备或正在推进的对应模块：

| 规划概念 | 当前工程对应 |
|---|---|
| `WorldState` | `slam_runtime_snapshot.py`、`runtime_state.py`、`llm_context.py` |
| `LocalObstacleSummary` | `slam_state.py`、`lidar_geometry.py` |
| 双目摘要 | `DepthCameraSummary`、`perception_fusion.py` |
| 任务队列 | `task_queue.py`、C++ `TaskQueueValidator`、`QueueExecutor` |
| 安全层 | Python `SafetySupervisor`、C++ `SafetyGate` |
| UI 反馈 | `operator_feedback`、`llm_feedback_results`、C++ operator panel |
| 弱网摘要 | `semantic_only` communication policy、weak UI mode |
| 巡检证据 | `capture_keyframe` 语义事件，后续接真实相机 |

下一步应优先把这些能力接到一个稳定 UI 与 state journal 中，而不是让 LLM 直接吃更多原始数据。

## 规划深化口径

外部规划稿中的主线可以吸收为当前项目的系统工程路线：

```text
WorldState 可信建模
  -> TaskQueue / ToolCall 动作协议
  -> SafetyGate 安全裁决
  -> SLAM Gateway / QueueExecutor 执行
  -> 语义通信与任务上报
  -> 运行日志记录
  -> GO2W 训练库沉淀
  -> MiniMind-GO2 或轻量规划模型迭代
```

这条路线的价值在于避免项目变成“用户说一句话，Qwen 输出 JSON，机器狗执行”的套壳 demo。真正要讲的是：模型看到的是可信 WorldState，能输出的是受限 ToolCall，能不能执行由 SafetyGate 和任务状态机决定，结果会进入日志和训练库。

### 术语对齐

| 规划稿术语 | 当前项目术语 |
|---|---|
| `WorldState` | `world_state`、`runtime_snapshot`、`planner_context` |
| `ToolCallSchema` | `TaskQueue IR`、`Action Registry` |
| `SafetyGate` | C++ `SafetyGate`、Python `SafetySupervisor` |
| `SLAMGateway` | `slam_llm_command_client`、`GatewayClient` |
| 任务状态机 | `queue_execution`、`operator_feedback`、后续 `state_journal` |
| 训练库 | `runtime_log_schema`、`failure_case_bank`、后续 JSONL |

### 第一版最关键字段

WorldState v1 不要一开始做太大，先保证这些字段稳定：

```text
localized
map_loaded
current_node
candidate_nodes
front_clearance_m
obstacle_status
detected_objects
network_level
task_phase
last_execution_result
motion_allowed
available_tools
```

多模态摘要统一附加：

```text
source
confidence
stale
latency_ms
timestamp_ms
```

### 赛道选择

推荐主线仍然是“大模型与智能体系统”，通感相关内容作为支撑实验：

- 弱网语义通信。
- 感知辅助通信策略。
- 多模态边缘节点。
- 任务成功率、带宽、时延、关键事件漏报率等可量化指标。

不要主报通感物理层波形、信道建模或完整 SAR 成像；这些方向可以作为后续增强或背景支撑。
