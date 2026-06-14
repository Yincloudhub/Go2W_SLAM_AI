# 多模态边缘自主机器狗系统规划

日期：2026-05-27
更新：2026-06-13

## 2026-06-12 比赛架构收束

后续实现以
[`go2w_competition_edge_autonomy_handoff_20260612.md`](go2w_competition_edge_autonomy_handoff_20260612.md)
为当前基线。本节优先于本文后续较早的 `SafetyGate`、双侧车和多执行路径描述。

核心调整：

- 比赛唯一真实执行链为 `TaskQueue -> MissionDecisionEngine -> SLAM Gateway -> Unitree SDK`。
- Gateway 是最终运动权威；上层安全检查用于预检和解释，不再叠加多个相互独立的权威。
- 所有传感器先转成带时间戳、新鲜度、置信度和标定状态的统一语义，再进入 `WorldState / PerceptionContext`。
- D435 深度摘要与 DeepYOLO 改为单一 RGBD 采集进程下的两个处理器，禁止两个侧车同时争抢相机。
- 弱网必须由实际通信策略执行器控制上传、缓存和补传，本地自治闭环不依赖网络。
- LLM 只做歧义理解、任务拆解、重规划和解释，不直接控制底层运动。
- 当前先不训练或 LoRA；先完成上下文、执行协议、约束输出、日志和评测闭环。

### 2026-06-13 P0-1 完成状态

统一 D435 感知服务已完成代码和真机无运动验收：

- 单一 `D435CaptureOwner` 持有 RealSense RGBD。
- 深度 ROI 与 YOLO 各自保留来源 frameset 的 frame sequence、sensor timestamp
  和 capture time；两条异步输出不要求最新序号相等。
- 深度独立刷新，不等待 YOLO 推理。
- `go2w_stereo_depth_sidecar.sh` 与 `go2w_deepyolo_sidecar.sh` 只转发到
  `go2w_d435_perception_sidecar.sh`。
- 主摘要与两个兼容摘要共享 `generation_id`。
- YOLO 缺失时只降级 `d435_yolo`，深度 freshness 保持独立。
- 真机 Python 回归 `261/261` 通过，服务验收后已停止。

P0-1 完成标记为 `p0-1-unified-d435-accepted-20260613`。下一阶段唯一任务是
`SensorEnvelope / PerceptionContext v1`，不得重新引入第二个 D435 owner。

### 2026-06-13 P0-2 schema/loaders 完成状态

P0-2 首个独立提交已建立统一感知输入契约：

- 所有来源先归一化为 `SensorEnvelope v1`，状态只允许
  `fresh/stale/offline/invalid/uncalibrated`。
- XT16、D435 depth、D435 YOLO、TI/NX 使用同一 Python loader 模块。
- 未来 XT16 IMU 和 Unitree 里程计先以明确 `offline` 的预留 envelope 出现，
  不伪造数据。
- 文件存在不能证明服务在线；必须结合 PID/bridge evidence、timestamp、
  stale budget 和 sequence。
- D435 depth 与 YOLO 保持独立 freshness，但共享单一 capture owner。
- XT16 是唯一主四向几何源；D435 depth 只能进入前向补充，不能在 XT16
  失效时自动晋升。
- LLM 输入只允许有界语义摘要，不包含原始点云、连续视频或雷达 ADC。
- `PerceptionContext` 固定 Gateway 为最终运动权威，且不新增第二套执行链。

本提交不修改 WorldState、LLM、UI 或 Gateway。下一独立提交才把 Python/C++
WorldState 的宽松 artifact normalization 替换为统一上下文，尤其禁止把缺失
timestamp 补成当前时间。

### 2026-06-13 P0-2 WorldState integration

Python 和 C++ WorldState reducers 已改为只接受完整
`PerceptionContext v1`：

- context 自身使用 `generated_at_ms + stale_ms` 失效，旧 context artifact
  不能继续提供 fresh 来源。
- 删除自由格式 `perception_summaries` 输入和缺失 timestamp 补当前时间逻辑。
- `WorldState.perception_summaries` 仅保留为 `context.sources` 的兼容投影。
- `detected_objects` 只从同一 context 的 `visual_objects` 投影。
- stale、畸形或违反 Gateway/LLM policy 的 context 整体 fail closed。

### 2026-06-13 P0-2 unified runtime consumers

- 单一 producer 原子写入 `artifacts/perception_context_v1.json`。
- Python Planner、WorldState 和 runtime log 复用同一已校验 context 对象。
- C++ OperatorPanel/LLM 只加载该 artifact，不再读取三份旧感知文件。
- Web UI 一次响应内从同一 `context_id` 投影兼容面板。
- context producer 不持有传感器、不启动 SLAM/Gateway、不控制底盘。
- artifact 缺失、畸形或 stale 时所有消费者 fail closed。

机器人已 fast-forward 到实现提交 `d8e9125` 并完成无运动验收：

- robot Python targeted `81/81`，full unittest `286/286`。
- C++ 全量构建通过，CTest `6/6`。
- loader、Planner、WorldState、runtime log、Web UI 的 `context_id` 一致。
- D435/context 服务停止，无 SLAM、Gateway 或底盘运动。

P0-2 完成标记为
`p0-2-perception-context-v1-accepted-20260613`。下一阶段唯一任务是 P0-3
决定与执行路径收口。

### 2026-06-13 P0-3 完成状态

- 任意最终 Planner 计划都先归一化为 `TaskQueue IR`；单目标、保持和人工确认
  不再绕过队列。
- 新增 `MissionDecision v1` schema 和确定性决定层。
- Python 闭环删除了单条 `slam_command` 直达 Gateway 的执行兜底。
- 真实导航只由 Python persistent supervised executor 承接。
- C++ 语义路由固定为 dry-run/诊断入口，真实导航转交 Python。
- Gateway 拒绝中的 safety reason、模式、时间和传感器 age 进入统一
  DecisionRecord。
- 断定位、地图错配、传感器过期和 Gateway 断网均已增加 fail-closed 测试。
- UI 和 runtime log 消费同一份 MissionDecision。

本地全量 Python `298/298` 通过。机器人 fast-forward 到 `cf68b0e` 后，
Python targeted `47/47`、full `298/298`、C++ build 和 CTest `6/6` 均通过；
已知拓扑点 `yin_siyuan_station` dry-run 输出 `dry_run_queue`，
`motion_allowed=false`。未启动 SLAM、Gateway 或底盘运动，三端提交一致且机器人
工作区干净。

P0-3 完成标记为
`p0-3-mission-decision-chain-accepted-20260613`。下一步唯一软件任务是 P0-4
弱网 journal、ack sequence 和重连补传执行器。

## 定位

当前项目不建议继续用“伪 6G”作为主表述。更稳的定位是：

> 面向弱链路和复杂室内场景的多模态边缘自主机器狗系统。

这条路线保留最初的弱网闭环想法，但把重点从“模拟某种通信制式”转成“弱链路下仍能完成本地自治、语义回传和安全巡检”。系统可以逐步接入本体 SLAM/LiDAR、本地 LLM、语音、拍照、双目深度相机、外接 NX + TI 毫米波雷达边缘节点，但每个模块都应是可插拔增强项，而不是主闭环的强依赖。

赛道选择上，建议主线放在“大模型与智能体系统”，弱网语义通信、感知辅助通信和多模态边缘节点作为支撑实验，而不是主报“通感一体物理层”。这样更贴合当前已有工程资产：GO2W、SLAM Gateway、MapRegistry、TaskQueue、SafetyGate、本地 Qwen3-4B、弱网摘要、运行日志和后续训练库。

推荐题目口径：

> 基于语义通信与边缘大模型的四足机器人智能体系统。

或：

> 面向弱链路场景的多模态边缘自主机器狗智能体系统。

## 最终呈现效果

建议最终演示收敛成一条主线：

```text
一句自然语言巡检任务
  -> 本地 LLM/确定性路由解析成任务队列
  -> SafetyGate 检查 SLAM、定位、障碍、弱网和传感器状态
  -> QueueExecutor 执行多点巡检
  -> UI 显示当前去哪、到哪了、为什么停、是否需要人工确认
  -> 语音/拍照/雷达/双目作为巡检证据和异常感知增强
  -> 弱网模式下只上传语义摘要和关键帧，不上传原始高频点云/视频
```

面向比赛展示时，不需要同时演示所有传感器。建议现场主演示只展示：

1. 自然语言下发巡检任务。
2. SLAM 有效时执行多点导航。
3. UI 实时显示任务队列、到达回复、安全原因。
4. 弱网模式下保留语义摘要，丢弃原始大流量数据。
5. 至少一个异常感知来源触发提醒，可以先用模拟雷达摘要，后续替换为真实 TI 雷达节点。

双目、TI 雷达、语音、拍照、SAR/搜救能力可以放在“扩展能力/后续接入”里，不需要都压在同一次演示中。

## 系统分层

```text
Operator UI / Voice Input
  -> C++ Operator Core / State Journal
      -> IntentNormalizer
          - deterministic alias match first
          - LLM service only for ambiguous commands
      -> TaskQueue IR
      -> SafetyGate policy
      -> QueueExecutor
      -> Action Registry
          - navigate
          - wait_until
          - capture_keyframe
          - speak
          - inspect_area
          - ask_confirm
      -> Perception Summary Bus
          - GO2W LiDAR / SLAM status
          - StereoDepthSummary
          - RadarDetectionSummary
          - PhotoKeyframeSummary
```

原则：

- 机器狗主闭环仍以 C++、SafetyGate、QueueExecutor 为核心。
- LLM 只负责意图解析、任务拆解和自然语言解释，不直接控制底层运动。
- 外部传感器只向主链路提供低频摘要，不阻塞 SLAM 轮询和到点等待。
- UI 只显示和输入，不直接拼底层运动命令。
- 弱网下优先保留任务状态、安全原因、异常摘要和关键帧索引。

## 深化主线

外部规划中的 `WorldState -> ToolCall -> SafetyGate -> 执行 -> 日志 -> 训练库` 可以作为后续深化主轴，但需要和当前代码术语对齐：

| 规划术语 | 当前工程术语 | 说明 |
|---|---|---|
| `WorldState` | `world_state` / `runtime_snapshot` / `planner_context` | 模型和 UI 看到的结构化状态 |
| `ToolCallSchema` | `TaskQueue IR` / `Action Registry` | LLM 或规则只能输出受限动作 |
| `SafetyGate` | C++ `SafetyGate` / Python `SafetySupervisor` | 所有运动前必须校验 |
| `SLAMGateway` | `slam_llm_command_client` / `GatewayClient` | 受控下发 Unitree SLAM 命令 |
| 任务状态机 | `queue_execution` / `operator_feedback` / 后续 `state_journal` | 记录任务处于规划、执行、到达、阻断还是完成 |
| 训练库 | `runtime_log_schema` / `failure_case_bank` / 后续 JSONL | 为后续 MiniMind-GO2 或轻量规划模型提供数据 |

### WorldState v1

第一版不要追求大而全，建议先固定 12 个关键字段：

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

多模态扩展必须给每个感知摘要加上：

```text
source
confidence
stale
latency_ms
timestamp_ms
```

这样 TI 雷达、双目、相机关键帧和 LiDAR 摘要可以统一进入 `WorldState`，而不会让 LLM 直接处理原始点云、连续视频或雷达 ADC。

### TaskQueue / ToolCall v1

后续所有 LLM、规则和 UI 生成的动作都应先进入统一队列，不直接下发底层命令。建议第一版动作集合：

```text
mapped_navigation / navigate
safe_hold
ask_human_confirm
request_relocalization
capture_keyframe
semantic_report
return_to_base
patrol_route
cancel_task
speak
inspect_area
```

与当前 C++ 实现对齐时，`mapped_navigation` 应落到 `navigate` step，`capture_keyframe` 先保持 dry-run/语义事件，后续再接真实相机命令。

### SafetyGate policy v1

SafetyGate 输出应从简单 allow/deny 逐步稳定为：

```text
allow
hold
slow
block
semantic_only
confirm
replan
```

必须拦截的情况：

- `localized=false` 时禁止 mapped navigation。
- `map_loaded=false` 时禁止 mapped navigation。
- `front_clearance_m` 低于阈值时禁止移动。
- `target_node` 不存在时转人工确认。
- 已经在目标附近时禁止重复导航。
- 弱网下请求 raw video 或 dense pointcloud 时改为 semantic report。
- 工具不在 `available_tools` 中时拒绝执行。

### 任务状态机 v1

任务不是一次模型输出，而是完整生命周期。建议状态：

```text
idle
planning
waiting_safety_check
executing_navigation
arrived
executing_after_arrival_action
reporting
completed
failed
blocked
not_localized
target_unknown
human_confirm_required
cancelled
```

这些状态应进入 `state_journal` 和 UI 反馈区域，让操作员能看到“现在去哪、到哪了、为什么停、是否需要确认”。

## 可选模块

### 1. 弱网语义闭环

目标不是证明某种真实 6G 网络，而是证明弱链路下机器人仍然能自治：

- 下行：传任务、目标和约束，不传连续遥控量。
- 上行：传 `operator_feedback`、`llm_feedback_results`、安全状态、雷达/双目摘要、关键帧索引。
- 限制或丢弃：raw video、dense pointcloud、full log、high-rate images。
- UI 展示：当前链路模式、保留了哪些语义数据、丢弃了哪些大流量数据、任务是否继续。

建议做一个演示开关：

```text
/weak on
```

打开后，UI 进入弱网摘要模式，LLM 只做低频或终态解释，主闭环继续由确定性状态驱动。

建议把弱网做成可量化实验，而不是概念展示：

| 模式 | 上传内容 | 作用 |
|---|---|---|
| 全量视频模式 | 持续视频流 | 传统基线 |
| 关键帧+语义模式 | 目标、位置、风险、关键帧 | 语义通信方案 |
| 纯语义模式 | 结构化状态、任务日志 | 极低带宽方案 |
| 本地智能体闭环 | 仅上报任务结果和异常事件 | 边缘智能方案 |

评估指标：

```text
平均上传带宽
单次任务上传数据量
端到端响应时延
弱网任务成功率
关键事件漏报率
操作员可理解性
人工接管次数
```

### 2. TI 毫米波雷达 + NX 边缘节点

建议把外接 NX + TI 雷达作为独立边缘感知节点，而不是直接绑死在机器狗主进程里。

```text
TI mmWave Radar
  -> NX Radar Edge Node
      -> point cloud / target tracking / occupancy / vital sign candidate
      -> RadarDetectionSummary
  -> GO2W Operator Core
      -> UI / LLM explanation
      -> calibrated SafetyGate adapter (later phase)
```

第一阶段不建议把目标定成完整合成孔径雷达成像。更稳的方向是“复杂环境搜救/巡检感知增强”：

- 弱光或烟雾下检测运动目标。
- 遮挡或光照不佳时提供人员存在候选。
- 输出目标距离、方位、速度、置信度。
- 在 UI 中作为“雷达告警区域”显示。
- 完成标定和静态验证后，再通过独立适配器在 SafetyGate 中触发 `slow`、`confirm` 或 `inspect_area`。

推荐摘要格式：

```json
{
  "source": "ti_mmwave_edge_01",
  "timestamp_ms": 0,
  "mode": "people_tracking",
  "detections": [
    {
      "range_m": 8.2,
      "azimuth_deg": -12.5,
      "doppler_mps": 0.3,
      "confidence": 0.76,
      "label": "moving_person_candidate"
    }
  ],
  "alert": "possible_person_ahead",
  "latency_ms": 80,
  "stale": false
}
```

硬件参考方向：

- TI IWR6843ISK 是 60GHz 毫米波传感器评估套件，适合先做点云/目标检测原型。
- TI mmWave SDK 可作为点云和 demo 配置的起点。
- Jetson Orin NX 可作为雷达边缘节点，承载 parser、跟踪、轻量视觉或语义融合。

这些硬件只作为可选方向，主链路应允许雷达节点缺席时继续运行。

截至 2026-06-02，仓库已预留 `artifacts/edge_perception_summary.json` 最新值接口，契约见
`docs/edge_perception_node_contract.md`。默认模式为 `semantic_only`：摘要可以进入 UI 和 LLM 上下文，
但 `safety_wired=false`，不会授权运动，也不会绕过 XT16、双目深度或 SLAM 的阻断。外部摘要限制在
256 KiB 内，`observations` 和 `events` 各最多接收 32 条，避免 NX 侧负载影响机器人主链路。

### 3. 双目深度相机

双目深度相机作为近距离障碍和空间理解增强，接入方式沿用 `DepthCameraSummary`：

- 单一 `D435CaptureOwner` 持有相机，深度 ROI 与 YOLO 共用 RGBD capture。
- 主链路只消费低频 ROI 净空摘要。
- 双目只能让安全判断更保守，不能放宽 LiDAR/SLAM 的阻断。
- 相机 stale、低置信度或超时后自动忽略。
- YOLO 变慢、缺失或失败不能阻塞深度摘要刷新。

推荐用途：

- 前方近距离障碍补充。
- 巡检目标附近的细节观察。
- 拍照前确认视野是否被遮挡。
- 无 SLAM 降级模式下的短距探测辅助。

### 4. 语音输入与语音反馈

语音不是运动控制入口，而是 operator input 的一种来源：

```text
microphone / ASR
  -> text command
  -> IntentNormalizer / LLM
  -> TaskQueue IR
```

建议 P0 先支持文本，P1 再接语音：

- ASR 输出必须经过同一套任务队列校验。
- 语音反馈只读 `operator_feedback` 和 `llm_feedback_results`。
- 弱网下可本地播报，不依赖远端 UI。
- 关键安全状态要短句播报，例如“定位未就绪，等待确认”“已到达目标点”“检测到疑似移动目标”。

### 5. 拍照与关键帧

`capture_keyframe` 应从语义事件逐步变成可配置动作：

- P0：dry-run 或记录语义关键帧事件。
- P1：接入本地相机命令，保存图片路径和缩略图。
- P2：弱网下只上传缩略图、图片哈希、时间戳和目标点，不上传连续视频。

推荐关键帧摘要：

```json
{
  "source": "front_camera",
  "timestamp_ms": 0,
  "target_node": "office_front",
  "image_path": "artifacts/keyframes/office_front_001.jpg",
  "thumbnail_path": "artifacts/keyframes/office_front_001_thumb.jpg",
  "quality": "ok",
  "caption": "office doorway keyframe"
}
```

### 6. 巡检功能

巡检是最适合比赛展示的外层任务。建议把巡检定义成任务模板，而不是一次性脚本：

```json
{
  "mission_type": "inspection",
  "targets": [
    {"node_id": "point_a", "actions": ["navigate", "capture_keyframe"]},
    {"node_id": "point_b", "actions": ["navigate", "inspect_area", "speak"]}
  ],
  "communication_policy": "semantic_only_when_weak",
  "on_anomaly": "slow_and_confirm"
}
```

巡检需要的最小闭环：

- 多点任务队列。
- 到达判定。
- 每个点的状态回复。
- 拍照或关键帧记录。
- 异常摘要接入：雷达/双目/LiDAR 任选其一即可。
- 结束报告：到达了哪些点、哪里失败、是否发现异常、性能是否超预算。

## UI 呈现建议

UI 不需要做成复杂控制台，第一版只需要五个稳定区域：

1. **任务队列**：计划去哪、当前执行到哪一步。
2. **世界状态**：SLAM、定位、LiDAR、双目、雷达、弱网状态。
3. **安全策略**：allow/hold/slow/block/confirm/replan 和原因。
4. **反馈显示屏**：`operator_feedback` + `llm_feedback_results`，用自然语言显示当前状态。
5. **巡检证据**：拍照关键帧、雷达告警、双目摘要、结束报告。

## 分阶段路线

### 2026-06-01 工程基线与近期推进

当前主链路已经达到“够用可演示”的工程基线，不再继续追求传感器侧车的极致资源压缩。已验证的闭环边界是：

```text
XT16 LiDAR
  -> Unitree SLAM / relocation
  -> C++ GatewayClient / OperatorPanel
  -> SemanticRouter / TaskQueue validator
  -> SafetyGate / QueueExecutor
  -> Web UI / LLM 输入口
```

走廊导航使用统一 `corridor_clearance_v1`：前向 `<0.80 m` 暂停，左右仅在距
名义机身边缘 `<0.20 m` 时暂停；左右 `0.20-0.60 m` 允许继续使用 Unitree
避障规划，但 Gateway 将真实导航速度限制为 `0.20 m/s`。该规则用于避免侧墙
低于 `0.8 m` 时误拦可通行走廊，不放宽 stale、未标定或低置信度阻断。

2026-06-14 XT16 成对静止场景确认左右名义半宽为 `0.30 m` 时不应再加横向
遮罩余量：右侧紧邻设备箱时 `0.30-0.35 m` 带持续有点，人工前移约 `0.5 m`
后 12/12 帧完全消失。当前只保留前后 `0.05 m` 自回波余量，左右余量为
`0.00 m`；该结论仍属于静态标定证据，不授权真实运动。

趴卧静止调试状态下，当前可作为基线认定：

- SLAM 与 XT16 可一键检查/启动，重定位后 UI 可读到 `loc=true`、`map=true`、`motion=false`、`safety=ok`。
- 中文 Web UI 已能显示机器狗回复、当前位置、视觉理解、安全策略和 LLM 任务输入。
- DeepYOLO / D435I 已收敛为统一 D435 服务：深度目标 `5-10 Hz`，YOLO
  约 `3 Hz`；视觉 stale 或设备离线时 UI 明确降级，主闭环继续按
  LiDAR + SLAM 策略运行。
- 资源优化以不影响实时主链路为边界：Web 与桥接器开销接近零，DeepYOLO 只在相机可用时按 resident 档常驻；`unitree_slam` 仍是最大 CPU 项，暂不在比赛前改厂商参数。
- 当前不把双目、TI 雷达、语音、拍照全部压进同一次演示，而是作为可插拔能力逐项接入。

接下来推进顺序建议：

1. **录点与站立复核**：机器人站起后先启动 SLAM / XT16，执行重定位，确认 `loc=true`、`safety=ok` 后录制真实拓扑点；`陈嘉瑜工位` 当前是趴卧标准位姿，真实导航前仍保留 standing verification。
2. **巡检任务模板**：先做多点任务队列、到点反馈、失败停止和结束报告；拍照可先记录 `capture_keyframe` 事件，再接真实相机命令。
3. **状态持久化**：补 `state_journal` 或长驻 operator core，保存任务队列、到达事件、SafetyGate 决策和人工确认记录，减少短进程状态丢失。
4. **LLM 使用边界**：确定性拓扑匹配优先，LLM 只处理模糊目标、任务拆解和自然语言解释；不微调模型，直到日志样本达到 500-1000 条。
5. **统一上下文**：把 D435、XT16 和 TI/NX 摘要纳入
   `SensorEnvelope / PerceptionContext v1`；不要重新引入独立相机 owner。
6. **弱网实验**：把 `/weak on` 做成可量化实验，比较全量视频、关键帧+语义、纯语义、本地智能体闭环四种模式的带宽、时延和任务成功率。
7. **TI 雷达 / NX**：只发布 `RadarDetectionSummary`，作为巡检异常告警和 `slow/confirm/inspect_area` 的输入，不把原始 ADC 或高频点云送入 LLM。

### P0：比赛主线最小闭环

- 一键启动 SLAM、LiDAR、operator panel。
- 文本输入巡检任务。
- 确定性路由优先，LLM 只处理模糊任务。
- 多点任务队列 dry-run 和实机低速验证。
- UI 显示任务队列、到达回复、安全原因。
- 弱网模式显示 semantic-only 策略。
- 拍照先以 `capture_keyframe` 事件记录。
- 固定 `WorldState v1`、`TaskQueue/ToolCall v1`、`SafetyGate policy v1`、运行日志 schema 和评估指标。
- 每次任务记录用户指令、WorldState、候选节点、LLM/规则输出、SafetyGate 结果、执行结果和人工修正。

### P1：可见能力增强

- `state_journal` 或长驻 operator core，减少短进程状态丢失。
- `SafetyGate` policy 化：`block/hold/slow/semantic_only/confirm/replan`。
- `capture_keyframe` 接真实相机命令。
- 增加 `speak` 动作，播报到达、阻断、异常。
- 引入模拟 `RadarDetectionSummary`，UI 展示雷达告警。
- 双目深度摘要接入 SafetyGate。
- 增加弱网实验脚本或演示开关，比较全量视频、关键帧+语义、纯语义和本地智能体闭环。
- 建立失败样本库：JSON 格式错误、目标匹配错误、安全拦截、执行失败、人工修正。

### P2：多模态边缘节点

- NX 上运行 TI 雷达 parser，发布 `RadarDetectionSummary`。
- 雷达告警先参与巡检任务；完成标定与故障测试后再接入 SafetyGate。
- LLM 常驻服务化，C++/Python 通过 HTTP 调用。
- 弱网模式压测：延迟、丢包、带宽限制下任务是否继续。
- 生成巡检结束报告。
- 当高质量日志达到 500-1000 条后，再评估 MiniMind-GO2 或其他轻量规划模型。

### P3：研究扩展

- 评估毫米波雷达更高级的成像或 SAR-like 能力。
- 评估本地模型微调，让小模型稳定输出任务队列 JSON。
- 接入更多边缘节点，例如热成像、气体传感器、固定摄像头。
- Qt/RViz2 图形化面板。

## 近期交付物

建议优先形成这些文件或模块：

```text
world_state_schema_v1.json
task_queue_schema_v1.json / tool_schema_v1.json
safety_gate_rules.md
task_state_machine.md
runtime_log_schema.json
eval_metric_plan.md
semantic_upload_policy.md
failure_case_bank.jsonl
```

其中已有基础的部分应优先复用当前代码：`task_queue.py`、C++ `task_queue_validator.cpp`、C++ `SafetyGate`、`execution_report.py`、`llm_feedback_results` 和 operator panel。

### 2026-05-27 工程推进记录

本轮先把 P0 闭环需要的“低频统一状态层”落地，不继续堆传感器功能：

- 已新增 `schemas/world_state_schema_v1.json`，作为 UI、LLM、SafetyGate 共享的低频世界状态契约。
- 已新增 `schemas/operator_display_state.schema.json`，用于 operator panel 的任务显示屏区域。
- 已新增 `schemas/runtime_log_schema.json`，约束后续 JSONL 运行日志，不记录连续视频、完整点云或原始雷达 ADC。
- 已新增 `src/edge_autonomy/world_state_v1.py`，把 SLAM/gateway 快照与 planner context 聚合成 `WorldState v1`。
- 已新增 `src/edge_autonomy/operator_display.py`，从 world state、task queue、queue execution 中抽取 UI 可直接显示的任务反馈。
- 已新增 `src/edge_autonomy/runtime_log.py`，形成后续评测、复盘和 MiniMind-GO2 数据积累的单条日志记录。
- 已新增 `scripts/go2w_startup_supervisor.py`，默认 dry-run，显式 `--run` 后只启动/检查 SLAM、雷达 driver、gateway 世界状态探针，不发送运动命令。

下一步应把 operator panel 内部从“直接打印 gateway world_state”升级为读取 `WorldState v1 + OperatorDisplayState`。当前 `scripts/run_go2w_operator_ui.sh` 已可通过 `scripts/start_go2w_runtime_stack.sh` 调用 startup supervisor，后续再把同一入口接到 Qt/RViz2 UI 启动按钮。

## 风险边界

- 不把完整 6G/SAR 成像作为 P0 承诺。
- 不让 LLM 直接输出速度控制或底层 SLAM 命令。
- 不在没有 SLAM、没有局部感知、没有人工确认的情况下执行长距离开环运动。
- 不把原始雷达 ADC、完整点云或连续视频放进 LLM。
- 不让外接传感器阻塞机器人主闭环。

## 对外表述

推荐表述：

> 本项目面向弱链路和复杂室内巡检场景，构建多模态边缘自主机器狗系统。系统以 GO2W 本体 SLAM 和 C++ 安全执行链路为核心，结合本地 LLM 任务理解、语义摘要回传、可选双目深度和 TI 毫米波雷达边缘节点，实现弱网下可解释、可降级、可扩展的自主巡检闭环。

避免表述：

- “我们实现了 6G。”
- “LLM 直接控制机器狗运动。”
- “没有 SLAM 也能任意远距离自主行走。”
- “TI 雷达已经完成 SAR 成像。”

## 参考资料

- TI IWR6843ISK：<https://www.ti.com/tool/IWR6843ISK>
- TI mmWave SDK：<https://www.ti.com/tool/MMWAVE-SDK>
- NVIDIA Jetson Orin：<https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/jetson-orin>
- NVIDIA Jetson Orin NX 16GB：<https://developer.nvidia.com/blog/boost-edge-ai-performance-with-the-new-nvidia-jetson-orin-nx-16gb/>
