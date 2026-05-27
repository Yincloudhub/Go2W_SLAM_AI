# 多模态边缘自主机器狗系统规划

日期：2026-05-27

## 定位

当前项目不建议继续用“伪 6G”作为主表述。更稳的定位是：

> 面向弱链路和复杂室内场景的多模态边缘自主机器狗系统。

这条路线保留最初的弱网闭环想法，但把重点从“模拟某种通信制式”转成“弱链路下仍能完成本地自治、语义回传和安全巡检”。系统可以逐步接入本体 SLAM/LiDAR、本地 LLM、语音、拍照、双目深度相机、外接 NX + TI 毫米波雷达边缘节点，但每个模块都应是可插拔增强项，而不是主闭环的强依赖。

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

### 2. TI 毫米波雷达 + NX 边缘节点

建议把外接 NX + TI 雷达作为独立边缘感知节点，而不是直接绑死在机器狗主进程里。

```text
TI mmWave Radar
  -> NX Radar Edge Node
      -> point cloud / target tracking / occupancy / vital sign candidate
      -> RadarDetectionSummary
  -> GO2W Operator Core
      -> SafetyGate / UI / LLM explanation
```

第一阶段不建议把目标定成完整合成孔径雷达成像。更稳的方向是“复杂环境搜救/巡检感知增强”：

- 弱光或烟雾下检测运动目标。
- 遮挡或光照不佳时提供人员存在候选。
- 输出目标距离、方位、速度、置信度。
- 在 UI 中作为“雷达告警区域”显示。
- 在 SafetyGate 中触发 `slow`、`confirm` 或 `inspect_area`。

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

### 3. 双目深度相机

双目深度相机作为近距离障碍和空间理解增强，接入方式沿用 `DepthCameraSummary`：

- 相机进程独立运行。
- 主链路只消费低频 ROI 净空摘要。
- 双目只能让安全判断更保守，不能放宽 LiDAR/SLAM 的阻断。
- 相机 stale、低置信度或超时后自动忽略。

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

### P0：比赛主线最小闭环

- 一键启动 SLAM、LiDAR、operator panel。
- 文本输入巡检任务。
- 确定性路由优先，LLM 只处理模糊任务。
- 多点任务队列 dry-run 和实机低速验证。
- UI 显示任务队列、到达回复、安全原因。
- 弱网模式显示 semantic-only 策略。
- 拍照先以 `capture_keyframe` 事件记录。

### P1：可见能力增强

- `state_journal` 或长驻 operator core，减少短进程状态丢失。
- `SafetyGate` policy 化：`block/hold/slow/semantic_only/confirm/replan`。
- `capture_keyframe` 接真实相机命令。
- 增加 `speak` 动作，播报到达、阻断、异常。
- 引入模拟 `RadarDetectionSummary`，UI 展示雷达告警。
- 双目深度摘要接入 SafetyGate。

### P2：多模态边缘节点

- NX 上运行 TI 雷达 parser，发布 `RadarDetectionSummary`。
- 雷达告警参与巡检任务和 SafetyGate。
- LLM 常驻服务化，C++/Python 通过 HTTP 调用。
- 弱网模式压测：延迟、丢包、带宽限制下任务是否继续。
- 生成巡检结束报告。

### P3：研究扩展

- 评估毫米波雷达更高级的成像或 SAR-like 能力。
- 评估本地模型微调，让小模型稳定输出任务队列 JSON。
- 接入更多边缘节点，例如热成像、气体传感器、固定摄像头。
- Qt/RViz2 图形化面板。

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
