# GO2W 2026-06-14 上下文归档与新 Session 交接

## 1. 本页用途

本页用于结束 2026-06-14 的 XT16 静态边界排查上下文，并给下一 Session 提供
唯一、可执行的起点。仓库文档是权威来源，Obsidian 只是同步视图。

权威阅读顺序：

1. `docs/go2w_session_handoff_20260614.md`
2. `docs/go2w_current_system_status_20260613.md`
3. `docs/go2w_competition_edge_autonomy_handoff_20260612.md`
4. `docs/go2w_runtime_operations_runbook.md`
5. `docs/mission_decision_v1.md`
6. `docs/perception_context_v1.md`

## 2. 版本与安全状态

- 分支：`agent/llm-on-robot`
- 本次归档前实现基线：`317808436e56495d111402ff5a52ef1a79774c53`
- 本页所在 Git 提交是归档后的权威版本；新 Session 开始时必须重新核对
  local、origin 和 robot HEAD，不能只依赖这里记录的旧哈希。
- 机器人连接地址按现场网络选择；2026-06-14 最近使用
  `unitree@192.168.3.17`。
- 密码不得写入代码、文档、日志或 Git。
- `uv.lock` 是无关未跟踪文件，不修改、不同步、不提交。
- `stash@{0}` 是废弃方向实验，不得恢复。
- Git 提交是机器人唯一代码来源，不允许长期保留覆盖式散文件。
- 本阶段验收结束后传感器测试服务、SLAM、Gateway 和底盘运动均应保持停止。

## 3. 已完成且不得重复的工作

### P0 主链

- P0-1：D435 深度与 DeepYOLO 使用单一相机 owner；旧 sidecar 入口只转发。
- P0-2：Planner、WorldState、C++ LLM、Web UI 和日志消费同一份
  `PerceptionContext v1`。
- P0-3：唯一真实执行链为
  `TaskQueue -> MissionDecisionEngine -> SLAM Gateway -> Unitree SDK`；
  Gateway 保持最终运动权威。

### XT16 静态工程验证

以下证据已经完成，不得在新 Session 中重新要求用户摆放前、左、右、后箱体，
也不得为了得到相同结论重新测量机身：

- 2026-06-06 前、左、右、后轴向静态场景，确认 XT16 原始坐标到机身坐标映射。
- 2026-06-13 走廊当前算法基线，确认前后自回波需要 `0.05 m` 纵向余量。
- 2026-06-14 右侧设备箱与前移约 `0.5 m` 开放场景成对差分，确认横向
  `0.30-0.35 m` 点带是外部箱体/线缆，不是机身自回波。
- 当前有效安全 footprint 为前、后、半宽 `0.30 m`；前后过滤余量
  `0.05 m`，左右过滤余量 `0.00 m`。

`0.30 m` 是经过静态工程筛选的有效安全外廓，不是精确 CAD 尺寸。只有安装位姿、
机身外廓或算法发生变化，或者正式放行审查指出某个明确缺失的 ground truth，
才补采对应证据。

## 4. “已做静态验证”与“未签发正式标定”的区别

`configs/perception/xt16_geometry_calibration.json` 仍保持
`pending_field_measurement`，不是因为前/左/右/后场景没做，而是现有 guard
把 `completed_stationary_measured_scenes` 定义为一套严格的五份正式 measured
artifact 台账，要求测量值、哈希、地图和 Git 记录齐全。

因此当前结论是：

- 轴向和有效 footprint 的静态工程验证：完成。
- 正式 measured ledger / `verified` 标定：未签发。
- 真实低速运动总验收：未执行。

后续若要从 `pending` 提升为 `verified`，先审计并复用已有 artifact，只补台账
真正缺少的测量或元数据，不能重新开始一轮无目标的静态探索。

## 5. 高度、低矮障碍与机腹盲区

- XT16 当前把机身坐标 `z=-0.25 m` 到 `-0.10 m` 作为独立
  `low_hazard` 带；四方向可见的线缆或低矮点簇会收紧净空。
- 低于 `min_z_m`、被机身遮挡或位于底盘正下方的区域仍是盲区。
- D435 只补前向深度视野，不能覆盖机腹、左右或后方，不能写成“底下只依赖
  深度相机”。
- 真实运动放行还需要 Unitree 原生地形/避障能力、低速起步、人工急停和有界
  现场验收；若任务要求可靠的机腹或落差覆盖，再评估专用向下传感器。

## 6. 下一步唯一任务

下一 Session 的唯一软件任务是 P0-4：

`CommunicationPolicyExecutor + append-only journal + ack sequence + reconnect replay`

验收重点：

1. 弱网或断网不阻塞本地 PerceptionContext、TaskQueue、MissionDecisionEngine
   和 Gateway。
2. 任务、决定、执行状态使用追加写 journal，进程重启后可恢复。
3. 上行消息有稳定 sequence，ack 后才能从待补传集合移除。
4. 重连只补传缺失记录，不重复执行已经决定或已经完成的运动任务。
5. 远端消息不能绕过 MissionDecisionEngine 或直接调用 Gateway。
6. 先完成本地和机器人无运动验收；未经用户明确允许，不启动 SLAM、Gateway
   或底盘运动。

XT16 正式标定 promotion、低速运动验收、TI/NX live transport 和 IMU/里程计
摘要继续作为 deferred issues，不与 P0-4 混成一个巨型提交。

2026-06-14 后续执行状态：P0-4 已完成实现、提交、推送、机器人 fast-forward
和本地/机器人纯 journal/dry-run 无运动验收。实现提交为 `c53f9a8`，机器人
Python `327/327` 通过；验收前后相关 GO2W 进程均为空。此归档不授权继续 XT16
复测、SLAM/Gateway 启动或真实运动。

### 2026-06-14 启动与受监督闭环整理

- P0-4 已完成，不再回退重做；当前主线是降低重启后的操作复杂度并准备受监督
  低速闭环。
- `start_go2w_runtime_stack.sh` 作为单一启动入口，负责 PTP、XT16、SLAM、
  统一 D435、PerceptionContext 和 Gateway 只读探测；它不自动重定位或运动。
- 正式 `xt16_geometry_calibration.json` 继续保持 `pending_field_measurement`。
  独立的 `xt16_supervised_release.json` 仅表示工程验证放行，不冒充正式标定。
- 工程放行必须显式启用、现场有人且可急停，速度上限为 `0.1 m/s`。侧向
  `<0.20 m`、后向 `<0.30 m`、前向 `<0.80 m` 的硬暂停保持不变。
- 功能不要求空旷房间；侧向 `0.20-0.60 m` 的走廊属于允许保守导航的范围。
  当前若实时净空低于硬暂停阈值，必须先查明实际障碍或感知原因，不能用工程模式
  绕过。

## 7. 新 Session 可直接使用的提示词

```text
继续推进 E:\GO2W_0 的 GO2W 比赛边缘自治项目。

先读取并以这些文档为权威：
1. docs/go2w_session_handoff_20260614.md
2. docs/go2w_current_system_status_20260613.md
3. docs/go2w_competition_edge_autonomy_handoff_20260612.md
4. docs/go2w_runtime_operations_runbook.md

当前分支是 agent/llm-on-robot。开始时只做状态核对：
- 核对 local、origin、robot HEAD 和工作区；
- Git 提交是机器人唯一代码来源；
- uv.lock 不修改、不提交；
- stash@{0} 不得恢复；
- 不启动 SLAM、Gateway、D435、XT16 或底盘运动。

重要纠偏：
- XT16 前、左、右、后轴向静态验证已经完成；
- 当前算法的走廊基线和右侧设备箱/开放场景成对差分已经完成；
- 有效安全 footprint 是前后/半宽 0.30 m，前后余量 0.05 m，左右余量 0.00 m；
- 不要再次要求测机身或重做前/左/后三场；
- pending_field_measurement 仅表示正式 measured ledger 尚未签发；
- D435 只补前向，不能覆盖机腹、左右和后方。

P0-1、P0-2、P0-3 已完成。下一步唯一任务是实现 P0-4：
CommunicationPolicyExecutor、append-only journal、ack sequence 和断线重连补传。
必须保证弱网不阻塞本地闭环，补传不导致任务重复执行，远端不能绕过
MissionDecisionEngine，Gateway 仍是最终运动权威。

先审阅现有弱网/任务状态代码和测试，给出小步实施计划，然后直接实现、测试、
同步 Obsidian、独立提交并推送。机器人只能 git fast-forward 到提交。
未经我明确允许，不进行真实运动验收。
```
