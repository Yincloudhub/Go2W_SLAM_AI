# GO2W 近场碰撞事故与恢复验证

> 本文保留 2026-06-01 事故和当时临时恢复方案。下文的 D435 三 ROI 硬门槛和
> 独立侧车描述已被当前架构取代。现行入口见
> `docs/go2w_runtime_operations_runbook.md` 与
> `docs/go2w_current_system_status_20260613.md`。

## 事故现象

2026-06-01，机器人从初始点执行多点导航时，起步右转后持续顶住右侧箱体，直到关节发热并停止工作。事故后应保持机器人趴下，完成静态检查和分级验证前不得恢复真实运动。

## 已确认的软件原因

1. 网关 `LidarGeometryPerception` 仍是阶段性占位实现，固定输出前后左右 `6.0m`。
2. 占位值此前没有 `source`、`stale`、`confidence` 标记，`SafetySupervisor` 将其当成真实近场距离。
3. C++ `SafetyGate` 只检查前方距离，没有拒绝缺少传感器来源的摘要，也没有检查左右近障。
4. 上层现场入口默认 `nav_mode=1`，但 Unitree SLAM 接口约定中 `mode=0` 才是绕障模式。
5. `QueueExecutor` 运行中发现阻断、网关连续失败或超时时，没有保证补发一次 `pause_navigation`。
6. 重启后视觉生产者未启动时，UI 将“未启动或离线”与“运行中数据过期”混成同一种显示。

## 与 XT16 重启时间问题的关系

2026-06-14 复验确认，XT16 在没有 GPS/PPS/NMEA 或本地 PTP master 时会进入
`Free Run`，并可能输出 `2020-05-20` 的旧时间。该状态会让 `xt16_driver`
丢弃时间异常帧，因此能直接阻断当前 SLAM 点云输入和重定位。

但没有证据表明它是 2026-06-01 撞箱子的直接原因。该事故已经确认的直接软件
原因仍是固定 `6.0m` 占位距离被当作真实安全数据、没有左右近障门槛、导航模式
错误以及失败时没有可靠补发暂停。当时 XT16 几何还不是正式运动安全来源。
因此应把两类故障分开：时间失同步解释“点云/重定位为什么不可用”，不能替代
已有的碰撞因果结论。

## 当时临时修正策略（已归档）

- 手工 `6.0m` 距离只允许作为调试占位，固定标记为 `manual_stub + stale`，不能解锁运动。
- 当时由 D435 轻量 ROI 摘要提供临时近场凭据。该方案只用于事故后的过渡验证。
- 当时使用 D435 前、左、右 ROI 门槛；该门槛不再是当前真实执行契约。
- 默认导航模式改为 `mode=0`，明确启用 Unitree SLAM 绕障。
- DeepYOLO 当时是可选侧车；当前已并入单一 D435 capture owner。
- XT16 点云几何后来成为四向主安全源，D435 只保留前向保守补充角色。

## 当前策略

- D435 depth 与 YOLO 共用唯一 `D435CaptureOwner`，旧入口只能转发。
- XT16 提供四向 geometry 和可见低矮风险带；未 verified 时继续 fail-closed。
- D435 只进入 `forward_supplements`，不能替代 XT16 左、右、后或底盘下方覆盖。
- 唯一执行链为
  `TaskQueue -> MissionDecisionEngine -> SLAM Gateway -> Unitree SDK`，
  Gateway 是最终运动权威。

## 上电后分级恢复

### 0. 人工检查

- 检查关节温度、异响、外壳、线缆和箱体碰撞位置。
- 机器人保持趴下，遥控器和物理停机手段在操作员手边。
- 不开启 UI 的“允许真实执行”。

### 1. 静态启动

```bash
cd /home/unitree/Go2W_SLAM_AI
bash scripts/run_go2w_operator_web.sh
```

该入口只自动启动或检查 XT16、SLAM 和轻量深度侧车，不应下发导航、重定位、建图或底盘运动命令。

### 2. 静态状态检查

```bash
bash scripts/go2w_d435_perception_sidecar.sh health
PYTHONPATH=src python3 scripts/go2w_agent_entry.py --status --pretty
```

必须看到：

```text
PerceptionContext.local_geometry.primary = fresh verified XT16
PerceptionContext.local_geometry.forward_supplements = fresh D435 depth when available
D435 owner instance / timestamp / sequence = valid
Gateway motion authority = false during static verification
```

### 3. 趴下阻断测试

在机器人仍趴下时，将箱体放在右侧近距离范围，确认 UI 的“运动安全门”显示锁定，`/execute on` 被拒绝。移开箱体后也只验证状态恢复，不进行导航。

### 4. 首次短距离运动

只有前三步通过后，才允许在开阔区域、双人值守、遥控器可立即接管的条件下进行 `0.2m` 级别短距离验证。先验证起步、原地转向和主动暂停，再逐步增加距离。

## 禁止事项

- 不允许用手工修改 JSON 距离绕过安全门。
- 不允许在 XT16 stale/uncalibrated、D435 前向摘要异常或任一可信来源报告近障时
  开启真实执行；D435 不负责左右/后方或底盘下方兜底。
- 不允许直接使用 `mode=1` 作为默认现场导航模式。
- 不允许在箱体、桌脚、电缆或人员紧邻机器人时做多点任务。
