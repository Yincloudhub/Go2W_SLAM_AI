# GO2W 闭环现场使用手册

## 事故后安全门

2026-06-01 近场碰撞后，真实运动入口已经改为 fail-closed。固定 `6.0m`
占位距离不能解锁导航。2026-06-12 的当前架构以已标定 XT16 为四向主安全
源；前视 D435 只允许收紧前方净空，不能替代左右和后方判断。XT16 尚未
完成正式标定，因此当前真实运动仍被正确阻断。

## 当前闭环边界

默认链路已经可以在机器人本机运行：

```text
XT16 LiDAR
  -> Unitree SLAM / relocation
  -> C++ GatewayClient
  -> SemanticRouter
  -> TaskQueue validator
  -> SafetyGate
  -> QueueExecutor
  -> operator Web UI / LLM 输入
```

DeepYOLO 是可选语义侧车，不参与运动许可。XT16 点云几何摘要是主安全
源；D435 是前视保守补充。任一来源只能增加谨慎，不能放宽 XT16 阻断。

## PerceptionContext v1 运行边界

P0-2 的统一读取入口位于：

```text
src/edge_autonomy/perception_context.py
```

运行时判定必须遵守：

- XT16：摘要 + `xt16_geometry_service/producer.pid` + 匹配进程。
- D435：统一摘要 + 嵌入 owner PID/state + 匹配 capture 进程。
- TI/NX：摘要 + transport manager 提供的 bridge-online evidence。
- 缺少生产者证据时，即使 artifact 内写着 `fresh` 也必须输出 `offline`。
- JSON/身份/类型/未来时间或 D435 序号回退异常输出 `invalid`。
- `local_geometry.primary` 只接受 fresh 且已验证标定的 XT16；D435 只进入
  `forward_supplements`。
- context 生成时重新计算有效 age，避免缓存中的旧 envelope 保持 fresh。
- context producer 不启动 D435、XT16、SLAM、Gateway 或底盘运动。

检查代码契约使用：

```bash
PYTHONPATH=src python3 -m unittest tests.test_perception_context
```

统一运行 artifact 为：

```text
artifacts/perception_context_v1.json
```

生命周期命令：

```bash
bash scripts/go2w_perception_context_sidecar.sh start
bash scripts/go2w_perception_context_sidecar.sh health
bash scripts/go2w_perception_context_sidecar.sh stop
```

该 sidecar 只读取紧凑摘要并发布 context，不持有相机，不启动 SLAM/Gateway。

P0-2 于 2026-06-13 完成机器人无运动验收。验收结束必须同时看到：

```text
d435_capture_owner=stopped
d435_summary_reducer=stopped
perception_context=stopped
```

### WorldState reducer 状态

Python/C++ WorldState reducer 已完成接口切换：

- 只接收 `perception_context`，不接收自由格式传感器 summary 列表。
- context 超过自身 `stale_ms`、结构错误或 policy 不一致时整体拒绝。
- `perception_summaries` 仅是 `context.sources` 的兼容投影。
- 未提供 fresh context 时显示 `perception_context_status=unavailable_or_stale`。

Python Planner、C++ OperatorPanel/LLM、Web UI 和 runtime log 已统一消费该
artifact。Web 的旧 lidar/depth/semantic/radar 响应字段仅是同一
`context_id` 的兼容投影，不再读取旧独立 artifact。

## 本地浏览器入口

机器人侧启动 UI：

```bash
cd /home/unitree/Go2W_SLAM_AI
bash scripts/run_go2w_operator_web.sh
```

Windows 本地建立 SSH 隧道：

```powershell
cd E:\GO2W_0
python scripts\go2w_web_tunnel.py
```

The tunnel reconnects lazily after a robot reboot or SSH transport failure. In
`auto` mode it probes the configured management addresses again, so switching
between direct cable and Wi-Fi does not require editing the source code.

浏览器打开：

```text
http://127.0.0.1:8765/
```

## 网络角色和换网方式

不要把 SSH 地址、gateway 网卡和 SLAM DDS 网卡混为一个配置：

- `GO2W_SSH_HOST`：Windows/MobaXterm 到机器人 NX 的管理地址。它会随 Wi-Fi、有线直连或现场路由变化。
- `GO2W_SSH_HOSTS`：本地 tunnel 自动探测的候选管理地址，按逗号分隔。默认依次尝试 `192.168.123.18,192.168.3.17`。
- `GO2W_NETWORK_INTERFACE`：机器人内部 Unitree gateway 使用的接口，默认 `eth0`。本地改用 Wi-Fi 或有线 SSH 不应自动改动它。
- `GO2W_DDS_INTERFACE`：仅用于 Unitree SLAM 的 CycloneDDS 绑定。通常保持自动；需要现场强制指定时再设置。

日常打开 UI 直接运行自动探测：

```powershell
python scripts\go2w_web_tunnel.py
```

有线直连时也可以显式指定，避免等待其他候选地址超时：

```powershell
python scripts\go2w_web_tunnel.py --ssh-host 192.168.123.18
```

现场地址变化时，优先临时设置环境变量，不需要修改代码：

```powershell
$env:GO2W_SSH_HOST='192.168.3.17'
$env:GO2W_SSH_HOSTS='192.168.3.17,192.168.123.18'
python scripts\go2w_web_tunnel.py
```

系统升级导致 CycloneDDS XML 中旧网卡名消失时，`scripts/start_go2w_slam_stack.sh` 会生成运行时副本，并优先选择同类型网卡、默认路由网卡和其他可用物理网卡。只做预检、不启动进程：

```bash
GO2W_SLAM_CONFIG_ONLY=1 bash scripts/start_go2w_slam_stack.sh
```

UI 默认使用自适应刷新：待机时每 5 秒更新，任务规划或导航时每 2 秒更新。手动点击“刷新状态”会立即强制读取一次。

## 静止调试命令

以下命令不会让底盘运动：

```bash
cd /home/unitree/Go2W_SLAM_AI

# 检查或启动 XT16 和 SLAM
bash scripts/start_go2w_slam_stack.sh

# 查看定位、安全门和世界状态
python3 scripts/go2w_agent_entry.py --status --pretty

# 只读取资源开销，默认采样 10 秒
bash scripts/snapshot_go2w_runtime_resources.sh

# 查看唯一 D435 感知服务；它同时产出深度和 YOLO 摘要
bash scripts/go2w_d435_perception_sidecar.sh status
bash scripts/go2w_d435_perception_sidecar.sh health

# 相机长时间无新帧时，只重启统一 D435 服务，不影响 SLAM
bash scripts/go2w_d435_perception_sidecar.sh restart-if-stale
```

如果机器人重启后未定位，在确认机器人实际位于对应锚点附近后，通过 UI 执行：

```text
/start-slam
/relocate mapping_origin confirm
/status
```

`relocate` 只做 SLAM 重定位，不会移动底盘。重定位锚点必须与机器人实际位置接近，否则可能失败或得到错误初值。

## D435 统一感知服务

比赛运行时只有 `D435CaptureOwner` 可以打开 RealSense。它在同一 RGBD
frameset 上记录 RealSense frame sequence、sensor timestamp 和主机接收时间。
深度 ROI 默认约 `7.5 Hz` 独立刷新，YOLO 按档位消费 latest frame；两条异步
输出各自保留来源帧元数据，不要求最新 sequence 相等。YOLO 变慢、报错或
没有事件时，深度摘要仍继续刷新。

主输出和迁移兼容视图：

```text
artifacts/d435_perception_summary.json
artifacts/stereo_depth_summary.json
artifacts/vision_semantic_summary.json
```

三份文件都有 `generation_id`。兼容视图先原子替换，主摘要最后替换，主摘要
可作为本轮发布事务的完成标记。旧入口
`go2w_deepyolo_sidecar.sh` 和 `go2w_stereo_depth_sidecar.sh` 仅转发到统一 manager，
不会再启动第二个 RealSense pipeline。

`start/stop/restart` 使用 manager 锁串行化。启动只有在 capture owner、reducer
和当前 owner 对应的 fresh 主摘要都通过后才返回成功。停止时即使 reducer PID
异常，也会继续尝试停止 capture owner。

artifact 文件存在不代表服务在线。判断输入可用必须同时检查：

```text
capture owner PID 与进程命令匹配
summary owner.pid 与当前 capture PID 一致
status=fresh 且 stale=false
timestamp_ms 在 freshness budget 内
generation_id 属于同一发布事务
```

服务停止后，历史摘要可保留作验收证据，但 loader 必须将其判为 stale/offline。

默认使用轻量常驻档：

```bash
cd /home/unitree/Go2W_SLAM_AI
bash scripts/go2w_d435_perception_sidecar.sh restart
```

可选档位：

```bash
# 轻量常驻：约 3 Hz 语义更新，适合长期在线
GO2W_D435_PROFILE=resident bash scripts/go2w_d435_perception_sidecar.sh restart

# 均衡展示：约 5 Hz 语义更新，适合现场演示
GO2W_D435_PROFILE=balanced bash scripts/go2w_d435_perception_sidecar.sh restart

# 全速诊断：只用于短时排障，不建议常驻
GO2W_D435_PROFILE=diagnostic bash scripts/go2w_d435_perception_sidecar.sh restart
```

档位只影响 YOLO 的取帧和推理节拍，不降低深度 ROI 的独立刷新。XT16、SLAM、
SafetyGate 和 QueueExecutor 不依赖 YOLO 成功。
如果 UI 显示视觉语义 `stale/ignored`，先运行 `health`；确认 stale 后再用
`restart-if-stale` 恢复。该命令只重启统一 D435 服务，不会触发 SLAM、
重定位或底盘运动。
如果 `rs-enumerate-devices -s` 没有枚举到 D435I，或系统没有 `/dev/video*`，
不要循环重启侧车；先检查相机 USB、供电和线缆。此时 UI 会把视觉显示为离线，
主链路仍按 XT16 LiDAR + SLAM 运行。

## P0-3 唯一决定与执行链

所有任务必须经过：

```text
TaskQueue -> MissionDecisionEngine -> Python supervisor -> SLAM Gateway -> Unitree SDK
```

操作员重点查看：

```text
mission_decision.decision
mission_decision.reason
mission_decision.execution_owner
mission_decision.preflight.gateway
operator_display.screen.mission_decision
operator_display.screen.motion_authority
```

安全含义：

- `dry_run_queue`：只预演，不运动。
- `await_confirmation`：需要人工确认，不运动。
- `hold`：Gateway 或任务要求保持，不运动。
- `reject`：队列、地图或拓扑无效，不运动。
- `execute_queue`：只表示可以进入 Python supervisor；Gateway 在下发前和运行中
  仍有最终否决权。

Gateway 断连时，真实执行 fail closed；dry-run 仍可输出队列和决定记录。
C++ 面板打开 `/execute on` 后，真实导航会转交 Python supervisor，C++ 内部
`QueueExecutor` 不拥有比赛导航租约。

### 2026-06-13 P0-3 部署记录

```text
implementation_commit: cf68b0ecca828a3881b69134a5b6ef6fd58fad3d
branch: agent/llm-on-robot
robot_python_targeted: 47/47 passed
robot_python_full_unittest: 298/298 passed
robot_cpp_build: passed
robot_ctest: 6/6 passed
robot_dry_run_target: yin_siyuan_station
robot_dry_run_decision: dry_run_queue
gateway_checked: false
motion_allowed: false
slam_started: false
gateway_started: false
services_stopped: true
robot_worktree: clean
```

P0-3 完成标记为
`p0-3-mission-decision-chain-accepted-20260613`。当前系统总览和操作者检查顺序见
`docs/go2w_current_system_status_20260613.md`。

## Dry-run 与真实执行

UI 默认是“仅预演”。机器人趴卧、锚点未站立复核、人员密集或现场未清空时，不要打开“允许真实执行”。

只做规划和安全检查：

```bash
python3 scripts/go2w_agent_entry.py \
  --command-b64 "<UTF-8 base64>" \
  --no-live-snapshot \
  --status \
  --pretty
```

真实执行必须显式增加：

```text
--execute
```

推荐先做不连接 Gateway 的纯预演：

```bash
python3 scripts/go2w_agent_entry.py \
  --go-b64 "<UTF-8 base64>" \
  --dry-run \
  --skip-gateway-check \
  --full-output
```

真实执行时禁止 `--skip-gateway-check`。只有操作者确认机器人站立、现场清空、
地图/定位正确、XT16 主几何源可信并准备好遥控器急停后，才增加 `--execute`。

中文从 Windows 或 SSH 发送时优先使用：

```bash
python3 scripts/go2w_encode_command.py --mode go "去目标点拍照，然后返回" --pretty
```

## XT16 走廊静止可视化

不启动 Unitree SLAM、Gateway 或底盘运动时，先锁定 XT16 的 PTP 时钟，再只
启动 `xt16_driver`，读取正式运行时话题 `/unitree/slam_lidar/points`：

```bash
sudo apt-get install linuxptp  # 机器人首次使用时执行一次
cd /home/unitree/Go2W_SLAM_AI
sudo scripts/go2w_xt16_ptp.sh start
```

`start` 必须返回 `xt16_ptp=healthy`，且雷达配置连续 5 次包含
`"PTPStatus":"Tracking ..."` 或 `"PTPStatus":"Locked ..."`，然后才能启动
`xt16_driver`。`start_go2w_slam_stack.sh` 默认在启动 driver 前执行同一健康检查；
PTP 未运行或已回到 `Free Run` 时直接失败，不允许带着异常时间启动 SLAM。
driver 初始化可能使雷达短暂显示 `Free Run` 后重新捕获 PTP，因此运行时检查最多
等待 `20 s` 重新取得连续 5 个健康样本；持续失锁仍然失败。

2026-06-13 重启后的故障样本中，雷达 UDP 包仍携带 `2020-05-20`，导致正式驱动
以时间异常丢弃整帧。XT16 是独立设备，不会自动读取机器人 Linux 系统时钟；
当雷达配置为 GPS、但没有 GPS/PPS/NMEA 锁定时，会进入 `Free Run` 并使用其
内部旧历元。恢复链路为
`机器人系统时钟 -> eth0 本地 PTP master -> XT16`，只在局域网内运行，不要求
互联网；但机器人系统时钟本身必须正确。

2026-06-14 复验还发现默认 `ptp4l` 的 10 ms 发送时间戳等待会在该软件时间戳
网卡上周期性超时，进程仍在但雷达会退回 `Free Run`。manager 现显式设置
`--tx_timestamp_timeout 1000`，并接受正常运行时交替出现的 `Tracking/Locked`。
实机空载连续 120 秒检查中只有一个 PTP 进程，偏差保持纳秒级；driver 初始化
期间出现过一次短暂 `Free Run`，随后在没有重启 PTP 的情况下自行恢复
`Tracking`，且正式点云持续发布。manager 默认启动最多等待 `90 s`，运行时
健康检查最多等待 `20 s`；启动超时会恢复 GPS、停止 `ptp4l` 并拒绝继续。
电脑建议通过有线地址连接：

```powershell
cd E:\GO2W_0
.\.venv\Scripts\python.exe scripts\visualize_xt16_over_ssh.py `
  --host 192.168.123.18 `
  --topic /unitree/slam_lidar/points `
  --record-jsonl artifacts\xt16_visual\corridor_baseline.jsonl
```

密码由终端交互输入，不使用命令行参数，也不写入 artifact。显示约定：

```text
red:    当前 footprint 及前后纵向 margin 内被删除的点
cyan:   footprint 外保留的机身高度点
orange: footprint 外保留的低矮风险点
gray:   高度带外诊断点
```

该工具同时显示当前算法的前、左、右、后净空和
`points_excluded_footprint`。关闭窗口会关闭 SSH 订阅。它是诊断工具，不会把
XT16 标记为 calibrated，也不会改变 Gateway 运动授权。

查看器默认保留最近 `3 s`、最多 `8` 帧点云，并按时间从亮到暗淡出，静止观察时
可以形成短时累积轮廓。按 `C` 清空历史点；机器人或目标移动后应清空一次，避免
残影被误认为当前障碍。可通过 `--trail-seconds`、`--trail-frames` 和
`--max-trail-points` 调整显示效果，这些参数只影响本地画面。

查看器只接受 `frame_id=rslidar` 的 `/unitree/slam_lidar/points`，使用已经通过
前、左、右、后纸箱差分确认的 XT16 机身轴向。`/utlidar/cloud`
（`frame_id=utlidar_lidar`，约 4k 点/帧）属于不同的 Unitree LiDAR
pipeline/frame，不得套用 XT16 轴向、footprint 或安全阈值。即使 topic 被错误
重映射，查看器也会在 `frame_id` 不是 `rslidar` 时停止。默认只显示至少连续
两帧落入同一 `0.08 m` 平面格的青色稳定实体点。橙色低矮层默认隐藏，按 `L`
切换；地面高度拒绝点和红色机身回波默认隐藏，按 `R` 切换。该显示过滤不参与
Gateway 运动授权。

2026-06-13 走廊静止基线确认前后纵向 filter-only margin 为 `0.05 m`。
2026-06-14 的成对场景进一步确认横向 margin 必须为 `0.00 m`：右侧紧邻设备箱
时，横向 `0.30-0.35 m` 带每帧有 `245-345` 个点；机器人由用户前移约 `0.5 m`
后，开放场景 12/12 帧该带为零点。说明这些点来自设备箱/线缆，不是自回波，
不能用横向 5 cm 遮罩吞掉。前后中央仍有稳定自回波，因此只保留纵向 5 cm。
低矮风险仅删除名义 footprint 内部点，机身外的线缆仍保留。净空仍从名义机身
边缘计算，XT16 标定状态仍为 `pending_field_measurement`。正式比较应短时只启
`xt16_driver`，读取运行时使用的 `/unitree/slam_lidar/points`，采样后立即停止
driver；不得因此启动 Unitree SLAM、Gateway 或底盘运动。

## 走廊净空与规划可移动性

`corridor_clearance_v1` 的净空以名义机身边缘为零点，不包含已过滤的机身回波：

```text
front: <1.50 m conservative, <0.80 m pause
side:  <0.60 m conservative, <0.20 m pause
rear:  <0.50 m conservative, <0.30 m pause
```

上表仍是 XT16 几何摘要的方向分级，不直接等同于受监督导航许可。
`semantic_mobility_v6` 延续 v5 的职责划分，并修正原生导航最低速度：

1. 注册点目标直接下发 Unitree `mode=0`，由原生规划器根据机器狗外形和局部地图
   判断起步转向与绕障。
2. GO2W 不再用固定的左/右单点净空近似机身旋转扫掠区域，也不在导航前抢先挪位。
3. 只有原生导航明确失败或持续无进展后，XT16 才根据当时的前、后、左、右净空
   生成不超过 `0.50 m` 的有界恢复候选。
4. 本地 LLM 只从候选中选择下一步；`MissionDecisionEngine` 复核候选和距离。
5. Gateway 使用 `SportClient::Move(vx, vy, 0)` 保持零角速度平移，速度不超过
   `0.10 m/s`，并按前、后、侧方向分别保留 XT16 净空。
6. 每一步结束后停止并刷新世界状态，再重新把原目标交给原生导航；最多默认三步。

这不是通用相对运动接口。LLM 不得输出任意速度或绕过候选边界，Gateway 仍是
最终运动权威。D435 前向近点在 XT16 前方清晰时必须由两张不同的新鲜帧确认。
受监督的 `mode=0` 原生导航运行期间，普通近障只记录为告警，不再由 GO2W
heartbeat 抢先暂停；SLAM/定位/传感器有效性、会话租约和显式紧急停止仍是硬停。

XT16 几何摘要正常以约 `5 Hz` 输出，但点云源时间戳、处理和文件交接会占用数百
毫秒。普通模式继续使用 `1 s` 新鲜度硬门；显式工程放行的 Unitree 原生导航使用
其最低有效速度 `0.20 m/s`，已验证来源的摘要最大允许年龄收紧为 `1.5 s`。
额外 `0.5 s` 容忍窗口内最大位移仍为 `0.10 m`；超过 `1.5 s`、时间戳缺失或
来源失效仍立即停止。内部有界平移脱困继续限制为 `0.10 m/s`。

Unitree 绕障调用位于 `robot/slam_gateway_refactor/src/slam_gateway.cpp` 的
`ROBOT_API_ID_POSE_NAV_PL (1102)`；目标 `mode=0` 表示避障模式。闭环在
1102 成功后显式调用 `ROBOT_API_ID_RESUME_NAV (1202)`，因为每次到达、超时或
失败都会执行 1201 暂停，而 SLAM 后端不会随着短生命周期 agent 退出而清除暂停态。

到达监控不再把所有任务固定限制为 25 秒。首次取得真实距离后，预算按
`距离 / 下发速度 * 1.8 + 15 s` 计算，同时保留 CLI 指定的最小监控时间。
默认 20 秒无运动看门狗同时观察平移和 yaw 变化；原地转向属于有效进展。
`0.08 m` 平移和 `0.12 rad` yaw 高于当前 SLAM 静止抖动，只用于识别底盘是否
真实运动，不是
左右净空、转向包络或碰撞许可阈值。

原生导航持续无进展时，闭环先暂停并刷新世界状态，让 LLM 策略层只从当前
前/后/左/右有界候选中选一步；`MissionDecisionEngine` 复核后执行一次零 yaw
平移，停止并再次刷新，然后把原目标交还 Unitree。不会连续盲走多步，也不会把
LLM 输出直接当底层速度。

未启用显式工程放行的正式模式继续使用全向严格策略。

## 注意事项

1. 真实执行前必须确认 `loc=true`、`map=true`、`motion=false`、`safety=ok`。
2. XT16 是主安全感知源。双目和 DeepYOLO 只能增加谨慎程度，不能解除 LiDAR 阻塞。
3. 当前 `unitree_slam` 是厂商二进制，CPU 开销需要先观测再调参。不要在比赛前临时修改 `/unitree/module/unitree_slam/config`。
4. 建图、写拓扑点、打开 RViz2 和真实执行都应由操作者显式确认，不应跟随 UI 启动自动触发。
5. 机器人重启、系统升级或雷达序列号变化后，先重新检查 XT16、SLAM 和重定位，再录点或执行任务。
6. 中文拓扑点写入注册表时遵守 UTF-8 校验流程，避免 PowerShell here-string 直接写中文 JSON。
## P0-4 通信 journal 无运动检查

以下命令只操作本地追加写 journal，不启动 SLAM、Gateway、传感器或底盘：

```bash
cd /home/unitree/Go2W_SLAM_AI
PYTHONPATH=src python3 scripts/go2w_communication_journal.py status --pretty
PYTHONPATH=src python3 scripts/go2w_communication_journal.py replay \
  --link-state disconnected --pretty
PYTHONPATH=src python3 scripts/go2w_communication_journal.py replay \
  --link-state recovered --pretty
```

默认文件为
`artifacts/communication/communication_journal_v1.jsonl`。远端累计 ack 通过：

```bash
PYTHONPATH=src python3 scripts/go2w_communication_journal.py ack \
  --ack-sequence <remote_ack_sequence> --pretty
```

验收必须确认 `execution_directive=false`，且补传 payload 不包含
`slam_command`、`target_pose`、`operator_ack`、原始视频或稠密点云。
