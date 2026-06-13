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

不启动 SLAM、Gateway 或底盘运动时，可以直接读取底层 `/utlidar/cloud`。建议
电脑通过有线地址连接：

```powershell
cd E:\GO2W_0
.\.venv\Scripts\python.exe scripts\visualize_xt16_over_ssh.py `
  --host 192.168.123.18 `
  --topic /utlidar/cloud `
  --record-jsonl artifacts\xt16_visual\corridor_baseline.jsonl
```

密码由终端交互输入，不使用命令行参数，也不写入 artifact。显示约定：

```text
red:    当前 footprint + margin 内被删除的点
cyan:   footprint 外保留的机身高度点
orange: footprint 外保留的低矮风险点
gray:   高度带外诊断点
```

该工具同时显示当前算法的前、左、右、后净空和
`points_excluded_footprint`。关闭窗口会关闭 SSH 订阅。它是诊断工具，不会把
XT16 标记为 calibrated，也不会改变 Gateway 运动授权。

## 注意事项

1. 真实执行前必须确认 `loc=true`、`map=true`、`motion=false`、`safety=ok`。
2. XT16 是主安全感知源。双目和 DeepYOLO 只能增加谨慎程度，不能解除 LiDAR 阻塞。
3. 当前 `unitree_slam` 是厂商二进制，CPU 开销需要先观测再调参。不要在比赛前临时修改 `/unitree/module/unitree_slam/config`。
4. 建图、写拓扑点、打开 RViz2 和真实执行都应由操作者显式确认，不应跟随 UI 启动自动触发。
5. 机器人重启、系统升级或雷达序列号变化后，先重新检查 XT16、SLAM 和重定位，再录点或执行任务。
6. 中文拓扑点写入注册表时遵守 UTF-8 校验流程，避免 PowerShell here-string 直接写中文 JSON。
