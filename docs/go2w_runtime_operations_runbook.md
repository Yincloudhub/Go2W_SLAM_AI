# GO2W 闭环现场使用手册

## 事故后安全门

2026-06-01 近场碰撞后，真实运动入口已经改为 fail-closed。固定 `6.0m` 占位距离不能解锁导航；当前阶段必须同时具备新鲜 D435 ROI 摘要，且前、左、右近场距离均通过安全门。恢复验证顺序见 `docs/go2w_near_field_collision_incident_2026-06-01.md`。

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

DeepYOLO 是可选语义侧车，不参与运动许可。D435 轻量 ROI 深度侧车当前是额外近场硬门槛：它退出或数据过期时，SLAM、定位和干跑仍可继续，但真实运动必须阻断。完成 XT16 点云几何摘要后，再切换为 LiDAR 主安全源与 D435 保守融合。

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

# 查看可选 DeepYOLO 侧车状态
bash scripts/go2w_deepyolo_sidecar.sh status

# 查看真实运动依赖的轻量 D435 深度安全侧车
bash scripts/go2w_stereo_depth_sidecar.sh health

# 检查视觉侧车是否真的还在刷新语义摘要
bash scripts/go2w_deepyolo_sidecar.sh health

# 如果 detector 进程还活着但相机长时间无新帧，安全地重启可选视觉侧车
bash scripts/go2w_deepyolo_sidecar.sh restart-if-stale
```

如果机器人重启后未定位，在确认机器人实际位于对应锚点附近后，通过 UI 执行：

```text
/start-slam
/relocate mapping_origin confirm
/status
```

`relocate` 只做 SLAM 重定位，不会移动底盘。重定位锚点必须与机器人实际位置接近，否则可能失败或得到错误初值。

## DeepYOLO 常驻档位

默认使用轻量常驻档：

```bash
cd /home/unitree/Go2W_SLAM_AI
bash scripts/go2w_deepyolo_sidecar.sh restart
```

可选档位：

```bash
# 轻量常驻：约 3 Hz 语义更新，适合长期在线
GO2W_DEEPYOLO_PROFILE=resident bash scripts/go2w_deepyolo_sidecar.sh restart

# 均衡展示：约 5 Hz 语义更新，适合现场演示
GO2W_DEEPYOLO_PROFILE=balanced bash scripts/go2w_deepyolo_sidecar.sh restart

# 全速诊断：只用于短时排障，不建议常驻
GO2W_DEEPYOLO_PROFILE=diagnostic bash scripts/go2w_deepyolo_sidecar.sh restart

# 如需检查画框效果，短时显式开启 overlay
GO2W_DEEPYOLO_PROFILE=diagnostic \
GO2W_DEEPYOLO_RENDER_OVERLAY=1 \
  bash scripts/go2w_deepyolo_sidecar.sh restart
```

档位只影响 DeepYOLO 可选语义侧车。XT16、SLAM、SafetyGate 和 QueueExecutor 不依赖该侧车。
如果 UI 显示视觉语义 `stale/ignored`，先运行 `health`；确认 stale 后再用
`restart-if-stale` 恢复。该命令只重启 DeepYOLO 和语义桥接器，不会触发 SLAM、
重定位或底盘运动。
如果 `rs-enumerate-devices -s` 没有枚举到 D435I，或系统没有 `/dev/video*`，
不要循环重启侧车；先检查相机 USB、供电和线缆。此时 UI 会把视觉显示为离线，
主链路仍按 XT16 LiDAR + SLAM 运行。

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

中文从 Windows 或 SSH 发送时优先使用：

```bash
python3 scripts/go2w_encode_command.py --mode go "去目标点拍照，然后返回" --pretty
```

## 注意事项

1. 真实执行前必须确认 `loc=true`、`map=true`、`motion=false`、`safety=ok`。
2. XT16 是主安全感知源。双目和 DeepYOLO 只能增加谨慎程度，不能解除 LiDAR 阻塞。
3. 当前 `unitree_slam` 是厂商二进制，CPU 开销需要先观测再调参。不要在比赛前临时修改 `/unitree/module/unitree_slam/config`。
4. 建图、写拓扑点、打开 RViz2 和真实执行都应由操作者显式确认，不应跟随 UI 启动自动触发。
5. 机器人重启、系统升级或雷达序列号变化后，先重新检查 XT16、SLAM 和重定位，再录点或执行任务。
6. 中文拓扑点写入注册表时遵守 UTF-8 校验流程，避免 PowerShell here-string 直接写中文 JSON。
