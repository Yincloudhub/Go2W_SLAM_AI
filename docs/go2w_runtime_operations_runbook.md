# GO2W 闭环现场使用手册

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

DeepYOLO 和双目深度是可选语义侧车。它们可以增强环境理解，但退出或数据过期时不得阻断 LiDAR-only 主链路，也不得放宽 LiDAR 的安全判断。

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
