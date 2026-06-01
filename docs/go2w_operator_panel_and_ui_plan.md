# GO2W 操作者面板与 UI 推进方案

## 背景

2026-05-26 现场调试暴露出四个问题：

1. 机器狗失去地图/SLAM 初始化后，gateway 会退回 `x=0,y=0`，此时不能把当前位置写成语义点。
2. 旧 `701门外走廊` 点带 `needs_calibration`，不能作为可靠远距离重定位锚点。
3. 只看 JSON 不适合现场操作，需要实时语义化世界状态。
4. 手工启动脚本如果没有执行权限会报 `Permission denied`，应统一由仓库入口脚本用 `bash` 启动。

本地网线直连记录：

- 机器人/GO2W SSH 地址：`192.168.123.18`
- SSH 用户：`unitree`
- 网卡接口：`eth0`

## 当前修正

- `scripts/start_go2w_slam_stack.sh`：新增仓库内 SLAM 启动脚本，入口通过 `bash` 调用，不依赖旧目录脚本权限。
- `scripts/go2w_startup_supervisor.py`：新增一体化启动/健康检查入口，默认 dry-run；显式 `--run` 后只启动/检查 SLAM、雷达 driver、gateway 世界状态探针，不发送运动命令。
- `scripts/go2w_agent_entry.py`：默认 SLAM 启动脚本改为仓库内脚本。
- `cpp/go2w_operator_panel`：新增 C++ 操作者面板原型。
- `src/edge_autonomy/world_state_v1.py` 与 `src/edge_autonomy/operator_display.py`：新增 UI/LLM 共用的低频状态和任务显示屏数据契约。
- `cpp/go2w_operator_panel`：补齐 C++ 面板内的建图、拓扑点预览/写入、RViz2 打开入口；所有会改变机器人/SLAM 状态的命令都要求显式 `confirm`。
- `scripts/start_go2w_rviz2.sh`：新增 RViz2 诊断启动脚本，只负责可视化进程，不发布运动命令。
- `scripts/go2w_operator_web.py` 与 `scripts/run_go2w_operator_web.sh`：新增薄 Web UI。浏览器只负责显示、按钮、确认弹窗和 LLM 输入，实际命令仍委托给 `go2w_operator_panel`。
- `scripts/go2w_web_tunnel.py`：新增本地 SSH TCP tunnel，方便把机器人侧 `127.0.0.1:8765` 映射成本机浏览器的 `127.0.0.1:8765`。
- `scripts/realsense_depth_summary.py`：新增 D435I/RealSense 低频 ROI 深度摘要导出脚本，Web UI 可只读显示，不传原始帧。UI stale 阈值通过 `GO2W_STEREO_STALE_MS`/`--stereo-stale-ms` 配置，默认 5000 ms，避免 1-2 Hz 展示层因为一次刷新抖动就误报失效。

## C++ 操作者面板职责

第一版不是完整 Qt/RViz2 插件，而是 C++ 终端面板，用来固化底层接口：

- 直接通过 `slam_llm_command_client eth0` 读取 `world_state`。
- 按中文显示定位、SLAM、最近语义点、导航状态、前方净空和安全状态。
- 支持弱网摘要模式，只显示定位、SLAM、最近点、导航、安全。
- 提供 LLM 中文输入口，内部转 UTF-8 base64 后调用 `go2w_agent_entry.py --go-b64 ... --human`。
- 默认不执行运动，必须 `/execute on` 后才会真实下发。
- 支持现场流程入口：`/mapping start confirm`、`/mapping end confirm [pcd]`、`/topology preview NAME`、`/topology add NAME confirm`、`/rviz2 start confirm`。
- `topology preview` 只读当前世界状态，不落盘；`topology add` 通过 gateway 的 `add_current_pose_waypoint` 写入 `/home/unitree/topology_points.json`，且 gateway 会拒绝非 fresh localization。

## 编译

在机器人或 Linux/NX 上：

```bash
cd ~/go2w_slam_agent/cpp
cmake -S . -B build
cmake --build build -j
```

## 运行

只看状态：

```bash
cd ~/go2w_slam_agent/cpp
./build/go2w_operator_panel --repo-root ~/go2w_slam_agent --watch 0
```

交互模式：

```bash
cd ~/go2w_slam_agent/cpp
./build/go2w_operator_panel --repo-root ~/go2w_slam_agent --current-node yin_siyuan_station
```

进入后可输入：

```text
/status
/watch 30
/weak on
/topology preview wp_station_01
/topology add wp_station_01 confirm
/rviz2 start confirm
/execute on
去赵博办公室门口拍照，然后回尹思园工位
```

浏览器 UI：

```bash
cd ~/go2w_slam_agent
./scripts/run_go2w_operator_web.sh
```

默认监听 `127.0.0.1:8765`，适合通过 MobaXterm/SSH tunnel 打开本机浏览器访问：

```bash
ssh -L 8765:127.0.0.1:8765 unitree@192.168.123.18
```

也可以用仓库脚本启动本地 tunnel：

```powershell
python .\scripts\go2w_web_tunnel.py
```

本地 tunnel 默认自动尝试有线和 Wi-Fi 管理地址。需要锁定当前链路时使用：

```powershell
python .\scripts\go2w_web_tunnel.py --ssh-host 192.168.123.18
$env:GO2W_SSH_HOSTS='192.168.3.17,192.168.123.18'
python .\scripts\go2w_web_tunnel.py
```

这里的 SSH 管理地址只影响 Windows 到 NX 的浏览器隧道。机器人内部 gateway 默认仍使用 `GO2W_NETWORK_INTERFACE=eth0`，Unitree SLAM 的 CycloneDDS 绑定由 `GO2W_DDS_INTERFACE` 单独控制。三者必须分层配置，避免换用有线直连后误改 SLAM 或运动链路。

如果需要在直连网段直接访问，可显式设置：

```bash
GO2W_WEB_HOST=0.0.0.0 ./scripts/run_go2w_operator_web.sh
```

## UI 操作策略

UI 的第一目标是“一打开就能看见系统是否可用”，不是“一打开就改变机器人状态”。建议保持以下策略：

1. 打开 `scripts/run_go2w_operator_ui.sh` 后自动启动/检查雷达 driver、SLAM 和 gateway 世界状态，但不自动建图、不自动重定位、不自动记录拓扑点。
2. 建图作为现场显式流程：需要新地图时输入 `/mapping start confirm`，结束时 `/mapping end confirm /home/unitree/test_xxx.pcd`；平时打开 UI 只看定位和已有地图。
3. 拓扑点分两步：先 `/topology preview NAME` 看当前 pose、SLAM/localization 状态，再 `/topology add NAME confirm` 写入。这样能避免 `x=0,y=0` 或 pose 过期时污染拓扑。考虑不同 SLAM 回调频率，写入门槛按 pose_age 策略判断：`localized/degraded/tracking` 且 `pose_age_ms<=2000`。
4. RViz2 是可视化诊断，不应成为主链路依赖。Mobaxterm/SSH 终端可直接看 C++ 面板；RViz2 需要 X11 forwarding 或机器狗/NX 本地图形桌面，启动失败时只提示，不阻塞 UI。
5. 浏览器 UI 默认 dry-run、默认 2 秒刷新一次状态。它通过短生命周期 C++ panel 会话读取状态和执行按钮命令，不直接打开新的高频 ROS2/SLAM 订阅。
6. 双目/深度相机只通过 `artifacts/stereo_depth_summary.json` 这类低频摘要进入 UI。原始彩色图、深度图和点云不进入 LLM/UI 边界。安全融合层可以用 300-500 ms 的严格 freshness，UI/LLM 反馈层用 `GO2W_STEREO_STALE_MS` 这类较宽松阈值，只表达“展示是否新鲜”，不直接影响运动许可。UI 应同时显示 `center_distance_m`/`center_window_m` 和 `front_clearance_m`，避免把整图有效率低误读成中心距离不可用。
7. 后续如果做 Qt UI，应复用同一套 C++ core 和 WorldState v1，不新造另一套实时轮询逻辑。

## 后续 Qt/RViz2 形态

建议沿用这套分层，不要直接把 LLM 写死进 RViz2：

```text
Browser/Qt/RViz2 Panel
  -> C++ Operator Core
      -> slam_llm_command_client / ROS2 world_state
      -> go2w_agent_entry.py 或本地 LLM HTTP service
      -> 安全规则和任务队列
```

这样 UI 只负责显示和输入，底层 C++ core 负责状态、弱网摘要、安全开关、命令封装；LLM 推理可以先保持 Python/llama.cpp server，后续再逐步 C++ 化。

## 下一轮面板接线

下一轮优先把面板内部的状态显示从“直接格式化 gateway world_state”改为：

```text
gateway world_state / SSH runtime snapshot
  -> WorldState v1
  -> OperatorDisplayState
  -> terminal panel / future Qt panel
```

显示频率按 `WorldState v1.refresh_policy` 执行：SLAM/雷达原始更新只给内部安全层，`WorldState` 聚合保持 2-5 Hz，UI 显示 1-2 Hz，LLM 反馈事件驱动并限制最小间隔。这样可以减少 UI 和 LLM 对实时闭环的影响。

## 现场注意

- `slam_health_failed`、`localization:not_started` 或 `pose_age_ms>2000` 时，禁止写入“当前位置标定点”。
- 若 `current_pose` 为零点或 `pose_age_ms=-1`，说明没有有效地图坐标。
- 删除旧点并创建新点，只能在 SLAM 有可信 live pose 后执行。

## 多模态巡检 UI 方向

后续 UI 可以逐步扩展为多模态边缘自主机器狗的操作台，但第一版仍应保持“显示和输入”的职责，不直接拼底层命令。建议五个稳定区域：

1. **任务队列**：显示巡检目标、当前 step、下一步动作和是否需要人工确认。
2. **世界状态**：显示 SLAM、定位、LiDAR、双目深度、TI 雷达/NX 节点、弱网模式。
3. **安全策略**：显示 `allow/hold/slow/block/confirm/replan` 以及触发原因。
4. **反馈显示屏**：显示 `operator_feedback` 和 `llm_feedback_results`，用于“现在去哪、到哪了、为什么停”的自然语言反馈。
5. **巡检证据**：显示拍照关键帧、雷达告警、双目摘要和结束报告。

语音、拍照、巡检、双目和雷达都应通过 action registry 或 perception summary bus 进入主链路：

```text
voice/asr -> text command -> TaskQueue IR
capture_keyframe -> keyframe summary
stereo depth -> DepthCameraSummary
NX + TI radar -> RadarDetectionSummary
inspection mission -> multi-step task queue
```

弱网模式下，UI 应明确显示 raw video、dense pointcloud、full log、high-rate images 被降级或丢弃，同时保留语义摘要、安全原因、到达事件、异常告警和关键帧索引。总路线见 `docs/multimodal_edge_autonomous_robot_plan.md`。
