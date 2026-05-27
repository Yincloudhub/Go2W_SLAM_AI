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
- `scripts/go2w_agent_entry.py`：默认 SLAM 启动脚本改为仓库内脚本。
- `cpp/go2w_operator_panel`：新增 C++ 操作者面板原型。

## C++ 操作者面板职责

第一版不是完整 Qt/RViz2 插件，而是 C++ 终端面板，用来固化底层接口：

- 直接通过 `slam_llm_command_client eth0` 读取 `world_state`。
- 按中文显示定位、SLAM、最近语义点、导航状态、前方净空和安全状态。
- 支持弱网摘要模式，只显示定位、SLAM、最近点、导航、安全。
- 提供 LLM 中文输入口，内部转 UTF-8 base64 后调用 `go2w_agent_entry.py --go-b64 ... --human`。
- 默认不执行运动，必须 `/execute on` 后才会真实下发。

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
/execute on
去赵博办公室门口拍照，然后回尹思园工位
```

## 后续 Qt/RViz2 形态

建议沿用这套分层，不要直接把 LLM 写死进 RViz2：

```text
Qt/RViz2 Panel
  -> C++ Operator Core
      -> slam_llm_command_client / ROS2 world_state
      -> go2w_agent_entry.py 或本地 LLM HTTP service
      -> 安全规则和任务队列
```

这样 UI 只负责显示和输入，底层 C++ core 负责状态、弱网摘要、安全开关、命令封装；LLM 推理可以先保持 Python/llama.cpp server，后续再逐步 C++ 化。

## 现场注意

- `slam_health_failed` 或 `localization:not_started` 时，禁止写入“当前位置标定点”。
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
