# GO2W 晚间串行任务闭环实测 Runbook

更新时间：2026-05-25

## 目标

今晚优先验证一句中文复合命令能否完成完整语义闭环：

```text
去701门外走廊拍照，然后回尹思园工位
```

期望链路：

```text
中文输入
-> UTF-8/base64 安全传输
-> registry 语义点匹配
-> task_queue 串行任务
-> 每段 gateway preflight
-> navigate_to_pose
-> 到点 pause
-> capture_keyframe 事件
-> 返回尹思园工位
-> brief / JSON / CSV 日志
```

## 上电后准备

进入机器狗仓库并更新代码：

```bash
cd /home/unitree/go2w_slam/go2w_edge_autonomy
git checkout agent/llm-on-robot
git pull
```

启动并检查 SLAM：

```bash
PYTHONPATH=src python3 scripts/go2w_agent_entry.py --ensure-slam --brief
PYTHONPATH=src python3 scripts/go2w_agent_entry.py --status --brief
```

如果定位未就绪，先用当前可信点重定位；例如机器狗在初始点附近：

```bash
PYTHONPATH=src python3 scripts/go2w_agent_entry.py --current-node initial_point --go "回初始点" --dry-run --brief
```

## 不运动验证

先干跑复合任务：

```bash
PYTHONPATH=src python3 scripts/go2w_agent_entry.py --go "去701门外走廊拍照，然后回尹思园工位" --dry-run --brief
```

如果终端中文不可靠，先编码：

```bash
python3 scripts/go2w_encode_command.py --mode go "去701门外走廊拍照，然后回尹思园工位" --pretty
```

然后执行输出里的 `--go-b64` 命令。

干跑必须看到：

```text
plan_mode = mapped_navigation
task_queue_targets = room_701_corridor|yin_siyuan_station
user_reply = 已解析为2个目标的串行任务：701门外走廊→尹思园工位
blocked_reason = dry run; pass --execute to send queued commands
```

## 低速真实执行

你在旁边拿遥控器兜底后再执行：

```bash
PYTHONPATH=src python3 scripts/go2w_agent_entry.py --go "去701门外走廊拍照，然后回尹思园工位" --execute --brief --arrival-monitor-s 35
```

默认已经是低速地形模式：

```text
nav_speed_mps = 0.3
nav_mode = 0
arrival_distance_m = 0.25
```

紧急暂停：

```bash
PYTHONPATH=src python3 scripts/go2w_agent_entry.py --pause --brief
```

## 当前限制

- `capture_keyframe` 目前默认记录语义事件；如果要接真实拍照，需要传 `--capture-command`，例如后续接相机截图脚本。
- `room_701_corridor`、`zhao_bo_office_front` 等点仍带 `needs_calibration` 时，真实导航需要人工观察误差。
- 队列执行已经逐段 preflight，但不是全局路径规划器；避障和底盘安全仍由 SLAM/网关/遥控器兜底。
- 如果到点误差大，先不要扩展更多任务，优先校准该点 live pose。

## 日志检查

默认日志：

```text
artifacts/robot_runs/go2w_agent_*.json
artifacts/robot_runs/go2w_agent_runs.csv
```

重点看：

```text
semantic_trace.matched_targets
planner.task_queue
queue_execution.events
execution.blocked_reason
target_distance_from_robot_m
final_distance_m
```
