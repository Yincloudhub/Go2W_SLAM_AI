# GO2W 当前系统状态与实操

更新：2026-06-13 20:05 +08:00

## 一句话结论

P0-1 单一 D435、P0-2 统一 PerceptionContext、P0-3 唯一决定与执行链已经完成
代码部署和机器人无运动验收。当前机器人是“代码已部署、全部测试服务已停止”的
安全停机状态，不是正在运行 SLAM 或自治导航。完整比赛闭环仍缺 P0-4 弱网
journal/ack/补传执行器、XT16 实测标定和最终低速运动验收。

## 版本与分支

```text
implementation commit:             cf68b0ecca828a3881b69134a5b6ef6fd58fad3d
local branch:                       agent/llm-on-robot
origin/agent/llm-on-robot at audit: cf68b0ecca828a3881b69134a5b6ef6fd58fad3d
robot HEAD at audit:                cf68b0ecca828a3881b69134a5b6ef6fd58fad3d
```

GitHub 默认分支仍是旧 `master`，提交停在 `583cf62`（2026-05-19）。当前比赛代码
在 `agent/llm-on-robot`。多个分支不会在同一个工作区自动混合，但 GitHub 页面默认
展示旧 `master` 会造成“代码没更新”的视觉误解。查看当前代码时必须先在分支选择器
切换到 `agent/llm-on-robot`。

机器人同时有两条管理网络：

- 有线 `eth0`：`192.168.123.18`
- Wi-Fi：`192.168.3.17`

这两个地址属于同一台机器人，不是两套代码。比赛执行链在机器人本机使用 `eth0`
访问 Unitree SDK；电脑侧 SSH/tunnel 可以按当前连线选择任一可达地址。

## 地图与拓扑点

实机默认注册表：

```text
configs/maps/go2w_real_site_map_registry.json
map_id=go2w_real_site
topology_nodes=7
```

七个节点是：

```text
initial_point
yin_siyuan_station
chen_jiayu_station
yang_shuyang_station
zhao_bo_office_front
room_701_corridor
nie_guoli_office_front
```

看到两个点时，打开的是开发示例
`configs/maps/go2w_map_registry.example.json`，其中 `test_current_main` 只有两个
示例节点。实机入口、Web UI、Python agent 和 C++ OperatorPanel 默认都使用七点
真实注册表；只有本地示例/仿真脚本默认使用两点文件。

## 当前机器人运行状态

2026-06-13 20:05 只读核查结果：

```text
git worktree: clean
GO2W process: none
D435 owner/reducer: stopped
XT16 geometry producer: stopped
PerceptionContext producer: stopped
SLAM: not started
Gateway: not started
chassis motion: not started
systemd/autostart/cron residue: none
```

磁盘上的历史 artifact 不是在线证据。现场一次性构建 PerceptionContext 时：

```text
d435_depth: offline
d435_yolo: offline
xt16_geometry: invalid (producer offline + calibration unverified)
ti_nx_radar: offline
imu/odometry adapters: offline
```

因此旧 D435 文件即使内部仍写着 `fresh`，也不会被当前 WorldState、Planner、UI
或日志当成在线输入。

## 当前闭环

```text
XT16 / D435 / TI-NX adapters
  -> PerceptionContext v1
  -> WorldState v1
  -> Planner / deterministic semantic router
  -> TaskQueue
  -> MissionDecisionEngine
  -> Python persistent supervised executor
  -> SLAM Gateway
  -> Unitree SDK
```

LLM 只读取低频摘要，不读取原始点云、连续视频或雷达 ADC，也不能直接控制底盘。
Gateway 保持最终运动权威。

## D435 状态

- 硬件当前可枚举：Intel RealSense D435I，序列号 `346222072418`。
- 统一服务是唯一设备 owner，深度 ROI 和 YOLO 共用一次 RGBD capture。
- 深度目标频率 5-10 Hz，resident 档 YOLO 约 3 Hz。
- 旧 `go2w_stereo_depth_sidecar.sh` 和 `go2w_deepyolo_sidecar.sh` 只转发到统一 manager。
- P0-1 真机验证曾在 3 秒内得到 12 个深度序列和 3 个 YOLO 序列。
- 当前服务已停止，因此当前状态是“硬件连接、代码可用、服务离线”。

只读检查：

```bash
bash scripts/go2w_d435_perception_sidecar.sh status
rs-enumerate-devices -s
```

## XT16 点云状态

机身自反射剔除已经实现，不需要再新增第二套过滤器。当前先把转换到机身坐标系后
落在以下矩形内的点删除：

```text
front=0.30 m
rear=0.30 m
half_width=0.30 m
filter_margin=0.05 m
```

输出会记录 `points_excluded_footprint`。D435 只允许保守地缩小前向净空，左右和
后方仍由 360 度 XT16 负责。当前参数仍是临时估计，状态为
`pending_field_measurement`，不能据此授权真实导航。

2026-06-06 已完成前、左、右、后四方向静态箱体验证，证明坐标轴方向正确：
前方箱体净空约 `0.50 m`，移除后约 `2.95 m`，后方箱体簇约 `0.67 m`。该批
artifact 使用旧 footprint 和旧单百分位算法，只能作为方向证据，不能替当前
空间簇/低矮风险算法签发最终标定。

实测机身尺寸不方便时，下一步先把机器人静止放到走廊空旷处，通过本地只读
可视化确认四方向自反射分布：

```powershell
cd E:\GO2W_0
.\.venv\Scripts\python.exe scripts\visualize_xt16_over_ssh.py `
  --host 192.168.123.18 `
  --topic /utlidar/cloud `
  --record-jsonl artifacts\xt16_visual\corridor_baseline.jsonl
```

脚本通过 SSH 在机器人侧临时订阅 PointCloud2，只向本地传输降采样点和当前算法
摘要。它不启动 SLAM、Gateway、D435 或运动。红色点是 footprint 剔除点，青色是
保留的机身高度点，橙色是低矮风险点。

2026-06-13 走廊静止基线已确认：正式 `rslidar` 点云前方净空 `6.0 m`，左右
净空中位数 `1.025/1.003 m`；旧 2 cm 余量保留了一个后部左右对称自回波簇。
同帧扫描显示 5 cm 是刚好去除该簇的最小余量，6-8 cm 不再删除额外点。因此
body-height filter-only margin 更新为 `0.05 m`，名义机身边缘仍为 `0.30 m`。
低矮风险不使用额外 margin，机身外近距离线缆仍保留。

## 完成度

| 阶段 | 状态 |
|---|---|
| P0-1 单一 D435 owner | 完成并验收 |
| P0-2 PerceptionContext / WorldState | 完成并验收 |
| P0-3 MissionDecision / 唯一执行链 | 完成并验收 |
| P0-4 弱网 journal / ack / 补传 | 未实现 |
| XT16 机身尺寸和五场标定 | 未完成 |
| TI/NX live transport | 预留接口，未接实流 |
| IMU/里程计统一运动摘要 | 预留接口，未实现 |
| SLAM + Gateway + 低速真实运动总验收 | 未执行 |

所以当前是“软件主链已收口、现场完整闭环未最终验收”，不能表述为比赛闭环已经
百分之百完善。
