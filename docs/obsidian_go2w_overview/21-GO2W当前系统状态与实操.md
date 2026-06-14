# GO2W 当前系统状态与实操

更新：2026-06-14（上下文归档）

## 一句话结论

P0-1 单一 D435、P0-2 统一 PerceptionContext、P0-3 唯一决定与执行链已经完成
代码部署和机器人无运动验收。当前机器人是“代码已部署、全部测试服务已停止”的
安全停机状态，不是正在运行 SLAM 或自治导航。完整比赛闭环仍缺 P0-4 弱网
journal/ack/补传执行器、XT16 正式 measured ledger 签发和最终低速运动验收。

## 版本与分支

```text
pre-archive implementation commit: 317808436e56495d111402ff5a52ef1a79774c53
local branch:                       agent/llm-on-robot
origin/agent/llm-on-robot at audit: 317808436e56495d111402ff5a52ef1a79774c53
robot HEAD at prior audit:          317808436e56495d111402ff5a52ef1a79774c53
```

归档后的权威提交是包含
`docs/go2w_session_handoff_20260614.md` 的分支 tip；新 Session 必须实时核对
local、origin 和 robot HEAD，不得把上面的归档前哈希当作永久当前值。

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
longitudinal_filter_margin=0.05 m
lateral_filter_margin=0.00 m
```

输出会记录 `points_excluded_footprint`。D435 只允许保守地缩小前向净空，左右和
后方仍由 360 度 XT16 负责。`0.30 m` 是当前已经过静态筛选的有效安全 footprint，
不是精确 CAD 尺寸。标定记录仍为 `pending_field_measurement`，因此不能据此授权
真实导航，但也不再把“重复测量机身、重做前/左/后三场”列为当前下一步。

2026-06-06 已完成前、左、右、后四方向静态箱体验证，证明坐标轴方向正确：
前方箱体净空约 `0.50 m`，移除后约 `2.95 m`，后方箱体簇约 `0.67 m`。该批
artifact 使用旧 footprint 和旧单百分位算法，只能作为方向证据，不能替当前
空间簇/低矮风险算法签发最终标定。

以下本地只读可视化入口已经用于完成走廊基线和成对场景证据，后续只在算法、
安装位姿或机身外廓变化时复用，不作为每个 Session 的重复动作：

```powershell
cd E:\GO2W_0
.\.venv\Scripts\python.exe scripts\visualize_xt16_over_ssh.py `
  --host 192.168.123.18 `
  --topic /unitree/slam_lidar/points `
  --record-jsonl artifacts\xt16_visual\corridor_baseline.jsonl
```

脚本通过 SSH 在机器人侧临时订阅 PointCloud2，只向本地传输降采样点和当前算法
摘要。机器人或雷达重启后，先执行 `sudo scripts/go2w_xt16_ptp.sh start` 并确认
返回 `xt16_ptp=healthy`，再启动 `xt16_driver`。健康状态允许雷达在正常的
`PTPStatus=Tracking/Locked` 间切换，但不允许 `Free Run`。该授时链路是机器人
`eth0` 到 XT16 的本地 PTP，不需要互联网。2026-06-13 重启故障已定位为 XT16
UDP 时间仍停留在 `2020-05-20`，PTP 恢复后正式 `rslidar` 点云恢复约 10 Hz、
约 62k 点/帧。

2026-06-14 无运动重定位复验中，`mapping_origin` 重定位 8/8 通过，随后连续性
10/10 通过；地图为 `/home/unitree/test.pcd`，距锚点约 `0.365-0.385 m`，航向
误差约 `1.3-2.2 deg`，没有发送运动命令。同期发现旧 PTP 参数会因发送时间戳
超时而在进程仍存活时失锁；将等待调整为 `1000 ms` 后连续 120 秒保持
`Tracking/Locked`。driver 初始化期间雷达可能短暂切到 `Free Run` 后自行重新
捕获，因此检查会在 `20 s` 有界窗口内等待连续 5 个健康样本；持续失锁仍然
fail-closed。SLAM 启动入口现默认执行该检查。
脚本拒绝把另一条 `frame_id=utlidar_lidar` 的 `/utlidar/cloud` 当作 XT16，并
对收到的每帧再次校验 `frame_id=rslidar`。
红色点是 footprint 剔除点，青色是保留的机身高度点，橙色是低矮风险点。

2026-06-13 走廊静止基线已确认：正式 `rslidar` 点云前方净空 `6.0 m`，左右
净空中位数 `1.025/1.003 m`；旧 2 cm 余量保留了一个后部左右对称自回波簇。
同帧扫描显示 5 cm 是刚好去除该簇的最小余量，6-8 cm 不再删除额外点。因此
body-height 纵向 filter-only margin 更新为 `0.05 m`，名义机身边缘仍为
`0.30 m`。低矮风险不使用额外 margin，机身外近距离线缆仍保留。

提交 `28340c4` 快进机器人后的第二次 34 帧复验中，左右净空中位数为
`1.024/1.003 m`，后部机身净空 33/34 帧为 `6.0 m`，没有支持成立的后部低矮
风险，证明原对称机身簇已删除且低矮风险路径未被 5 cm margin 吞掉。该次开放
前向没有足够方向支撑，输出为 `null + missing_required_roi:front`，系统保持
stale，没有把“无点”猜成“安全”。因此本次只验收机身边界过滤，不签发 XT16
正式标定。

2026-06-14 又完成一组右侧紧邻设备箱与前移约 `0.5 m` 后开放场景的静止差分。
设备箱场景 13 帧的右/后净空中位数为 `0.093/0.057 m`，
`points_excluded_footprint` 中位数为 `7262`；开放场景 12 帧分别为
`0.464/0.819 m` 和 `913`。关键证据是右侧 `0.30-0.35 m` 带在箱体场景每帧
出现 `245-345` 个点，而开放场景 12/12 帧完全消失。因此这些点是设备箱/线缆，
不是机器狗自回波，横向 5 cm 遮罩会误吞真实近障。当前实现改为前后保留
`0.05 m`、左右使用 `0.00 m` 余量；离线重算后箱体场景正确阻断右/后，开放场景
没有阻断方向。本次仍只是静止边界验收，不代表 XT16 已正式标定或允许真实运动。

实现提交 `599d4cc` 已于 `2026-06-14T10:49:32+08:00` 在机器人上通过 Git
fast-forward 部署。机器人全量 Python `315/315`、Gateway CTest `3/3`、C++
CTest `6/6` 通过。部署后的 12 个连续实时样本序号从 `146` 更新到 `159`，
右/后净空中位数为 `0.463/0.814 m`，每帧总点数中位数为 `62463`，
footprint 剔除点中位数为 `893`，12/12 均无阻断方向。输出始终保留
`uncalibrated_xt16_geometry`，不会提前授权运动。验收结束后 geometry、
`xt16_driver` 和 PTP 已停止，未启动 SLAM、Gateway、D435 或底盘运动。

XT16 对低矮点并非完全忽略：当前 `-0.25 m` 到 `-0.10 m` 为独立
`low_hazard` 带，四个方向可见的线缆/低矮簇会收紧净空。但低于 `min_z_m`、
被机身遮挡或位于底盘正下方的区域仍是盲区。D435 只能补前向视野，不能单独
覆盖底盘下方、左右或后方；真实运动时还必须依赖 Unitree 原生地形/避障能力、
低速起步和人工急停，必要时再增加专用向下传感器。

针对“侧墙较近但导航路径可通行”的误拦问题，`corridor_clearance_v1` 将阈值
改为按名义机身边缘净空分级：

```text
front: pause <0.80 m, conservative <1.50 m
side:  pause <0.20 m, conservative <0.60 m
rear:  pause <0.30 m, conservative <0.50 m
conservative navigation speed cap: 0.20 m/s
```

侧墙不再因为低于 `0.8 m` 就全局暂停。Gateway 是唯一阈值权威，并在保守模式
实际钳制导航速度，而不是只显示提示。stale、未标定、低置信度或缺少方向值仍
失败关闭；因此这项策略修正本身不代表当前已允许真实运动。

## 完成度

| 阶段 | 状态 |
|---|---|
| P0-1 单一 D435 owner | 完成并验收 |
| P0-2 PerceptionContext / WorldState | 完成并验收 |
| P0-3 MissionDecision / 唯一执行链 | 完成并验收 |
| P0-4 弱网 journal / ack / 补传 | 未实现 |
| XT16 轴向/有效 footprint 静态工程验证 | 完成，不重复采集 |
| XT16 正式 measured ledger / verified 标定 | 未签发 |
| TI/NX live transport | 预留接口，未接实流 |
| IMU/里程计统一运动摘要 | 预留接口，未实现 |
| SLAM + Gateway + 低速真实运动总验收 | 未执行 |

所以当前是“软件主链已收口、现场完整闭环未最终验收”，不能表述为比赛闭环已经
百分之百完善。
