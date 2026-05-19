# 从备份恢复出的当前基线

## 1. 已确认的有效资产

当前工作区 `E:\GO2W_0` 原本是空的，真正可恢复的上下文来自：

- `D:\go2_backup\go2_backup_2026-03-02_173310.tar.gz`
- `D:\go2_backup\go2_state_2026-03-02_173310.tar.gz`
- 需求文档 `C:\Users\c\Desktop\面向弱带宽远程交互的四足机器人语义感知—边缘自治系统(1).md`

这份主备份不是单纯源码包，而是通过下面的命令打出来的整机用户态快照：

```bash
sudo tar -czf ~/backup/go2_backup_$(date +%F).tar.gz /etc /home/unitree 2>/dev/null
```

这意味着：

- `/etc` 和 `/home/unitree` 里的信息可恢复
- 真正位于 `/unitree/module/...` 的工作区不一定被打包进去

## 2. 已恢复出的关键运行事实

### 2.1 ROS2 与 CycloneDDS 环境

`home/unitree/.bashrc` 显示这台机器当时使用：

- `ROS2 Foxy`
- `CycloneDDS`
- `rmw_cyclonedds_cpp`
- 网口 `eth0`

这说明你的 GO2W 侧开发链是按 ROS2 + CycloneDDS 来搭的，而不是纯 Python 脚本直连。

### 2.2 曾实际运行过的 SLAM / Navigation 组合

从 `.bash_history` 和 `.ros/log/.../launch.log` 已确认这台机器至少跑过下面几类流程：

- `hesai_lidar`
- `livox_ros_driver2`
- `lio_sam_ros2`
- `nav2_costmap`
- `dog_control_B1_one`
- `go2_pid_tracing`

其中一组关键启动链如下：

1. `hesai_lidar_node`
2. `dog_control_B1_one`
3. `lio_sam_ros2_IniPoseFromText`
4. `lio_sam_ros2_lidar_checkout`
5. `lio_sam_ros2_dogOdomForReloc`
6. `lio_sam_ros2_imuPreintegration`
7. `lio_sam_ros2_imageProjection`
8. `lio_sam_ros2_featureExtraction`
9. `lio_sam_ros2_globalLocalize`
10. `nav2_costmap_inflation_layer`
11. `nav2_costmap_collision_checker`
12. `CloudToScanForCostmap`
13. `nav2_costmap_static_layer`

另一组链路里还单独启动了：

- `go2_pid_tracing`

这表明当时不是“只有宇树官方导航”，而是已经做过 ROS2 侧的定位 / 栅格 / 路径跟踪集成。

### 2.3 现成的自研节点痕迹

从日志名和历史命令可以确认曾经存在这些自研包或节点：

- `graph_pid_ws`
- `graph_msg`
- `graph_process`
- `occ_grid_mapping`
- `pid_tracing`
- `QT_Server`
- `send_cmd`
- `task`
- `template_matching`
- `go2_control_by_sdk`
- `unitree_interfaces`
- `custom_interface`
- `main_process`
- `path_management`
- `graph_visual`

其中 `main_process` 和 `path_management` 的日志表明，当时系统已经具备：

- 图节点 / 边形式的路径规划
- 子目标逐段下发
- 根据控制反馈推进边级导航

也就是说，真正有价值的创新层其实已经不只 SLAM，本质上是：

- 底层定位 / 建图
- 图级路径规划
- GO2 控制执行
- 自定义状态机

## 3. 已确认的缺口

最重要的缺口是：

- `.bash_history` 明确出现过 `/unitree/module/graph_pid_ws/`
- 但这份备份只打包了 `/etc` 和 `/home/unitree`
- 因此 `/unitree/module/graph_pid_ws` 的源码大概率没有被备份进来

这意味着当前不能假设我们已经拥有完整的老工作区源码。

另外一个可见问题是：

- `CloudToScanForCostmap` 在一次运行中发生过 `exit code -11`

所以就算以后把旧链路全找回来，也不能直接默认它是稳定可复用的。

## 4. 当前最稳的工程判断

不要把“深改宇树官方 SLAM”当成第一步。

当前更稳的路线是：

1. 保留官方 SLAM / Navigation 作为黑盒底座
2. 在其外面定义稳定接口
3. 先做结构化世界状态、任务层、安全监督层
4. 再通过 ROS2 适配器把官方底层和自研模块接起来

这样做有三个直接收益：

- 不会一开始就陷进底层 SLAM 细节
- 可以先在离线日志和仿真输入上验证语义层
- 后面即使换 `LIO-SAM`、官方定位、混合定位，上层逻辑仍可保留

## 5. 当前建议的第一批落地目标

先做下面四件事，而不是先碰官方 SLAM 内核：

1. 固化 `WorldState` 协议
2. 固化 `NavigationSubgoal` 协议
3. 建一个 `SlamNavigationAdapter`
4. 建一个 `SafetySupervisor`

等这四层固定后，再去接：

- 官方位姿 / 地图 / 导航反馈 topic
- YOLO / 多传感器结果
- 远端指令和弱网状态

## 6. 还需要你后续补回的关键资料

如果你手头还有下面任意一个，优先级都很高：

- `/unitree/module/graph_pid_ws` 的完整源码
- `0_unitree_slam.sh`
- `dog_control_B1_one` 的源码
- `pid_tracing` 的源码与配置
- 任何自定义 `launch.py` / `yaml` 文件

如果这些资料能补回来，我们就能从“接口重建”快速升级到“系统复原 + 演进”。
