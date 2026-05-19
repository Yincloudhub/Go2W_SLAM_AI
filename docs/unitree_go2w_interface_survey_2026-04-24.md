# GO2W 接口摸底记录

目标：只读摸清 `unitree@192.168.3.17` 这台机器狗 NX 当前的接口形态，不改远端代码，不启动新服务，不写远端文件。

时间：2026-04-24  
目标主机：`unitree@192.168.3.17`

---

## 1. 先给结论

当前这台机器上，底层 SLAM 不是你旧备份里那种 `/unitree/module/graph_pid_ws` 源码工作区，而是一个已经部署好的二进制包：

- `/unitree/module/unitree_slam`

真正还带源码形态、适合后续继续摸和接接口的内容，主要在家目录：

- `/home/unitree/cyclonedds_ws`
- `/home/unitree/go2_bridge`
- `/home/unitree/NX_radar_fleet`
- `/home/unitree/go2_mycode`
- `/home/unitree/unitree_sdk2`

这几个目录都 **不是 git 仓库**，更像是直接拷过去的工作目录或快照。

---

## 2. 这次怎么查的

这次摸底分三步：

1. 先确认机器身份、目录结构、ROS 环境。
2. 再确认 `/unitree/module/unitree_slam` 是源码包还是二进制包。
3. 最后确认当前运行态里能直接观察到哪些 topic / service / action / DDS 接口。

这三步的意义不同：

- 第一步回答“东西放哪了”。
- 第二步回答“后面是改源码还是接接口”。
- 第三步回答“现在实际能和系统对话的入口是什么”。

---

## 3. 机器基本信息

远端主机信息：

- 主机名：`ubuntu`
- 用户：`unitree`
- 当前目录：`/home/unitree`
- 系统：`Linux ubuntu 5.10.104-tegra ... aarch64`

ROS 环境：

- `/opt/ros/foxy`
- `/opt/ros/noetic`

`/home/unitree/.bashrc` 里和当前通信相关的关键设置：

- `source /opt/ros/foxy/setup.bash`
- 如果存在，则 `source ~/cyclonedds_ws/install/local_setup.bash`
- `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`
- `export ROS_DOMAIN_ID=0`
- `unset CYCLONEDDS_URI`

解释：

- 当前主链是 `ROS2 Foxy + CycloneDDS`。
- 系统默认走 `rmw_cyclonedds_cpp`。
- `ROS_DOMAIN_ID=0`，后面做适配层时必须对齐。

---

## 4. 为什么说当前 SLAM 是部署包，不是源码工程

### 4.1 目录证据

`/unitree/module` 下面当前只有一套核心目录：

- `/unitree/module/unitree_slam`

它的结构很像发布包，而不像工作区：

- `bin/`
- `lib/`
- `config/`
- `rviz2/`
- `example/`

其中二进制和动态库直接就在那里：

- `bin/unitree_slam`
- `bin/mid360_driver`
- `bin/xt16_driver`
- `bin/keyDemo`
- `lib/libslam_server.so`
- `lib/libgraph_function.so`
- `lib/libgrid_map_core.so`
- `lib/libsport_control.so`

这说明当前机器上的 `unitree_slam` 更接近“已编译部署产物”。

### 4.2 和旧备份的差异

你之前备份里最关键的路径是：

- `/unitree/module/graph_pid_ws`

这台机器当前没有这个目录，取而代之的是：

- `/unitree/module/unitree_slam`

所以现在不能按“恢复旧 graph_pid_ws 工作区”的思路直接接着走。

---

## 5. 现在真正像源码工作区的目录有哪些

### 5.1 `~/cyclonedds_ws`

这是当前最像主开发工作区的目录。

它的 `src` 下有这些包：

- `bridge_pkg`
- `dog_command_center`
- `go2_bridge_pkg`
- `my_imu_subscriber`
- `radar_start_eval`
- `cyclonedds`
- `rmw_cyclonedds_cpp`
- `test_talker`
- `unitree_api`
- `unitree_go`
- `unitree_hg`
- `unitree_ros2_example`

这说明当前机器的“可继续开发层”主要在 `cyclonedds_ws`，而不是 `/unitree/module`。

### 5.2 `~/go2_bridge`

这是一个很明确的桥接脚本目录，不是仓库。

它里面有：

- `start_go2_bridge.sh`
- `stop_go2_bridge.sh`
- `go2_eth_sub.py`
- `go2_wlan_pub.py`
- `eth0.xml`
- `wlan0.xml`

作用是把 `eth0` 和 `wlan0` 上的 DDS/ROS 数据桥接起来。

### 5.3 `~/NX_radar_fleet`

这是另一个明显的应用层目录，偏多机采集/编队控制。

里面主要是：

- `radar_fleet_control.py`
- `radar_station_remote.py`
- `run_radar_fleet_capture.sh`
- `run_radar_fleet_ui.sh`
- `radar_fleet_config.json`

它通过 SSH 去调远端雷达侧 NX，不是底层 SLAM 包。

### 5.4 `~/go2_mycode`

这是你自己的实验代码目录，主要是：

- `gst_reader`
- `yolo_deploy`
- `yolo_cpp`
- `yolo_env`

这部分和你文档里的“语义层”方向是对得上的。

### 5.5 `~/unitree_sdk2`

这是 Unitree 官方 SDK2 源码包目录，带 `example/go2w` 等示例。

它更像 SDK 依赖和示例，不是你自己的主业务目录。

### 5.6 git 状态

这几个目录都查过：

- `~/go2_bridge`
- `~/go2_mycode`
- `~/NX_radar_fleet`
- `~/unitree_sdk2`
- `~/cyclonedds_ws`

结果都是：

- `GIT=NO`

解释：

- 现在机器上的目录是“工作副本”或“拷贝结果”，不是标准 git repo。
- 后面如果要系统化接手，最好把本地整理后的东西重新纳入版本控制。

---

## 6. `unitree_slam` 暴露了哪些直接接口

这一部分最重要，因为后面做适配层就要接这些入口。

### 6.1 来自 `example/src/keyDemo.cpp` 的接口

`keyDemo.cpp` 直接暴露了三类关键信息：

#### 订阅的 DDS topic

- `rt/slam_info`
- `rt/slam_key_info`

#### 调用的服务名

- `slam_operate`

#### 服务 API ID

- `1901`：停止节点
- `1801`：开始建图
- `1802`：结束建图
- `1804`：开始重定位
- `1102`：导航到目标位姿
- `1201`：暂停导航
- `1202`：恢复导航

这组信息非常关键，因为它说明你现在至少有两种接法：

1. 直接走 `unitree_sdk2` 的 Client 调 `slam_operate`
2. 走运行态 DDS topic 做状态观测

### 6.2 `slam_info` / `slam_key_info` 的 JSON 结构

从示例代码能看到：

- `rt/slam_info` 里至少会有 `type == "pos_info"`
- 其中 `data.currentPose` 包含：
  - `x`
  - `y`
  - `z`
  - `q_x`
  - `q_y`
  - `q_z`
  - `q_w`

`rt/slam_key_info` 里至少会有：

- `type == "task_result"`
- `data.is_arrived`
- `data.targetNodeName`

这意味着，如果你只想先接“定位状态 + 任务到达反馈”，其实不必先碰内部 SLAM 算法，先订阅这两个 topic 就够做一版状态适配器。

### 6.3 导航目标的请求结构

示例里的导航请求是一个 JSON，核心字段：

- `data.targetPose.x`
- `data.targetPose.y`
- `data.targetPose.z`
- `data.targetPose.q_x`
- `data.targetPose.q_y`
- `data.targetPose.q_z`
- `data.targetPose.q_w`
- `data.mode`
- `data.speed`

这正好可以映射到你本地仓库里的 `NavigationGoal` / `NavigationSubgoal` 模型。

---

## 7. 配置文件里能确认的输入输出 topic

这部分比运行态还重要，因为它告诉你“系统设计上认为哪些 topic 是正式接口”。

### 7.1 机器人本体状态输入

在 `config/slam_interfaces_server_config/param.yaml` 里，不同机型都引用：

- `odom_topic: rt/dog_odom`
- `imu_topic: rt/dog_imu_raw`

这说明 `dog_odom` 和 `dog_imu_raw` 是 `unitree_slam` 侧消费的底层状态输入。

### 7.2 激光输入

不同雷达配置不一样：

#### Mid360

- `rt/utlidar/imu_livox_mid360`
- `rt/utlidar/cloud_livox_mid360`

或者处理后的：

- `rt/unitree/slam_lidar/imu`
- `rt/unitree/slam_lidar/points`

#### XT16

- `rt/rslidar_points`

或者处理后的：

- `rt/unitree/slam_lidar/points`

### 7.3 Mapping 输出

在 `config/pl_mapping/*.yaml` 里可以确认：

- `rt/unitree/slam_mapping/points`
- `rt/unitree/slam_mapping/odom`

### 7.4 Relocation 输出

在 `config/pl_relocation/*.yaml` 里可以确认：

- `rt/unitree/slam_relocation/points`
- `rt/unitree/slam_relocation/odom`
- `rt/unitree/slam_relocation/global_map`
- `rt/unitree/slam_relocation/local_map`

### 7.5 Grid Map 相关输出

在 `config/gridmap_config/config.yaml` 里可以确认：

- 输入点云：`rt/unitree/slam_lidar/points`
- 输入里程计：`rt/unitree/slam_relocation/odom`
- 输出栅格：`rt/gridmap`

解释：

- 这套系统是“激光前端 -> mapping / relocation -> gridmap”分层的。
- 如果你后面做语义层，优先订阅 `slam_relocation/odom`、`gridmap`、`slam_lidar/points` 会比直接碰内部库稳得多。

---

## 8. 当前运行态实际看到的接口

### 8.1 先说一个关键现象

`ros2 node list` 结果是空的，`ros2 service list` 和 `ros2 action list` 也是空的。  
但 `ros2 topic list -t` 能看到大量 topic。

这说明当前系统里有不少接口是 **bare DDS endpoint**，不是标准 ROS2 graph 里注册出来的节点。

这个现象在 `ros2 topic info --verbose` 里被直接证实了：

- 节点名显示为 `_CREATED_BY_BARE_DDS_APP_`

这件事非常关键，因为它决定了你后面调试时不要只盯着 `ros2 node list`。

### 8.2 当前能看到的运行态 topic

当前系统里能直接枚举到几类 topic：

#### Unitree API / 本体接口

- `/api/.../request`
- `/api/.../response`
- `/lf/lowstate`
- `/lf/sportmodestate`
- `/lowcmd`
- `/sportmodestate`
- `/wirelesscontroller`

#### 当前 SLAM / 激光链路

- `/uslam/frontend/cloud_world_ds`
- `/uslam/frontend/odom`
- `/uslam/localization/cloud_world`
- `/uslam/localization/odom`
- `/uslam/navigation/global_path`
- `/uslam/server_log`
- `/utlidar/cloud`
- `/utlidar/cloud_base`
- `/utlidar/cloud_deskewed`
- `/utlidar/grid_map`
- `/utlidar/height_map`
- `/utlidar/height_map_array`
- `/utlidar/imu`
- `/utlidar/lidar_state`
- `/utlidar/range_info`
- `/utlidar/range_map`
- `/utlidar/robot_odom`
- `/utlidar/robot_pose`
- `/utlidar/server_log`
- `/utlidar/voxel_map`
- `/utlidar/voxel_map_compressed`

#### 旧链路或并行链路痕迹

- `/lio_sam_ros2/mapping/odometry`
- `/qt_command`
- `/qt_add_node`
- `/qt_add_edge`
- `/query_result_node`
- `/query_result_edge`

### 8.3 几个关键 topic 的当前状态

#### `/utlidar/cloud`

- 类型：`sensor_msgs/msg/PointCloud2`
- 发布者数量：`1`
- 订阅者数量：`1`
- 端点类型：裸 DDS

说明：

- 当前激光点云链路是活的。

#### `/uslam/frontend/odom`

- 类型：`nav_msgs/msg/Odometry`
- 发布者数量：`0`
- 订阅者数量：`1`

说明：

- 前端里程计这一路现在看起来没有上游在发，至少当前时刻如此。

#### `/qt_command`

- 类型：`unitree_interfaces/msg/QtCommand`
- 发布者数量：`1`
- 订阅者数量：`0`
- 端点类型：裸 DDS

说明：

- 图导航 / Qt 交互链路还有残留接口。
- 但它当前不是完整的 ROS2 节点通信链，而更像底层 DDS 端点直接发。

---

## 9. 一个需要特别注意的矛盾点

`/unitree/module/unitree_slam/config/dds_config.json` 里仍然出现了旧路径：

- `/home/unitree/graph_pid_ws/src/slam_server_interfaces/config/cyclonedds.xml`

但当前机器上主目录已经不是 `graph_pid_ws`。

这意味着至少有一种可能：

1. 这是历史遗留配置，没有实际被当前进程使用。
2. 部分部署脚本仍沿用旧工程的配置习惯。

工程判断：

- 这类配置不要直接信任。
- 后面接适配层时，应以“当前运行态 topic + 当前实际被读取的配置文件”作为准绳，而不是只看单个 json。

---

## 10. `go2_bridge` 在做什么

这个目录值得单独写，因为它很可能是你后面把多网口、弱网链路和语义回传接起来的切入点。

### 10.1 启动方式

`start_go2_bridge.sh` 会起两个进程：

1. `go2_eth_sub.py`
2. `go2_wlan_pub.py`

并且对两个进程分别切换：

- `CYCLONEDDS_URI=file://.../eth0.xml`
- `CYCLONEDDS_URI=file://.../wlan0.xml`

这说明它在按网卡拆 DDS 域接入。

### 10.2 它订阅什么

`go2_eth_sub.py` 订阅：

- `/lf/lowstate`
- `/lf/sportmodestate`

然后把这些数据转成 JSON，经 UDP 发到：

- `127.0.0.1:15000`（默认）

### 10.3 它发布什么

`go2_wlan_pub.py` 把 UDP 收到的 JSON 再发布成：

#### 原始 topic

- `/go2_bridge/raw/lowstate_json`
- `/go2_bridge/raw/sportmodestate_json`
- `/go2_bridge/raw/lowstate`
- `/go2_bridge/raw/sportmodestate`

#### 标准 ROS2 topic

- `/go2_bridge/imu`
- `/go2_bridge/joint_states`
- `/go2_bridge/odom`
- `/go2_bridge/twist`

工程意义：

- 如果你后面要做“弱带宽远程语义交互”，`go2_bridge` 这种模式很有参考价值。
- 它已经展示了一种“把 Unitree 原始状态先规整成稳定中间层”的方法。

---

## 11. `NX_radar_fleet` 在做什么

这个目录不是 SLAM 本体，而是“多站点远端采集控制层”。

它的关键点：

- 通过 SSH 控远端雷达侧 NX
- 配置文件是 `radar_fleet_config.json`
- 默认控制侧 IMU topic 是 `/utlidar/imu`
- 默认会 source：
  - `/opt/ros/foxy/setup.bash`
  - `~/cyclonedds_ws/install/setup.bash`

说明：

- 这个目录证明当前系统已经在往“边缘自治 + 多机协同采集”方向走。
- 你的论文方向和这部分是同向的，但它不是底层 SLAM 包。

---

## 12. 这次摸底后，接口层应该怎么分

后面如果你要在当前项目里接适配层，我建议直接按下面四层来拆。

### 12.1 本体状态层

先接：

- `/lf/lowstate`
- `/lf/sportmodestate`
- `rt/dog_odom`
- `rt/dog_imu_raw`

### 12.2 激光 / SLAM 层

优先观察：

- `/utlidar/cloud`
- `/utlidar/imu`
- `/utlidar/robot_pose`
- `/utlidar/robot_odom`
- `/uslam/localization/odom`
- `/uslam/navigation/global_path`

### 12.3 图导航 / 任务层

优先观察：

- `/qt_command`
- `/qt_add_node`
- `/qt_add_edge`
- `/query_result_node`
- `/query_result_edge`
- `rt/slam_info`
- `rt/slam_key_info`
- `slam_operate`

### 12.4 桥接 / 弱网层

优先观察：

- `/go2_bridge/raw/*`
- `/go2_bridge/imu`
- `/go2_bridge/joint_states`
- `/go2_bridge/odom`
- `/go2_bridge/twist`

---

## 13. 当前最稳的开发判断

当前不建议先尝试“改 `unitree_slam` 本体”。

理由很直接：

1. 它是部署好的二进制包，不是现成源码工作区。
2. 当前机器上的真正源码层在 `~/cyclonedds_ws`、`~/go2_bridge`、`~/NX_radar_fleet`。
3. 运行态里大量接口是 DDS 端点，不是标准 ROS2 节点，直接改底层的调试成本会很高。

更稳的做法是：

1. 先做一个 `SlamNavigationAdapter`
2. 先把 `rt/slam_info`、`rt/slam_key_info`、`slam_operate` 接出来
3. 再把 `/utlidar/*`、`/uslam/*` 接成 `WorldState`
4. 最后再决定要不要碰二进制底层外面的桥接脚本

---

## 14. 下一轮建议怎么继续摸

如果下一轮还坚持只读，我建议按下面顺序做：

1. 先把 `rt/slam_info` 和 `rt/slam_key_info` 的真实 JSON 抓一份样本
2. 再把 `slam_operate` 的请求/响应抓一份样本
3. 然后确认 `/utlidar/*` 和 `/uslam/*` 各 topic 的发布频率与字段稳定性
4. 最后再看 `qt_command` 这条链是不是还真正参与导航

原因：

- 这四步能直接决定适配层怎么写。
- 到这一步之前，都还不需要改远端任何代码。

---

## 15. 这次摸底里最值得记住的三句话

1. 当前机器上的 SLAM 主体是 `/unitree/module/unitree_slam` 二进制部署包，不是旧的 `graph_pid_ws`。
2. 当前真正可继续开发的源码层在 `~/cyclonedds_ws`、`~/go2_bridge`、`~/NX_radar_fleet`、`~/go2_mycode`。
3. 当前系统大量接口是 bare DDS endpoint，所以调试时不能只看 `ros2 node list`。
