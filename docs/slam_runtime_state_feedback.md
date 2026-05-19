# SLAM 与点云状态返回实施说明

本文档记录当前已经推进完成的 P0 状态返回能力。目标不是把完整点云直接喂给 LLM，而是先把机器狗底层状态整理成稳定、低带宽、可判断的结构化 JSON。

---

## 1. 当前已经实现什么

新增文件：

```text
src/edge_autonomy/runtime_state.py
scripts/slam_runtime_snapshot.py
tests/test_runtime_state.py
```

它们负责：

| 模块 | 作用 |
|---|---|
| `runtime_state.py` | 解析进程、LiDAR 状态、PointCloud2 摘要、重定位 odom |
| `slam_runtime_snapshot.py` | 通过 SSH 只读采集机器狗运行状态 |
| `test_runtime_state.py` | 验证解析逻辑不会轻易改坏 |

当前采集是“一次性快照”，不是常驻服务。这样做风险最低，适合先摸清接口稳定性。

---

## 2. 怎么运行

在本机仓库：

```powershell
cd E:\GO2W_0
python .\scripts\slam_runtime_snapshot.py --host 192.168.123.18 --username unitree --password 123 --map-id test_current_main --map-path /home/unitree/test.pcd --pretty
```

如果不想在命令里写密码，可以设置环境变量：

```powershell
$env:GO2W_SSH_PASSWORD="123"
python .\scripts\slam_runtime_snapshot.py --host 192.168.123.18 --username unitree --map-id test_current_main --map-path /home/unitree/test.pcd --pretty
```

这个脚本只读，不会发导航、不切图、不修改机器狗文件。

---

## 3. 当前真实快照结果

2026-05-17 现场采集到的关键状态：

```json
{
  "health_status": "ok",
  "localization_status": "localized_or_tracking",
  "processes": {
    "unitree_slam": true,
    "xt16_driver": true,
    "slam_keyboard_client": true,
    "slam_llm_command_client": false
  },
  "lidar_state": {
    "alive": true,
    "cloud_frequency_hz": 15.019,
    "imu_frequency_hz": 247.208,
    "cloud_size": 57761,
    "error_state": 0
  },
  "live_pointcloud": {
    "alive": true,
    "topic": "/unitree/slam_lidar/points",
    "frame_id": "rslidar",
    "width": 56014,
    "height": 1,
    "point_step": 22,
    "is_dense": true
  },
  "relocation_odom": {
    "alive": true,
    "topic": "/unitree/slam_relocation/odom",
    "frame_id": "map",
    "child_frame_id": "rslidar",
    "x": -0.161,
    "y": -0.015,
    "z": -0.017,
    "yaw": -0.021
  }
}
```

这说明当前链路已经具备：

```text
雷达在线
点云在线
SLAM 在线
重定位 odom 在线
当前地图坐标下有连续位姿
```

---

## 4. 为什么不直接返回完整点云

当前 `/unitree/slam_lidar/points` 单帧约 5 到 6 万点，频率约 15Hz。完整点云适合 RViz 和底层算法，不适合直接给 LLM 或弱网远端。

上层应该拿到的是：

```json
{
  "pointcloud_alive": true,
  "cloud_frequency_hz": 15.0,
  "cloud_size": 56000,
  "front_clearance_m": 2.4,
  "left_clearance_m": 1.2,
  "right_clearance_m": 0.9,
  "vertical_obstacles": []
}
```

也就是说，点云要先在边缘侧压缩成状态摘要。

---

## 5. 当前状态 JSON 应该怎么给 LLM

LLM 不需要看到 `/unitree/slam_lidar/points` 的原始内容。LLM 应该看到这种摘要：

```json
{
  "map": {
    "map_id": "test_current_main",
    "map_path": "/home/unitree/test.pcd"
  },
  "slam": {
    "health_status": "ok",
    "localization_status": "localized_or_tracking",
    "pose": {
      "x": -0.161,
      "y": -0.015,
      "yaw": -0.021
    }
  },
  "lidar": {
    "alive": true,
    "cloud_frequency_hz": 15.0,
    "cloud_size": 56014,
    "error_state": 0
  },
  "allowed_actions": [
    "navigate_to_verified_node",
    "pause_navigation",
    "request_human_confirm"
  ]
}
```

这才是后续本地 LLM 闭环策略应该消费的输入。

---

## 6. `slam_info` 和 `slam_key_info` 当前现象

初始快照里：

```text
/slam_info: 部分场景有样本
/slam_key_info: 暂无样本
```

后续实际从 `wp_0` 低速导航到 `wp_1` 后，已经确认 `/slam_info` 是关键状态源。它会持续发布：

```text
type=pos_info    当前位姿、pcdName、address
type=robot_data  电池、电机温度、CPU、内存等本体状态
type=ctrl_info   导航控制状态、目标点、是否到达
```

这次没有从 `/slam_key_info` 抓到数据，后续任务状态应优先解析 `/slam_info` 的 `ctrl_info`。

真实到达事件样式如下：

```json
{
  "type": "ctrl_info",
  "errorCode": 0,
  "info": "The navigation point has been reached. Node id is No.9999",
  "data": {
    "is_arrived": true,
    "stateMachine": {
      "state": "FINISHED",
      "vx": 0.0,
      "vy": 0.0,
      "vyaw": 0.0
    },
    "targetNodeName": 9999,
    "targetPose": {
      "x": 3.258938789367676,
      "y": -2.2877299785614014,
      "yaw": -1.6144882440567017
    }
  }
}
```

这意味着任务层判断“到达”的第一规则应是：

```text
/slam_info JSON 中 type == ctrl_info
并且 data.is_arrived == true
或 data.stateMachine.state == FINISHED
```

这意味着 P0 状态返回先按下面优先级实现：

| 状态 | 当前来源 |
|---|---|
| SLAM 是否在线 | 进程 + topic |
| 雷达是否在线 | `/utlidar/lidar_state` + `/unitree/slam_lidar/points` |
| 当前位姿 | `/unitree/slam_relocation/odom` + `/slam_info type=pos_info` |
| 导航任务结果 | `/slam_info type=ctrl_info` |
| 本体状态 | `/slam_info type=robot_data` |
| 备用事件源 | `/slam_key_info`，当前未抓到有效样本 |

---

## 7. 下一步应该推进什么

建议按这个顺序：

```text
1. 把 slam_runtime_snapshot.py 跑稳定，作为现场诊断工具。
2. 在切换 PCD 前后各跑一次快照，确认 map_id、map_path、odom 是否一致。
3. 发一个近距离导航目标，抓 /slam_key_info 的到达事件样本。
4. 把一次性 snapshot 升级成 1Hz 常驻状态发布器。
5. 增加 LocalObstacleSummary，先输出前/左/右/后 clearance。
6. 再加竖直障碍/柱体检测，输出 GeometryObjects。
```

其中第 3 步很关键。只有抓到导航结果事件，任务层才能知道“目标到底到了没有”。

---

## 8. 柱体识别放在哪里

柱体识别建议作为 P1/P2，不要阻塞 P0。

推荐接口：

```json
{
  "type": "geometry_object",
  "object_id": "cylinder_001",
  "shape": "vertical_cylinder",
  "frame_id": "base_link",
  "center": {
    "x": 1.8,
    "y": -0.4,
    "z": 0.6
  },
  "radius_m": 0.12,
  "height_m": 1.1,
  "distance_m": 1.84,
  "stability": "single_frame",
  "risk_level": "medium"
}
```

第一版算法不要做复杂模型，先按几何规则：

```text
裁剪 0.2m 到 6m 范围
去地面点
体素降采样
欧式聚类
筛选窄半径、高竖直结构
输出柱体候选
```

等状态层和通行摘要稳定后，再接这个模块。

---

## 9. 当前验收结论

当前已经可以回答：

```text
SLAM 是否启动？
雷达是否启动？
点云是否在线？
点云频率和点数大概是多少？
当前重定位 odom 是否在线？
当前机器人在 map 坐标系的大概位姿是多少？
```

这一步完成后，项目才具备继续做“导航事件返回”和“点云几何感知”的基础。

---

## 10. 室内速度与噪声策略

真实测试中发现：

```text
0.15m/s 可以安全验证接口，但四足机器人容易表现为踏步挪动，室内噪声明显。
0.35m/s 可以稳定完成 wp_1 -> wp_0，但仍可能有踏步感。
已验证路线建议尝试 0.45m/s，使步态更连续。
```

推荐策略：

| 场景 | 速度 |
|---|---:|
| 第一次验证新点 | `0.15-0.20m/s` |
| 路线已验证但通道较窄 | `0.35m/s` |
| 室内已验证路线且希望减少跺脚 | `0.45m/s` |

如果后续仍然跺脚，应优先检查：

1. 目标点是否太近，导致机器人频繁微调。
2. 目标 yaw 是否和当前朝向差异过大，导致原地转向。
3. 是否刚到点后还在持续纠偏，应在 `ctrl_info is_arrived=true` 后立即 pause 或进入 hold。
4. 是否可以把短距离目标合并，减少碎片化到点。
