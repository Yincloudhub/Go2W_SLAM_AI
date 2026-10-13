# Unitree SLAM mode 官方语义与开源导航项目 (2026-06-16)

## 官方 mode 定义

来源：Unitree 官方文档 (`doc-cdn.unitree.com/6/111/en/6_111_en`)：

```
"mode": 1,  #Obstacle mode, 1 is stop, 0 is bypass,
             #(In obstacle-avoidance mode, a navigable area with a width
             # of at least 0.8m is required, and obstacles must not be
             # wider than 0.5m)
```

| mode | 官方名称 | 实际行为 | 约束 |
|------|---------|---------|------|
| **0** | bypass（绕障） | **走直线**，只能绕过宽度 < 0.5m 的小障碍，需通道宽 ≥ 0.8m | 遇墙/大障碍傻掉 |
| **1** | stop（停障） | 全局 A\* 路径规划 + 遇未知障碍停机 | keyDemo 实测走通复杂路径 |

**keyDemo 实测**：
- mode=1：`track:[[8888,9999]]` 是"动态路径"哨兵码，⏺ 正常运行
- mode=0：直线导航，遇墙直接乱转 / 无法规划

## 当前 Gateway 强制 mode=0 的问题

`safety_supervisor.cpp` 在 supervised release 下设置 `motion_direction = "unitree_pose_navigation_mode_0"`，
`llm_command_processor.cpp:294` 强制拒绝 mode != 0。这导致所有导航使用 mode=0——只能走直线，
无法通过 PCD 全局规划绕墙。

## 社区开源导航项目（均不依赖 Unitree slam_operate）

| 项目 | Stars | 方案 |
|------|:---:|------|
| jizhang-cmu/autonomy_stack_go2 | 487 | 自研 SLAM + 路线规划 + 避障，纯 Go2 内置传感器 |
| botbotrobotics/BotBrain | 222 | Web UI + 自主导航，3D 打印外壳 |
| andy-zhuo-02/go2_ros2_toolbox | 163 | SLAM Toolbox + Nav2，跑在 Go2 EDU 扩展坞 |
| Sayantani-Bhattacharya/unitree_go2_nav | 110 | RTAB-Map + Nav2，Unitree SDK 高层控制 |
| yehna-kim/unitree-go2-waypoint-nav | 5 | Docker: FAST-LIO + RTABMap + Nav2 + MID360 |

```
https://github.com/andy-zhuo-02/go2_ros2_toolbox
https://github.com/Sayantani-Bhattacharya/unitree_go2_nav
https://github.com/jizhang-cmu/autonomy_stack_go2
https://github.com/yehna-kim/unitree-go2-waypoint-nav
https://github.com/botbotrobotics/BotBrain
```

**共同点**：全部绕过 Unitree slam_operate，自行建图+导航（SLAM Toolbox / RTAB-Map / FAST-LIO + Nav2）。
对我们的选型启示：要么继续用 Unitree SLAM 但切 mode=1 + 重新建连续 PCD，要么走社区主流路线自搭 Nav2。

## `[[8888,9999]]` 不是错误

keyDemo mode=1 导航成功时 Unitree 返回 `track:[[8888,9999]], edgelist:[[9999]]`——
这是动态路径的哨兵/占位码，表示路径规划成功且为动态轨迹，不是空路径或错误。
机器人确实在走，且反复输出 "There are obstacles nearby, please be careful"。
