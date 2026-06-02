---
created: 2026-05-15
updated: 2026-05-15
status: draft
type: design
tags:
  - 机器狗/SLAM
  - 机器狗/PCD
  - 机器狗/重定位
  - 机器狗/拓扑地图
  - 机器狗/SLAMGateway
---

# SLAM 地图与重定位子系统

## 当前底层事实

当前 GO2W 的 SLAM 主体是：

```text
/unitree/module/unitree_slam
```

它更像官方二进制部署包，不是适合第一阶段直接修改的源码工程。当前应优先接外部接口：

```text
rt/slam_info
rt/slam_key_info
slam_operate
/utlidar/*
/uslam/*
```

## 关键接口

| 接口 | 作用 | 内部映射 |
|---|---|---|
| `rt/slam_info` | 当前位姿 JSON | `CurrentPose`、`LocalizationState` |
| `rt/slam_key_info` | 任务结果、是否到达 | `NavigationTaskState` |
| `slam_operate 1102` | 目标位姿导航 | `submit_navigation_goal()` |
| `slam_operate 1201` | 暂停导航 | `pause_navigation()` |
| `slam_operate 1202` | 恢复导航 | `resume_navigation()` |
| `slam_operate 1804` | 重定位 | `start_relocation()` |

详见：

- [[slam接口/06-Unitree本地SLAM接口含义速查]]

## PCD 地图边界

`PCD` 是几何地图，不是万能定位索引。

必须区分：

```text
map_loaded
relocation_required
relocating
localized
degraded
lost
map_mismatch
```

不能假设：

```text
只要机器人在 PCD 覆盖范围内，就一定能自动定位。
```

更稳的策略：

```text
地图加载后，必须通过上次位姿、人工初始点或重定位锚点完成定位验证。
定位可信后才允许导航。
```

## 拓扑点与重定位锚点

拓扑点用于任务规划：

```text
701_center
701_door_inside
corridor_patrol_a
```

重定位锚点用于地图切换和定位初始化：

```text
701_door_inside_anchor
corridor_701_door_outside_anchor
```

它们可以重合，但语义不同。

## 701 到走廊示例流程

```text
1. 用户说：去 701 外面的走廊巡视。
2. MapManager 选择 701.pcd。
3. SLAM Gateway 加载或确认 701 地图。
4. LocalizationManager 检查定位是否已验证。
5. 如果未验证，使用 701 锚点或人工初始位姿重定位。
6. localized 后，导航到 701_door_inside。
7. 到达门口后暂停。
8. MapManager 切换 corridor_7f.pcd。
9. 使用 corridor_701_door_outside_anchor 重定位。
10. 走廊定位成功后继续巡视。
```

## 第一阶段开发目标

先完成：

```text
parse_slam_info()
parse_slam_key_info()
get_slam_health()
submit_navigation_goal()
pause_navigation()
resume_navigation()
get_localization_state()
```

验收标准：

1. 能读当前位姿。
2. 能判断位姿是否过期。
3. 能知道导航是否到达。
4. 能短距离低速导航。
5. 能在定位异常或障碍物出现时暂停。
