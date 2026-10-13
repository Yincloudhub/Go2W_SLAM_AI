# Session G — 底盘运动回退与重启

## 背景

Session F 自研了 `nav_core.py` + `sport_bridge.cpp`，绕过 Unitree `slam_operate` 导航，直接调 `SportClient::Move/Stop` 控制底盘。

非运动测试（Gateway 连接、状态读取、数据新鲜度、pipe drain）全部通过。但实机运动测试发现致命问题：

**`SportClient::Move(0, 0, vyaw)` 不生效。** 机器人站立状态下发旋转命令，DDS ACK 正常但几乎不转。更危险的是：`stopMove()` 不是"停止运动"而是触发趴下，导致机器人反复起立-趴下-起立，甚至在某个状态关节失力直接垮倒。

## 根因

我们跳过了对 Go2 底层运动状态机的理解。Go2 的 `SportClient` API 不是裸速度接口——它内部有站立/行走/趴下的状态转换。`Move(0,0,0)` 和 `stopMove()` 的语义不是我们假设的"速度归零"和"急停"。

需要从 Go2 官方 SDK 文档/示例入手，理解正确的运动控制方式，而不是继续在现有假设上打补丁。

## 已完成且应保留

| 组件 | 说明 |
|------|------|
| `waypoint_graph.py` | PCD 连通图 + BFS，离线构建，已可用 |
| `nav_core.py` 的 Gateway reader 部分 | 持久会话 + 位姿/净空缓存 + 数据新鲜度检查，实机验证通过 |
| `nav_core.py` 的 `move_to_xy` 控制循环骨架 | 闭环坐标追踪逻辑正确，只是底盘命令通道有问题 |
| `sport_bridge.cpp` 的 deadman 机制 | 500ms watchdog，安全方向正确 |
| Gateway 重定位流程 | SLAM + PCD 加载已验证 |

## 新 session 要做的事

1. **查 Go2 官方 SDK 文档**：`SportClient::Move` 的正确参数、前置条件、状态机
2. **查 keyDemo 源码**：官方示例如何调用运动 API
3. **确认 Go2 的运动模式**：是否需要 `StandUp()` / `RecoveryStand()` 等前置调用
4. **确定 `stopMove` 的真实语义**：为什么会趴下？是否有替代方案（如 `Move(0,0,0)` 或 `StandDown` vs `StopMove`）
5. **在理解清楚之前，不要发任何底盘命令**
6. **如果 Go2 的 SportClient 不支持安全的持续速度控制**，考虑备选方案（如 ROS2 `/cmd_vel` topic、DDS 其他 API）

## 不要做

- 不要重新启用 Unitree `slam_operate navigate_to_pose`
- 不要引入 Nav2
- 不要在没理解 API 语义的情况下修改 `sport_bridge.cpp`
- 不要远程发 Move 命令
