# Gateway 重定位协议与陷阱 (2026-06-16)

> 2026-06-16 实机验证总结

## Gateway session 模式

- `--persistent-world-state-session`：**只读**模式。`relocate` 操作被拒绝(`world_state_session_is_read_only`)。
- `--persistent-navigation-session`：需要 session_token + navigation lease，适合运动命令。
- bare 模式（无 flag）：适合一次性 mute 操作（如 relocate）。

## Gateway 命令契约

### `request_id` 必须字段

所有命令 JSON 都必须包含 `"request_id"` 字段，否则 Gateway 返回：

```json
{"accepted":false,"reason":"request_id_required","request_id":null}
```

### relocate 命令合约

```json
{
  "action": "relocate",
  "map_id": "go2w_real_site",
  "map_path": "/home/unitree/test.pcd",
  "anchor_id": "mapping_origin",
  "operator_ack": true,
  "initial_pose": {
    "name": "mapping_origin",    // ← 必须等于 anchor_id！
    "x": 0, "y": 0, "z": 0,
    "q_x": 0, "q_y": 0, "q_z": 0, "q_w": 1
  },
  "request_id": "reloc_1"
}
```

**强制校验**（`navigation_target_authorizer.cpp`）：
1. `initial_pose.value("name")` 必须等于 `anchor_id`（line 348），否则 `relocalization_initial_pose_name_mismatch`
2. 所有 pose 字段 (x,y,z,q_x,q_y,q_z,q_w) 必须与注册锚点一致，容差 1e-4
3. anchor 必须在 `relocalization_anchors`（活跃），不能在 `archived_relocalization_anchors`
4. anchor status 必须以 `verified` 开头

### 活跃重定位锚点

当前注册表 `go2w_real_site_map_registry.json` 中：
- ✅ `mapping_origin` — status=`verified_startup_relocalization_20260601`
- ✅ `yin_siyuan_station` — status=`verified_startup_relocalization_20260615`
- ❌ `initial_point` — 在 `archived_relocalization_anchors`（fail ICP 509）

**用 `go2w_accept.sh relocate mapping_origin confirm` 做重定位。** 不要用 `initial_point`。

### 重复重定位被拒

定位已 healthy 时再次 relocate 会被拒：`localization is already healthy and fresh; recovery relocation is not allowed`。这不是错误——说明 SLAM 已经定位好了。

## Gateway 短命进程陷阱

`communicate()` 调用会关闭 stdin，导致进程退出。**进程一退出，DDS 订阅销毁，所有未到达的消息丢失。** Gateway 的 `get_world_state` 需要持续运行的进程。

**正确做法**：持久会话 + 线程读 stdout（见 `scripts/ws_live.py`）。

## DDS/ROS2 桥接问题

gateway 通过 DDS 订阅 `rt/slam_info`（`ChannelSubscriber`），但 Unitree SLAM 实际通过 ROS2 发布 `/slam_info`。CycloneDDS 应该桥接两者，但需验证：

```bash
# ROS2 侧（数据存在）
ros2 topic echo /slam_info --no-arr | head -3

# DDS 侧（Gateway 能收到吗？用持久会话验证）
python3 scripts/ws_live.py
```

## Coordinate system validation (2026-06-16)

经实机验证——机器人推至 yin_siyuan_station、nie_guoli_office_front、mapping_origin 三个位置，SLAM 实时位姿与注册坐标对比：

| 位置 | SLAM pose | 注册坐标 | 偏差 |
|------|-----------|----------|------|
| yin_siyuan | (1.50, 0.36) | (1.60, 0.36) | ~0.09m |
| nie_guoli | (3.30, -4.03) | (3.25, -3.45) | ~0.58m |
| mapping_origin | (0.22, -0.20) | (0, 0) | ~0.30m |

**PCD 坐标系与 SLAM 坐标系已对齐。** 之前的"导航方向不对"问题是因为 PCD 未加载到 SLAM（`loc: not_started`），不是坐标轴问题。
