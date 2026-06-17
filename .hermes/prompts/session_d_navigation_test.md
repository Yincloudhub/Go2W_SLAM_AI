# Session D — 导航闭环测试 + 标点继续

> 2026-06-17 | 承接 Session C (topology_closed_loop)

## 当前状态

### 机器人
- **在线**: 192.168.3.17, unitree/123
- **SLAM**: unitree_slam 运行中, 已重定位到 map_701 (mapping_origin_701)
- **当前位置**: wp_e643bc (701门外走廊中心点, x=-5.56, y=0.08)
- **XT16 geometry**: 刚重启, health=ok, age_ms<1000, 但**不稳定, 长时间静置会停**

### 注册表 (go2w_multi_map_registry_v2.json)
- **map_701**: 6 拓扑节点 + 2 过渡锚点 (坐标全部真实, 非占位)
- **map_701_left**: 0 节点 (待标)
- **map_terrace_wc**: 0 节点 (待标)
- symlink: go2w_real_site_map_registry.json → go2w_multi_map_registry_v2.json

### map_701 节点列表

| node_id | 名称 | 类型 | x | y | yaw |
|---------|------|------|---|---|-----|
| wp_60497f | 尹思园工位 | attributed | 0.343 | 0.242 | 0.055 |
| wp_b738c7 | 701中心过道端点 | corridor_endpoint, rotation_point | 1.924 | -0.285 | -0.060 |
| wp_bbcb4c | 赵博办公室门口 | attributed, corridor_endpoint, rotation_point | 1.806 | -5.455 | -1.600 |
| wp_a684c6 | 杨书洋工位 | attributed | -1.315 | -4.942 | 3.083 |
| wp_2ae043 | 701右侧过末端节点 | rotation_point | -2.310 | -5.279 | 0.251 |
| wp_e643bc | 701门外走廊中心点 | corridor_endpoint, rotation_point | -5.528 | 0.131 | 3.121 |
| transition_701_to_701_left | 过渡→701左侧 | transition | 2.519 | 3.689 | 1.611 |
| transition_701_to_terrace | 过渡→露台 | transition | -5.807 | -7.653 | -1.526 |

### 刚完成的事
1. ✅ 交互式标点脚本 `snapshot_waypoint.py` — 5步流程, 多类型支持, 无占位
2. ✅ map_701 全部 6 节点标定完成 (带属性: person/area/connects/rotation_space)
3. ✅ 过渡锚点坐标从 field_acceptance 快照提取并填入注册表
4. ✅ 全部节点加 `live_verified` tag (导航资格必需)
5. ✅ DEFAULT_MAP_PATH 改为 map_701.pcd
6. ✅ build_multi_pcd.sh relocate bug 修复 (外层 break)
7. ✅ 机器狗环境清理 (临时脚本归档, 旧 CSV 归档)

## 导航测试 — 当前阻塞

**命令**:
```bash
python3 scripts/run_robot_closed_loop.py \
  --command wp_60497f --map-id map_701 \
  --map-path /home/unitree/maps/staging/map_701.pcd \
  --execute --nav-speed-mps 0.2 --nav-mode 0 \
  --robot-password 123 --summary
```

**上次结果**: `STATUS:BLOCKED  REASON: safety disallows navigation: local_obstacle_not_fresh`
- 根因: XT16 geometry sidecar 停了 2+ 小时, 障碍数据过期
- 已修复: 重启 geometry sidecar → summary_health=ok, age_ms<1000

**⚠️ 关键**: `--map-id` 必须用 `map_701` (不是 `go2w_real_site`), 因为 go2w_real_site 的 topology_nodes 是空的。

**导航前检查清单**:
```bash
# 1. XT16 geometry 新鲜?
bash scripts/go2w_xt16_geometry_sidecar.sh health
# 必须: summary_health=ok, age_ms<1000

# 2. 如果停了, 重启:
GO2W_XT16_SUPERVISED_RELEASE=1 GO2W_XT16_GEOMETRY_CALIBRATED=0 \
  bash scripts/go2w_xt16_geometry_sidecar.sh restart

# 3. 确认 SLAM 定位
bash scripts/go2w_accept.sh status
```

## 待完成

### P0: 导航闭环测试
- [ ] 从 wp_e643bc → wp_60497f (尹思园) 单点导航成功
- [ ] 多段导航测试: 尹思园 → 过道中心 → 赵博 → 杨书洋 → 末端

### P1: 继续标点
- [ ] 过渡锚点实际切换测试: 推到 transition_701_to_701_left (2.52, 3.69) → relocate mapping_origin_701_left
- [ ] 标 map_701_left 节点 (initial_point, chen_jiayu, room_701_corridor)
- [ ] 切换到 terrace → 标 terrace_entrance, wc_corridor_end

### P2: 修复
- [ ] `live_verified` tag 应该在 snapshot_waypoint.py 里自动加 (目前只在节点上手动补了)
- [ ] path_validator.py 多处硬编码 `/home/unitree/test.pcd` → 改为读 registry
- [ ] XT16 geometry sidecar 稳定性 — 为什么反复停?

## 切换 PCD 命令

```bash
# 从 701 → 701_left (推车到 transition_701_to_701_left 位置后)
bash scripts/build_multi_pcd.sh relocate --anchor mapping_origin_701_left

# 从 701 → terrace (推车到 transition_701_to_terrace 位置后)
bash scripts/build_multi_pcd.sh relocate --anchor mapping_origin_terrace
```

## 关键文件

| 文件 | 用途 |
|------|------|
| `scripts/snapshot_waypoint.py` | 交互式标点 (python3 scripts/snapshot_waypoint.py) |
| `scripts/build_multi_pcd.sh` | 建图/切换/重定位工具箱 |
| `scripts/run_robot_closed_loop.py` | 导航闭环主脚本 |
| `configs/maps/go2w_multi_map_registry_v2.json` | 多 PCD 注册表 (权威源) |
| `artifacts/snapshot_log.jsonl` | 标点审计日志 |
| `artifacts/field_acceptance/` | 过渡锚点快照 (坐标已提取) |
| `C:\Users\c\Desktop\map_701_waypoints.md` | 8 点可视化文档 |

## 桌面文档

`C:\Users\c\Desktop\map_701_waypoints.md` — 含全部 8 点坐标 + ASCII 地图 + JSON 数据，可给 Codex 做可视化。
