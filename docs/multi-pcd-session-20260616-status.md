# 多 PCD 建图 — 2026-06-16 完成状态

> Hermes Agent 整理，基于 2026-06-16 晚 Session

## 已建 PCD

| 图 | PCD 路径 | 大小 | 状态 |
|----|---------|------|------|
| map_701 | `/home/unitree/maps/staging/map_701.pcd` | ~1.2MB | ✅ 已建，重定位验证通过 |
| map_701_left | `/home/unitree/maps/staging/map_701_left.pcd` | ~342KB | ✅ 已建，重定位验证通过 |
| map_terrace_wc | `/home/unitree/maps/staging/map_terrace_wc.pcd` | ? | ✅ 已建，重定位验证通过 |

## 图拓扑

```
                  map_701（中心区：yin_siyuan, zhao_bo, nie_guoli）
                 /                          \
    map_701_left（左侧：initial_point,    map_terrace_wc（露台+厕所：
      chen_jiayu, room_701_corridor）       terrace_entrance）
```

## 过渡锚点（⚠️ 需要填入坐标）

| 过渡 | 锚点 ID | reverse_anchor | 状态 |
|------|--------|---------------|------|
| 701 → 701_left | `transition_701_to_701_left` | `mapping_origin_701_left` | TODO 填坐标 |
| 701 → terrace | `transition_701_to_terrace` | `mapping_origin_terrace` | TODO 填坐标 |
| 701_left → 701 | `transition_701_left_to_701` | `mapping_origin_701` | TODO 填坐标 |
| terrace → 701 | `transition_terrace_to_701` | `mapping_origin_701` ⚠️ | **应该用 transition_701_to_terrace** |

### ⚠️ 已知问题：terrace→701 的 reverse_anchor

当前注册表 `reverse_anchor = "mapping_origin_701"`，但 `mapping_origin_701` 的 pose 是 `(0,0,0)`——机器人不在 701 原点时 relocation 会失败。正确应该是 `"transition_701_to_terrace"`，这个锚点有露台过渡点在 701 帧里的真实坐标（从 mark-transition snapshot 得来）。

**修法**：把 `transition_701_to_terrace` 加入 `go2w_real_site.relocalization_anchors`，status 改为 verified，然后 `reverse_anchor` 指向它。

## 拓扑节点（全部 TODO 填坐标）

| 图 | 节点 ID | 名称 |
|----|--------|------|
| map_701 | yin_siyuan_station | 尹思园工位 |
| map_701 | zhao_bo_office_front | 赵波办公室前 |
| map_701 | nie_guoli_office_front | 聂国力办公室前 |
| map_701_left | initial_point | 初始点 |
| map_701_left | chen_jiayu_station | 陈家宇工位 |
| map_701_left | room_701_corridor | 701室走廊 |
| map_terrace_wc | terrace_entrance | 露台入口 |

## 运行时配置（机器人上）

- `go2w_real_site_map_registry.json` → symlink → `go2w_multi_map_registry_v2.json`
  - 原因：Gateway C++ 硬编码读取旧路径，symlink 让所有脚本透明读 V2
- `go2w_real_site.pcd_path` 在 relocate 前自动同步（脚本处理）
- `go2w_real_site.relocalization_anchors` 在 relocate 前自动合并（脚本处理）

## 下次继续

1. 修改 `transition_terrace_to_701` 的 reverse_anchor
2. 逐个 snapshot 所有 TODO 拓扑节点
3. 把 snapshot 坐标填入注册表
4. 所有锚点 status → verified
5. 测试跨 PCD 导航：`select_waypoint.py yin_siyuan terrace_entrance`
