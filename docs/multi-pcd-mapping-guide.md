# 多 PCD 建图与切换操作指南

> 更新于 2026-06-16 | Hermes Agent 编写

## 核心思路

不用提前标定过渡锚点。建图的自然流程就是标定流程：

```
建图1 → 重定位验证 → 推车到图2起点 → snapshot 过渡锚点（在图1帧中）
→ 不退车 → 开始建图2（从同一位置）→ 图2的 mapping_origin 就是过渡锚点
```

## 场地分区（三图中心辐射）

```
                  map_701（中心区）
                 /                \
    map_701_left（左侧）    map_terrace_wc（露台+厕所走廊）
```

- **map_701**：中心枢纽。包含 yin_siyuan, zhao_bo, nie_guoli。
- **map_701_left**：701 左侧。包含 initial_point, chen_jiayu, room_701_corridor。
- **map_terrace_wc**：7 楼露台与厕所走廊。节点待现场定。

---

## 操作流程

### 第一张：建 map_701

```bash
cd /home/unitree/Go2W_SLAM_AI

# 从零开始 — 机器人推到 701 原点
bash scripts/build_multi_pcd.sh map_701 --next map_701_left
```

脚本会引导：
1. 启动 SLAM 栈
2. 推到 mapping_origin → 确认
3. 开始建图 → 推车扫描 701 中心区 + 外走廊
4. 结束建图 → 保存 PCD
5. Snapshot mapping_origin
6. **→ 推到 map_701_left 的建图起点 → snapshot 过渡锚点 `transition_701_to_701_left`**
7. 输出坐标，提示填注册表

> 🔴 **推车扫描时务必覆盖到两个方向的过渡点位置**（map_701_left 起点 + 露台起点），否则过渡锚点 snapshot 时不在 PCD 范围内会导致定位失败。

### 第一张（续）：同样记录露台过渡点（可选）

```bash
# 如果需要，再跑一次 --next map_terrace_wc
bash scripts/build_multi_pcd.sh map_701 --next map_terrace_wc --skip-slam-start
```

> `--skip-slam-start` 因为 SLAM 还活着，不用重启。脚本会跳到过渡锚点录制步骤。

### 第二张：建 map_701_left

```bash
# 机器人已经在 map_701_left 的建图原点（上一步没动过）
bash scripts/build_multi_pcd.sh map_701_left --from-transition
```

`--from-transition` 表示机器人已在原点（跳过定位提示）。建完后 snapshot topology nodes。

### 第三张（如需）：建 map_terrace_wc

```bash
# 如果前面已录了 map_701 → terrace 过渡点：
bash scripts/build_multi_pcd.sh map_terrace_wc --from-transition
```

---

## 建完后：填入注册表

### 1. 过渡锚点坐标

建图时脚本输出的坐标，填入 `go2w_multi_map_registry_v2.json`：

```json
// map_701.transition_anchors[0]:
{
  "anchor_id": "transition_701_to_701_left",
  "connects_to": "map_701_left",
  "reverse_anchor": "mapping_origin_701_left",
  "pose": { "x": 1.5, "y": -2.3, ... }   // ← 从 snapshot 输出填入
}
```

> **reverse_anchor 规则**：始终指向目标图的 `mapping_origin_anchor_id`。因为过渡点就是下一张图的建图原点（同一物理位置），只是在不同坐标帧中坐标不同。

### 2. 拓扑节点坐标

每个图的 topology_nodes 逐个 snapshot 后填入。

### 3. Anchor status

验证通过后改 `"status": "verified"`。

---

## PCD 切换

```bash
# 从 701 切到 701左侧
bash scripts/switch_pcd.sh map_701 map_701_left

# 从 701左侧切回 701
bash scripts/switch_pcd.sh map_701_left map_701
```

脚本自动查找过渡锚点 → relocate 到目标图。

---

## LLM 跨图路径规划

```bash
# 查询从 yin_siyuan 到 chen_jiayu 的路径
python3 scripts/select_waypoint.py yin_siyuan chen_jiayu

# 输出：
# Step 1: Navigate yin_siyuan → transition_701_to_701_left (in map_701)
# Step 2: SWITCH to map_701_left
# Step 3: Navigate mapping_origin_701_left → chen_jiayu_station
```

---

## 连通图速查

```
map_701 ──► map_701_left    (via mapping_origin_701_left)
map_701 ──► map_terrace_wc  (via mapping_origin_terrace)
map_701_left ──► map_701    (via mapping_origin_701)
map_terrace_wc ──► map_701  (via mapping_origin_701)
```

---

## 工具速查

| 脚本 | 用法 |
|------|------|
| `build_multi_pcd.sh <map> --next <next_map>` | 建图 + 录过渡锚点 |
| `build_multi_pcd.sh <map> --from-transition` | 从过渡点建图（机器人已在原点） |
| `switch_pcd.sh <from> <to>` | PCD 切换 |
| `select_waypoint.py <from_node> <to_node>` | LLM 跨图路径规划 |
| `path_validator.py reachability` | 全场地可达性分析 |
