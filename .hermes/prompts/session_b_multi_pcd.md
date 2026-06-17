# Session B: 多PCD建图 + 拼接 + 切换

## 背景
- GO2W 机器狗，Unitree SLAM，XT16 激光雷达，NX 板载
- 项目路径：`/home/unitree/Go2W_SLAM_AI`（本地 `E:\GO2W_0`）
- 当前只有一张 PCD（`/home/unitree/test.pcd`），走廊覆盖不全，A* 找不到路径
- 场地分多个区域（701左侧办公区、701右侧办公区、走廊、702区域）
- 已有 `go2w_multi_map_registry_v2.json` 多 PCD 注册表框架（含过渡锚点）
- 已有 `go2w_accept.sh` 脚本（snapshot、relocate、verify）

## 目标

### 1. 建图流程与脚本（P0）
- **分区方案**（示例，现场可调整）：
  ```
  ┌──────────────┬──────────────┐
  │  PCD 1:       │  PCD 2:       │
  │  701左侧       │  701右侧       │
  │  (尹思园/陈嘉瑜)│  (聂国篱/赵博) │
  ├──────────────┴──────────────┤
  │        PCD 3: 走廊+初始点    │
  └─────────────────────────────┘
  ```
- 每张 PCD 建图脚本：
  1. 启动 SLAM（已有 `start_go2w_slam_stack.sh`）
  2. 机器狗推到该区域 mapping_origin，记录 mapping_origin 位姿
  3. 用 keyDemo 或 Gateway API 开始建图（`start_mapping`）
  4. 推机器狗完整扫描该区域（遥控器建图模式）
  5. 回到起点，结束建图（`end_mapping`），保存 PCD
  6. 立即标定该区域的 mapping_origin 和所有拓扑点（`go2w_accept.sh snapshot`）
- **PCD 命名规范**: `map_701_left.pcd`、`map_701_right.pcd`、`map_corridor.pcd`
- **流程自动化**: 写一个 `scripts/build_multi_pcd.sh`，引导用户按步骤操作，每步给出提示和确认

### 2. PCD 拼接/切换（P0）
- **过渡锚点设计**: 两张 PCD 的重叠区域（如走廊口），各标一个过渡锚点
  ```
  PCD 1 走廊口锚点: transition_701_left_to_corridor
  PCD 3 对应锚点:   transition_corridor_to_701_left
  ```
- **切换逻辑**（`scripts/switch_pcd.sh` 或 Python）:
  1. 暂停当前导航
  2. 保存当前位姿
  3. 重定位到新 PCD（用过渡锚点）
  4. 恢复导航到下一目标
- **多 PCD 注册表**：每张 PCD 的 `go2w_multi_map_registry_v2.json` 包含：
  - `map_id`, `pcd_path`
  - `mapping_origin_anchor_id`
  - `topology_nodes`（只含该区域内节点）
  - `transition_anchors`（跨 PCD 的过渡点对）
  - `relocalization_anchors`

### 3. 路点注册 + 选择（P0）
- **建完每张 PCD 后立刻标点**：
  ```bash
  bash scripts/go2w_accept.sh snapshot <node_id>
  ```
- **路点元数据增强**（在 registry JSON 中）：
  - `pcd_id`: 所属 PCD
  - `open_area`: 附近是否有旋转空间（手动标注）
  - `is_turn_point`: 是否转弯节点
  - `next_hops`: 可直达的下一跳路点列表（手动或程序生成）
- **路点选择程序**（`scripts/select_waypoint.py`）：
  - 输入：当前位置、目标区域
  - 输出：推荐的中间路点序列
  - 基于路点连通图做简单 BFS

### 4. 路点间可达性验证（P1）
- 扩展 `path_validator.py`：
  - `check_path_between_nodes` 支持多 PCD
  - `build_multi_pcd_reachability_graph()` 构建全场地路点连通图
  - 输出：哪些路点对之间可以直接导航（无障碍）、哪些需要中间路点

## 技术约束
- 建图用 keyDemo（`/unitree/module/unitree_slam/bin/keyDemo`）或 Gateway API
- PCD 保存路径务必确认（`/home/unitree/test.pcd` 会被覆盖，改名前备份）
- 建图时 mode 无关紧要（只是扫描，不导航）
- 每张 PCD 建完后立刻验证重定位（`go2w_accept.sh relocate`）
- Python 3.8.10 兼容
- 所有操作脚本加 `--dry-run` 模式

## 产出
1. `scripts/build_multi_pcd.sh` — 多 PCD 建图引导脚本
2. `scripts/switch_pcd.sh` — PCD 切换脚本
3. 更新 `configs/maps/go2w_multi_map_registry_v2.json`
4. 每张 PCD 的拓扑节点注册数据
5. `scripts/select_waypoint.py` — 路点选择器
6. 扩展 `path_validator.py`（多 PCD 可达性分析）
7. `docs/multi-pcd-mapping-guide.md` — 操作文档
