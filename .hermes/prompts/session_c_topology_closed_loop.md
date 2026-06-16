# Session 交接 — 2026-06-16 多 PCD 建图

> Hermes Agent 整理

## 已完成

| 项目 | 状态 |
|------|------|
| 三张 PCD 建成 | ✅ map_701 (1.2MB), map_701_left (342KB), map_terrace_wc |
| 三图重定位全通 | ✅ |
| 注册表 V2 三图中心辐射拓扑 | ✅ |
| 建图/切换工具链 | ✅ build_multi_pcd.sh, switch_pcd.sh, select_waypoint.py |
| 节点类型框架 | ✅ attributed / corridor_endpoint / rotation_point |
| Gateway 兼容 | ✅ symlink, sync pcd_path, map_id 全部改用 go2w_real_site |

## 待完成（下次 Session）

### P0 — 标点 + 填坐标
- [ ] 逐个 snapshot 所有 TODO 拓扑节点（见 `docs/multi-pcd-session-20260616-status.md`）
- [ ] 过渡锚点填入 mark-transition snapshot 坐标
- [ ] 所有锚点 status → verified

### P1 — 过渡锚点修复
- [ ] `transition_terrace_to_701.reverse_anchor` 改为 `transition_701_to_terrace`（当前指向 mapping_origin_701，pose 不对）
- [ ] 把 `transition_701_to_terrace` 加入 go2w_real_site.relocalization_anchors（含真实坐标）

### P2 — 闭环导航
- [ ] 测试同图导航：`run_robot_closed_loop.py --command yin_siyuan --map-id go2w_real_site --map-path .../map_701.pcd`
- [ ] 测试跨图路径：`select_waypoint.py yin_siyuan terrace_entrance`
- [ ] 验证 `map_registry.py` 的 navigate/relocate 命令 Gateway 兼容（已修复 map_id）

### P3 — 审阅
- [ ] `run_robot_closed_loop.py` 的 `DEFAULT_MAP_PATH` 仍是旧 `/home/unitree/test.pcd`
- [ ] `mission_decision.py` 多 PCD 兼容性
- [ ] Journal 恢复逻辑是否阻塞新导航

## 关键文件

| 文件 | 用途 |
|------|------|
| `scripts/build_multi_pcd.sh` | 原子建图子命令 |
| `scripts/switch_pcd.sh` | PCD 切换 |
| `scripts/select_waypoint.py` | BFS 跨图路径 |
| `src/edge_autonomy/path_validator.py` | 多 PCD 可达性 |
| `src/edge_autonomy/map_registry.py` | 注册表加载 + Gateway 命令构造 |
| `configs/maps/go2w_multi_map_registry_v2.json` | 三图注册表 |
| `docs/multi-pcd-session-20260616-status.md` | 当前状态快照 |
| `docs/multi-pcd-mapping-guide.md` | 操作手册 |

## 机器人配置

- `go2w_real_site_map_registry.json` → symlink → `go2w_multi_map_registry_v2.json`
- PCD 暂存：`/home/unitree/maps/staging/`
- 备份：`go2w_real_site_map_registry.json.bak`

## 本次修复的 Bug 清单

1. `gateway_cmd()` Python 内联 JSON true → NameError（改为 sys.argv[1]）
2. Gateway 返回 20+ 嵌套 dict，objs[-1] 是 sensor health 不含 accepted
3. Gateway 硬编码 map_id="go2w_real_site"（全部改用此值）
4. Gateway 校验 pcd_path 必须匹配注册表（relocate 前自动 sync）
5. go2w_accept.sh 不兼容多 PCD（symlink 绕过）
6. switch_pcd.sh reverse_anchor 推导错误（改为读字段）
7. switch_pcd.sh next() StopIteration（加 default）
8. map_registry.py navigate/relocate 返回 V2 map_id 而非 go2w_real_site
