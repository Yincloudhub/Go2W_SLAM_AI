# GO2W SLAM 优化与局限总览

> 权威文档，更新于 2026-06-16。
> 整合了 2026-06-15/16 源码审查 + 硬件验证 + Unitree 官方文档的所有 SLAM 结论。
> 本文件是 SLAM 相关决策的唯一入口；旧文档中的矛盾内容以本文件为准。

---

## 1. 当前 SLAM 栈

```
XT16 激光雷达 (外挂, ROS2 /unitree/slam_lidar/points)
      ↓
unitree_slam (官方模块, DDS slam_operate 服务)
      ↓
SLAM_INFO_TOPIC (位姿) + rt/slam_info
      ↓
C++ Gateway (slam_llm_command_client, stdin/stdout JSON)
      ↓
Python 层 (run_robot_closed_loop.py → plan → Gateway → Unitree 1102)
```

DDS 与 ROS2 的分界线：点云走 ROS2，位姿/导航指令走 DDS。Python 不直接碰 DDS。

---

## 2. 已验证的优化（保持项）

### 2.1 mode=0 语义与田字格路点策略

Unitree 官方文档原文（doc-cdn.unitree.com/6/111/en）：

| mode | 名称 | 行为 | 约束 |
|------|------|------|------|
| 0 | bypass | 走直线 + 局部绕小障 | 障碍<0.5m 宽，通道≥0.8m，距离≤10m |
| 1 | stop | 全局 A* 规划 + 遇未知障碍停 | keyDemo 实测可走通复杂路径 |

**策略**：保持 mode=0（Gateway 在 supervised release 下强制），用田字格路点把
拐角拆成多段直线，每段直线内靠 Unitree 局部绕障。不切 mode=1。

**`track:[[8888,9999]]` 哨兵码**：两种含义——
- keyDemo mode=1 导航成功时 = 动态路径占位码，**不是错误**，机器人正常在走；
- mode=0 下出现且机器人不动 = PCD 地图内无连通路径，**需重建 PCD**。

### 2.2 多 PCD 注册表与过渡锚点

`configs/maps/go2w_multi_map_registry_v2.json`（version 2）：
- 每张 PCD 独立 map profile：自己的 mapping_origin + topology_nodes
- 共享 relocalization_anchors
- transition_anchors 标记 PCD 重叠区域切换点
- V1 注册表已归档（`go2w_real_site_map_registry_v1_archived.json`），不混用

### 2.3 重定位锚点管理

- 活跃可用：`mapping_origin`、`yin_siyuan_station`（均 verified）
- 已归档：`initial_point`（ICP 509 失败）、`zhao_bo_office_front`、`room_701_corridor`
- relocate 契约：`initial_pose.name` 必须等于 anchor_id；pose 字段容差 1e-4；
  必须非持久会话模式；锚点冲突检查（active vs archive 同名即拒绝）

### 2.4 PCD 密度标定（路径预检基础）

实测 `/home/unitree/test.pcd`：49,288 点 / 768m²，平均 64 pts/m²。
开阔走廊中心密度可达 53 pts/0.25m 圆 → 默认阈值 15 会全拦。
**结论**：`check_path` 结果只做参考（advisory），不做硬拦截依据。
阈值必须按实际 PCD 标定；自适应网格（40×40）二值化阈值随细胞面积缩放。

### 2.5 XT16 自遮挡过滤

XT16 装在腹部，原地旋转时看到自己的腿/底盘（front=0.07m 等）。
`SELF_OCCLUSION_RADIUS_M=0.15`：clearance < 0.15m 视为"无有效测量"→ 2.0m 开放空间。
**不要把 0.07m 读数当成障碍**——先判断是不是自遮挡。

### 2.6 坐标轴对齐验证（2026-06-16 实机三位置）

yin_siyuan / nie_guoli / mapping_origin 三位置偏差：0.09m / 0.58m / 0.30m。
**坐标系已对齐**。之前的"方向不对"根因是 PCD 未加载到 SLAM（`loc: not_started`），
不是坐标轴问题。修复流程：先确认 SLAM 已定位，再做导航；定位正常仍方向错才怀疑坐标轴。

### 2.7 前向主传感器：D435 优先

`nav_core.py`：前向净空取 D435 立体深度，移除与 XT16 的 min 融合
（XT16 自遮挡读数会污染前向净空）。D435 是前向唯一权威。

### 2.8 supervised release 安全链

唯一策略名 `safe_guard`。速度限制链（取最严格）：
`xt16_supervised_release.json (0.2m/s) → registry pose.speed (0.5) → CLI`。
Gateway 强制 mode=0 是有意设计（有人监督的低速阶段保持 Unitree 原生绕障开着）。

---

## 3. 已知局限（不可修复/需绕开项）

### 3.1 中层避障 API 不可用（硬性限制）

| 尝试 | 结果 |
|------|------|
| `SportClient::SwitchAvoidMode()` (DDS 2058) | 3203 "Api not implement error" |
| `SportClient::FreeAvoid(true)` (DDS 2048) | 3203 |
| ROS2 `/obstacle_avoidance_mode` 服务 | 不存在 |
| SlamGateway 跨服务注册 sport API | 无效 |

**根因**：Unitree 标注 AIR 型号不支持 Free Avoid；此 Go2 固件不支持这两个 sport API。
与 XT16 外挂无关（API 发给运动控制器，不经过激光雷达）。

**后果**：只有两层防护——规划层（mode=0 路径规划）+ 安全层（XT16 safe_guard），
**没有中层实时反应层**。导航前必须 path_validator 预检，否则会直接撞上去。
不要继续追 SwitchAvoidMode/FreeAvoid，这是死路。

### 3.2 mode=0 能力边界

- 只能绕宽度 < 0.5m 的小障碍；墙、工位隔断等大结构绕不过
- 需要通道宽 ≥ 0.8m；目标距离 ≤ 10m
- pose navigation 沿直线去目标（非全局规划）
- 大尺度绕行必须由上层（LLM + path_validator）拆成多段

### 3.3 Unitree 全局 A* 会找"数学合法但物理不可行"路径

PCD 栅格上存在墙角缝隙、工位边缘等数学可通的路径。**不是定位问题、不是坐标问题**——
规划器找到了一条不该存在的路径。对策：path_validator 出发前预检 + LLM 粗粒度地图推理绕道。

### 3.4 PCD 建图累积漂移

从原点走 2m，PCD 在该区域畸变约 1m——漂移被 bake 进地图坐标系。
表现：物理到达目标但系统报告还剩 1m+；RViz2 里位置又对。
修复：多 PCD 分区建图（每区独立 PCD 减小漂移累积）。

### 3.5 initial_point 重定位失败（ICP 509）

`initial_point` 已归档为 `candidate_failed_icp509_20260601`，不能用于重定位。
活跃锚点只有 `mapping_origin` 和 `yin_siyuan_station`。

### 3.6 DDS-ROS2 桥接隐患

Gateway 通过 DDS 读 `rt/slam_info`，但 SLAM 发布的是 ROS2 `/slam_info`，
桥接可能不通。重定位 API 返回 succeed 但 SLAM 可能未实际执行 →
用 `go2w_accept.sh` 验证，不要只看 API 返回值。

### 3.7 外挂硬件遮挡 XT16

背上的 TI 雷达架子进入 XT16 360° 扫描视野 → 左侧持续 0.06~0.09m 读数 →
`left_obstacle_confidence_too_low`。supervised release 下跳过置信度检查绕过。
长期方案：调整架子位置或给 geometry 服务加角度盲区 mask。

### 3.8 社区共识：绕过 slam_operate

主流开源项目（CMU autonomy_stack_go2 487★、go2_ros2_toolbox 163★ 等）
全部绕过 Unitree slam_operate，用 SLAM Toolbox / RTAB-Map / FAST-LIO + Nav2 自建导航。
若后续需全局 A* 绕墙，两条路：① Unitree SLAM 切 mode=1 + 连续 PCD；
② 走社区路线自搭 Nav2。当前阶段维持 ① 的 mode=0 + 田字格拆段。

---

## 4. 决策记录

| 决策 | 结论 | 日期 |
|------|------|------|
| mode 选择 | 保持 mode=0，田字格多段直线 | 2026-06-16 |
| 中层避障 | 不追 SwitchAvoidMode/FreeAvoid（3203 死路） | 2026-06-16 |
| 路径预检 | advisory-only，不做硬拦截（阈值未标定） | 2026-06-16 |
| 坐标轴 | 已验证对齐，勿再怀疑；先查 SLAM 定位状态 | 2026-06-16 |
| 重定位锚点 | 只用 mapping_origin + yin_siyuan_station | 2026-06-16 |
| PCD 策略 | 多 PCD 分区 + 过渡锚点（V2 注册表） | 2026-06-16 |
| 导航架构 | LLM 看图推理多跳 ≠ 硬编码路由表 | 2026-06-16 |

---

## 5. 相关文档索引

| 主题 | 文档 |
|------|------|
| 绕障完整分析 | skill references/gateway-obstacle-avoidance-20260616.md |
| mode 官方语义 | skill references/nav-mode-official-docs-20260616.md |
| 坐标轴诊断 | skill references/pcd-slam-axis-misalignment-20260616.md |
| PCD 阈值标定 | skill references/pcd-threshold-calibration-20260616.md |
| Gateway 重定位契约 | skill references/gateway-relocation-20260616.md |
| SLAM 状态返回 | docs/slam_runtime_state_feedback.md |
| Gateway 流程协议 | docs/slam_gateway_flow_and_protocols.md |
| 多 PCD 建图指南 | docs/multi-pcd-mapping-guide.md |
| 代码架构 | docs/complete-code-architecture.md |

> 本文由 Hermes Agent 整合于 2026-06-16，覆盖 2026-06-15/16 全部 SLAM 实测结论。
