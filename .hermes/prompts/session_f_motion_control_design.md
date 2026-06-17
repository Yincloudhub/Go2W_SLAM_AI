# Session F — GO2W 自研运动控制系统设计

> 2026-06-17 | 承接 Session E (nav_review)

---

## 1. 背景与前因

### 1.1 为什么放弃 Unitree 导航

Session D 实测发现：Unitree `slam_operate` 的 `navigate_to_pose` API（DDS 1102）在 PCD relocate 后**无法驱动机器人移动**。mode=0/1 均返回 `[[8888,9999]]` 哨兵码（plan + resume succeed，但机器人原地不动）。keyDemo 建图期间能导航，存盘 PCD 后却不能——推测 Unitree SLAM 全局 A* 路径规划仅在建图模式（active mapping）下可用，relocate 模式只提供定位、不提供 occupancy grid 给规划器。

**决策：放弃 `slam_operate` 导航，自研底盘控制。**

### 1.2 已有基础（Session D/E 成果）

| 组件 | 文件 | 功能 | 状态 |
|------|------|------|------|
| PCD 连通图 | `scripts/waypoint_graph.py` (262行) | 从注册表加载 waypoint → PCD 密度采样预验每对路点直线是否穿墙 → 输出可通边 + BFS 路由 | ✅ 可用 |
| 底盘控制 | `robot/.../src/sport_bridge.cpp` (115行) | stdin JSON → Unitree SportClient::Move(vx,vy,vyaw) / Stop()，绕过 slam_operate | ✅ 已编译 |
| 闭环导航 | `scripts/waypoint_nav.py` (282行) | SLAM 位姿 → 最近路点 → BFS → 逐 hop（转体→前进→到达检测） | ⚠️ 有结构性缺陷 |
| PCD 地图 | `staging/map_701.pcd` 等 3 张 | ~10 万点/张，中心辐射拓扑 (701→701_left, 701→terrace) | ✅ |
| 路点 | 注册表 topology_nodes | 10 点（6 拓扑 + 4 过渡），已标定 | ✅ |
| 感知 | XT16 geometry sidecar | 前/左/右/后四向净空，250ms 刷新 | ✅ |
| 定位 | SLAM relocate | Gateway 持久会话读位姿 (x, y, yaw) | ✅ |

### 1.3 用户顶层设计理念

用户明确提出运动控制的核心框架：

1. **路点不需要全连通**：不是 10 个点的完全图——通过路点**链式串联**盘活整个地图。`waypoint_graph` 的 PCD 预验已经保证了只有物理可达的边才进图。
2. **LLM 提取两种意图**：
   - **运动意图（Motion）**："去哪里"——目标路点
   - **行为意图（Behavior）**："做什么"——拍照、TTS 语音、等待
3. **SLAM 提供定位，但底盘控制自己做**：绕过 Unitree 导航 API
4. **闭环公式**：理解栅格地图 → 路点串联 → 风控 → 完成闭环

---

## 2. 当前架构问题诊断

### 2.1 `waypoint_nav.py` 的四个结构性缺陷

| # | 问题 | 根因 | 后果 |
|---|------|------|------|
| ① | **sport_bridge 每次新进程** | `send_sport()` 用 `Popen + communicate`：启动 sport_bridge → 发 move(duration_ms=N) → C++ 侧 sleep(N) → stop() → 进程退出。若进程在 sleep 期间崩溃，stop() 永远不执行 | 机器人可能不停，稳定性黑洞 |
| ② | **Gateway 位姿每次都重建会话** | `get_slam_pose()` 每次启动 Gateway `--persistent-world-state-session` → 等 3 秒 DDS 就绪 → 读一次 → kill。一个 3-hop 导航起 9+ 次 Gateway | 慢（每次 3-8 秒开销），DDS 订阅反复抖动 |
| ③ | **运动全是开环** | 旋转：`duration = diff/ROTATE_SPEED` → 猜一个时间 → sleep → 不管转没转到。前进：`duration = dist/speed` → 猜一个时间 → sleep → 不管走没走到 | 走不准，累积误差 |
| ④ | **运动期间零风控** | `move_forward()` 只有 `send_sport` + `sleep`，不轮询 XT16。只在 hop 之间才读一次位姿 | 撞了才知道，急停没有用 |

### 2.2 根因总结

四个问题的根因是同一个：**缺少一个跨 hop 存活的"导航会话"**。当前设计把每个运动原语（读位姿、旋转、前进）都实现为独立的短命子进程，每次清理状态、下次重建。

---

## 3. 目标系统架构

### 3.1 整体分层

```
┌─────────────────────────────────────────────────────────┐
│                      用户 / LLM                          │
│  "去露台叫杨书洋回来"                                      │
└──────────────────────┬──────────────────────────────────┘
                       ▼
┌─ Intent Parser (agent_intent.py) ───────────────────────┐
│ LLM 自然语言 → 结构化意图                                  │
│ {                                                        │
│   motion:   {target: "terrace_wc_entry"},                │
│   behavior: {action: "tts", text: "杨书洋回来"}           │
│ }                                                        │
└──────────────────────┬──────────────────────────────────┘
                       ▼
┌─ Route Planner (waypoint_graph.py) ─────────────────────┐
│ 输入: target_waypoint_id                                  │
│ 输出: BFS hop 序列 [wp_a, wp_b, wp_c]                    │
│ 离线构建: PCD 密度采样预验边 (waypoint_graph.json)         │
└──────────────────────┬──────────────────────────────────┘
                       ▼
┌─ Motion Controller (waypoint_nav.py) ───────────────────┐
│ 输入: hop 序列 + NavigationSession                        │
│ 逻辑: 逐 hop → 转体对准 → 闭环前进（带风控）→ 到达检测     │
│ 不直接操作底盘——全部通过 nav_core 的原语                   │
└──────────────────────┬──────────────────────────────────┘
                       ▼
┌─ Navigation Core (nav_core.py) ⬅ NEW ──────────────────┐
│ 持有一个 sport_bridge (持久进程) + 一个 Gateway (持久进程) │
│                                                          │
│ 闭环运动原语:                                             │
│   .get_pose()        → (x, y, yaw)  从 Gateway 读       │
│   .rotate_to(yaw)    → 循环读 yaw 直到对准               │
│   .move_ahead(dist)  → 闭环前进 + XT16 风控              │
│   .front_clearance() → float         从 Gateway 读       │
│   .stop()            → 急停                              │
│                                                          │
│ 风控策略（内嵌于 move_ahead 循环中）:                      │
│   front ≥ 0.50m  →  全速 0.20 m/s                       │
│   0.30 ≤ front < 0.50m → 降速 0.10 m/s（不停）          │
│   front < 0.30m  →  stop → 返回 BLOCKED                 │
│   SLAM loc:lost  →  stop → 返回 LOST                    │
└──────┬──────────────────────┬───────────────────────────┘
       ▼                      ▼
┌──────────────┐    ┌──────────────────┐
│ sport_bridge │    │  Gateway (只读)   │
│ (持久stdin)  │    │  world_state     │
│ Move/Stop    │    │  位姿+XT16净空   │
└──────┬───────┘    └────────┬─────────┘
       │                     │
       ▼                     ▼
   Unitree SDK          SLAM (rt/slam_info)
   DDS 1001/1002        XT16 (perception_context)
```

### 3.2 数据流：一次完整导航

```
"去露台叫杨书洋回来"
  │
  ├─ 1. agent_intent.parse() → {motion, behavior}
  │
  ├─ 2. waypoint_graph.route(current_waypoint, "terrace_wc_entry")
  │      → [wp_60497f, wp_e643bc, terrace_wc_entry]
  │
  ├─ 3. with NavigationSession() as nav:
  │      for hop in route:
  │        ├─ bearing = nav.bearing_to(hop)
  │        ├─ nav.rotate_to(bearing)         ← 闭环: 读 yaw → 调整 → 对准
  │        ├─ result = nav.move_ahead(dist)  ← 闭环: 读位姿+XT16 → 前进/降速/停
  │        └─ if result != ARRIVED: break
  │
  └─ 4. agent_intent.execute_behavior(behavior)
         → TTS "杨书洋回来"
```

---

## 4. 详细组件设计

### 4.1 `nav_core.py` — NavigationSession（核心新建，~200行）

**职责**：持有一个 sport_bridge 持久进程 + 一个 Gateway 持久进程，提供闭环运动原语。上层（waypoint_nav）不直接碰进程管理。

```python
class NavigationSession:
    """
    Usage:
        with NavigationSession() as nav:
            nav.rotate_to(1.57)
            result = nav.move_ahead(2.5)
    """
    
    # ── 生命周期 ──
    def __enter__(self):
        # 1. 启动 sport_bridge 持久进程 (stdin PIPE)
        # 2. 启动 Gateway --persistent-world-state-session
        # 3. 启动 reader 线程读 Gateway stdout → 更新 _latest_pose, _latest_clearance
        # 4. 等 DDS 就绪（最多 5s）
        return self
    
    def __exit__(self):
        self.stop()
        self._bridge.stdin.close(); self._bridge.terminate()
        self._gateway.stdin.close(); self._gateway.terminate()
    
    # ── 查询 ──
    def get_pose(self) -> (float, float, float):
        """从 Gateway reader 线程的最新缓存读 (x, y, yaw)"""
        return self._latest_pose
    
    def front_clearance(self) -> float:
        """从 Gateway world_state 缓存读前向净空（米）"""
        return self._latest_front
    
    # ── 运动原语（闭环） ──
    def rotate_to(self, target_yaw: float, tolerance=0.1) -> bool:
        """
        闭环旋转: 发 vyaw → 循环读 yaw → 偏差<tolerance → stop
        循环内每 200ms 检查一次，最大 5 秒超时
        Returns: True=对准, False=超时
        """
    
    def move_ahead(self, distance_m: float, max_segment=1.0) -> str:
        """
        闭环前进 + 风控。
        每次最多走 max_segment 米（安全分段），内部循环:
          每 200ms:
            ├─ 读 SLAM 位姿 → 算剩余距离
            │   └─ < 0.3m → stop → return 'ARRIVED'
            ├─ 读 front_clearance
            │   ├─ ≥ 0.50m → 全速 0.20 m/s
            │   ├─ 0.30-0.50m → 降速 0.10 m/s (不 stop, 继续)
            │   └─ < 0.30m → stop → return 'BLOCKED'
            └─ SLAM loc:lost → stop → return 'LOST'
        Returns: 'ARRIVED' | 'BLOCKED' | 'LOST' | 'TIMEOUT'
        """
    
    def stop(self):
        """急停。向 sport_bridge 发 {"stop":true}"""
```

**关键设计决策**：

- **move_ahead 不设 duration_ms**：sport_bridge 收到 `{"vx":0.2}` 后持续前进，不自动 stop。Python 侧通过轮询位姿 + XT16 决定何时发 `{"stop":true}`。
- **max_segment=1.0m**：单次 move_ahead 最多走 1 米。超过 1 米的 hop 分成多段，每段结束后重新读位姿校准。防止开环累积误差。
- **风控是渐进式的**：降速优先，极近才停。符合用户"不要硬邦邦阈值"的偏好。

### 4.2 `waypoint_nav.py` — 重构（282→~150行）

**改动**：用 `NavigationSession` 替换所有进程管理代码。删除 `send_sport()`、`get_slam_pose()`、`rotate_to_yaw()`、`move_forward()` —— 全部改用 `nav.xxx()`。

```python
def navigate_to(target_node: str, speed_mps=0.2):
    """从当前位置导航到目标路点。"""
    graph = load_graph()
    
    with NavigationSession() as nav:
        # 1. 获取当前位姿
        x, y, yaw = nav.get_pose()
        
        # 2. 匹配最近路点
        nearest_id, _ = nearest_waypoint(graph, x, y)
        
        # 3. BFS 路由
        route = bfs_route(graph, nearest_id, target_node)
        if not route:
            return f"NO_ROUTE: {nearest_id} → {target_node}"
        
        print(f"Route ({len(route)-1} hops): {' → '.join(route)}")
        
        # 4. 逐 hop 执行
        for i, node_id in enumerate(route):
            if i == 0:
                continue  # 跳起始点
            
            target = graph['waypoints'][node_id]
            tx, ty = target['x'], target['y']
            
            # 转体对准
            bearing = math.atan2(ty - nav.get_pose()[1], tx - nav.get_pose()[0])
            if not nav.rotate_to(bearing):
                return f"ROTATE_TIMEOUT at hop {i}"
            
            # 闭环前进（内部带风控）
            dist = math.hypot(tx - nav.get_pose()[0], ty - nav.get_pose()[1])
            result = nav.move_ahead(dist)
            
            if result == 'BLOCKED':
                return f"BLOCKED at hop {i}: front obstacle < 0.30m"
            if result == 'LOST':
                return f"LOST at hop {i}: SLAM localization lost"
            if result == 'TIMEOUT':
                return f"TIMEOUT at hop {i}"
            
            print(f"  ✅ hop {i} → {node_id} ARRIVED")
        
        return "COMPLETED"
```

### 4.3 `sport_bridge.cpp` — 小改（~5行新增）

**改动**：加 5 秒超时安全网。`move()` 后如果 5 秒内没有新命令（move 或 stop），自动 stop。这是纯兜底——正常流程 Python 会在到达后主动发 stop。

```cpp
// 改动: move() 成功后记录时间，加 watchdog
if (duration_ms > 0) {
    // 现有逻辑: sleep(duration) → stop
} else {
    // 新增: duration_ms=0 表示持续移动，Python 侧控制 stop
    // 加 5s 兜底: 5 秒内无新 stdin 输入 → auto stop
}
```

### 4.4 `agent_intent.py` — 意图解析（新建，~80行）

**职责**：LLM 自然语言 → 结构化意图，分离运动和行为。

```python
# LLM prompt 模板
PARSE_PROMPT = """
你是一个意图解析器。用户给出自然语言指令，你输出 JSON。

运动意图 (motion): 用户想去的目标路点 ID
行为意图 (behavior): 到达后要执行的动作

可用路点: {waypoint_list}
可用行为: tts(文本播报), photo(拍照), wait(等待N秒), none(无行为)

示例:
  "去露台叫杨书洋回来" →
  {{"motion": {{"target": "terrace_wc_entry"}},
    "behavior": {{"action": "tts", "text": "杨书洋，回来了！"}}}}

  "去尹思园工位拍张照" →
  {{"motion": {{"target": "wp_60497f"}},
    "behavior": {{"action": "photo", "label": "yin_siyuan"}}}}

  "回充电桩" →
  {{"motion": {{"target": "charging_station"}},
    "behavior": {{"action": "none"}}}}
"""

def parse_intent(llm_text: str, waypoints: dict) -> dict:
    """调用 LLM 解析意图 → 返回结构化 dict"""

def execute_behavior(behavior: dict):
    """执行行为意图"""
    if behavior['action'] == 'tts':
        subprocess.run(['espeak', '-v', 'zh', behavior['text']])
    elif behavior['action'] == 'photo':
        # 调 D435 拍照
        pass
    elif behavior['action'] == 'wait':
        time.sleep(behavior.get('seconds', 5))
```

### 4.5 `waypoint_graph.py` — 不变

当前的 PCD 预验 + BFS 路由已经满足需求。唯一可能需要的是把 `DENSITY_WALL_THRESHOLD` 按实际 PCD 点数做自适应校准（见已知问题），但不阻塞运动控制重构。

---

## 5. 实现计划

| 步骤 | 内容 | 依赖 | 改动量 | 可并行 |
|------|------|------|--------|--------|
| **Step 1** | 新建 `scripts/nav_core.py` — NavigationSession 类 | 无 | ~200 行 | — |
| **Step 2** | 小改 `sport_bridge.cpp` — 加 5s 兜底超时 | 无 | ~5 行 C++ | 可并行 Step 1 |
| **Step 3** | 重构 `scripts/waypoint_nav.py` — 改用 nav_core | Step 1 | 282→~150 行 | — |
| **Step 4** | 新建 `scripts/agent_intent.py` — 意图解析 | Step 3 | ~80 行 | 可与 Step 3 并行 |
| **Step 5** | 机器人编译部署 + 实测 | Step 1-4 | — | — |

### Step 1 实现细节

`nav_core.py` 核心流程：

```
__enter__:
  1. sport_bridge = Popen([BRIDGE, 'eth0'], stdin=PIPE, stdout=PIPE, text=True)
  2. gateway = Popen([CLIENT, 'eth0', '--persistent-world-state-session'],
                     stdin=PIPE, stdout=PIPE, stderr=STDOUT, text=True, bufsize=1)
  3. 启动 reader 线程: 循环读 gateway.stdout → json.loads → 
     if 'world_state' in obj: 更新 self._latest_pose, self._latest_front
  4. time.sleep(3) 等 DDS 订阅就绪

move_ahead 循环:
  while remaining_dist > 0.3m:
    sleep(0.2)
    pose = self._latest_pose  # reader 线程持续更新
    clearance = self._latest_front
    
    if self._loc_lost:        return 'LOST'
    if clearance < 0.30:      self._send_bridge({"stop":True}); return 'BLOCKED'
    
    speed = 0.10 if clearance < 0.50 else 0.20
    self._send_bridge({"vx": speed})  # 持续发速度命令
    
  self._send_bridge({"stop":True})
  return 'ARRIVED'
```

---

## 6. 风控策略

### 设计原则

用户明确反对硬邦邦的固定阈值。风控采用渐进式策略：

| 前向净空 | 动作 | 恢复条件 |
|----------|------|----------|
| ≥ 0.50m | 全速 0.20 m/s | — |
| 0.30m ~ 0.50m | 降速 0.10 m/s，继续前进 | 净空恢复 ≥ 0.50m 自动提回全速 |
| < 0.30m | **STOP**（真正危险，硬停） | 人工确认后重试 |
| XT16 连续 1s 无数据 | **STOP**（传感器挂了） | 人工确认 |
| SLAM `loc: lost` | **STOP**（定位丢了） | 人工确认 |

### 为什么这样设计

- **降速不硬停**（0.30-0.50m）：机器狗在走廊里 XT16 看到墙的侧面读数 0.30-0.50m 很常见——不是真障碍，是通道边界。降速即可。
- **0.30m 才硬停**：机器人本体半径约 0.2m，0.30m 是真正快要碰到的距离。在 supervised release + 人工旁站 + 急停在手条件下，这个阈值是务实且安全的。
- **XT16 自遮挡已过滤**：XT16 装在腹部，自遮挡读数 <0.15m（已在 Gateway 层过滤为 2.0m 开放空间）。不会误触发。

---

## 7. 与现有组件的关系

| 组件 | 关系 | 说明 |
|------|------|------|
| Gateway C++ | **只读 world_state** | nav_core 只取位姿 + XT16 净空，不依赖 Gateway 导航命令 |
| Gateway relocate | **独立运行** | 导航前手工 relocate，不在 nav_core 内处理 |
| `go2w_accept.sh` | **不变** | relocate/verify 流程不变 |
| `build_multi_pcd.sh` | **不变** | 建图/标点流程不变 |
| `snapshot_waypoint.py` | **不变** | 交互式标点流程不变 |
| `waypoint_graph.py` | **只读 graph JSON** | nav_core 不重建图，只加载已有 JSON |
| `run_robot_closed_loop.py` | **逐渐替代** | 当前闭环脚本（2000+ 行）的导航部分逐步由 nav_core + waypoint_nav 替代。初期并行运行，nav_core 验证稳定后废弃旧导航路径 |
| 云端 LLM (`gateway_server.py`) | **互补** | 云端 LLM 看懂粗粒度地图后推 hop plan → 本地 nav_core 执行。云端负责规划，nav_core 负责执行 |

---

## 8. 不做的事

- ❌ 不改 Gateway C++ 导航逻辑（已经放弃 Unitree navigate_to_pose）
- ❌ 不改 `waypoint_graph.py`（PCD 预验 + BFS 够用）
- ❌ 不搞自定义 PID 控制器（robot 速度低、距离短，闭环读取位姿 + 速度命令足够）
- ❌ 不改 XT16 几何服务或 Gateway 安全判定（继续用现有 world_state 输出）
- ❌ 不做四阶段上线流程、故障注入、图搜索优化等重型工程化（用户明确反对过度设计）

---

## 9. 风险与缓解

| 风险 | 缓解 |
|------|------|
| sport_bridge 持久进程 stdin 阻塞 | reader 线程独立，主循环不受阻塞影响 |
| Gateway world_state reader 线程 json 解析失败 | try/except + 保留上一次有效数据，1 秒无有效数据 → 报 LOST |
| Python 进程崩溃导致 robot 不停 | sport_bridge 5s 超时兜底（Step 2）+ 人工急停在手 |
| SLAM 位姿更新延迟 | 200ms 轮询间隔配合 0.2m/s 速度，两次轮询之间最多走 4cm |
| 多 PCD 切换后位姿跳变 | 导航前由人工 relocate 确认定位，nav_core 不处理 PCD 切换 |

---

## 10. 总结

```
Session D:  发现 Unitree 导航不可用 → 决策自研
Session E:  建 PCD 连通图 + sport_bridge + 开环导航原型 → 识别 4 个结构性问题
Session F:  本设计 —— 引 NavigationSession 统一进程管理 + 闭环运动 + 渐进风控

架构变更:
  旧: waypoint_nav.py (每个原语独立起进程, 开环, 无风控)
  新: nav_core.py (持久会话) ← waypoint_nav.py (路由+调度)
       agent_intent.py (LLM 意图解析)

改动量: ~350 行新代码 + ~130 行重构 + 5 行 C++
核心交付: 稳定走通 wp_60497f → wp_e643bc 的闭环导航
```
