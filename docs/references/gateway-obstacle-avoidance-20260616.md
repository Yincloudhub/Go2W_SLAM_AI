# Gateway 绕障能力完整分析

> 2026-06-16，基于源码审查 + 硬件验证 + Unitree 官方文档。

## 三层绕障架构（理论 vs 实际）

| 层 | 机制 | 理论 | 实际（此 Go2） |
|---|---|---|---|
| 规划层 | `navigate_to_pose` mode=0 | Unitree SLAM 在 PCD 栅格上规划绕障路径 | ✅ 可用。但官方限制可通行宽度≥0.8m、障碍宽≤0.5m、距离≤10m |
| 反应层 | `SwitchAvoidMode` / L1 激光雷达 | GO2 实时激光雷达检测 → 内圈停/外圈慢 | ❌ 3203 "Api not implement"。此 Go2 固件不支持 |
| 安全层 | XT16 safe_guard | 前向<0.80m → HARD PAUSE | ⚠️ supervised release 下被降级为 advisory |

## SwitchAvoidMode / FreeAvoid 调用链

```
Gateway Init() → SportClient::SwitchAvoidMode() → DDS API 2058 → GO2 运动控制器
                                                                    ↓
                                                          返回 3203 (Api not implement)
```

**错误码对照**（Unitree SDK）:
```cpp
UT_DECL_ERR(UT_ROBOT_ERR_SERVER_API_NOT_IMPL, 3203, "Api not implement error.")
```

**尝试过的调用方式**（全部返回 3203）:
- `SportClient::SwitchAvoidMode()` — DDS API 2058
- `SportClient::FreeAvoid(true)` — DDS API 2048
- 通过 SlamGateway 的 Client("slam_operate") 注册 sport API → 跨服务调用无效
- ROS2 `/obstacle_avoidance_mode` 服务 → go2_interfaces 模块不存在

**根因**: Unitree 文档标注 "AIR model currently does not support Free Avoid"。此 Go2 大概率是 Air 型号或固件版本不支持。与 XT16 外挂无关——L1/Mid360 替换为 XT16 不影响此 API 的可用性（API 发给运动控制器，不经过激光雷达）。

## 🔴 supervised release 下 XT16 前向被降级

**文件**: `robot/slam_gateway_refactor/src/safety_supervisor.cpp` lines 101-129

**当前逻辑（有问题）**:
```cpp
if (obstacle.supervised_release_active) {
    // 检查 front_clearance < 0.80m 但只设为 advisory
    const bool local_obstacle_advisory =
        obstacle.front_clearance_m < obstacle_policy::kFrontPauseM || ...;
    d.allow_navigation = true;  // ← 仍然允许导航！
    d.should_pause = false;     // ← 不暂停！
    d.reason = "..._advisory";
    return d;  // ← 提前返回，跳过了下面的硬阻断逻辑
}
// 下面的硬阻断（lines 132-158）永远走不到：
if (obstacle.front_clearance_m < obstacle_policy::kFrontPauseM) {
    d.allow_navigation = false;  // ← 被上面的 return 跳过了
}
```

**后果**: 2026-06-15 22:45 日志显示，前向从 0.649m 进入危险区后，XT16 报告 `recommended_action=pause`，但 Gateway 仍允许导航。等软件层反应过来发 pause 时，Unitree 返回 `errorCode=13`，此时 front 已到 0.03m。

**正确修复**: 在 supervised release 分支**之前**插入前向硬阻断检查，确保无论是否 supervised release，前向<0.80m 都硬拦。

## Unitree 官方 mode=0 能力边界

来源: [Unitree SLAM and Navigation Services Interface](https://doc-cdn.unitree.com/6/111/en/6_111_en)

- `mode=0`: 绕障模式。规划路径时避开 PCD 地图中的占据栅格。
- `mode=1`: 停障模式。不尝试绕行，检测到障碍即停。
- 可通行宽度 ≥ 0.8m
- 障碍物宽度 ≤ 0.5m  
- 目标距离 ≤ 10m
- pose navigation 沿直线去目标（非全局规划）

**这意味着**: mode=0 适合绕开椅子、箱子等小障碍，**不适合绕开工位隔断、墙壁等大结构**。对于工位→走廊→办公室这种大尺度绕行，必须在发给 1102 之前由上层（LLM + path_validator）拆成多段。

## Gateway C++ 编译陷阱

### `llm_command_main.cpp` 绕过 Init()

```cpp
// llm_command_main.cpp:154 — 错误写法
gateway.initApis();  // 直接调 initApis，跳过了 Init() 里的任何逻辑

// 正确写法
gateway.Init();       // Init() 内部调用 initApis() + 额外初始化
```

任何加到 `SlamGateway::Init()` 的逻辑在命令行模式下不会执行，除非改为 `gateway.Init()`。

### 跨服务 API 调用

`SlamGateway` 继承 `Client("slam_operate")`，只能调 SLAM 服务 API（1102, 1201, 1801 等）。调 sport API（2048, 2058）需独立 `SportClient` 实例。

## 修复方案参考

详见 `C:\Users\c\Desktop\hermes_maps\go2w_fix_plan.md` 和 `fix_package\README.md`。

三个改动：
1. `safety_supervisor.cpp`: 前向硬阻断（+8行）
2. `local_llm_planner.py`: coarse_map 硬拦截 + 自动多跳（~40行替换）
3. `path_validator.py`: `find_clear_two_hop` 函数（+25行）
