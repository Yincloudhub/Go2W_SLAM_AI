# GO2W 机器狗完整代码架构与命令执行流程

> 更新于 2026-06-16 | 目标受众：项目开发者、Hermes Agent

---

## 目录

1. [项目文件结构](#1-项目文件结构)
2. [硬件-软件分层全景](#2-硬件-软件分层全景)
3. [启动流程：从关机到待命](#3-启动流程从关机到待命)
4. [命令下发完整链路](#4-命令下发完整链路)
5. [核心 Python 库详解](#5-核心-python-库详解)
6. [C++ Gateway 详解](#6-c-gateway-详解)
7. [感知管道：从传感器到安全裁决](#7-感知管道从传感器到安全裁决)
8. [安全架构](#8-安全架构)
9. [世界状态与内部协议](#9-世界状态与内部协议)
10. [关键 Topics、Topics、API IDs](#10-关键-topics-api-ids)
11. [数据文件清单](#11-数据文件清单)

---

## 1. 项目文件结构

```
E:\GO2W_0 (本地) / /home/unitree/Go2W_SLAM_AI (机器人 NX)
│
├── scripts/                          # 可执行入口和 shell 脚本
│   ├── run_robot_closed_loop.py      # ★ 主入口：命令→规划→决策→执行
│   ├── go2w_agent_entry.py           # 自治 Agent 入口（含规划循环）
│   ├── gateway_server.py             # HTTP REST API（Hermes 远程调机器人）
│   ├── go2w_startup_supervisor.py    # 启动管理器
│   ├── go2w_supervised_acceptance.py # 监督验收脚本
│   ├── start_go2w_slam_stack.sh      # ★ 启动 SLAM 全栈
│   ├── go2w_xt16_geometry_sidecar.sh # ★ XT16 几何服务管理
│   ├── go2w_d435_perception_sidecar.sh # D435 感知服务管理
│   ├── xt16_lidar_geometry_summary.py  # ★ XT16 点云→四向净空
│   ├── d435_perception_summary.py      # D435 RGBD→深度+YOLO摘要
│   ├── perception_context_service.py   # ★ 感知上下文生产者
│   ├── xt16_supervised_release_guard.py # 监督发布守卫
│   ├── xt16_calibration_guard.py      # 标定守卫
│   ├── capture_keyframe.py            # 关键帧拍照
│   └── ... (其他工具脚本)
│
├── src/edge_autonomy/                # ★ Python 核心库
│   ├── __init__.py
│   ├── models.py                     # 共享数据模型（Pose2D, MapReference, RobotState...）
│   ├── slam_state.py                 # SLAM 状态模型（CurrentPose, LocalizationState, SlamHealth...）
│   ├── slam_topics.py                # Unitree DDS/ROS2 话题解析
│   ├── slam_adapter.py               # SLAM 适配器抽象 + InMemory/Replay 实现
│   ├── slam_health_monitor.py        # SLAM 健康监控
│   ├── xt16_geometry.py              # ★ XT16 点云→四向净空算法
│   ├── lidar_geometry.py             # 激光几何工具
│   ├── obstacle_policy.py            # ★ 障碍物策略阈值（FRONT_PAUSE_M=0.80 等）
│   ├── perception_fusion.py          # LiDAR + D435 深度融合
│   ├── perception_context.py         # ★ 统一感知上下文（SensorEnvelope 加载/校验）
│   ├── map_registry.py               # ★ 地图注册表（PCD、拓扑节点、重定位锚点）
│   ├── llm_context.py                # ★ LLM 上下文构建（规划上下文 + 相对运动检测）
│   ├── local_llm_planner.py          # ★ 本地 LLM 规划器（Qwen 4B → TaskQueue）
│   ├── world_state_v1.py             # ★ 统一世界状态聚合
│   ├── task_queue.py                 # ★ TaskQueue IR 定义与校验
│   ├── mission_decision.py           # ★ 任务决策引擎（Registry/Topology/Gateway 三道门）
│   ├── gateway_safety.py             # Gateway 安全裁决检查
│   ├── chassis_controller.py         # ★ Gateway 客户端（PersistentGatewaySession）
│   ├── path_validator.py             # PCD 密度采样路径校验 + 粗粒度地图
│   ├── communication_policy.py       # 弱网通信策略 + Journal
│   ├── operator_display.py           # 操作员面板显示状态
│   ├── runtime_log.py                # 运行时日志
│   ├── runtime_state.py              # 运行时快照
│   ├── safety.py                     # 安全模块
│   ├── service_supervision.py        # 服务监管
│   └── runtime_readiness.py          # 运行时就绪检查
│
├── robot/slam_gateway_refactor/      # ★ C++ Gateway（运动权威）
│   ├── src/
│   │   ├── slam_gateway.cpp          # ★ SlamGateway 核心类
│   │   ├── safety_supervisor.cpp     # ★ 安全监督器
│   │   ├── llm_command_processor.cpp # ★ 命令处理器（JSON→Unitree API）
│   │   └── llm_command_main.cpp      # ★ main()：持久会话入口
│   ├── include/slam_gateway/
│   │   ├── slam_gateway.hpp          # SlamGateway 头文件
│   │   ├── safety_supervisor.hpp     # 安全监督器头文件
│   │   ├── llm_command_processor.hpp # 命令处理器头文件
│   │   ├── obstacle_policy.hpp       # C++ 障碍物策略
│   │   └── navigation_target_authorizer.hpp # 导航目标授权器
│   └── build/
│       └── slam_llm_command_client   # ★ 编译产物（Python 调用的可执行文件）
│
├── configs/
│   ├── maps/
│   │   ├── go2w_real_site_map_registry.json      # 真实场地地图注册表
│   │   ├── go2w_multi_map_registry_v2.json       # 多 PCD 注册表 V2
│   │   └── go2w_real_site_map_registry_v1_archived.json # V1 归档
│   └── perception/
│       ├── xt16_geometry_calibration.json        # XT16 标定记录
│       └── xt16_supervised_release.json          # 监督发布记录
│
├── artifacts/                        # 运行时产物（JSON 日志和状态文件）
│   ├── lidar_geometry_summary.json   # XT16 几何摘要（C++ Gateway 读取）
│   ├── d435_perception_summary.json  # D435 感知摘要
│   ├── perception_context_v1.json    # ★ 统一感知上下文
│   ├── communication/               # 通信 Journal
│   └── robot_runs/                  # 导航运行日志
│
├── tests/                            # 测试
└── docs/                             # 文档
```

---

## 2. 硬件-软件分层全景

```
┌─────────────────────────────────────────────────────────────┐
│                      比赛演示端 / Hermes Agent               │
│  "去赵博那" → 通过 SSH/HTTP 向机器人 NX 发命令              │
└─────────────────────────────────────────────────────────────┘
                            │
                            ▼
┌─────────────────────────────────────────────────────────────┐
│              Python 层 (NX 上运行，边缘自治)                  │
│                                                              │
│  ┌───────────────┐  ┌───────────────┐  ┌─────────────────┐ │
│  │ LLM Planner   │  │ MissionDecision│  │ WorldState V1   │ │
│  │ (Qwen 3 4B)   │  │ Engine         │  │ (统一世界状态)   │ │
│  └───────┬───────┘  └───────┬───────┘  └────────┬────────┘ │
│          │                  │                    │          │
│  ┌───────┴───────┐  ┌───────┴───────┐  ┌────────┴────────┐ │
│  │ PlannerContext│  │ 三道门判决     │  │ PerceptionCtx  │ │
│  │ + TaskQueue   │  │ Registry/     │  │ Loader          │ │
│  │ (IR)          │  │ Topology/     │  │ (SensorEnvelope)│ │
│  │               │  │ Gateway       │  │                 │ │
│  └───────────────┘  └───────────────┘  └────────┬────────┘ │
│                                                  │          │
│  ┌───────────────────────────────────────────────┴────────┐ │
│  │         ChassisController / PersistentGatewaySession    │ │
│  │         (stdin/stdout JSON 协议与 C++ Gateway 通信)      │ │
│  └────────────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────────┘
                            │ (stdin/stdout JSON)
                            ▼
┌─────────────────────────────────────────────────────────────┐
│            C++ Gateway 层 (唯一运动权威)                      │
│                                                              │
│  ┌──────────────────┐  ┌──────────────────┐                │
│  │ SafetySupervisor │  │ LlmCommandProc   │                │
│  │ (SLAM健康/定位/  │  │ (JSON→UnitreeAPI │                │
│  │  障碍物/安全裁决) │  │  参数绑定/校验)   │                │
│  └────────┬─────────┘  └────────┬─────────┘                │
│           │                     │                           │
│  ┌────────┴─────────────────────┴─────────┐                │
│  │            SlamGateway                   │                │
│  │  (DDS 话题订阅 / WorldState 构建 /      │                │
│  │   Unitree API 调用 / 导航状态机)        │                │
│  └────────────────────────┬───────────────┘                │
└───────────────────────────┼────────────────────────────────┘
                            │ (DDS RPC)
                            ▼
┌─────────────────────────────────────────────────────────────┐
│                Unitree SDK / SLAM 层                         │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐     │
│  │ unitree_slam │  │ xt16_driver  │  │ DDS middleware│     │
│  │ (定位/建图/  │  │ (激光雷达驱动)│  │ (CycloneDDS) │     │
│  │  路径规划)   │  │              │  │              │     │
│  └──────┬───────┘  └──────┬───────┘  └──────────────┘     │
└─────────┼──────────────────┼──────────────────────────────┘
          │                  │
          ▼                  ▼
┌─────────────────────────────────────────────────────────────┐
│                    硬件层 (GO2W 本体)                        │
│  XT16 LiDAR | D435 RGBD | IMU | 电机 | 电池 | ...          │
└─────────────────────────────────────────────────────────────┘
```

### 分层职责边界

| 层 | 负责 | 不负责 |
|----|------|--------|
| Python Planner | 任务理解、自然语言→TaskQueue、多段路径规划 | 底盘控制、速度指令、安全裁决 |
| Python MissionDecision | 三道门校验（Registry/Topology/Gateway）、TaskQueue 合法性 | 最终运动执行 |
| C++ SafetySupervisor | SLAM 状态、定位质量、障碍物净空、安全裁决 | 自然语言理解 |
| C++ SlamGateway | WorldState 聚合、Unitree API 调用封装、导航状态机 | LLM 输入生成 |
| Unitree SLAM | 定位、建图、路径规划（A*）、导航执行 | 安全策略 |

---

## 3. 启动流程：从关机到待命

### 3.1 SLAM 栈启动

```bash
bash scripts/start_go2w_slam_stack.sh
```

**启动顺序（严格有序）**：

```
1. CycloneDDS 网络接口准备
   ├── 检查/重写 cyclonedds.xml（确保 eth0 可用）
   └── 导出 CYCLONEDDS_URI

2. XT16 PTP 时间同步检查
   ├── 雷达与 NX 间的离线局域网授时
   └── 未同步则拒绝启动（fail-closed）

3. xt16_driver 启动（Unitree 二进制）
   ├── 发布 ROS2 话题: /unitree/slam_lidar/points
   ├── 等待进程出现 + 稳定性等待 4s
   └── 验证话题有数据（6s 超时，无数据则重启 driver）

4. unitree_slam 启动（Unitree 二进制）
   ├── 订阅 /unitree/slam_lidar/points
   ├── 发布 DDS 话题: SLAM_INFO_TOPIC (pose) + SLAM_KEY_INFO_TOPIC (task result)
   ├── 等待进程 + 验证 /slam_info 话题
   └── 启动 DDS service: slam_operate (navigation API endpoint)

5. XT16 Geometry Sidecar 启动（可选）
   ├── go2w_xt16_geometry_sidecar.sh restart-if-stale
   ├── 订阅 /unitree/slam_lidar/points (ROS2)
   ├── 5 Hz 限速处理
   └── 产出: artifacts/lidar_geometry_summary.json

6. D435 Perception Sidecar 启动
   ├── go2w_d435_perception_sidecar.sh
   ├── D435 RGBD @ 15 FPS
   ├── 深度摘要 @ 5-10 Hz
   ├── YOLO @ ~3 Hz
   └── 产出: artifacts/d435_perception_summary.json

7. PerceptionContext Service 启动
   ├── perception_context_service.py
   ├── 聚合所有传感器摘要
   └── 产出: artifacts/perception_context_v1.json
```

### 3.2 启动后验证

```bash
# 关键进程检查
ps aux | grep -E 'xt16_driver|unitree_slam|xt16_lidar_geometry|perception_context|d435_perception'

# 话题检查
ros2 topic list | grep slam_lidar

# Gateway 健康检查
./robot/slam_gateway_refactor/build/slam_llm_command_client eth0 <<< '{"action":"get_world_state"}'
```

---

## 4. 命令下发完整链路

### 4.1 总览

以用户说 **"去赵博那"** 为例，完整执行路径：

```
用户命令 "去赵博那" (中文自然语言)
    │
    ▼
┌─────────────────────────────────────────────────────────────┐
│ Step 1: 运行时快照 (build_snapshot)                          │
│   收集: Gateway world_state + 注册表 + 感知上下文            │
└─────────────────────────────────────────────────────────────┘
    │
    ▼
┌─────────────────────────────────────────────────────────────┐
│ Step 2: 规划上下文 (build_planner_context)                   │
│   聚合: 拓扑节点列表 + 粗粒度PCD地图 + capability_contract   │
│        + 当前位置 + 感知摘要 + 网络状态 + 电池状态            │
└─────────────────────────────────────────────────────────────┘
    │
    ▼
┌─────────────────────────────────────────────────────────────┐
│ Step 3: LLM 规划 (run_local_llm_planner → Qwen 4B)          │
│   prompt = system_prompt + planner_context                   │
│   输出: LocalPlan JSON                                       │
│   {                                                          │
│     "plan_id": "plan_xxx",                                   │
│     "mode": "mapped_navigation",                             │
│     "steps": [                                               │
│       {"step_id":"1","tool":"set_communication_policy",...}, │
│       {"step_id":"2","tool":"create_navigation_subgoal",     │
│        "arguments":{"map_id":"go2w_real_site",               │
│                     "target_node":"zhao_bo_office_front"}},  │
│       {"step_id":"3","tool":"wait_until",...}                │
│     ]                                                        │
│   }                                                          │
└─────────────────────────────────────────────────────────────┘
    │
    ▼
┌─────────────────────────────────────────────────────────────┐
│ Step 4: Plan → TaskQueue + SLAM Command                      │
│   plan_to_task_queue(plan) → TaskQueue IR (中间表示)         │
│   plan_to_slam_command(plan) → {target_node, target_pose}    │
│   包含: map_id, target_pose (x,y,yaw), mode=0, speed=0.2     │
└─────────────────────────────────────────────────────────────┘
    │
    ▼
┌─────────────────────────────────────────────────────────────┐
│ Step 5: 三道门判决 (MissionDecision)                         │
│                                                              │
│   [门1] Registry Gate:                                       │
│     → 注册表是否允许执行？map_id 是否存在？status==real？     │
│                                                              │
│   [门2] Topology Gate:                                       │
│     → 目标节点是否有阻塞标签（needs_calibration 等）？        │
│     → coarse_map 密度采样是否穿墙？（已降级为 advisory）      │
│                                                              │
│   [门3] Gateway Preflight:                                   │
│     → 调用 C++ Gateway get_world_state                       │
│     → 检查: accepted==true, safety.allow_navigation==true    │
│     → 检查: SLAM 健康, 定位状态, 传感器新鲜度                 │
│     → 返回: allowed/rejected                                 │
│                                                              │
│   三道全过 → decision = "execute_queue"                      │
│   任意阻止 → decision = "reject" 或 "hold"                   │
└─────────────────────────────────────────────────────────────┘
    │ (仅当 decision == "execute_queue" 且 --execute)
    ▼
┌─────────────────────────────────────────────────────────────┐
│ Step 6: 执行 (run_supervised_navigation_session)             │
│                                                              │
│   6a. 启动 PersistentGatewaySession                          │
│       → spawn: slam_llm_command_client eth0                  │
│               --persistent-navigation-session                │
│       → 等待: navigation_session_ready (含 session_token)    │
│                                                              │
│   6b. 发送导航命令                                            │
│       → stdin JSON:                                          │
│         {                                                    │
│           "action": "navigate_to_pose",                       │
│           "map_id": "go2w_real_site",                        │
│           "map_path": "/home/unitree/test.pcd",              │
│           "target_node": "zhao_bo_office_front",             │
│           "target_pose": {                                   │
│             "name": "zhao_bo_office_front",                  │
│             "x": 3.5, "y": -1.2,                             │
│             "q_x": 0, "q_y": 0, "q_z": 0.707, "q_w": 0.707, │
│             "mode": 0,   ← 主动绕障                          │
│             "speed": 0.2 ← 受 supervised release 限制         │
│           },                                                 │
│           "operator_ack": true,                              │
│           "navigation_session_token": "<token>"              │
│         }                                                    │
│                                                              │
│   6c. 导航监控循环 (2 Hz)                                     │
│       → heartbeat (每 500ms，维持 session lease)             │
│       → get_world_state 轮询 (1 Hz)                          │
│       → 检查: distance_to_goal < arrival_distance_m？         │
│       → 检查: nav_state == "arrived"？                       │
│       → 超时检测 (stall 20s 无进展 → recovery)               │
│       → LLM 反馈给操作员 (每 5s)                              │
│                                                              │
│   6d. 到达或异常                                              │
│       → arrived: pause_navigation → return ok                │
│       → timeout/stalled: 触发 recovery 策略                  │
│         → 分析四向净空 → 选择最佳平移方向 → reposition       │
│       → 完成: 关闭 session (auto-pause + stdin close)        │
└─────────────────────────────────────────────────────────────┘
    │
    ▼
┌─────────────────────────────────────────────────────────────┐
│ Step 7: C++ Gateway 内部处理 (llm_command_processor.cpp)     │
│                                                              │
│   7a. process(cmd) 入口                                      │
│       → 安全边界: 拒绝 raw_api_id                             │
│       → 识别 action: "navigate_to_pose"                       │
│                                                              │
│   7b. 参数校验                                                │
│       → hasNavigationSessionAuthority(token)                 │
│       → hasOperatorAck(confirm)                              │
│       → map_path 与当前 SLAM 加载的地图匹配？                 │
│       → NavigationTargetAuthorizer 检查目标合法性             │
│       → validatePoseJson() 检查数值合法性                     │
│                                                              │
│   7c. 安全裁决 (SafetySupervisor.evaluate)                    │
│       → SLAM 健康: status=="ok" 或 "degraded"？              │
│       → 定位: status=="localized"？                          │
│       → 障碍物源: 是 lidar_pointcloud？                       │
│       → 障碍物新鲜度: age_ms < 1000ms？                      │
│       → supervised release 激活时:                            │
│         - 跳过 per-direction confidence 检查                  │
│         - 前向障碍 → advisory（不硬拦）                       │
│         - motion_direction = "unitree_pose_navigation_mode_0" │
│       → 非 supervised release:                               │
│         - front_clearance < 0.80m → pause                    │
│         - side_clearance < 0.20m → pause                     │
│         - rear_clearance < 0.30m → pause                     │
│                                                              │
│   7d. mode=0 强制检查                                         │
│       → 若 safety.motion_direction ==                        │
│         "unitree_pose_navigation_mode_0" 且 mode != 0        │
│         → 拒绝: "supervised_navigation_requires_             │
│                    unitree_avoidance_mode_0"                  │
│                                                              │
│   7e. 速度修正                                                │
│       → mode=0: speed = max(speed, 0.2)                      │
│       → conservative mode: speed = min(speed, 0.2)           │
│                                                              │
│   7f. 调用 Unitree SDK                                       │
│       → gateway.submitNavigationGoal(goal)                   │
│         ├── callApi(ROBOT_API_ID_POSE_NAV_PL, json)         │
│         │   → DDS RPC → Unitree SLAM 接收导航目标            │
│         ├── callApi(ROBOT_API_ID_RESUME_NAV, json)          │
│         │   → 启动实际运动                                    │
│         └── 更新内部 nav_state_: state="running"             │
│                                                              │
│   7g. 等待结果（异步）                                        │
│       → SLAM_KEY_INFO_TOPIC 回调:                            │
│         type=="task_result" → is_arrived 或 failed           │
│       → 超时 60s → state="timeout"                           │
│       → 运行时安全检查（每 100ms）                             │
└─────────────────────────────────────────────────────────────┘
    │
    ▼
┌─────────────────────────────────────────────────────────────┐
│ Step 8: Unitree SLAM 执行                                    │
│                                                              │
│   → 全局 A* 在 PCD 栅格地图上规划路径                        │
│   → 局部规划器（mode=0 时主动绕障）                           │
│   → 底盘电机控制 + IMU 反馈                                   │
│   → 持续发布 SLAM_INFO_TOPIC（pose 更新）                     │
│   → 到达目标: 发布 SLAM_KEY_INFO_TOPIC (is_arrived=true)     │
└─────────────────────────────────────────────────────────────┘
```

### 4.2 命令类型总览

| 命令 | 入口 | LLM 参与 | 说明 |
|------|------|----------|------|
| 自然语言导航（"去赵博那"） | `run_robot_closed_loop.py --command "去赵博那"` | ✅ 需要（Qwen 4B 推理出拓扑节点） |
| 精确节点导航 | `run_robot_closed_loop.py --command zhao_bo_office_front` | hybrid 模式：确定性路由优先，歧义时 LLM |
| HTTP 导航 | `curl .../navigate -d '{"target":"..."}'` | 内部调 run_robot_closed_loop.py |
| 状态查询 | `--command status` 或 `curl .../status` | ❌ 不需要 |
| 暂停 | `curl .../pause` | ❌ 直接调 Gateway pause |
| 重定位 | `curl .../relocate -d '{"anchor":"..."}'` | ❌ 调 Gateway relocate |
| 自治 Agent | `go2w_agent_entry.py --command "去巡检"` | ✅ 带规划循环和自动恢复 |

### 4.3 run_robot_closed_loop.py 参数速查

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `--command` | 必填 | 自然语言命令或拓扑节点 ID |
| `--registry` | `configs/maps/go2w_real_site_map_registry.json` | 地图注册表 |
| `--map-id` | `go2w_real_site` | 当前使用的地图 |
| `--map-path` | `/home/unitree/test.pcd` | PCD 文件路径 |
| `--execute` | False | 真正执行（不加则为 dry run） |
| `--nav-speed-mps` | 0.0（使用注册表速度） | 导航速度覆盖 |
| `--nav-mode` | None（使用注册表 mode） | mode=0 绕障, mode=1 停障 |
| `--prompt-mode` | `hybrid` | `hybrid`/`full`：LLM 调用策略 |
| `--summary` | False | 输出精简为 1-3 行（推荐日常使用） |
| `--robot-password` | 无 | 跳过密码提示（部分场景不生效） |

---

## 5. 核心 Python 库详解

### 5.1 chassis_controller.py — Gateway 客户端

**PersistentGatewaySession** 是 Python 与 C++ Gateway 通信的核心桥梁：

```python
# 创建会话（spawn C++ 进程，stdin/stdout JSON 协议）
session = PersistentGatewaySession(
    client_path="robot/slam_gateway_refactor/build/slam_llm_command_client",
    network_interface="eth0",
    timeout_s=30,
    navigation_session=True,   # 持久的运动会话（含 lease）
)

# 发送 JSON 命令，等待响应
result = session.command({
    "action": "navigate_to_pose",
    "target_node": "zhao_bo_office_front",
    "target_pose": {...},
    "map_id": "go2w_real_site",
    "map_path": "/home/unitree/test.pcd",
    "operator_ack": True,
})
# result = {"accepted": True, "action": "navigate_to_pose", ...}

# 心跳（维持 session lease）
session.heartbeat()  # 每 500ms 一次

# 关闭
session.close()  # 先 pause_navigation，再 close stdin
```

**调用链**：
```
PersistentGatewaySession
  → spawn: slam_llm_command_client eth0 --persistent-navigation-session
  → stdin JSON → C++ main() → LlmCommandProcessor::process()
  → stdout JSON ← C++ 返回
```

**ChassisController** 是更上层的封装，提供注册表查询和预检功能。

### 5.2 perception_context.py — 统一感知上下文

**核心职责**：将所有传感器来源包装成统一的 `SensorEnvelope`，每个传感器有明确的状态（fresh/stale/offline/invalid/uncalibrated）。

```python
# 传感器状态定义
STATUS_VALUES = {"fresh", "stale", "offline", "invalid", "uncalibrated"}

# 每个传感器被包装为:
{
    "source_id": "xt16_geometry",        # 唯一源标识
    "source_kind": "lidar_geometry",     # 传感器类型
    "status": "fresh",                   # fresh/stale/offline/invalid/uncalibrated
    "confidence": 0.85,                  # 0.0-1.0
    "age_ms": 230,                       # 数据年龄
    "sequence": 42,                      # 单调序号
    "producer": "xt16_geometry_sidecar", # 生产者进程名
    "producer_instance_id": "boot_id:pid:start_ticks", # 进程实例
    "status_reasons": [...],             # 状态原因
    "payload": {                         # 传感器特定数据
        "front_clearance_m": 3.5,
        "left_clearance_m": 0.45,
        ...
    }
}
```

**传感器来源**：

| source_id | 生产者 | 数据文件 | stale_ms |
|-----------|--------|----------|----------|
| `xt16_geometry` | `xt16_geometry_sidecar` | `lidar_geometry_summary.json` | 1000ms |
| `d435_depth` | `d435_perception_sidecar` | `d435_perception_summary.json` | 1000ms |
| `d435_yolo` | `d435_perception_sidecar` | `d435_perception_summary.json` | 3000ms |
| `nx_ti_radar` | (TI/NX bridge) | (future) | (future) |

**Freshness 判定链**：
1. 文件存在？→ 读 JSON
2. 进程在线？→ 检查 PID + cmdline 匹配
3. 序列单调？→ `SensorSequenceTracker` 防止序列回退
4. 年龄合格？→ `age_ms <= stale_ms`
5. 生产者未声明 stale？→ 检查 `stale` flag 和 `stale_reasons`
6. 标定/发布状态？→ `calibrated` 或 `supervised_release`

**PerceptionContext v1** 聚合结构：
```json
{
  "schema": "go2w_perception_context_v1",
  "generated_at_ms": 1234567890,
  "robot_motion": {...},
  "local_geometry": {...},
  "visual_objects": [...],
  "radar_tracks": [...],
  "risk_events": [...],
  "sources": [各 SensorEnvelope],
  "degraded_capabilities": ["xt16_geometry:stale", "d435_yolo:offline"]
}
```

### 5.3 xt16_geometry.py — 点云→净空算法

**处理管道**：
```
ROS2 Topic /unitree/slam_lidar/points
    → xt16_lidar_geometry_summary.py (5 Hz)
        → 去自遮挡（footprint 过滤，SELF_OCCLUSION_RADIUS_M=0.15）
        → 坐标变换（forward_axis=y, lateral_axis=x, forward_sign=-1）
        → Z 轴过滤（min_z=-0.25, body_min_z=-0.10, max_z=1.20）
        → 四向 ROI 分箱（front/side/rear 各有 half_width）
        → 空间分箱（support_bin_m=0.05）
        → 聚类（clearance_cluster_gap_m=0.15）
        → 最近邻取第 10 百分位值作为净空
        → 计算 per-direction 置信度
    → artifacts/lidar_geometry_summary.json
```

**输出结构**：
```json
{
  "type": "local_obstacle_summary",
  "schema_version": 2,
  "source": "lidar_pointcloud",
  "timestamp_ms": 1234567890,
  "front_clearance_m": 3.5,
  "left_clearance_m": 0.45,
  "right_clearance_m": 1.2,
  "rear_clearance_m": 2.8,
  "roi_confidence": {"front": 0.9, "left": 0.6, "right": 0.85, "rear": 0.8},
  "blocked_directions": [],
  "recommended_action": "normal",
  "stale": false,
  "parameters": {
    "calibrated": false,
    "supervised_release": {"active": true, "release_id": "...", "max_speed_mps": 0.2}
  }
}
```

### 5.4 obstacle_policy.py — 障碍物策略阈值

```python
FRONT_PAUSE_M = 0.80   # 前方 < 0.80m → 暂停
FRONT_SLOW_M  = 1.50   # 前方 < 1.50m → 减速
SIDE_PAUSE_M  = 0.20   # 侧方 < 0.20m → 暂停
SIDE_SLOW_M   = 0.60   # 侧方 < 0.60m → 减速
REAR_PAUSE_M  = 0.30   # 后方 < 0.30m → 暂停
REAR_SLOW_M   = 0.50   # 后方 < 0.50m → 减速
```

**判定逻辑**：
- `recommended_action(clearance)` → "pause" / "go_slow" / "normal"
- `blocked_directions(clearance)` → ["front", "left", ...]
- `narrow_passage(clearance)` → True (双侧都 < 0.60m)

### 5.5 map_registry.py — 地图注册表

```python
MapRegistry {
    maps: [MapProfile]        # 每张 PCD 地图一个 profile
}

MapProfile {
    map_id: str              # "go2w_real_site"
    status: str              # "real" | "simulation" | "archived"
    pcd_path: str            # "/home/unitree/test.pcd"
    mapping_origin_anchor_id: str
    topology_nodes: [TopologyNode]  # 已注册的导航目标点
    relocalization_anchors: [...]
    transition_anchors: [...]       # 多 PCD 过渡点
    references: [...]               # V2 registry 地图引用
}

TopologyNode {
    node_id: str             # "zhao_bo_office_front"
    name: str                # "赵博办公室门口"
    aliases: [str]           # ["赵博", "zhao_bo"]
    pose: UnitreePose {      # 注册坐标
        x, y, z,
        q_x, q_y, q_z, q_w,
        yaw, speed, mode
    }
    tags: [str]              # ["live_verified"] 或 ["needs_calibration"]
    node_type: str           # "waypoint" | "charging_point" | ...
    description: str         # 语义描述
    area: str                # "701实验室"
    exit_direction: str      # "west"（角落节点的出口方向）
    corner_enclosed: bool    # 是否为角落封闭节点
}
```

**导航资格判定**（`topology_target_allows_navigation`）：
- tags 中含 `disabled`, `ui_disabled`, `deleted` → 永久阻塞
- tags 中含 `needs_calibration`, `needs_standing_verification` → 阻塞（需标定）
- 其他 → 允许

### 5.6 local_llm_planner.py — LLM 规划器

**双模式**：
- **hybrid**：确定性解析优先（直接匹配拓扑节点名/别名），歧义时才调 LLM
- **full**：总是调 LLM

**LLM 调用**：
```python
# Qwen 3 4B Q4_K_M GGUF 模型
# llama.cpp 推理，~13.5 tok/s
result = run_local_llm_planner(
    planner_context,
    LocalCommandBackend("MODEL_PATH=... scripts/ask_qwen.sh --ctx 4096 ..."),
    system_prompt=DEFAULT_SYSTEM_PROMPT,
    max_tokens=512,
    timeout_s=30,
    prompt_mode="hybrid",
)
```

**Plan 结构（LLM 输出）**：
```json
{
  "plan_id": "plan_xxx",
  "mode": "mapped_navigation",          // mapped_navigation|mapless_scout|safe_hold|human_confirm
  "confidence": 0.85,
  "reason": "目标 zhao_bo_office_front 已注册，SLAM 正常，路径可通",
  "steps": [
    {"step_id": "1", "tool": "create_navigation_subgoal",
     "arguments": {"map_id": "go2w_real_site", "target_node": "zhao_bo_office_front"}},
    {"step_id": "2", "tool": "wait_until",
     "arguments": {"condition": "arrived"}}
  ],
  "communication_policy": {"mode": "normal", "send": [...], "drop": [...], "reason": "..."},
  "requires_human_ack": false
}
```

**Plan → TaskQueue**：
```
plan_to_task_queue(plan, context) → {
    queue_id, mode: "sequential", status: "planned",
    source: "llm_fallback" | "semantic_topology",
    targets: ["zhao_bo_office_front"],
    steps: [
        {task_id, action: "navigate", target_node: "...", status: "pending"},
        {task_id, action: "wait_until", status: "pending"},
    ],
    communication_policy: {...}
}
```

### 5.7 mission_decision.py — 三道门判决

```python
build_mission_decision(
    task_queue,                         # LLM 产出的 TaskQueue
    execute_requested,                   # --execute flag
    registry_allowed, registry_reason,   # [门1] 地图注册表
    topology_allowed, topology_reason,   # [门2] 拓扑节点标签
    gateway_checked, gateway_allowed,    # [门3] C++ Gateway 预检
    gateway_reason, gateway_state,
)
```

**决策逻辑**（优先级从高到低）：
1. TaskQueue 非法 → `reject` (invalid_task_queue)
2. 需要人工确认 → `await_confirmation`
3. 纯 hold 无导航 → `hold`
4. 无导航步骤 → `reject` (no_navigation_task)
5. Dry run → `dry_run_queue`
6. Registry 不准 → `reject` (registry_blocked)
7. Topology 不准 → `reject` (topology_blocked)
8. Gateway 未检查 → `reject` (gateway_not_checked)
9. Gateway 拒绝 → `hold` (gateway_blocked)
10. 全部通过 → `execute_queue`（motion_allowed=True）

### 5.8 world_state_v1.py — 世界状态

**构建 unified WorldState**，消费者包括：LLM Planner、Operator Panel、Runtime Log、UI。

```python
build_world_state_v1(
    gateway_state,      # C++ Gateway 返回的 world_state
    planner_context,    # Planner 上下文
    task_phase,         # 当前任务阶段
    perception_context, # 统一感知上下文
    motion_allowed,     # MissionDecision 是否放行
    network_level,      # normal/weak/disconnected
)
```

**输出字段**：
- `task_phase`: idle/planning/executing_navigation/arrived/completed/blocked/...
- `localization`: status + pose_age_ms + confidence
- `slam_health`: status + slam_alive
- `local_obstacle`: front/left/right/rear clearance + recommended_action
- `safety`: allow_navigation + reason + speed_limit
- `available_tools`: 当前可用的工具列表
- `link_quality`: 网络状态

---

## 6. C++ Gateway 详解

### 6.1 SlamGateway 类

**构造时**：
```cpp
SlamGateway::SlamGateway() {
    // 订阅 DDS 话题
    sub_slam_info_->InitChannel(slamInfoHandler);       // pose 更新
    sub_slam_key_info_->InitChannel(slamKeyInfoHandler); // 导航任务结果
    // 默认手动净空（无传感器时 6m 全开放）
    lidar_perception_.setManualClearance(6.0, 6.0, 6.0, 6.0);
}
```

**initApis() 注册的 Unitree API**：
| API ID | 功能 |
|--------|------|
| `ROBOT_API_ID_POSE_NAV_PL` | 提交导航目标（含 A* 规划） |
| `ROBOT_API_ID_PAUSE_NAV` | 暂停导航 |
| `ROBOT_API_ID_RESUME_NAV` | 恢复导航 |
| `ROBOT_API_ID_STOP_NODE` | 停止节点 |
| `ROBOT_API_ID_START_MAPPING_PL` | 开始建图 |
| `ROBOT_API_ID_END_MAPPING_PL` | 结束建图并保存 PCD |
| `ROBOT_API_ID_START_RELOCATION_PL` | 开始重定位 |

### 6.2 SafetySupervisor 评估流程

```cpp
SafetyDecision evaluate(SlamHealth, LocalizationState, LocalObstacleSummary) {
    // 1. SLAM 健康检查
    if (health.status == "failed" || !slam_alive) → slam_health_failed

    // 2. 定位检查
    if (localization.status != "localized") → localization_not_valid

    // 3. 障碍物源检查
    if (source not in {lidar_pointcloud, lidar_pointcloud+stereo_depth})
        → local_obstacle_source_not_trusted

    // 4. 障碍物新鲜度检查
    if (stale && !supervised_stale_grace) → local_obstacle_not_fresh

    // 5. 监督发布检查
    if (supervised_release && 参数无效) → supervised_release_invalid

    // 6. Per-direction 置信度检查（仅非 supervised release）
    if (!supervised_release_active) {
        if (front_confidence < 0.15) → 拒绝
        if (left_confidence < 0.15)  → 拒绝
        if (right_confidence < 0.15) → 拒绝
    }

    // 7. Supervised release 分支
    if (supervised_release_active) {
        SLAM health degraded → 拒绝
        前向/侧向障碍物 → advisory（allow_navigation=true, conservative）
        motion_direction = "unitree_pose_navigation_mode_0"
        speed_limit = supervised_max_speed_mps (0.2)
    }

    // 8. 标准（非 supervised）分支
    front_clearance < 0.80 → front_obstacle_too_close (hard block)
    recommended_action == "pause" → pause
    side_clearance < 0.20 → side_obstacle_too_close
    rear_clearance < 0.30 → rear_obstacle_too_close
    recommended_action == "go_slow" → conservative + low speed

    // 9. 默认通过
    allow_navigation = true, mode = "normal"
}
```

### 6.3 LlmCommandProcessor 命令处理

**支持的动作**：

| action | operator_ack | session_token | 说明 |
|--------|:---:|:---:|------|
| `get_world_state` | - | - | 返回完整 WorldState JSON |
| `navigate_to_pose` | ✅ | ✅ | 导航到注册拓扑点 |
| `supervised_reposition` | ✅ | ✅ | 监督平移（四向 bounded translation） |
| `pause_navigation` | - | - | 暂停当前导航 |
| `relocate` | ✅ | - | 重定位到锚点 |
| `start_mapping` | ✅ | - | 开始建图 |
| `end_mapping` | ✅ | - | 结束建图 |
| `add_current_pose_waypoint` | ✅ | - | 添加当前位置为路点 |
| `stop_slam` | ✅ | - | 停止 SLAM |

### 6.4 llm_command_main.cpp — 两种会话模式

**模式 1: `--persistent-navigation-session`（导航会话）**
- 创建 session_token 和 navigation lease（2s 超时）
- 每 500ms 发送心跳维持 lease
- 运动命令（navigate_to_pose, supervised_reposition）必须携带 session_token
- 运行时检查（map_identity 不变、localization 有效、safety 允许）
- 退出时自动 pause_navigation

**模式 2: `--persistent-world-state-session`（世界状态会话）**
- 无 navigation lease
- 仅支持 `get_world_state` 和被动状态查询
- Python 预检用这个轻量模式

### 6.5 DDS 话题数据流

```
Unitree SLAM
    │
    ├── SLAM_INFO_TOPIC (DDS channel)
    │   → StringMsg { type: "pos_info", data: { currentPose: {x,y,z,q_x...}, pcdName, address } }
    │   → slamInfoHandler() 回调
    │       ├── 校验: 四元数范数 [0.5, 1.5]
    │       ├── 校验: 源时间戳单调递增
    │       ├── 更新: current_pose_ (含 map_id, map_path)
    │       ├── 更新: last_pose_update_ms_
    │       └── 更新: distance_to_goal_m
    │
    └── SLAM_KEY_INFO_TOPIC (DDS channel)
        → StringMsg { type: "task_result", data: { is_arrived: true, targetNodeName: "..." } }
        → slamKeyInfoHandler() 回调
            ├── arrived: nav_state_.state = "arrived", is_arrived_ = true
            └── not_arrived: nav_state_.state = "failed"
```

### 6.6 getLocalObstacleSummary 数据源

```cpp
LocalObstacleSummary SlamGateway::getLocalObstacleSummary() {
    return lidar_perception_.getFusedSummaryOrFallback(
        "artifacts/lidar_geometry_summary.json",  // XT16 几何摘要
        1000,                                      // max age ms
        "artifacts/stereo_depth_summary.json",      // D435 深度（兼容）
        1000                                       // max age ms
    );
}
```

**读取逻辑**：
1. 尝试读取 `lidar_geometry_summary.json`（XT16 LiDAR）
2. 如果有效（age_ms < 1000, trusted），使用它
3. 如果无效，尝试读取 `stereo_depth_summary.json`（D435 前向深度）
4. 两者都无效 → 回退到手动设置的默认值（6m 全开放，但标记为 not trusted）
5. 融合：LiDAR 提供 360° 净空，D435 只可降低前向净空

---

## 7. 感知管道：从传感器到安全裁决

### 7.1 数据流图

```
┌──────────┐   ROS2 topic        ┌──────────────────┐   JSON file
│ XT16     │ ═══════════════════►│ xt16_lidar_       │──────────────┐
│ LiDAR    │  /unitree/slam_     │ geometry_summary  │              │
│ 600rpm   │  lidar/points       │ (Python, 5 Hz)    │              │
└──────────┘                     └──────────────────┘              │
                                                                    ▼
┌──────────┐   直接采集           ┌──────────────────┐   JSON    ┌───────────────┐
│ D435     │────────────────────►│ d435_perception_  │─────────►│perception_    │
│ RGBD     │  15 FPS             │ summary           │          │context_v1.json│
│          │                     │ (深度5-10Hz,      │          │               │
└──────────┘                     │  YOLO 3Hz)        │          │ 统一感知上下文 │
                                  └──────────────────┘          │               │
                                                                └───────┬───────┘
┌──────────┐   (future)          ┌──────────────────┐                  │
│ TI Radar │────────────────────►│ nx_ti_radar       │──────────────────┘
│ + NX     │                      │ bridge            │
└──────────┘                     └──────────────────┘
```

### 7.2 Sidecar 服务管理

**XT16 几何服务**：
```bash
# 状态
bash scripts/go2w_xt16_geometry_sidecar.sh status

# 启动（监督发布模式）
GO2W_XT16_SUPERVISED_RELEASE=1 GO2W_XT16_GEOMETRY_CALIBRATED=0 \
  bash scripts/go2w_xt16_geometry_sidecar.sh restart

# 健康检查
bash scripts/go2w_xt16_geometry_sidecar.sh health
# → summary_health=ok age_ms=230 ...
```

**D435 感知服务**：
```bash
bash scripts/go2w_d435_perception_sidecar.sh restart-if-stale
```

**PerceptionContext 服务**：
```bash
pkill -f perception_context_service
cd /home/unitree/Go2W_SLAM_AI
nohup python3 scripts/perception_context_service.py \
  --repo-root /home/unitree/Go2W_SLAM_AI \
  --output artifacts/perception_context_v1.json \
  --interval-ms 250 > /dev/null 2>&1 &
```

### 7.3 传感器状态传播

```
传感器采集 → sidecar 写入 JSON → perception_context 读取 → SensorEnvelope
                                                              │
                                            ┌─────────────────┘
                                            ▼
                              ┌──────────────────────────┐
                              │ status 判定 (5 种)        │
                              │                          │
                              │ fresh:    在线+新鲜       │
                              │ stale:    在线+过期       │
                              │ offline:  进程不在        │
                              │ invalid:  数据格式错误    │
                              │ uncalibrated: 未标定     │
                              └──────────────────────────┘
                                            │
                              ┌─────────────┴─────────────┐
                              ▼                           ▼
                    Python Planner               C++ Gateway
                    (读 perception_context)       (读 lidar_geometry_summary)
                    → WorldState                 → LocalObstacleSummary
                    → LLM Context                → SafetySupervisor
```

---

## 8. 安全架构

### 8.1 安全决策链

```
C++ SafetySupervisor (唯一安全权威)
    │
    ├── 检查 SLAM 健康 (slam_alive, status)
    ├── 检查定位 (localized? pose_age < 500ms?)
    ├── 检查障碍物源 (trusted? lidar_pointcloud?)
    ├── 检查障碍物新鲜度 (age_ms < 1000ms?)
    ├── 检查监督发布 (release_id, max_speed)
    │
    ├── [Supervised Release 激活]
    │   ├── 跳过 per-direction confidence 检查
    │   ├── 前向障碍 → advisory (allow=true, conservative)
    │   ├── 侧向/后向障碍 → advisory (allow=true, conservative)
    │   └── motion_direction = "unitree_pose_navigation_mode_0"
    │
    └── [标准模式]
        ├── front < 0.80m → HARD BLOCK
        ├── side < 0.20m  → HARD BLOCK
        ├── rear < 0.30m  → HARD BLOCK
        └── go_slow → conservative
```

### 8.2 速度限制链（优先级从高到低）

```
1. xt16_supervised_release.json → max_speed_mps = 0.2
2. Gateway SafetySupervisor → safety.speed_limit_mps (conservative 时 = 0.2)
3. Map Registry → topology_node.pose.speed (默认 0.5)
4. CLI --nav-speed-mps

最终 = min(所有层)
当前 effective speed = 0.2 m/s (受 supervised release 限制)
```

### 8.3 拦截门完整列表

| # | 拦截门 | 位置 | 触发条件 |
|---|--------|------|----------|
| 1 | Registry Gate | `run_robot_closed_loop.py:main()` | map_id 非 real，status 非允许 |
| 2 | Topology Gate | `llm_context.py` | 目标节点标签含 disabled/needs_calibration/needs_standing_verification |
| 3 | coarse_map Advisory | `local_llm_planner.py:1255-1270` | PCD 密度采样沿直线超标（已降级为 advisory log） |
| 4 | MissionDecision | `mission_decision.py` | TaskQueue 非法 / 三道门任意未过 |
| 5 | Gateway Preflight | `gateway_safety.py` | gateway response accepted != True |
| 6 | SafetySupervisor | `safety_supervisor.cpp` | SLAM/定位/障碍物/标定 不通过 |
| 7 | mode=0 强制 | `llm_command_processor.cpp:294` | supervised release 下 mode != 0 |
| 8 | map_path 匹配 | `llm_command_processor.cpp:260` | 请求 map_path ≠ 当前 SLAM 加载的地图 |
| 9 | Session Guard | `llm_command_main.cpp` | 导航会话 lease 过期/map identity 变化/localization 失效 |

---

## 9. 世界状态与内部协议

### 9.1 WorldState V1

```json
{
  "schema": "go2w_world_state_v1",
  "timestamp_ms": 1234567890,
  "task_phase": "executing_navigation",
  "localization": {
    "status": "localized",
    "pose_age_ms": 230,
    "map_id": "go2w_real_site"
  },
  "slam_health": {
    "status": "ok",
    "slam_alive": true,
    "localization_alive": true
  },
  "robot_pose": {"x": 1.5, "y": 0.23, "yaw": 1.57},
  "local_obstacle": {
    "front_clearance_m": 3.5,
    "left_clearance_m": 0.45,
    "right_clearance_m": 1.2,
    "rear_clearance_m": 2.8,
    "recommended_action": "normal"
  },
  "safety": {
    "allow_navigation": true,
    "reason": "supervised_unitree_avoidance_available",
    "recommended_mode": "conservative",
    "speed_limit_mps": 0.2
  },
  "navigation": {
    "state": "running",
    "target_node": "zhao_bo_office_front",
    "distance_to_goal_m": 2.3
  },
  "available_tools": ["safe_hold", "ask_human_confirm", "capture_keyframe", "speak"],
  "link_quality": {
    "level": "normal",
    "bandwidth_kbps": 5000
  },
  "perception_sources": {
    "xt16_geometry": "fresh",
    "d435_depth": "fresh",
    "d435_yolo": "stale"
  }
}
```

### 9.2 TaskQueue IR（中间表示）

```json
{
  "queue_id": "q_1234567890",
  "mode": "sequential",
  "status": "planned",
  "source": "llm_fallback",
  "targets": ["zhao_bo_office_front"],
  "steps": [
    {
      "task_id": "step_1",
      "action": "navigate",
      "target_node": "zhao_bo_office_front",
      "status": "pending"
    },
    {
      "task_id": "step_2",
      "action": "wait_until",
      "status": "pending"
    }
  ],
  "communication_policy": {
    "mode": "normal",
    "send": ["task_state", "navigation_feedback"],
    "drop": ["raw_video", "dense_pointcloud"]
  }
}
```

### 9.3 内部协议传输路径

```
PerceptionContext V1 (JSON file)
    → WorldStateBuilder → WorldState V1
    → LlmContextBuilder → Planner Context (LLM input)
    → MissionDecisionBuilder → MissionDecision

TaskQueue IR
    → MissionDecision.verify()
    → ChassisController → Gateway (JSON over stdin/stdout)

Gateway World State (C++ → JSON → Python)
    → gateway_safety.py → allow/deny
    → world_state_v1.py → WorldState V1
    → operator_display.py → UI
    → runtime_log.py → 日志
```

---

## 10. 关键 Topics、API IDs

### 10.1 ROS2 Topics

| Topic | 类型 | 生产者 | 消费者 | 频率 |
|-------|------|--------|--------|------|
| `/unitree/slam_lidar/points` | PointCloud2 | xt16_driver | xt16_lidar_geometry_summary.py | ~10 Hz |
| `/slam_info` | (ROS2) | unitree_slam | (诊断) | ~10 Hz |

### 10.2 DDS Channels

| Channel | 方向 | 数据 | 频率 |
|---------|------|------|------|
| `SLAM_INFO_TOPIC` | unitree_slam → Gateway | pose (x,y,z,q_x,q_y,q_z,q_w) + pcdName + address | ~10 Hz |
| `SLAM_KEY_INFO_TOPIC` | unitree_slam → Gateway | task_result (is_arrived, targetNodeName) | 事件驱动 |

### 10.3 DDS Services (Unitree API)

| API ID | 服务 | 方向 | 参数 |
|--------|------|------|------|
| `ROBOT_API_ID_POSE_NAV_PL` | navigation submit | Gateway → unitree_slam | target_pose (x,y,z,q_x,q_y,q_z,q_w,mode,speed) |
| `ROBOT_API_ID_PAUSE_NAV` | pause | Gateway → unitree_slam | {} |
| `ROBOT_API_ID_RESUME_NAV` | resume | Gateway → unitree_slam | {} |
| `ROBOT_API_ID_STOP_NODE` | stop | Gateway → unitree_slam | {} |
| `ROBOT_API_ID_START_MAPPING_PL` | mapping start | Gateway → unitree_slam | slam_type |
| `ROBOT_API_ID_END_MAPPING_PL` | mapping end | Gateway → unitree_slam | address (PCD path) |
| `ROBOT_API_ID_START_RELOCATION_PL` | relocation | Gateway → unitree_slam | init_pose + address |

### 10.4 JSON 文件（artifacts/）

| 文件 | 生产者 | 消费者 | 更新频率 |
|------|--------|--------|----------|
| `lidar_geometry_summary.json` | xt16_lidar_geometry_summary.py | C++ Gateway (SafetySupervisor) | 5 Hz |
| `d435_perception_summary.json` | d435_perception_summary.py | perception_context_service.py | 深度 5-10 Hz, YOLO ~3 Hz |
| `perception_context_v1.json` | perception_context_service.py | Python Planner, UI, Runtime Log | 4 Hz (250ms) |
| `communication_journal_v1.jsonl` | run_robot_closed_loop.py | 弱网同步 | 事件驱动 |

---

## 11. 数据文件清单

### 11.1 配置文件

| 文件 | 用途 |
|------|------|
| `configs/maps/go2w_real_site_map_registry.json` | 真实场地地图注册（拓扑节点坐标、标签、PCD路径） |
| `configs/maps/go2w_multi_map_registry_v2.json` | 多 PCD 注册表 V2（含过渡锚点） |
| `configs/perception/xt16_geometry_calibration.json` | XT16 标定记录 |
| `configs/perception/xt16_supervised_release.json` | 监督发布记录（max_speed=0.2） |

### 11.2 机器人上关键文件

| 路径 | 用途 |
|------|------|
| `/home/unitree/test.pcd` | PCD 点云地图（SLAM 建图产物） |
| `/home/unitree/Go2W_SLAM_AI/` | 项目根（Git 仓库） |
| `/home/unitree/models/Qwen_Qwen3-4B-Instruct-2507-Q4_K_M.gguf` | 本地 LLM 模型 |
| `/unitree/module/unitree_slam/bin/unitree_slam` | Unitree SLAM 二进制 |
| `/unitree/module/unitree_slam/bin/xt16_driver` | XT16 激光雷达驱动 |

### 11.3 运行时日志

| 路径 | 内容 |
|------|------|
| `artifacts/robot_runs/go2w_agent_*.json` | 导航运行完整日志 |
| `artifacts/slam_stack/xt16_driver.log` | XT16 驱动日志 |
| `artifacts/slam_stack/unitree_slam.log` | Unitree SLAM 日志 |
| `artifacts/xt16_geometry_service/producer.log` | XT16 几何服务日志 |
| `artifacts/communication/communication_journal_v1.jsonl` | 通信 Journal |

---

## 附录：关键函数调用追踪

### 用户命令 "去赵博那" 的函数调用栈

```
main()                                          [run_robot_closed_loop.py]
├── build_snapshot(args)                        [llm_context.py:build_snapshot]
│   └── run_gateway_command(get_world_state)    [chassis_controller.py]
│       └── subprocess: slam_llm_command_client --persistent-world-state-session
│           └── LlmCommandProcessor::process(get_world_state)
│               └── SlamGateway::buildWorldStateJson()
│                   ├── getCurrentPose()         → 锁 + current_pose_
│                   ├── getLocalizationState()   → 基于 pose_age_ms 判定
│                   ├── getSlamHealth()          → 基于 last_pose_age_ms
│                   ├── getNavigationTaskState() → 锁 + nav_state_
│                   ├── getLocalObstacleSummary()→ 读 lidar_geometry_summary.json
│                   └── getSafetyDecision()      → SafetySupervisor::evaluate()
│
├── load_perception_context_file()              [perception_context.py]
│   ├── load_xt16_geometry_envelope()           → SensorEnvelope
│   ├── load_d435_envelopes()                   → [SensorEnvelope x 2]
│   └── validate_perception_context()
│
├── build_planner_context()                      [llm_context.py]
│   ├── 聚合 topology_nodes（从注册表）
│   ├── 聚合 world_state_summary（从 Gateway）
│   ├── 聚合 perception 摘要
│   ├── 聚合 capability_contract
│   ├── 聚合 link_quality
│   └── 构建 LLM prompt context
│
├── run_local_llm_planner()                      [local_llm_planner.py]
│   ├── (hybrid mode) 确定性解析
│   │   └── 匹配 "赵博" → zhao_bo_office_front
│   ├── 或 (full mode) 调 LLM
│   │   └── subprocess: ask_qwen.sh --prompt "..." 
│   │       → Qwen 4B GGUF 推理 → JSON Plan
│   └── 返回 {plan, task_queue, elapsed_s, ...}
│
├── plan_to_slam_command()                       [llm_context.py]
│   └── 从 plan 提取 target_node → 查注册表坐标 → slam_command
│
├── plan_to_task_queue()                         [local_llm_planner.py]
│   └── plan.steps → TaskQueue IR
│
├── build_mission_decision()                     [mission_decision.py]
│   ├── validate_task_queue(task_queue)
│   ├── registry_allows_execution()
│   ├── topology_target_allows_navigation() x N
│   └── gateway_allows_navigation(gateway_state)
│       → 最终决定: execute_queue / hold / reject
│
├── (if execute) run_supervised_navigation_session() [run_robot_closed_loop.py]
│   ├── PersistentGatewaySession(cmd)
│   │   → spawn: --persistent-navigation-session
│   │   → 等待: navigation_session_ready
│   ├── session.command(navigate_to_pose)
│   │   → stdin JSON → C++ LlmCommandProcessor::process()
│   │       ├── 安全校验（token, ack, map_path, safety）
│   │       ├── 速度修正
│   │       └── gateway.submitNavigationGoal(goal)
│   │           ├── callApi(ROBOT_API_ID_POSE_NAV_PL) → DDS → unitree_slam
│   │           └── callApi(ROBOT_API_ID_RESUME_NAV) → 开始运动
│   ├── 导航监控循环
│   │   ├── heartbeat (500ms)
│   │   ├── get_world_state 轮询 (1s)
│   │   ├── 距离/到达检测
│   │   ├── stall/progress 检测
│   │   └── LLM 操作员反馈 (5s)
│   └── close() → pause_navigation → close stdin
│
└── 输出: JSON 或 --summary 简化为 1-3 行
```

---

> 本文档由 Hermes Agent 基于 `E:\GO2W_0` 仓库源码（分支 `agent/llm-on-robot`）编写于 2026-06-16。
> 覆盖文件: `scripts/run_robot_closed_loop.py`, `src/edge_autonomy/*.py`, `robot/slam_gateway_refactor/src/*.cpp`
