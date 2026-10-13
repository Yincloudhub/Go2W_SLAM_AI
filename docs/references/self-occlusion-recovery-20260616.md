# XT16 自遮挡 & Recovery 修复 + Mode 陷阱

> 会话: 2026-06-15  
> 提交: `449d619` (自遮挡) + `7d5266d` (评分) + `b0087d6` (mode=1)

## 🔴 导航模式陷阱

**Unitree mode 0 = 直走不绕障，mode 1 = 路径规划+绕障。**

keyDemo 源码 (`/unitree/module/unitree_slam/example/src/keyDemo.cpp`) 默认 `int mode = 1`。但注册表中所有 pose 创建时硬编码了 `"mode": 0`。这意味着从 Day 1 开始所有导航命令都在用直走模式。

### 验证方法
```bash
grep '"mode"' configs/maps/go2w_real_site_map_registry.json
# 所有结果必须是 "mode": 1
```

### 修复
注册表 18 处 `"mode": 0` → `"mode": 1`。现在不传 `--nav-mode` 参数时默认用 1（绕障）。

---

## XT16 自遮挡问题

### 现象
safe_guard 报告三面阻塞: `front=0.07m left=0.002m right=0.033m rear=1.01m`，实际四面无障碍物。

### 根因
XT16 LiDAR 安装在机器狗腹部下方，旋转扫描时看到自身的腿/底盘/机腹。SLAM 系统过滤了这些自遮挡点，但 safe_guard/closed_loop 的 `local_obstacle` 数据直接来自 Gateway C++，**没有自遮挡过滤**。

### 修复
在 `build_recovery_translation_analysis()` 中添加:

```python
SELF_OCCLUSION_RADIUS_M = 0.15  # robot body radius
for direction in ("forward", "left", "right", "backward"):
    clearance = clearances[direction]
    if clearance < SELF_OCCLUSION_RADIUS_M:
        clearance = 2.0  # treat as open space
```

同时修复旋转回退的自遮挡判断 (`check_clearance` 同样加过滤)。

### ⚠️ 注意
- 此修复只影响 **Python recovery 逻辑**，不影响 Gateway C++ 的安全判定
- 如果真有 0.10m 以内的障碍物，recovery 会误判为开放——但这在实际场景中极少
- Gateway 的 `recommended_action` 和 `blocked_directions` 字段仍然是原始值

---

## Reposition 评分修复

### 旧公式问题
```python
score = min(clearance, 2.0) * 0.20 + goal_progress_m * 0.50
```
50% goal_progress 权重过高。后退后 goal_progress 稍差 → 前向（撞墙方向）反而最高分 → 走错方向。

### 新公式
```python
clearance_bonus = min(clearance, 2.0) * 0.35
progress_bonus = max(0.0, goal_progress_m / max(target_distance_m, 0.1)) * 2.0 * 0.35
novelty = 0.3 if direction not in _RECOVERY_HISTORY else 0.0
score = clearance_bonus + progress_bonus + novelty
```

---

## 旋转角度修复

- 旧: `(90, 'right'), (-90, 'left'), (180, 'back')` — 90° 扫过太大空间
- 新: `(45, 'right'), (-45, 'left'), (90, 'back')` — 45° 更安全

---

## PersistentGatewaySession API

构造函数参数: `client_path` (不是 gateway_client)，方法: `command()` (不是 get_world_state)：

```python
from edge_autonomy.chassis_controller import PersistentGatewaySession
s = PersistentGatewaySession(
    client_path='robot/slam_gateway_refactor/build/slam_llm_command_client',
    network_interface='eth0', timeout_s=10)
r = s.command({'action':'get_world_state','map_id':'go2w_real_site','map_path':'/home/unitree/test.pcd'})
pose = r.get('world_state',{}).get('current_pose',{}).get('pose',{})
print(f\"x={pose.get('x'):.4f} y={pose.get('y'):.4f} yaw={pose.get('yaw'):.4f}\")
s.close()
```
