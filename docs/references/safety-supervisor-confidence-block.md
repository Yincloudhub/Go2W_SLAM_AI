# Safety Supervisor 置信度检查拦截 supervised release 导航

> 2026-06-16 实测发现

## 问题

`safety_supervisor.cpp` 第 79-99 行的三个方向置信度检查（`front_confidence`/`left_confidence`/`right_confidence >= 0.15`）位于 `supervised_release_active` 分支（第 101 行）**之前**，supervised release 无法绕过。

当 XT16 几何服务不填充 `left_confidence` 等字段（为 null）时，Gateway 当成 0 处理 → 不通过置信度检查 → 导航被拦。

**报错信息**:
```
STATUS:BLOCKED
REASON: gateway feedback failed repeatedly: navigation heartbeat rejected:
  navigation_session_safety_blocked:left_obstacle_confidence_too_low
```

## 根因链

```
XT16 几何服务只产生 roi_confidence（per ROI，都是 1.0）
但不产生 front_confidence / left_confidence / right_confidence（per direction，null）
        ↓
Gateway 的 LocalObstacleSummary 中 confidence = null → 当 0
        ↓
safety_supervisor.cpp:86 检查 left_confidence >= 0.15 → false → HOLD
        ↓
supervised_release_active 分支在第 101 行，没机会执行
```

## 可能导致 confidence=null 的原因

1. XT16 几何服务不生成 per-direction confidence（当前实现）
2. 安装的 TI 雷达架子遮挡了 XT16 部分视角，导致某方向有效点数不足
3. XT16 自遮挡（机器狗自己的腿/底盘在视野内）

## 修复

将三个置信度检查包在 `if (!obstacle.supervised_release_active)` 块内。提交 6eafb14：

```cpp
// Per-direction confidence checks are skipped under supervised release.
// Rationale: operator presence + emergency stop provide the safety net,
// and XT16 geometry may report null confidence for directions partially
// occluded by mounted hardware (e.g. TI radar bracket on the back).
// The actual clearance values (front/left/right_clearance_m) are still
// checked below and will catch real obstacles regardless of confidence.
if (!obstacle.supervised_release_active) {
    if (!(obstacle.front_confidence >= kMinimumObstacleConfidence)) { ... }
    if (!(obstacle.left_confidence >= kMinimumObstacleConfidence)) { ... }
    if (!(obstacle.right_confidence >= kMinimumObstacleConfidence)) { ... }
}
```

**理由**：supervised release 已有操作员旁站 + 急停在手 + 0.2m/s 低速。实际障碍仍由 clearance_m 值检查捕获（不受 confidence 影响）。非 supervised 模式保持原检。

## 导航拦截门更新

在原拦截门表中新增：

| 拦截门 | 标志 | 典型 trigger |
|--------|------|-------------|
| 置信度检查 | `left_obstacle_confidence_too_low` / `front_obstacle_confidence_too_low` | XT16 几何服务不填 per-direction confidence，或被安装硬件遮挡 |
