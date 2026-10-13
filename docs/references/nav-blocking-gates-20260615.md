# 导航拦截门诊断 (2026-06-15)

系统有多个独立的拦截门，报错信息不同。被拦后先确定是哪个门，不要混为一谈。

## 拦截门列表（执行顺序）

| # | 拦截门 | 函数/模块 | 典型报错 | 修复 |
|---|--------|----------|---------|------|
| 1 | 导航资格 | `_node_navigation_eligible()` | `target is not verified for live navigation` | 节点 tag 去掉 `needs_calibration`，加 `live_verified` |
| 2 | coarse_map | `apply_context_policy_overrides()` | `all direct paths blocked by walls` | **已降级为 advisory**，不拦截 |
| 3 | registry | `preflight` | `registry does not allow execution` | map status 非 `real`/`simulation` |
| 4 | Gateway 预检 | `gateway_safety` | `gateway rejected` / `slam_health_failed` | 定位丢失、传感器过期、安全策略否决 |

## coarse_map 校验演变

| 提交 | 行为 | 问题 |
|------|------|------|
| `73277fe` | 仅 LLM prompt 提示 | 无强制校验 |
| `2edce2d` | 强制拦截 → human_confirm | **太激进**: 直线采样在走廊环境中误拦所有路径 |
| `e385c87` | advisory-only（建议日志） | 当前版本 |

## 教训

**PCD 直线采样 ≠ 实际可通行性。** Unitree 全局规划器使用完整 PCD 占据栅格找路径，能在走廊中绕行。两个节点之间的直线穿过墙壁是正常的——不代表机器人不能到达。直线检测只适合空旷环境。

## 典型误判案例

用户跑 `nie_guoli_office_front` → 被拦 → 以为是 coarse_map 挡的。实际诊断：
- `BLOCKING_NAVIGATION_TAGS = {"needs_calibration", ...}`
- `nie_guoli_office_front` 的 tags 含 `needs_calibration`
- `_node_navigation_eligible()` 返回 False → plan 降级为 `human_confirm`
- 报错: `target is not verified for live navigation`

**两个门顺序不同、报错不同——不要混为一谈。**
