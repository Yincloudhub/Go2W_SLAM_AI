# 旋转路点约定 (2026-06-16)

## 设计原则
- 每个需要拐弯的地方放一个旋转路点，选在**开阔空间**（走廊交叉口、门前空地）
- 相邻路点之间必须**直视可达**（无障碍物遮挡），mode=0 走直线
- LLM 规划路径时自动穿过旋转路点，不做硬编码路由

## 路点命名
`rotation_<区域>_<描述>`，如 `rotation_corridor_junction`、`rotation_701_door_exit`

## 路点元数据
每个旋转路点在注册表中需标注：
```json
{
  "node_id": "rotation_xxx",
  "type": "rotation_waypoint",
  "pre_rotation_safe": true,
  "open_area": true,
  "exit_direction": "north",
  "description": "走廊交叉口，四周开阔，可原地旋转"
}
```

## 导航流程（代码待实现）
1. 到达旋转路点 → 停稳
2. 计算轴承误差（当前朝向 vs 下一路点方向）
3. < 30° → 跳过转体，直接发下一段导航
4. ≥ 30° → 检查侧向净空 → 发送 0.3m 近点 `navigate_to_pose` 完成转体
5. 净空不够（不该发生）→ reposition 后退 0.25m → 重试

## 待改文件
- `scripts/run_robot_closed_loop.py` — 插入 `pre_navigation_alignment()`
- `src/edge_autonomy/llm_context.py` — prompt 增加旋转路点说明
