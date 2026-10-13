# PCD/SLAM 坐标轴错位诊断

> 2026-06-16 实机验证发现

## 症状

- 导航命令执行后，机器人往**相反方向或错误象限**移动
- PCD 里两个节点的坐标差值和物理位置对不上
- 用户说"聂国篱在我前方（+Y），但机器人往后方走（-Y）"

## 根因

PCD 建图时的 Y 轴指向可能与 Unitree SLAM 的 Y 轴指向**相反**（差 180°）。
也可能是建图时机器狗的初始朝向记录错误。

**关键判断**：不是 SLAM 定位失准——`yin_siyuan_station` 报的坐标是对的。是注册表里其他节点的坐标按 PCD 的 Y 轴给的，但那个 Y 轴方向和你认为的前向不一致。

## 诊断

```bash
# 1. 确认 SLAM 定位正常
bash scripts/go2w_accept.sh status

# 2. 确认当前物理位置对应的注册坐标是否合理
python3 -c "
import json
r = json.load(open('configs/maps/go2w_real_site_map_registry.json'))
for m in r['maps']:
    for n in m['topology_nodes']:
        p = n['pose']
        print(f\"{n['node_id']}: ({p['x']:.2f}, {p['y']:.2f})\")
"

# 3. 对比：物理上从 A 走到 B 的方向 vs PCD 里 A→B 的向量
```

## 修复

**不能在代码里修（改坐标系会影响所有节点）**。必须**重标定受影响节点**的注册坐标：

```bash
# 机器狗推到节点实际物理位置（SLAM 已定位）：
bash scripts/go2w_accept.sh snapshot <node_id>
```

如果多个节点坐标都偏了，回到 `mapping_origin` 用 SLAM 重新标定所有节点。

## 教训

- 建完 PCD 后，不要假设坐标系方向已对齐
- 第一次导航前，先跑短距已知路径（如 yin_siyuan → 最近走廊口）验证方向
- 如果机器人走的方向明显不对——先怀疑 PCD 坐标轴，不要怀疑代码逻辑
