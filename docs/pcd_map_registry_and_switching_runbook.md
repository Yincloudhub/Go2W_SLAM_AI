# GO2W PCD 地图切换与拓扑点仓库实施手册

> **历史文档（2026-06-12 起停用）**：本文中的双 PCD、`test_current_main`
> 和裸 JSON 重定位示例不得用于当前实机。当前只使用逻辑地图
> `go2w_real_site` 与物理文件 `/home/unitree/test.pcd`；重定位必须通过
> 活动 verified 锚点和监督验收入口完成。

本文档解决一个具体问题：不要再靠记忆管理 `/home/unitree/test.pcd`、`/home/unitree/test513.pcd` 和 `topology_points.json`，而是把它们放进一个“地图集 registry”，由配置文件统一绑定：

1. 当前使用哪张 PCD。
2. 这张 PCD 有哪些可尝试重定位的锚点。
3. 这张 PCD 对应哪些拓扑点。
4. RViz2 应该看哪些 topic。
5. 切换 PCD 时应该发给 `slam_llm_command_client` 什么 JSON。

当前第一版不改宇树官方 `unitree_slam`，也不改机器狗上的 C++ 客户端，只在本仓库新增配置和命令生成工具。

---

## 1. 先明确事实

你实测得到的现象是：

```text
机器狗在建图起点附近可以重定位。
机器狗离建图起点较远时，加载同一张 PCD 可能无法重定位。
```

所以现在不能按下面这种错误方式理解：

```text
只要 PCD 覆盖这个区域，机器狗就一定能在任意位置找到自己。
```

当前应该按下面这种方式使用：

```text
PCD + 初始位姿锚点 + 当前真实摆放位置接近该锚点
```

也就是说，切换 PCD 的本质不是“RViz 换一张图”，而是调用 Unitree SLAM 的重定位接口：

```json
{
  "action": "relocate",
  "map_path": "/home/unitree/test.pcd",
  "initial_pose": {
    "x": 0.0,
    "y": 0.0,
    "z": 0.0,
    "q_x": 0.0,
    "q_y": 0.0,
    "q_z": 0.0,
    "q_w": 1.0
  }
}
```

这里的 `initial_pose` 非常重要。它告诉 SLAM：“我大概从地图里的哪个位置开始匹配”。没有这个初值，远距离全局重定位就很容易失败。

---

## 2. 新增文件

本次新增的核心文件是：

```text
configs/maps/go2w_map_registry.example.json
schemas/map_registry.schema.json
src/edge_autonomy/map_registry.py
scripts/map_registry_cli.py
tests/test_map_registry.py
```

它们的分工如下：

| 文件 | 作用 |
|---|---|
| `configs/maps/go2w_map_registry.example.json` | 地图集配置，记录 PCD、锚点、拓扑点、RViz topic |
| `schemas/map_registry.schema.json` | 地图集 JSON 的结构约束 |
| `src/edge_autonomy/map_registry.py` | 加载地图集、查询地图、生成重定位/导航 JSON |
| `scripts/map_registry_cli.py` | 命令行工具，用来生成可粘贴到机器狗客户端的 JSON |
| `tests/test_map_registry.py` | 防止配置和命令生成逻辑改坏 |

---

## 3. 当前地图集里有什么

当前配置里先登记了两张机器狗上的 PCD：

| map_id | PCD | 状态 |
|---|---|---|
| `test_current_main` | `/home/unitree/test.pcd` | 当前主测试地图 |
| `test513_candidate` | `/home/unitree/test513.pcd` | 候选旧地图 |

`test_current_main` 下面先放了 3 个重定位锚点：

| anchor_id | 含义 | 当前状态 |
|---|---|---|
| `mapping_origin` | 建图起点默认锚点，位姿为原点 | 目前最可靠 |
| `701_entrance_hallway_mid_anchor` | 已保存拓扑点 `wp_0`，语义为 701入口过道中间 | 已完成导航验证，重定位能力仍需单独验证 |
| `nie_guoli_office_front_anchor` | 已保存拓扑点 `wp_1`，语义为聂国篱办公室前方 | 已完成导航验证，重定位能力仍需单独验证 |

同时把已有两个拓扑点改成了更清楚的语义占位：

| node_id | 原始点 | 当前建议含义 |
|---|---|---|
| `701_entrance_hallway_mid` | `wp_0` | 701入口过道中间 |
| `nie_guoli_office_front` | `wp_1` | 聂国篱办公室前方 |

注意：这些是当前现场语义命名，后续如果物理空间重新建图或拓扑点重采，需要重新校准坐标。

---

## 4. 在本机生成切换 PCD 的命令

在 Windows PowerShell 里进入仓库：

```powershell
cd E:\GO2W_0
$env:PYTHONPATH="E:\GO2W_0\src"
```

查看当前登记了哪些地图：

```powershell
python .\scripts\map_registry_cli.py list
```

查看主地图详情：

```powershell
python .\scripts\map_registry_cli.py show --map-id test_current_main
```

生成“加载 `/home/unitree/test.pcd` 并使用建图起点锚点重定位”的 JSON：

```powershell
python .\scripts\map_registry_cli.py relocate --map-id test_current_main --anchor-id mapping_origin
```

输出会是一行 JSON，形态类似：

```json
{"action":"relocate","map_id":"test_current_main","map_path":"/home/unitree/test.pcd","anchor_id":"mapping_origin","initial_pose":{"name":"mapping_origin","x":0.0,"y":0.0,"z":0.0,"q_x":0.0,"q_y":0.0,"q_z":0.0,"q_w":1.0,"yaw":0.0,"speed":0.0,"mode":0}}
```

这就是你要发给机器狗 `slam_llm_command_client` 的内容。

---

## 5. 在机器狗上执行重定位

机器狗上电后，先确认雷达和 SLAM 已启动。你之前已经启动过这两个：

```text
/unitree/module/unitree_slam/bin/xt16_driver
/unitree/module/unitree_slam/bin/unitree_slam
```

然后 SSH 到机器狗：

```powershell
ssh unitree@192.168.123.18
```

进入客户端目录：

```bash
cd /home/unitree/slam_gateway_refactor
```

启动结构化命令客户端：

```bash
./build/slam_llm_command_client eth0
```

把第 4 节生成的一行 JSON 粘贴进去，回车。这个动作才是真正的“切换 PCD + 使用指定初始位姿重定位”。

如果要直接在机器狗上单次执行，也可以用：

```bash
printf '%s\n' '{"action":"relocate","map_path":"/home/unitree/test.pcd","initial_pose":{"x":0.0,"y":0.0,"z":0.0,"q_x":0.0,"q_y":0.0,"q_z":0.0,"q_w":1.0}}' | ./build/slam_llm_command_client eth0
```

---

## 6. RViz2 里怎么看

RViz2 不是“主动切 PCD”的地方。RViz2 是显示当前 ROS2/DDS 桥出来的 topic。

在当前机器狗启动 `unitree_slam` 后，重点看这些：

| RViz Display 类型 | Topic | 作用 |
|---|---|---|
| `PointCloud2` | `/unitree/slam_lidar/points` | 实时雷达点云 |
| `PointCloud2` | `/unitree/slam_relocation/global_map` | 重定位加载后的全局地图 |
| `PointCloud2` | `/unitree/slam_relocation/points` | 重定位过程/定位点云 |
| `PointCloud2` | `/unitree/slam_mapping/points` | 建图过程点云 |
| `Odometry` | `/unitree/slam_relocation/odom` | 重定位后的机器人位姿 |
| `Map` 或 `OccupancyGrid` | `/gridmap` | 栅格地图，如果当前 SLAM 发布 |
| `Map` 或 `OccupancyGrid` | `/planner_map` | 规划地图，如果当前 SLAM 发布 |

切换 PCD 后，RViz 里真正会变化的是 `/unitree/slam_relocation/global_map` 这类 topic 的内容。你不是在 RViz 里选文件，而是先通过 `relocate` 加载 PCD，然后 RViz 显示底层发布出来的新地图。

如果你只是想离线看 PCD 长什么样，推荐先用 CloudCompare 或 Open3D 打开 `.pcd`。RViz2 更适合看“机器人当前正在使用的地图和位姿”。

---

## 7. 拓扑点仓库怎么设置

不要再只用一个全局 `/home/unitree/topology_points.json` 表示所有地图。长期应该改成：

```text
/home/unitree/maps/
  701_room.pcd
  701_room.topology.json
  corridor_7f.pcd
  corridor_7f.topology.json
```

每张 PCD 对应一个拓扑点文件。当前第一版为了兼容已经跑通的客户端，`test_current_main` 仍然指向：

```text
/home/unitree/topology_points.json
```

但我们在仓库的 registry 里已经把它抽象成：

```json
{
  "map_id": "test_current_main",
  "pcd_path": "/home/unitree/test.pcd",
  "topology_path": "/home/unitree/topology_points.json"
}
```

以后你要新增 701 地图，就新增一段：

```json
{
  "map_id": "701_room",
  "name": "701实验室",
  "pcd_path": "/home/unitree/maps/701_room.pcd",
  "topology_path": "/home/unitree/maps/701_room.topology.json",
  "relocalization_anchors": [],
  "topology_nodes": [],
  "topology_edges": []
}
```

---

## 8. 怎么新增一个拓扑点

现在最稳的方式是：

1. 机器狗先在可靠锚点重定位成功。
2. 手动或低速导航到目标位置。
3. 在 RViz 确认位置合理。
4. 读取当前位姿。
5. 把这个点写进 `configs/maps/go2w_map_registry.example.json` 的 `topology_nodes`。
6. 如果这个点也能作为启动重定位点，再写进 `relocalization_anchors`。

拓扑点示例：

```json
{
  "node_id": "701_door_inside",
  "name": "701门内侧",
  "node_type": "door",
  "anchor_id": "701_door_inside_anchor",
  "aliases": ["701门口", "门内"],
  "tags": ["701", "door", "startup_candidate"],
  "pose": {
    "x": 3.20,
    "y": -2.30,
    "z": 0.0,
    "yaw": -1.57,
    "speed": 0.4,
    "mode": 0
  },
  "description": "701门内侧，机器人可从这里进入走廊。"
}
```

如果这个点能用于重定位，再加锚点：

```json
{
  "anchor_id": "701_door_inside_anchor",
  "name": "701门内侧重定位锚点",
  "status": "verified",
  "allowed_radius_m": 1.0,
  "allowed_yaw_error_deg": 20.0,
  "pose": {
    "x": 3.20,
    "y": -2.30,
    "z": 0.0,
    "yaw": -1.57,
    "speed": 0.0,
    "mode": 0
  },
  "description": "机器狗放在 701 门内侧约 1m 内，朝向接近走廊方向时可重定位。"
}
```

---

## 9. 怎么生成到拓扑点的导航命令

生成导航到“聂国篱办公室前方”对应节点的 JSON：

```powershell
python .\scripts\map_registry_cli.py navigate-node --map-id test_current_main --node-id 国篱师兄门口
```

现场低速验证时可以覆盖速度：

```powershell
python .\scripts\map_registry_cli.py navigate-node --map-id test_current_main --node-id 国篱师兄门口 --speed 0.45
```

速度建议：

| 场景 | 建议速度 |
|---|---:|
| 首次验证陌生路线 | `0.15-0.20m/s` |
| 已验证室内短距离且需要减少踏步 | `0.45m/s` |
| 路线稳定但通道较窄 | `0.35m/s` |
| 未做安全层前 | 不建议直接用 `0.8m/s` |

说明：`0.15m/s` 是安全验证速度，对四足机器人会显得像“踏步挪动”，噪声更明显；路线确认后应提高到 `0.45m/s` 左右，运动会更连贯。室内不要为了安静盲目降到极低速，低速反而容易跺脚。

输出类似：

```json
{"action":"navigate_to_pose","map_id":"test_current_main","target_node":"nie_guoli_office_front","target_pose":{"name":"nie_guoli_office_front","x":3.258938789367676,"y":-2.2877299785614014,"z":-0.08693132549524307,"q_x":0.011335774324834347,"q_y":0.013992785476148129,"q_z":-0.722239077091217,"q_w":0.6914089918136597,"yaw":-1.6144882723819336,"speed":0.45,"mode":0}}
```

执行前必须满足：

```text
SLAM 已启动
雷达已启动
当前地图已成功重定位
RViz 中位姿稳定
目标点距离较近
现场有人看护
```

---

## 10. 推荐现场工作流

第一次梳理地图时按这个顺序：

```text
1. 把机器狗放回建图起点附近。
2. 启动雷达和 unitree_slam。
3. 用 mapping_origin 对 /home/unitree/test.pcd 做重定位。
4. RViz2 看 /unitree/slam_relocation/global_map 和 /unitree/slam_relocation/odom。
5. 如果定位稳定，再低速移动到一个明确位置，例如 701门内侧。
6. 保存或读取当前位姿。
7. 把这个点写入 topology_nodes。
8. 把机器狗断电或重启 SLAM 后放回该点附近，测试它能否作为 relocalization_anchor。
9. 能稳定成功 3 次以上，再把 anchor status 改成 verified。
10. 继续扩展下一个点。
```

不要一开始就做很大的地图和很长的拓扑链。先做 3 个点：

```text
起点
701门口
走廊起点
```

这 3 个点跑通以后，LLM 才有可靠的语义地图可用。

---

## 11. 当前最重要的验收标准

第一阶段不要追求“任意点开机”。先验收下面这些：

| 能力 | 合格标准 |
|---|---|
| 地图登记 | `map_registry_cli.py list` 能看到所有 PCD |
| 地图切换 | `relocate` 命令能生成正确 JSON |
| 起点重定位 | `mapping_origin` 附近能稳定成功 |
| RViz 显示 | 能看到全局地图、实时点云、定位 odom |
| 拓扑命名 | `wp_0/wp_1` 被改成有含义的节点 |
| 短距离导航 | 重定位成功后能低速到一个近点 |
| 锚点扩展 | 新锚点需要现场验证后才能标记 `verified` |

这个流程跑通后，项目才真正具备“复用 PCD + 拓扑点仓库 + LLM 规划接口”的基础。
