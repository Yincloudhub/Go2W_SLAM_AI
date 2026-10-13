# XT16 Geometry Sidecar 重启流程

> 2026-06-16 诊断记录。原始故障：xt16_geometry 显示 `offline`，age_ms=44906638（~12.5 小时无数据）。

## 故障模式对比

| 模式 | perception_context 显示 | 根因 | 修复 |
|------|------------------------|------|------|
| sequence_rollback | `invalid` + `sequence_rollback` | perception_context 内部状态机卡住 | 重启 perception_context_service |
| **offline (本次)** | `offline`，age_ms 巨大 | geometry sidecar 服务未运行 | 重启 go2w_xt16_geometry_sidecar.sh |

## 诊断步骤

```bash
# 1. 确认 XT16 驱动在跑
ps aux | grep xt16_driver

# 2. 确认 geometry sidecar 是否在跑
bash scripts/go2w_xt16_geometry_sidecar.sh status
# 正常输出: xt16_geometry=running pid=...
# 异常输出: xt16_geometry=stopped

# 3. 确认 lidar_geometry_summary.json 是否在更新
stat artifacts/lidar_geometry_summary.json
# 如果 mtime 超过 10 秒 → stale

# 4. 确认 perception_context 里 xt16_geometry 状态
python3 -c "import json; d=json.load(open('artifacts/perception_context_v1.json')); [print(s['source_id'],s['status'],s.get('age_ms')) for s in d['sources']]"
```

## 修复命令

```bash
# 以 supervised release 模式重启 geometry sidecar
GO2W_XT16_SUPERVISED_RELEASE=1 GO2W_XT16_GEOMETRY_CALIBRATED=0 \
  bash scripts/go2w_xt16_geometry_sidecar.sh restart
```

⚠️ **不要用 CALIBRATED=1**：会被拒绝 (`calibration_rejected reason=calibration status is pending_field_measurement, not verified`)。必须用 `CALIBRATED=0` + `SUPERVISED_RELEASE=1` 组合。

重启后验证：
```bash
python3 -c "import json; d=json.load(open('artifacts/perception_context_v1.json')); \
  xt=[s for s in d['sources'] if s['source_id']=='xt16_geometry']; \
  print('status:', xt[0]['status'], 'reasons:', xt[0]['status_reasons']) if xt else print('not found'); \
  print('degraded:', d.get('degraded_capabilities'))"
# 期望: status=fresh reasons=[] degraded=不含xt16_geometry
```

### 为什么需要这两个环境变量？

1. **`GO2W_XT16_SUPERVISED_RELEASE=1`**: XT16 当前标定状态是 `pending_field_measurement`（未完成正式 5 场景标定）。不带此变量时，geometry service 以 `uncalibrated` 模式启动，产出数据被标记 `stale_reasons=['uncalibrated_xt16_geometry']`，Gateway 安全层会拒绝导航。

2. **`GO2W_XT16_GEOMETRY_CALIBRATED=0`**: 跳过标定尝试。默认 `auto` 会先尝试 calibration guard，失败后才 fallback 到 supervised release。设为 0 跳过此步骤加快启动。

### 常见错误

- ❌ 只用 `restart-if-stale`：默认 `SUPERVISED_RELEASE_REQUESTED=0`，以 uncalibrated 模式启动
- ❌ 只重启 perception_context_service：geometry sidecar 是独立服务，重启 PC 不解决问题
- ❌ 忘记设 `GO2W_XT16_GEOMETRY_CALIBRATED=0`：会先等 calibration guard 超时

## 感知上下文深度诊断（perception_context 直接解读）

当 XT16 geometry sidecar 在跑但导航仍被安全层拦（`local_obstacle_not_fresh`），直接查 perception_context：

```bash
cat artifacts/perception_context_v1.json | python3 -c "
import sys,json
d=json.load(sys.stdin)
xt=[s for s in d.get('sources',[]) if s.get('source_id')=='xt16_geometry']
if xt:
    x=xt[0]
    print('status:', x.get('status'))           # 期望 fresh
    print('reasons:', x.get('status_reasons'))   # 期望 []
    print('age_ms:', x.get('age_ms'))            # 期望 <1000
    print('calibration:', x.get('calibration_status'))  # pending 也没关系
print('degraded:', d.get('degraded_capabilities'))  # 不应含 xt16_geometry
"
```

**关键判断**：
- `status_reasons` 为空 → XT16 数据被安全层接受
- `degraded_capabilities` 不含 `xt16_geometry` → 系统不再降级 XT16
- 重启 sidecar 后 perception_context 需要 2-3 秒才会更新状态

```bash
# 等 3 秒后
bash scripts/go2w_xt16_geometry_sidecar.sh health
# 预期: summary_health=ok age_ms=... stale_reasons=[]

python3 -c "
import json
d = json.load(open('artifacts/perception_context_v1.json'))
s = [s for s in d['sources'] if s['source_id'] == 'xt16_geometry'][0]
print(f'status={s[\"status\"]} age_ms={s.get(\"age_ms\")}')
# 预期: status=fresh age_ms=<1000
"
```

## 相关文件

- `scripts/go2w_xt16_geometry_sidecar.sh` — 服务管理器
- `scripts/xt16_lidar_geometry_summary.py` — ROS2 PointCloud2 → JSON 转换
- `configs/perception/xt16_supervised_release.json` — engineering_validated 配置
- `configs/perception/xt16_geometry_calibration.json` — pending_field_measurement 标定状态
