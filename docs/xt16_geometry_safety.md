# XT16 Geometry Safety Contract

## Confirmed body-frame mapping

Static differential tests on 2026-06-06 confirmed:

- physical forward: raw point-cloud `-Y`
- physical rear: raw point-cloud `+Y`
- physical left: raw point-cloud `+X`
- physical right: raw point-cloud `-X`
- vertical: raw point-cloud `+Z`

These values are now the producer defaults.

## Footprint

The current pending footprint is a provisional symmetric estimate based on the
centrally mounted XT16:

- front: `0.30 m`
- rear: `0.30 m`
- half-width: `0.30 m`
- filter-only margin: `0.05 m`

The same nominal offset is used in all four directions. It is not evidence that
the complete moving envelope fits inside a `0.60 x 0.60 m` square. Expanding the
mask to suppress an unexplained return can hide a real obstacle and is
prohibited; shrinking the footprint can overstate reported clearance.

The margin applies only when rejecting self-returns at the body boundary and at
or above `body_min_z_m`. Low hazards below that height are rejected only inside
the nominal footprint, so cables or floor-level obstacles immediately outside
the body remain visible. Reported obstacle clearance remains measured from the
nominal footprint, not from the expanded filter boundary.
The values remain `pending_field_measurement`, and navigation remains blocked,
until five stationary measured scenes confirm the physical body-edge
relationship and the safety thresholds.

The footprint rejection is already active in `src/edge_autonomy/xt16_geometry.py`.
Each summary reports `points_excluded_footprint`; no second point-cloud filter
should be added. The next field action is to measure the distance from the XT16
center to the front, rear, left, and right physical body edges, update the four
footprint values, and then collect five stationary measured scenes.

Earlier static work did include all four directions. The 2026-06-06 artifacts
confirmed the axis mapping with front, left, right, and rear boxes. They used a
`0.35 m` front/rear and `0.32 m` half-width footprint plus the earlier
single-percentile algorithm. The current implementation uses a `0.30 m`
symmetric footprint, a `0.05 m` filter margin, spatial cluster support, and a
separate low-hazard band. The older artifacts remain valid axis evidence but
cannot certify the current geometry implementation.

When physical measurement is inconvenient, first collect a stationary corridor
baseline with the read-only local viewer:

```powershell
.\.venv\Scripts\python.exe scripts\visualize_xt16_over_ssh.py `
  --host 192.168.123.18 `
  --topic /utlidar/cloud `
  --record-jsonl artifacts\xt16_visual\corridor_baseline.jsonl
```

This raw topic is available without starting SLAM or Gateway. It is useful for
inspecting body self-returns. Final runtime comparison must later repeat against
`/unitree/slam_lidar/points`, which is the processed topic consumed by the XT16
geometry sidecar.

## 2026-06-13 Stationary Corridor Baseline

The robot was rebooted and placed stationary in a corridor with open front and
rear space and at least `0.8 m` physical side clearance. Only `xt16_driver` was
started temporarily; Unitree SLAM, Gateway, D435, and chassis motion remained
stopped.

Processed `/unitree/slam_lidar/points` results over 34 frames:

```text
points_per_frame: about 62,100
front_clearance_m: 6.0
left_clearance_m: median 1.025
right_clearance_m: median 1.003
rear_clearance_m with 0.02 m margin: median 0.021
```

The rear return was a symmetric 127-point self cluster:

```text
rear clearance from nominal body edge: 0.000-0.041 m
lateral position: approximately -0.12 m and +0.12 m
vertical position: -0.096 to -0.084 m
```

Same-frame margin comparison:

```text
0.02 m -> rear 0.020 m
0.03 m -> rear 0.032 m
0.04 m -> rear 0.042 m
0.05 m -> rear no return inside 6.0 m
0.06 m -> no additional points removed
0.08 m -> no additional points removed
```

The minimum stable body-height filter-only margin is therefore `0.05 m`. The
nominal footprint remains `0.30 m`; obstacle clearance is still reported from
that nominal edge. The low-hazard band does not use the extra margin.
Calibration remains `pending_field_measurement`.

### Post-deployment no-motion verification

Commit `28340c41fb6300865b19177301f34aaa6b47bb0a` was fast-forwarded to the
robot and verified with another 34 processed frames:

```text
points_per_frame median: 62,027
left_clearance_m median: 1.024
right_clearance_m median: 1.003
rear_body_clearance_m: 6.0 in 33/34 frames
rear_low_hazard_clearance_m: no supported return
points_excluded_footprint median: 913.5
front_clearance_m: null
```

The rear body cluster was removed without converting a low-hazard return into
clear space. The single rear frame without a confirmed value remained stale.
The open forward direction had no supported ROI return in this capture, so it
correctly remained `null` with `missing_required_roi:front`; no-return was not
guessed as clear. This result accepts the body-boundary filter change but does
not complete XT16 field calibration.

The ignored local evidence file is
`artifacts/xt16_visual/20260613_corridor_rslidar_margin05_20s.jsonl`, SHA-256
`270A042800A7966B622C5BFF4C77C186008E88B172F51A81763E76EE05EFDAF8`.

## Schema version 2

Legacy directional fields remain authoritative and backward compatible:

```json
{
  "front_clearance_m": 1.2,
  "left_clearance_m": 0.5,
  "right_clearance_m": 6.0,
  "rear_clearance_m": 0.9
}
```

Each legacy value is the conservative minimum of:

- `body_clearance_m`: supported obstacle clusters above `body_min_z_m`
- `low_hazard_clearance_m`: supported cable and near-ground clusters

Additional fields expose class-specific confidence, pending candidates, and
directions where a healthy cloud had no return inside the configured range.
Unknown class values are JSON `null`; consumers must not infer them from the
legacy aggregate.

## Cluster support

Clearance is calculated from the nearest supported distance cluster, not from
one percentile over the complete directional ROI. A supported cluster requires:

- at least `min_points_per_roi` points
- at least `min_spatial_bins` occupied lateral/height bins

Repeated identical points cannot create a confirmed obstacle. A smaller near
cluster becomes a pending candidate. Pending candidates inside the directional
stop envelope make the summary stale so the gateway remains fail-closed.

For a healthy full XT16 cloud, a direction with no supported or pending return
inside the range is reported clear to `range_m` and listed under
`summary.body_no_return_directions`.

## Runtime policy

The sidecar uses verified-record auto mode by default:

```text
GO2W_XT16_GEOMETRY_CALIBRATED=auto
```

`auto` enables calibrated output only when the repository record passes the
complete guard: verified status, non-empty identity, five accepted artifacts
with hashes, accepted error, and exact runtime-parameter equality. A pending or
invalid record stays uncalibrated and navigation remains fail-closed. Value `0`
can still force diagnostic uncalibrated mode; value `1` makes a guard failure a
startup error.

### Corridor clearance policy v1

Clearance thresholds are measured from the nominal body edge after self-return
filtering, not from the XT16 center:

| Direction | Conservative speed | Pause |
|---|---:|---:|
| Front | `< 1.50 m` | `< 0.80 m` |
| Left/right | `< 0.60 m` | `< 0.20 m` |
| Rear | `< 0.50 m` | `< 0.30 m` |

Side clearance below `0.8 m` is no longer a global navigation block. A corridor
with supported side returns between `0.20 m` and `0.60 m` remains navigable, but
Gateway returns `recommended_mode=conservative` and clamps the submitted
navigation speed to `0.20 m/s`. A confirmed side return below `0.20 m` still
pauses. Unitree navigation remains in obstacle-avoidance mode `0`.

This policy does not override stale, uncalibrated, low-confidence, or missing
directional data. The 2026-06-13 corridor capture still has
`missing_required_roi:front`, so it cannot authorize movement until formal XT16
calibration and a supervised no-motion preflight pass.
