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

The prone GO2W wheel and leg envelope requires a larger exclusion footprint
than the original prototype:

- front: `0.35 m`
- rear: `0.45 m`
- half-width: `0.40 m`

This removes supported self-return clusters at the rear legs and right wheel.

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

The sidecar remains uncalibrated by default:

```text
GO2W_XT16_GEOMETRY_CALIBRATED=0
```

Static validation does not authorize robot motion. Set the value to `1` only
after a supervised movement test confirms that body, low-hazard, and no-return
semantics remain stable while the robot moves.

Any confirmed rear hazard under `0.6 m` currently produces `pause`. Directional
motion authorization can relax this later, but reverse commands must first
check the rear blocked direction explicitly.
