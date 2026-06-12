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
- filter-only margin: `0.02 m`

The same nominal offset is used in all four directions. It is not evidence that
the complete moving envelope fits inside a `0.60 x 0.60 m` square. Expanding the
mask to suppress an unexplained return can hide a real obstacle and is
prohibited; shrinking the footprint can overstate reported clearance.

The margin applies only when rejecting self-returns at the body boundary.
Reported obstacle clearance remains measured from the nominal footprint, not
from the expanded filter boundary.
The values remain `pending_field_measurement`, and navigation remains blocked,
until five stationary measured scenes confirm the physical body-edge
relationship and the safety thresholds.

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

Any confirmed rear hazard under `0.6 m` currently produces `pause`. Directional
motion authorization can relax this later, but reverse commands must first
check the rear blocked direction explicitly.
