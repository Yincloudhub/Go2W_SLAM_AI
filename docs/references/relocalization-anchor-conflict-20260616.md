# Relocalization Anchor Conflict

> Date: 2026-06-16
> Root cause: Anchors present in both active and archive registries

## Problem

Adding a new relocalization anchor (e.g., `yin_siyuan_station`, `nie_guoli_office_front`) to
`go2w_real_site_map_registry.json` causes accept.sh to fail:

```
accepted: False
reason: map 'go2w_real_site' has anchors present in both active and archive: 
  nie_guoli_office_front, yin_siyuan_station
```

This blocks even `mapping_origin` relocation.

## Root Cause

`go2w_accept.sh` checks both:
- Active: `configs/maps/go2w_real_site_map_registry.json`
- Archive: `configs/maps/go2w_real_site_map_registry_v1_archived.json` (V1 backup)

If an `anchor_id` exists in BOTH, it's a conflict. The original V1 archive has topology nodes
that mirror the active registry, and adding an anchor with the same ID creates a duplicate.

## Fix

**Revert**: Remove newly added anchors from active registry, keeping only `mapping_origin`:

```python
import json
path = 'configs/maps/go2w_real_site_map_registry.json'
reg = json.load(open(path))
for m in reg['maps']:
    if m['map_id'] == 'go2w_real_site':
        m['relocalization_anchors'] = [
            a for a in m['relocalization_anchors'] 
            if a['anchor_id'] == 'mapping_origin'
        ]
json.dump(reg, open(path, 'w'), indent=2, ensure_ascii=False)
```

## Current State

Only `mapping_origin` is available for relocation. If new anchors are needed:
1. Remove the conflicting entry from the archived V1 copy first
2. Then add to active registry
3. Verify with `go2w_accept.sh relocate <anchor_id> confirm`
