from __future__ import annotations

import argparse
import json
import sys
from collections import deque
from pathlib import Path
from typing import Any

import numpy as np
from PIL import Image, ImageDraw

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from render_pcd_topdown import parse_pcd_xyz  # noqa: E402


DEFAULT_BASE = Path(r"C:\Users\c\Desktop\map_701_matlab_visualization")
DEFAULT_MODEL = DEFAULT_BASE / "map_701_matlab_data.json"
DEFAULT_PCD = DEFAULT_BASE / "robot_pull" / "home__unitree__maps__staging__map_701.pcd"
DEFAULT_OUTPUT = DEFAULT_BASE / "map_701_office_floorplan_background.png"
DEFAULT_META = DEFAULT_BASE / "map_701_office_floorplan_meta.json"


def _node_bounds(model: dict[str, Any], margin_m: float) -> dict[str, float]:
    xs = [float(n["x"]) for n in model["nodes"]]
    ys = [float(n["y"]) for n in model["nodes"]]
    for region in model.get("regions", []):
        xs.extend([float(region["x_min"]), float(region["x_max"])])
        ys.extend([float(region["y_min"]), float(region["y_max"])])
    return {
        "min_x": min(xs) - margin_m,
        "max_x": max(xs) + margin_m,
        "min_y": min(ys) - margin_m,
        "max_y": max(ys) + margin_m,
    }


def _neighbor_count(mask: np.ndarray, radius: int = 1) -> np.ndarray:
    padded = np.pad(mask.astype(np.uint8), radius, mode="constant")
    out = np.zeros(mask.shape, dtype=np.uint16)
    size = radius * 2 + 1
    for dy in range(size):
        for dx in range(size):
            out += padded[dy : dy + mask.shape[0], dx : dx + mask.shape[1]]
    return out


def _dilate(mask: np.ndarray, radius: int = 1) -> np.ndarray:
    return _neighbor_count(mask, radius) > 0


def _erode(mask: np.ndarray, radius: int = 1) -> np.ndarray:
    return _neighbor_count(mask, radius) == (radius * 2 + 1) ** 2


def _connected_components(mask: np.ndarray) -> list[list[tuple[int, int]]]:
    height, width = mask.shape
    seen = np.zeros(mask.shape, dtype=bool)
    comps: list[list[tuple[int, int]]] = []
    for y in range(height):
        for x in range(width):
            if not mask[y, x] or seen[y, x]:
                continue
            q: deque[tuple[int, int]] = deque([(y, x)])
            seen[y, x] = True
            comp: list[tuple[int, int]] = []
            while q:
                cy, cx = q.popleft()
                comp.append((cy, cx))
                for ny in range(max(0, cy - 1), min(height, cy + 2)):
                    for nx in range(max(0, cx - 1), min(width, cx + 2)):
                        if (ny == cy and nx == cx) or seen[ny, nx] or not mask[ny, nx]:
                            continue
                        seen[ny, nx] = True
                        q.append((ny, nx))
            comps.append(comp)
    return comps


def _component_summary(
    comp: list[tuple[int, int]],
    cell_size_m: float,
    bounds: dict[str, float],
    *,
    kind: str,
) -> dict[str, Any]:
    ys = [p[0] for p in comp]
    xs = [p[1] for p in comp]
    min_col, max_col = min(xs), max(xs)
    min_row, max_row = min(ys), max(ys)
    width_m = (max_col - min_col + 1) * cell_size_m
    height_m = (max_row - min_row + 1) * cell_size_m
    area_m2 = len(comp) * cell_size_m * cell_size_m
    return {
        "kind": kind,
        "cells": len(comp),
        "area_m2": round(area_m2, 3),
        "bbox": {
            "x_min": round(bounds["min_x"] + min_col * cell_size_m, 3),
            "x_max": round(bounds["min_x"] + (max_col + 1) * cell_size_m, 3),
            "y_min": round(bounds["min_y"] + min_row * cell_size_m, 3),
            "y_max": round(bounds["min_y"] + (max_row + 1) * cell_size_m, 3),
        },
    }


def build_floorplan(
    model: dict[str, Any],
    pcd_path: Path,
    output_png: Path,
    output_meta: Path,
    *,
    cell_size_m: float,
    px_per_cell: int,
    threshold: int,
    min_component_cells: int,
    tall_span_m: float,
    object_span_m: float,
    object_min_cells: int,
    margin_m: float,
) -> dict[str, Any]:
    bounds = _node_bounds(model, margin_m)
    width = int(np.ceil((bounds["max_x"] - bounds["min_x"]) / cell_size_m))
    height = int(np.ceil((bounds["max_y"] - bounds["min_y"]) / cell_size_m))
    counts = np.zeros((height, width), dtype=np.uint16)
    z_values: list[list[list[float]]] = [[[] for _ in range(width)] for _ in range(height)]

    points = parse_pcd_xyz(pcd_path)
    selected = 0
    for x, y, z in points:
        if not (bounds["min_x"] <= x <= bounds["max_x"] and bounds["min_y"] <= y <= bounds["max_y"]):
            continue
        col = int((x - bounds["min_x"]) / cell_size_m)
        row = int((y - bounds["min_y"]) / cell_size_m)
        if 0 <= row < height and 0 <= col < width:
            counts[row, col] += 1
            z_values[row][col].append(float(z))
            selected += 1

    z_p05 = np.full((height, width), np.nan, dtype=np.float32)
    z_p95 = np.full((height, width), np.nan, dtype=np.float32)
    z_span = np.zeros((height, width), dtype=np.float32)
    for row in range(height):
        for col in range(width):
            vals = z_values[row][col]
            if not vals:
                continue
            ordered = np.array(sorted(vals), dtype=np.float32)
            p05 = ordered[int(round((len(ordered) - 1) * 0.05))]
            p95 = ordered[int(round((len(ordered) - 1) * 0.95))]
            z_p05[row, col] = p05
            z_p95[row, col] = p95
            z_span[row, col] = p95 - p05

    high_density = counts >= threshold
    neighbor = _neighbor_count(high_density, radius=1)
    vertical = (counts >= max(4, threshold - 2)) & (z_span >= tall_span_m)
    wall_candidate = vertical & (neighbor >= 2)
    wall_candidate = _dilate(wall_candidate, radius=1)
    wall_candidate = _erode(wall_candidate, radius=1)

    object_candidate = (
        (counts >= max(4, threshold - 3))
        & (z_span >= object_span_m)
        & ~wall_candidate
    )
    object_candidate = _dilate(object_candidate, radius=1)
    object_candidate = _erode(object_candidate, radius=1)

    wall_mask = np.zeros_like(wall_candidate)
    object_mask = np.zeros_like(object_candidate)
    summaries: list[dict[str, Any]] = []
    for comp in _connected_components(wall_candidate):
        if len(comp) < min_component_cells:
            continue
        summary = _component_summary(comp, cell_size_m, bounds, kind="wall_or_partition")
        summaries.append(summary)
        for row, col in comp:
            wall_mask[row, col] = True

    for comp in _connected_components(object_candidate & ~wall_mask):
        if len(comp) < object_min_cells:
            continue
        summary = _component_summary(comp, cell_size_m, bounds, kind="large_obstruction")
        summaries.append(summary)
        for row, col in comp:
            object_mask[row, col] = True

    scale = px_per_cell
    img = Image.new("RGB", (width * scale, height * scale), (250, 249, 245))
    draw = ImageDraw.Draw(img)
    fine_grid = (232, 231, 226)
    major_grid = (212, 211, 205)
    for col in range(width + 1):
        x = col * scale
        color = major_grid if col % round(1.0 / cell_size_m) == 0 else fine_grid
        draw.line((x, 0, x, height * scale), fill=color, width=1)
    for row in range(height + 1):
        y = height * scale - row * scale
        color = major_grid if row % round(1.0 / cell_size_m) == 0 else fine_grid
        draw.line((0, y, width * scale, y), fill=color, width=1)

    # Draw retained structural cells. Image row 0 is top, grid row 0 is min_y.
    for row in range(height):
        for col in range(width):
            y0 = (height - row - 1) * scale
            x0 = col * scale
            if wall_mask[row, col]:
                fill = (54, 58, 61)
            elif object_mask[row, col]:
                fill = (136, 141, 145)
            else:
                continue
            draw.rectangle((x0, y0, x0 + scale - 1, y0 + scale - 1), fill=fill)

    # Outline connected structures for a plan-drawing look.
    keep = wall_mask | object_mask
    edge_mask = keep & (_neighbor_count(keep, radius=1) < 9)
    for row in range(height):
        for col in range(width):
            if not edge_mask[row, col]:
                continue
            y0 = (height - row - 1) * scale
            x0 = col * scale
            draw.rectangle((x0, y0, x0 + scale - 1, y0 + scale - 1), outline=(35, 38, 40), width=1)

    output_png.parent.mkdir(parents=True, exist_ok=True)
    img.save(output_png)
    meta = {
        "image": str(output_png),
        "bounds_m": bounds,
        "cell_size_m": cell_size_m,
        "px_per_cell": px_per_cell,
        "grid_width": width,
        "grid_height": height,
        "threshold_points_per_cell": threshold,
        "min_component_cells": min_component_cells,
        "tall_span_m": tall_span_m,
        "object_span_m": object_span_m,
        "object_min_cells": object_min_cells,
        "pcd_points_total": len(points),
        "pcd_points_in_bounds": selected,
        "wall_cells": int(wall_mask.sum()),
        "large_obstruction_cells": int(object_mask.sum()),
        "occupied_cells_raw": int(high_density.sum()),
        "occupied_cells_retained": int(keep.sum()),
        "components_retained": summaries,
        "note": "PCD-derived height-aware top-down occupancy; dark cells indicate likely walls/partitions, mid-gray cells indicate large obstructions.",
    }
    output_meta.write_text(json.dumps(meta, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return meta


def main() -> int:
    parser = argparse.ArgumentParser(description="Build map_701 office-style floorplan background from PCD occupancy.")
    parser.add_argument("--model", default=str(DEFAULT_MODEL))
    parser.add_argument("--pcd", default=str(DEFAULT_PCD))
    parser.add_argument("--output-png", default=str(DEFAULT_OUTPUT))
    parser.add_argument("--output-meta", default=str(DEFAULT_META))
    parser.add_argument("--cell-size-m", type=float, default=0.08)
    parser.add_argument("--px-per-cell", type=int, default=9)
    parser.add_argument("--threshold", type=int, default=8)
    parser.add_argument("--min-component-cells", type=int, default=18)
    parser.add_argument("--tall-span-m", type=float, default=0.85)
    parser.add_argument("--object-span-m", type=float, default=0.45)
    parser.add_argument("--object-min-cells", type=int, default=18)
    parser.add_argument("--margin-m", type=float, default=0.65)
    args = parser.parse_args()

    model = json.loads(Path(args.model).read_text(encoding="utf-8"))
    meta = build_floorplan(
        model,
        Path(args.pcd),
        Path(args.output_png),
        Path(args.output_meta),
        cell_size_m=args.cell_size_m,
        px_per_cell=args.px_per_cell,
        threshold=args.threshold,
        min_component_cells=args.min_component_cells,
        tall_span_m=args.tall_span_m,
        object_span_m=args.object_span_m,
        object_min_cells=args.object_min_cells,
        margin_m=args.margin_m,
    )
    print(f"wrote {meta['image']}")
    print(
        "cells raw={raw} retained={retained} components={components}".format(
            raw=meta["occupied_cells_raw"],
            retained=meta["occupied_cells_retained"],
            components=len(meta["components_retained"]),
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
