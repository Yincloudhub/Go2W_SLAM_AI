from __future__ import annotations

import argparse
import json
import math
import struct
from pathlib import Path
from typing import Any


def parse_pcd_xyz(path: Path) -> list[tuple[float, float, float]]:
    raw = path.read_bytes()
    marker = b"DATA "
    marker_index = raw.find(marker)
    if marker_index < 0:
        raise ValueError("PCD header missing DATA line")
    header_end = raw.find(b"\n", marker_index)
    if header_end < 0:
        raise ValueError("PCD DATA line is not terminated")
    header = raw[:header_end].decode("ascii", errors="replace")
    body = raw[header_end + 1 :]

    fields: list[str] = []
    size: list[int] = []
    type_: list[str] = []
    count: list[int] = []
    points = 0
    data_kind = ""
    for line in header.splitlines():
        parts = line.strip().split()
        if not parts:
            continue
        key = parts[0].upper()
        values = parts[1:]
        if key == "FIELDS":
            fields = values
        elif key == "SIZE":
            size = [int(v) for v in values]
        elif key == "TYPE":
            type_ = values
        elif key == "COUNT":
            count = [int(v) for v in values]
        elif key == "POINTS":
            points = int(values[0])
        elif key == "DATA":
            data_kind = values[0].lower()

    if data_kind != "binary":
        raise ValueError(f"only binary PCD is supported for now, got {data_kind!r}")
    if fields[:3] != ["x", "y", "z"] or size[:3] != [4, 4, 4] or type_[:3] != ["F", "F", "F"]:
        raise ValueError(f"expected first fields x/y/z as float32, got fields={fields} size={size} type={type_}")
    if not count:
        count = [1] * len(fields)

    point_step = sum(s * c for s, c in zip(size, count))
    if len(body) < point_step * points:
        raise ValueError("PCD body is shorter than expected")

    xyz: list[tuple[float, float, float]] = []
    for i in range(points):
        offset = i * point_step
        x, y, z = struct.unpack_from("<fff", body, offset)
        if math.isfinite(x) and math.isfinite(y) and math.isfinite(z):
            xyz.append((x, y, z))
    return xyz


def percentile(values: list[float], pct: float) -> float:
    if not values:
        return 0.0
    ordered = sorted(values)
    index = min(len(ordered) - 1, max(0, round((len(ordered) - 1) * pct)))
    return ordered[index]


def render(points: list[tuple[float, float, float]], output_png: Path, output_meta: Path, *, px_per_m: float, padding_px: int) -> dict[str, Any]:
    from PIL import Image, ImageDraw

    if not points:
        raise ValueError("no valid points")

    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    zs = [p[2] for p in points]
    min_x, max_x = percentile(xs, 0.01), percentile(xs, 0.99)
    min_y, max_y = percentile(ys, 0.01), percentile(ys, 0.99)
    min_z, max_z = percentile(zs, 0.02), percentile(zs, 0.98)
    width = max(100, int((max_x - min_x) * px_per_m) + padding_px * 2)
    height = max(100, int((max_y - min_y) * px_per_m) + padding_px * 2)

    img = Image.new("RGB", (width, height), (246, 248, 250))
    pixels = img.load()
    for x, y, z in points:
        if x < min_x or x > max_x or y < min_y or y > max_y:
            continue
        px = int((x - min_x) * px_per_m) + padding_px
        py = height - (int((y - min_y) * px_per_m) + padding_px)
        z_norm = 0.5 if max_z <= min_z else max(0.0, min(1.0, (z - min_z) / (max_z - min_z)))
        shade = int(65 + 150 * z_norm)
        if 0 <= px < width and 0 <= py < height:
            pixels[px, py] = (shade, shade + 10 if shade < 245 else shade, shade + 18 if shade < 237 else shade)

    draw = ImageDraw.Draw(img)
    # Draw map axes and origin marker if visible.
    def map_to_pixel(x: float, y: float) -> tuple[int, int]:
        return int((x - min_x) * px_per_m) + padding_px, height - (int((y - min_y) * px_per_m) + padding_px)

    ox, oy = map_to_pixel(0.0, 0.0)
    if 0 <= ox < width and 0 <= oy < height:
        draw.line((ox - 10, oy, ox + 10, oy), fill=(220, 40, 40), width=2)
        draw.line((ox, oy - 10, ox, oy + 10), fill=(220, 40, 40), width=2)
        draw.text((ox + 6, oy + 6), "origin", fill=(180, 20, 20))

    meta = {
        "image": str(output_png),
        "width_px": width,
        "height_px": height,
        "px_per_m": px_per_m,
        "padding_px": padding_px,
        "bounds_m": {"min_x": min_x, "max_x": max_x, "min_y": min_y, "max_y": max_y, "min_z": min_z, "max_z": max_z},
        "pixel_to_map": {
            "x": "min_x + (pixel_x - padding_px) / px_per_m",
            "y": "min_y + ((height_px - pixel_y) - padding_px) / px_per_m",
        },
    }
    output_png.parent.mkdir(parents=True, exist_ok=True)
    img.save(output_png)
    output_meta.write_text(json.dumps(meta, ensure_ascii=False, indent=2), encoding="utf-8")
    return meta


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Render binary XYZ PCD as a top-down annotation image.")
    parser.add_argument("pcd")
    parser.add_argument("--output-png", required=True)
    parser.add_argument("--output-meta", required=True)
    parser.add_argument("--px-per-m", type=float, default=80.0)
    parser.add_argument("--padding-px", type=int, default=40)
    return parser


def main() -> int:
    args = build_parser().parse_args()
    points = parse_pcd_xyz(Path(args.pcd))
    meta = render(points, Path(args.output_png), Path(args.output_meta), px_per_m=args.px_per_m, padding_px=args.padding_px)
    print(json.dumps(meta, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
