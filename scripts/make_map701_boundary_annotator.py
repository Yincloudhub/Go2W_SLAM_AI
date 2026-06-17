from __future__ import annotations

import argparse
import base64
import io
import json
import math
import sys
from pathlib import Path
from typing import Any

from PIL import Image, ImageDraw

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from render_pcd_topdown import parse_pcd_xyz  # noqa: E402


DEFAULT_BASE = Path(r"C:\Users\c\Desktop\map_701_matlab_visualization")
DEFAULT_MODEL = DEFAULT_BASE / "map_701_matlab_data.json"
DEFAULT_PCD = DEFAULT_BASE / "robot_pull" / "home__unitree__maps__staging__map_701.pcd"
DEFAULT_OUTPUT = Path(r"C:\Users\c\Desktop\map_701_boundary_annotator.html")


HTML_TEMPLATE = r"""<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>map_701 边界描绘工具</title>
  <style>
    :root {
      color-scheme: light;
      --ink: #17202a;
      --muted: #53606f;
      --line: #d4d9df;
      --panel: #ffffff;
      --bg: #f3f5f6;
      --blue: #1463c2;
      --green: #11864b;
      --orange: #d96c12;
      --purple: #7b2cbf;
      --wall: #24282b;
      --obstacle: #8a8f94;
    }
    * { box-sizing: border-box; }
    body {
      margin: 0;
      font-family: "Microsoft YaHei", "Segoe UI", Arial, sans-serif;
      color: var(--ink);
      background: var(--bg);
      display: grid;
      grid-template-columns: minmax(0, 1fr) 390px;
      height: 100vh;
      overflow: hidden;
    }
    main { overflow: auto; padding: 14px; }
    aside {
      overflow: auto;
      border-left: 1px solid var(--line);
      background: var(--panel);
      padding: 14px;
    }
    h1 { margin: 0 0 10px; font-size: 18px; }
    h2 { margin: 16px 0 8px; font-size: 13px; color: #384351; }
    .hint { color: var(--muted); font-size: 12px; line-height: 1.45; margin-bottom: 10px; }
    .stage-shell {
      display: inline-block;
      position: relative;
      background: #fbfaf6;
      border: 1px solid #b8bec6;
      box-shadow: 0 1px 6px rgba(0,0,0,.12);
    }
    #stage {
      position: relative;
      width: __WIDTH__px;
      height: __HEIGHT__px;
      overflow: hidden;
      cursor: crosshair;
    }
    .bg-layer, #overlay {
      position: absolute;
      left: 0;
      top: 0;
      width: 100%;
      height: 100%;
    }
    #bgTrace { opacity: 1; }
    #bgAuto { opacity: 0; }
    #overlay { z-index: 3; }
    .toolbar {
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: 8px;
      margin: 8px 0 12px;
    }
    label { display: block; margin: 9px 0 4px; font-size: 12px; color: #3d4856; }
    select, input, button, textarea {
      width: 100%;
      font: inherit;
      border-radius: 6px;
    }
    select, input, textarea {
      border: 1px solid #c8d0d8;
      background: #fff;
      padding: 8px;
    }
    textarea {
      min-height: 220px;
      resize: vertical;
      font-family: Consolas, "Courier New", monospace;
      font-size: 11px;
      white-space: pre;
    }
    button {
      border: 0;
      padding: 9px;
      color: #fff;
      background: #1768c9;
      cursor: pointer;
    }
    button.secondary { background: #46515f; }
    button.warn { background: #b44b2b; }
    .row { display: flex; gap: 8px; align-items: center; margin: 8px 0; }
    .row input[type="checkbox"] { width: auto; }
    .small { font-size: 11px; color: var(--muted); }
    .stats {
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: 6px;
      font-size: 12px;
      color: #354252;
    }
    .pill {
      border: 1px solid #d9dee5;
      border-radius: 6px;
      padding: 6px;
      background: #f7f8fa;
    }
    .node-label {
      pointer-events: none;
      paint-order: stroke;
      stroke: rgba(255,255,255,.9);
      stroke-width: 4px;
      stroke-linejoin: round;
      fill: #111820;
      font-size: 12px;
    }
    .feature-label {
      pointer-events: none;
      paint-order: stroke;
      stroke: rgba(255,255,255,.9);
      stroke-width: 4px;
      stroke-linejoin: round;
      fill: #18202a;
      font-size: 13px;
      font-weight: 600;
    }
    .vertex { cursor: grab; }
    .vertex:active { cursor: grabbing; }
  </style>
</head>
<body>
  <main>
    <div class="stage-shell">
      <div id="stage">
        <img id="bgTrace" class="bg-layer" src="__TRACE_IMAGE_URI__" alt="PCD trace base">
        <img id="bgAuto" class="bg-layer" src="__AUTO_IMAGE_URI__" alt="auto floorplan">
        <svg id="overlay" viewBox="0 0 __WIDTH__ __HEIGHT__" xmlns="http://www.w3.org/2000/svg"></svg>
      </div>
    </div>
  </main>
  <aside>
    <h1>map_701 边界描绘工具</h1>
    <div class="hint">
      在底图上点击描点。墙/隔断默认是折线，大件遮挡默认是闭合多边形。
      画完一条后点“完成当前边界”，最后下载 JSON。
    </div>
    <div class="stats">
      <div class="pill">坐标系: map_701 / map</div>
      <div class="pill">点位: 8</div>
      <div class="pill" id="coord">x=?, y=?</div>
      <div class="pill" id="count">features=0</div>
    </div>
    <label>绘制类型</label>
    <select id="kind">
      <option value="wall">墙 / 隔断折线</option>
      <option value="large_obstruction">大件遮挡多边形</option>
      <option value="no_go_boundary">不可通行边界</option>
      <option value="door_opening">门洞 / 通行口</option>
    </select>
    <label>标签</label>
    <input id="label" value="">
    <div class="toolbar">
      <button id="finish">完成当前边界</button>
      <button id="undoPoint" class="secondary">撤销上一点</button>
      <button id="undoFeature" class="secondary">撤销上一条</button>
      <button id="clearCurrent" class="warn">清空当前</button>
    </div>
    <div class="row">
      <input id="showNodes" type="checkbox" checked>
      <span>显示 8 个拓扑点</span>
    </div>
    <div class="row">
      <input id="showAuto" type="checkbox">
      <span>叠加自动高度底图</span>
    </div>
    <div class="row">
      <input id="snapGrid" type="checkbox">
      <span>吸附到 0.05 m 网格</span>
    </div>
    <h2>继续编辑已有 JSON</h2>
    <input id="loadFile" type="file" accept=".json,application/json">
    <h2>导出</h2>
    <div class="toolbar">
      <button id="download">下载 JSON</button>
      <button id="copy" class="secondary">复制 JSON</button>
    </div>
    <div class="small">
      下载文件名会是 <code>map_701_manual_boundaries.json</code>。保存到桌面后我可以直接读取并重画 MATLAB fig。
    </div>
    <label>当前 JSON</label>
    <textarea id="jsonOut" spellcheck="false"></textarea>
  </aside>
<script>
const model = __MODEL_JSON__;
const traceMeta = __TRACE_META_JSON__;
const W = traceMeta.width_px;
const H = traceMeta.height_px;
const bounds = traceMeta.bounds_m;
let features = [];
let current = [];
let dragState = null;

const stage = document.getElementById("stage");
const overlay = document.getElementById("overlay");
const kindEl = document.getElementById("kind");
const labelEl = document.getElementById("label");
const jsonOut = document.getElementById("jsonOut");
const coordEl = document.getElementById("coord");
const countEl = document.getElementById("count");

function isClosedKind(kind) {
  return kind === "large_obstruction";
}

function colorForKind(kind) {
  if (kind === "wall") return "#24282b";
  if (kind === "large_obstruction") return "#7e858a";
  if (kind === "no_go_boundary") return "#b6422e";
  if (kind === "door_opening") return "#0c8a52";
  return "#1768c9";
}

function nodeColor(type) {
  if (type === "transition") return "#7b2cbf";
  if (type === "corridor_endpoint") return "#11864b";
  if (type === "rotation_point") return "#d96c12";
  return "#1463c2";
}

function mapToPx(pt) {
  return {
    x: (pt.x - bounds.min_x) / (bounds.max_x - bounds.min_x) * W,
    y: H - (pt.y - bounds.min_y) / (bounds.max_y - bounds.min_y) * H
  };
}

function pxToMap(px) {
  let x = bounds.min_x + px.x / W * (bounds.max_x - bounds.min_x);
  let y = bounds.min_y + (H - px.y) / H * (bounds.max_y - bounds.min_y);
  if (document.getElementById("snapGrid").checked) {
    x = Math.round(x / 0.05) * 0.05;
    y = Math.round(y / 0.05) * 0.05;
  }
  return {x, y};
}

function eventPx(event) {
  const rect = stage.getBoundingClientRect();
  return {
    x: (event.clientX - rect.left) * W / rect.width,
    y: (event.clientY - rect.top) * H / rect.height
  };
}

function esc(value) {
  return String(value ?? "").replace(/[&<>"']/g, ch => ({
    "&": "&amp;", "<": "&lt;", ">": "&gt;", "\"": "&quot;", "'": "&#39;"
  }[ch]));
}

function pathData(points, closed) {
  if (!points.length) return "";
  const px = points.map(mapToPx);
  let d = `M ${px[0].x.toFixed(1)} ${px[0].y.toFixed(1)}`;
  for (let i = 1; i < px.length; i++) d += ` L ${px[i].x.toFixed(1)} ${px[i].y.toFixed(1)}`;
  if (closed && points.length > 2) d += " Z";
  return d;
}

function centroid(points) {
  if (!points.length) return {x: 0, y: 0};
  return {
    x: points.reduce((s, p) => s + p.x, 0) / points.length,
    y: points.reduce((s, p) => s + p.y, 0) / points.length
  };
}

function payload() {
  return {
    version: 1,
    map_id: "map_701",
    frame_id: "map",
    created_for: "matlab_office_floorplan",
    source: {
      desktop_model: "C:/Users/c/Desktop/map_701_matlab_visualization/map_701_matlab_data.json",
      remote_pcd: model.source?.remote_pcd || "/home/unitree/maps/staging/map_701.pcd"
    },
    features
  };
}

function updateJson() {
  jsonOut.value = JSON.stringify(payload(), null, 2);
  countEl.textContent = `features=${features.length}`;
}

function renderNodes(parts) {
  if (!document.getElementById("showNodes").checked) return;
  for (const n of model.nodes || []) {
    const p = mapToPx(n);
    const c = nodeColor(n.primary_type);
    const marker = n.primary_type === "transition" ? "polygon" : "circle";
    if (marker === "polygon") {
      parts.push(`<polygon points="${p.x},${p.y - 9} ${p.x + 8},${p.y + 7} ${p.x - 8},${p.y + 7}" fill="${c}" stroke="#111" stroke-width="1.5"></polygon>`);
    } else {
      parts.push(`<circle cx="${p.x}" cy="${p.y}" r="6" fill="${c}" stroke="#fff" stroke-width="2"></circle>`);
    }
    const yaw = Number(n.yaw || 0);
    const x2 = p.x + Math.cos(yaw) * 34;
    const y2 = p.y - Math.sin(yaw) * 34;
    parts.push(`<line x1="${p.x}" y1="${p.y}" x2="${x2}" y2="${y2}" stroke="#c91f1f" stroke-width="2"></line>`);
    parts.push(`<text class="node-label" x="${p.x + 8}" y="${p.y - 8}">${esc(n.node_id)}</text>`);
  }
}

function renderFeature(feature, featureIndex, parts) {
  const kind = feature.kind;
  const closed = Boolean(feature.closed);
  const color = colorForKind(kind);
  const d = pathData(feature.points, closed);
  if (!d) return;
  const fill = closed ? color : "none";
  const opacity = kind === "large_obstruction" ? 0.48 : 0.18;
  const strokeWidth = kind === "wall" ? 8 : 5;
  parts.push(`<path d="${d}" fill="${fill}" fill-opacity="${closed ? opacity : 0}" stroke="${color}" stroke-width="${strokeWidth}" stroke-linecap="round" stroke-linejoin="round"></path>`);
  feature.points.forEach((pt, pointIndex) => {
    const p = mapToPx(pt);
    parts.push(`<circle class="vertex" data-feature="${featureIndex}" data-point="${pointIndex}" cx="${p.x}" cy="${p.y}" r="5" fill="#fff" stroke="${color}" stroke-width="2"></circle>`);
  });
  if (feature.label) {
    const c = mapToPx(centroid(feature.points));
    parts.push(`<text class="feature-label" x="${c.x + 6}" y="${c.y - 6}">${esc(feature.label)}</text>`);
  }
}

function render() {
  const parts = [];
  renderNodes(parts);
  features.forEach((f, i) => renderFeature(f, i, parts));
  if (current.length) {
    const f = {
      kind: kindEl.value,
      label: labelEl.value,
      closed: isClosedKind(kindEl.value),
      points: current
    };
    renderFeature(f, -1, parts);
  }
  overlay.innerHTML = parts.join("");
  bindVertexDrag();
  updateJson();
}

function finishCurrent() {
  if (current.length < 2) return;
  const kind = kindEl.value;
  const feature = {
    id: `${kind}_${String(features.length + 1).padStart(3, "0")}`,
    kind,
    label: labelEl.value.trim(),
    closed: isClosedKind(kind),
    points: current.map(p => ({x: Number(p.x.toFixed(4)), y: Number(p.y.toFixed(4))}))
  };
  features.push(feature);
  current = [];
  labelEl.value = "";
  render();
}

function bindVertexDrag() {
  for (const el of overlay.querySelectorAll(".vertex")) {
    el.addEventListener("pointerdown", event => {
      event.preventDefault();
      dragState = {
        feature: Number(el.dataset.feature),
        point: Number(el.dataset.point)
      };
      el.setPointerCapture(event.pointerId);
    });
    el.addEventListener("pointermove", event => {
      if (!dragState) return;
      const pt = pxToMap(eventPx(event));
      if (dragState.feature >= 0) {
        features[dragState.feature].points[dragState.point] = pt;
      } else {
        current[dragState.point] = pt;
      }
      render();
    });
    el.addEventListener("pointerup", event => {
      dragState = null;
      el.releasePointerCapture(event.pointerId);
    });
  }
}

stage.addEventListener("pointermove", event => {
  const pt = pxToMap(eventPx(event));
  coordEl.textContent = `x=${pt.x.toFixed(2)}, y=${pt.y.toFixed(2)}`;
});

stage.addEventListener("click", event => {
  if (dragState) return;
  if (event.target.classList && event.target.classList.contains("vertex")) return;
  const pt = pxToMap(eventPx(event));
  current.push({x: Number(pt.x.toFixed(4)), y: Number(pt.y.toFixed(4))});
  render();
});

stage.addEventListener("dblclick", event => {
  event.preventDefault();
  finishCurrent();
});

document.getElementById("finish").addEventListener("click", finishCurrent);
document.getElementById("undoPoint").addEventListener("click", () => { current.pop(); render(); });
document.getElementById("clearCurrent").addEventListener("click", () => { current = []; render(); });
document.getElementById("undoFeature").addEventListener("click", () => { features.pop(); render(); });
document.getElementById("showNodes").addEventListener("change", render);
document.getElementById("showAuto").addEventListener("change", event => {
  document.getElementById("bgAuto").style.opacity = event.target.checked ? "0.75" : "0";
});
kindEl.addEventListener("change", render);
labelEl.addEventListener("input", render);

document.addEventListener("keydown", event => {
  if (event.key === "Enter" && current.length) finishCurrent();
  if (event.key === "Escape") { current = []; render(); }
  if ((event.ctrlKey || event.metaKey) && event.key.toLowerCase() === "z") {
    if (current.length) current.pop(); else features.pop();
    render();
  }
});

document.getElementById("download").addEventListener("click", () => {
  const blob = new Blob([JSON.stringify(payload(), null, 2)], {type: "application/json"});
  const a = document.createElement("a");
  a.href = URL.createObjectURL(blob);
  a.download = "map_701_manual_boundaries.json";
  a.click();
  URL.revokeObjectURL(a.href);
});

document.getElementById("copy").addEventListener("click", async () => {
  await navigator.clipboard.writeText(JSON.stringify(payload(), null, 2));
});

document.getElementById("loadFile").addEventListener("change", async event => {
  const file = event.target.files[0];
  if (!file) return;
  const text = await file.text();
  const loaded = JSON.parse(text);
  features = Array.isArray(loaded.features) ? loaded.features : [];
  current = [];
  render();
});

render();
</script>
</body>
</html>
"""


def _bounds_from_model(model: dict[str, Any], margin_m: float) -> dict[str, float]:
    if model.get("floorplan_meta", {}).get("bounds_m"):
        return {k: float(v) for k, v in model["floorplan_meta"]["bounds_m"].items()}
    xs = [float(item["x"]) for item in model.get("nodes", [])]
    ys = [float(item["y"]) for item in model.get("nodes", [])]
    for region in model.get("regions", []):
        xs.extend([float(region["x_min"]), float(region["x_max"])])
        ys.extend([float(region["y_min"]), float(region["y_max"])])
    return {
        "min_x": min(xs) - margin_m,
        "max_x": max(xs) + margin_m,
        "min_y": min(ys) - margin_m,
        "max_y": max(ys) + margin_m,
    }


def _encode_png(image: Image.Image) -> str:
    buf = io.BytesIO()
    image.save(buf, format="PNG")
    return "data:image/png;base64," + base64.b64encode(buf.getvalue()).decode("ascii")


def _image_file_to_data_uri(path: Path) -> str:
    if not path.exists():
        return _encode_png(Image.new("RGBA", (1, 1), (255, 255, 255, 0)))
    return "data:image/png;base64," + base64.b64encode(path.read_bytes()).decode("ascii")


def _percentile(values: list[float], pct: float) -> float:
    if not values:
        return 0.0
    ordered = sorted(values)
    idx = min(len(ordered) - 1, max(0, round((len(ordered) - 1) * pct)))
    return ordered[idx]


def render_trace_base(
    model: dict[str, Any],
    pcd_path: Path,
    *,
    width_px: int,
    margin_m: float,
) -> tuple[Image.Image, dict[str, Any]]:
    bounds = _bounds_from_model(model, margin_m)
    aspect = (bounds["max_y"] - bounds["min_y"]) / (bounds["max_x"] - bounds["min_x"])
    height_px = int(round(width_px * aspect))
    img = Image.new("RGB", (width_px, height_px), (252, 250, 244))
    draw = ImageDraw.Draw(img)

    def map_to_px(x: float, y: float) -> tuple[int, int]:
        px = int((x - bounds["min_x"]) / (bounds["max_x"] - bounds["min_x"]) * width_px)
        py = int(height_px - (y - bounds["min_y"]) / (bounds["max_y"] - bounds["min_y"]) * height_px)
        return px, py

    # Grid first: light 0.25 m grid and stronger 1 m grid.
    x0 = math.floor(bounds["min_x"] * 4) / 4
    while x0 <= bounds["max_x"]:
        px, _ = map_to_px(x0, bounds["min_y"])
        color = (218, 216, 209) if abs(x0 - round(x0)) < 1e-6 else (236, 234, 228)
        draw.line((px, 0, px, height_px), fill=color, width=1)
        x0 += 0.25
    y0 = math.floor(bounds["min_y"] * 4) / 4
    while y0 <= bounds["max_y"]:
        _, py = map_to_px(bounds["min_x"], y0)
        color = (218, 216, 209) if abs(y0 - round(y0)) < 1e-6 else (236, 234, 228)
        draw.line((0, py, width_px, py), fill=color, width=1)
        y0 += 0.25

    points = [
        p
        for p in parse_pcd_xyz(pcd_path)
        if bounds["min_x"] <= p[0] <= bounds["max_x"] and bounds["min_y"] <= p[1] <= bounds["max_y"]
    ]
    zs = [p[2] for p in points]
    z_low = _percentile(zs, 0.03)
    z_high = _percentile(zs, 0.97)
    z_span = max(0.1, z_high - z_low)
    for x, y, z in points:
        t = max(0.0, min(1.0, (z - z_low) / z_span))
        # Low points remain faint, high vertical structure becomes dark.
        shade = int(214 - 150 * t)
        color = (shade, min(232, shade + 8), min(238, shade + 14))
        px, py = map_to_px(x, y)
        if 0 <= px < width_px and 0 <= py < height_px:
            img.putpixel((px, py), color)

    # Axes and origin.
    ox, oy = map_to_px(0.0, 0.0)
    draw.line((ox - 24, oy, ox + 48, oy), fill=(45, 45, 45), width=2)
    draw.line((ox, oy + 24, ox, oy - 48), fill=(45, 45, 45), width=2)
    draw.text((ox + 8, oy + 8), "origin", fill=(60, 60, 60))
    draw.rectangle((0, 0, width_px - 1, height_px - 1), outline=(160, 165, 170), width=1)

    meta = {
        "width_px": width_px,
        "height_px": height_px,
        "bounds_m": bounds,
        "pcd_points_in_bounds": len(points),
        "note": "Height-shaded raw PCD crop for manual boundary tracing.",
    }
    return img, meta


def main() -> int:
    parser = argparse.ArgumentParser(description="Create a standalone HTML tool for manually tracing map_701 boundaries.")
    parser.add_argument("--model", default=str(DEFAULT_MODEL))
    parser.add_argument("--pcd", default=str(DEFAULT_PCD))
    parser.add_argument("--output", default=str(DEFAULT_OUTPUT))
    parser.add_argument("--width-px", type=int, default=1320)
    parser.add_argument("--margin-m", type=float, default=0.65)
    args = parser.parse_args()

    model = json.loads(Path(args.model).read_text(encoding="utf-8"))
    trace_img, trace_meta = render_trace_base(
        model,
        Path(args.pcd),
        width_px=args.width_px,
        margin_m=args.margin_m,
    )
    auto_uri = _image_file_to_data_uri(Path(model.get("floorplan_background_png", "")))
    html = (
        HTML_TEMPLATE.replace("__WIDTH__", str(trace_meta["width_px"]))
        .replace("__HEIGHT__", str(trace_meta["height_px"]))
        .replace("__TRACE_IMAGE_URI__", _encode_png(trace_img))
        .replace("__AUTO_IMAGE_URI__", auto_uri)
        .replace("__MODEL_JSON__", json.dumps(model, ensure_ascii=False))
        .replace("__TRACE_META_JSON__", json.dumps(trace_meta, ensure_ascii=False))
    )
    output = Path(args.output)
    output.write_text(html, encoding="utf-8")
    print(output)
    print(f"pcd_points_in_bounds={trace_meta['pcd_points_in_bounds']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
