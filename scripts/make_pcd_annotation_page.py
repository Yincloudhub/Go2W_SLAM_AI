from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any


HTML_TEMPLATE = """<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1" />
  <title>PCD topology annotation</title>
  <style>
    body {{
      margin: 0;
      font-family: Arial, "Microsoft YaHei", sans-serif;
      background: #f4f6f8;
      color: #17202a;
      display: grid;
      grid-template-columns: minmax(0, 1fr) 420px;
      height: 100vh;
    }}
    main {{ overflow: auto; padding: 16px; }}
    aside {{
      border-left: 1px solid #d8dee6;
      background: #ffffff;
      padding: 14px;
      overflow: auto;
    }}
    .canvas-wrap {{
      position: relative;
      display: inline-block;
      background: #fff;
      box-shadow: 0 1px 4px rgba(0,0,0,.12);
    }}
    img {{ display: block; max-width: none; }}
    .marker {{
      position: absolute;
      width: 12px;
      height: 12px;
      margin-left: -6px;
      margin-top: -6px;
      border: 2px solid #fff;
      border-radius: 50%;
      background: #0b7cff;
      box-shadow: 0 0 0 2px #0b7cff;
      pointer-events: none;
    }}
    .known {{
      position: absolute;
      transform: translate(-50%, -50%);
      white-space: nowrap;
      font-size: 12px;
      color: #151b23;
      text-shadow: 0 1px 2px #fff, 0 -1px 2px #fff, 1px 0 2px #fff, -1px 0 2px #fff;
      cursor: grab;
      user-select: none;
    }}
    .known.dragging {{ cursor: grabbing; }}
    .known-dot {{
      display: inline-block;
      width: 10px;
      height: 10px;
      border-radius: 50%;
      border: 2px solid #fff;
      background: #e5534b;
      box-shadow: 0 0 0 2px #e5534b;
      vertical-align: middle;
      margin-right: 5px;
    }}
    .known.adjusted .known-dot {{
      background: #22a447;
      box-shadow: 0 0 0 2px #22a447;
    }}
    label {{
      display: block;
      margin: 10px 0 4px;
      font-size: 13px;
      color: #3f4b5a;
    }}
    input, button {{
      box-sizing: border-box;
      width: 100%;
      font: inherit;
    }}
    input {{
      border: 1px solid #c9d1dc;
      border-radius: 6px;
      padding: 8px;
    }}
    button {{
      margin-top: 10px;
      border: 0;
      border-radius: 6px;
      padding: 9px 10px;
      background: #0b7cff;
      color: white;
      cursor: pointer;
    }}
    button.secondary {{ background: #46515f; }}
    pre {{
      white-space: pre-wrap;
      word-break: break-word;
      background: #f5f7fa;
      border: 1px solid #d8dee6;
      border-radius: 6px;
      padding: 8px;
      font-size: 12px;
    }}
    .hint {{
      margin: 8px 0 12px;
      font-size: 12px;
      line-height: 1.45;
      color: #52616f;
    }}
    .row {{
      display: flex;
      align-items: center;
      gap: 8px;
      margin: 8px 0;
      font-size: 13px;
    }}
    .row input {{ width: auto; }}
  </style>
</head>
<body>
  <main>
    <div class="canvas-wrap" id="wrap">
      <img id="map" src="{image_name}" width="{width}" height="{height}" />
    </div>
  </main>
  <aside>
    <h2>PCD topology annotation</h2>
    <div class="hint">
      Red dots are registry nodes. Drag a red dot to correct it; adjusted dots turn green.
      Blue dots are newly clicked points. Use the copied JSON to update the registry.
    </div>
    <div class="row"><input id="dragKnown" type="checkbox" checked /><span>Drag existing registry points</span></div>
    <label>node_id</label>
    <input id="nodeId" value="wp_new" />
    <label>name</label>
    <input id="name" value="new target" />
    <label>aliases, comma separated</label>
    <input id="aliases" value="new target" />
    <label>yaw(rad)</label>
    <input id="yaw" value="0" />
    <label>speed(m/s)</label>
    <input id="speed" value="0.3" />
    <button id="add">Add clicked point</button>
    <button id="copy" class="secondary">Copy JSON</button>
    <button id="copyAdjusted" class="secondary">Copy adjusted registry nodes only</button>
    <label>output</label>
    <pre id="output">Click the map, or drag an existing registry point.</pre>
  </aside>
<script>
const meta = {meta_json};
const knownNodes = {known_nodes_json};
const nodes = [];
const adjusted = new Map();
let last = null;
let dragging = null;
const wrap = document.getElementById('wrap');
const img = document.getElementById('map');
const output = document.getElementById('output');

function pixelToMap(px, py) {{
  const b = meta.bounds_m;
  return {{
    x: b.min_x + (px - meta.padding_px) / meta.px_per_m,
    y: b.min_y + ((meta.height_px - py) - meta.padding_px) / meta.px_per_m
  }};
}}

function mapToPixel(x, y) {{
  const b = meta.bounds_m;
  return {{
    px: (x - b.min_x) * meta.px_per_m + meta.padding_px,
    py: meta.height_px - ((y - b.min_y) * meta.px_per_m + meta.padding_px)
  }};
}}

function eventToImagePixel(event) {{
  const rect = img.getBoundingClientRect();
  const scaleX = img.naturalWidth / rect.width;
  const scaleY = img.naturalHeight / rect.height;
  return {{
    px: (event.clientX - rect.left) * scaleX,
    py: (event.clientY - rect.top) * scaleY
  }};
}}

function cloneNodeWithPose(node, x, y) {{
  const copy = JSON.parse(JSON.stringify(node));
  copy.pose = copy.pose || {{}};
  copy.pose.x = x;
  copy.pose.y = y;
  if (!copy.tags) copy.tags = [];
  if (!copy.tags.includes('needs_calibration')) copy.tags.push('needs_calibration');
  copy.description = (copy.description || 'PCD adjusted registry point') + ' Adjusted in annotation HTML.';
  return copy;
}}

function show(value) {{
  output.textContent = JSON.stringify(value, null, 2);
}}

function addMarker(px, py) {{
  const el = document.createElement('div');
  el.className = 'marker';
  el.style.left = px + 'px';
  el.style.top = py + 'px';
  wrap.appendChild(el);
}}

function addKnownNode(node) {{
  const pose = node.pose || {{}};
  if (typeof pose.x !== 'number' || typeof pose.y !== 'number') return;
  const p = mapToPixel(pose.x, pose.y);
  if (p.px < 0 || p.py < 0 || p.px > meta.width_px || p.py > meta.height_px) return;
  const el = document.createElement('div');
  el.className = 'known';
  el.dataset.nodeId = node.node_id;
  el.style.left = p.px + 'px';
  el.style.top = p.py + 'px';
  el.innerHTML = '<span class="known-dot"></span>' + node.node_id + ' / ' + (node.name || '');
  wrap.appendChild(el);
  el.addEventListener('pointerdown', (event) => {{
    if (!document.getElementById('dragKnown').checked) return;
    event.preventDefault();
    dragging = {{el, node}};
    el.classList.add('dragging');
    el.setPointerCapture(event.pointerId);
  }});
  el.addEventListener('pointermove', (event) => {{
    if (!dragging || dragging.el !== el) return;
    const p = eventToImagePixel(event);
    el.style.left = p.px + 'px';
    el.style.top = p.py + 'px';
    const m = pixelToMap(p.px, p.py);
    const adjustedNode = cloneNodeWithPose(node, m.x, m.y);
    adjusted.set(node.node_id, adjustedNode);
    el.classList.add('adjusted');
    show(Array.from(adjusted.values()));
  }});
  el.addEventListener('pointerup', (event) => {{
    if (!dragging || dragging.el !== el) return;
    el.releasePointerCapture(event.pointerId);
    el.classList.remove('dragging');
    dragging = null;
  }});
}}

for (const node of knownNodes) addKnownNode(node);

img.addEventListener('click', (event) => {{
  if (dragging) return;
  const p = eventToImagePixel(event);
  const m = pixelToMap(p.px, p.py);
  last = {{pixel_x: p.px, pixel_y: p.py, x: m.x, y: m.y}};
  addMarker(p.px, p.py);
  show(last);
}});

document.getElementById('add').addEventListener('click', () => {{
  if (!last) return;
  const nodeId = document.getElementById('nodeId').value.trim();
  const name = document.getElementById('name').value.trim() || nodeId;
  const aliases = document.getElementById('aliases').value.split(',').map(v => v.trim()).filter(Boolean);
  const yaw = Number(document.getElementById('yaw').value || 0);
  const speed = Number(document.getElementById('speed').value || 0.3);
  const qz = Math.sin(yaw / 2);
  const qw = Math.cos(yaw / 2);
  nodes.push({{
    node_id: nodeId,
    name,
    node_type: 'annotated_waypoint',
    aliases,
    tags: ['real_site', 'pcd_annotated', 'needs_calibration'],
    pose: {{x: last.x, y: last.y, z: 0, q_x: 0, q_y: 0, q_z: qz, q_w: qw, speed, mode: 0}},
    description: 'PCD top-down annotation point'
  }});
  show(nodes);
}});

document.getElementById('copy').addEventListener('click', async () => {{
  const payload = [...Array.from(adjusted.values()), ...nodes];
  await navigator.clipboard.writeText(JSON.stringify(payload, null, 2));
  show(payload);
}});

document.getElementById('copyAdjusted').addEventListener('click', async () => {{
  const payload = Array.from(adjusted.values());
  await navigator.clipboard.writeText(JSON.stringify(payload, null, 2));
  show(payload);
}});
</script>
</body>
</html>
"""


def load_known_nodes(registry_path: Path | None, map_id: str | None) -> list[dict[str, Any]]:
    if registry_path is None:
        return []
    registry = json.loads(registry_path.read_text(encoding="utf-8"))
    maps = registry.get("maps", [])
    selected = None
    for item in maps:
        if map_id is None or item.get("map_id") == map_id:
            selected = item
            break
    if not selected:
        return []
    nodes = selected.get("topology_nodes", [])
    return nodes if isinstance(nodes, list) else []


def main() -> int:
    parser = argparse.ArgumentParser(description="Create a local HTML page for PCD top-down point annotation.")
    parser.add_argument("--meta", required=True)
    parser.add_argument("--image", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--registry")
    parser.add_argument("--map-id")
    args = parser.parse_args()

    meta_path = Path(args.meta)
    image_path = Path(args.image)
    output_path = Path(args.output)
    registry_path = Path(args.registry) if args.registry else None
    meta = json.loads(meta_path.read_text(encoding="utf-8"))
    known_nodes = load_known_nodes(registry_path, args.map_id)
    html = HTML_TEMPLATE.format(
        image_name=image_path.name,
        width=meta["width_px"],
        height=meta["height_px"],
        meta_json=json.dumps(meta, ensure_ascii=False),
        known_nodes_json=json.dumps(known_nodes, ensure_ascii=False),
    )
    output_path.write_text(html, encoding="utf-8")
    print(output_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
