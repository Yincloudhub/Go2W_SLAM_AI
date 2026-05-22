from __future__ import annotations

import argparse
import json
from pathlib import Path


HTML_TEMPLATE = """<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1" />
  <title>PCD 拓扑点标注</title>
  <style>
    body {{
      margin: 0;
      font-family: Arial, "Microsoft YaHei", sans-serif;
      background: #f4f6f8;
      color: #17202a;
      display: grid;
      grid-template-columns: minmax(0, 1fr) 360px;
      height: 100vh;
    }}
    main {{
      overflow: auto;
      padding: 16px;
    }}
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
    img {{
      display: block;
      max-width: none;
    }}
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
    label {{
      display: block;
      margin: 10px 0 4px;
      font-size: 13px;
      color: #3f4b5a;
    }}
    input, textarea, button {{
      box-sizing: border-box;
      width: 100%;
      font: inherit;
    }}
    input, textarea {{
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
    button.secondary {{
      background: #46515f;
    }}
    pre {{
      white-space: pre-wrap;
      word-break: break-word;
      background: #f5f7fa;
      border: 1px solid #d8dee6;
      border-radius: 6px;
      padding: 8px;
      font-size: 12px;
    }}
  </style>
</head>
<body>
  <main>
    <div class="canvas-wrap" id="wrap">
      <img id="map" src="{image_name}" width="{width}" height="{height}" />
    </div>
  </main>
  <aside>
    <h2>拓扑点标注</h2>
    <label>node_id</label>
    <input id="nodeId" value="wp_new" />
    <label>中文名称</label>
    <input id="name" value="新目标点" />
    <label>别名，逗号分隔</label>
    <input id="aliases" value="新目标点" />
    <label>yaw(rad)</label>
    <input id="yaw" value="0" />
    <label>speed(m/s)</label>
    <input id="speed" value="0.3" />
    <button id="add">加入当前点击点</button>
    <button id="copy" class="secondary">复制 JSON</button>
    <label>输出</label>
    <pre id="output">点击地图选择点。</pre>
  </aside>
<script>
const meta = {meta_json};
const nodes = [];
let last = null;
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

function addMarker(px, py) {{
  const el = document.createElement('div');
  el.className = 'marker';
  el.style.left = px + 'px';
  el.style.top = py + 'px';
  wrap.appendChild(el);
}}

img.addEventListener('click', (event) => {{
  const rect = img.getBoundingClientRect();
  const scaleX = img.naturalWidth / rect.width;
  const scaleY = img.naturalHeight / rect.height;
  const px = (event.clientX - rect.left) * scaleX;
  const py = (event.clientY - rect.top) * scaleY;
  const m = pixelToMap(px, py);
  last = {{pixel_x: px, pixel_y: py, x: m.x, y: m.y}};
  addMarker(px, py);
  output.textContent = JSON.stringify(last, null, 2);
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
    tags: ['real_site', 'pcd_annotated'],
    pose: {{x: last.x, y: last.y, z: 0, q_x: 0, q_y: 0, q_z: qz, q_w: qw, speed, mode: 0}},
    description: 'PCD俯视图点击标注点'
  }});
  output.textContent = JSON.stringify(nodes, null, 2);
}});

document.getElementById('copy').addEventListener('click', async () => {{
  await navigator.clipboard.writeText(JSON.stringify(nodes, null, 2));
}});
</script>
</body>
</html>
"""


def main() -> int:
    parser = argparse.ArgumentParser(description="Create a local HTML page for PCD top-down point annotation.")
    parser.add_argument("--meta", required=True)
    parser.add_argument("--image", required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()

    meta_path = Path(args.meta)
    image_path = Path(args.image)
    output_path = Path(args.output)
    meta = json.loads(meta_path.read_text(encoding="utf-8"))
    html = HTML_TEMPLATE.format(
        image_name=image_path.name,
        width=meta["width_px"],
        height=meta["height_px"],
        meta_json=json.dumps(meta, ensure_ascii=False),
    )
    output_path.write_text(html, encoding="utf-8")
    print(output_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
