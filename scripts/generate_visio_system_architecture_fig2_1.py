# -*- coding: utf-8 -*-
"""Generate a compact editable Visio system architecture diagram.

The output .vsdx uses native Visio objects only: panels, rounded module boxes,
editable text, and straight arrow segments.
"""

from __future__ import annotations

from pathlib import Path
from typing import Sequence

import win32com.client as win32


ROOT = Path(__file__).resolve().parents[1]
OUT_DIR = ROOT / "docs" / "assets"
VSDX_PATH = OUT_DIR / "fig2_1_system_architecture.vsdx"
PNG_PATH = OUT_DIR / "fig2_1_system_architecture.png"

PAGE_W = 16.0
PAGE_H = 9.0

COLORS = {
    "bg": "RGB(251, 252, 253)",
    "line": "RGB(75, 85, 99)",
    "muted_line": "RGB(102, 112, 125)",
    "text": "RGB(31, 41, 55)",
    "left_group": "RGB(238, 244, 248)",
    "middle_group": "RGB(234, 246, 242)",
    "right_group": "RGB(244, 241, 248)",
    "sensor": "RGB(232, 241, 248)",
    "sar": "RGB(223, 241, 234)",
    "optical": "RGB(234, 243, 255)",
    "fusion": "RGB(221, 245, 255)",
    "world": "RGB(255, 242, 204)",
    "schedule": "RGB(238, 245, 232)",
    "result": "RGB(232, 244, 241)",
    "remote": "RGB(242, 240, 248)",
    "note": "RGB(245, 248, 250)",
}


def cell(shape, name: str, formula: str) -> None:
    try:
        shape.CellsU(name).FormulaU = formula
    except Exception:
        pass


def set_text_style(
    shape,
    size_pt: float,
    *,
    bold: bool = False,
    color: str = COLORS["text"],
    align: str = "center",
    valign: str = "middle",
) -> None:
    align_map = {"left": "0", "center": "1", "right": "2"}
    valign_map = {"top": "0", "middle": "1", "bottom": "2"}
    cell(shape, "Char.Font", 'FONT("Times New Roman")')
    cell(shape, "Char.AsianFont", 'FONT("SimSun")')
    cell(shape, "Char.Size", f"{size_pt} pt")
    cell(shape, "Char.Style", "1" if bold else "0")
    cell(shape, "Char.Color", color)
    cell(shape, "Para.HorzAlign", align_map[align])
    cell(shape, "VerticalAlign", valign_map[valign])
    cell(shape, "TxtAngle", "0 deg")
    cell(shape, "LeftMargin", "0.04 in")
    cell(shape, "RightMargin", "0.04 in")
    cell(shape, "TopMargin", "0.02 in")
    cell(shape, "BottomMargin", "0.02 in")


def draw_text(
    page,
    x1: float,
    y1: float,
    x2: float,
    y2: float,
    text: str,
    size_pt: float,
    *,
    bold: bool = False,
    align: str = "center",
    valign: str = "middle",
    fill: str | None = None,
):
    shape = page.DrawRectangle(x1, y1, x2, y2)
    shape.Text = text
    cell(shape, "LinePattern", "0")
    if fill:
        cell(shape, "FillPattern", "1")
        cell(shape, "FillForegnd", fill)
    else:
        cell(shape, "FillPattern", "0")
    set_text_style(shape, size_pt, bold=bold, align=align, valign=valign)
    return shape


def draw_round_rect(
    page,
    x1: float,
    y1: float,
    x2: float,
    y2: float,
    *,
    fill: str,
    line: str = COLORS["line"],
    weight_pt: float = 1.25,
    rounding_in: float = 0.055,
):
    shape = page.DrawRectangle(x1, y1, x2, y2)
    cell(shape, "FillPattern", "1")
    cell(shape, "FillForegnd", fill)
    cell(shape, "LineColor", line)
    cell(shape, "LineWeight", f"{weight_pt} pt")
    cell(shape, "Rounding", f"{rounding_in} in")
    return shape


def draw_panel(page, x1: float, y1: float, x2: float, y2: float, title: str, fill: str):
    panel = draw_round_rect(page, x1, y1, x2, y2, fill=fill, weight_pt=1.25, rounding_in=0.08)
    draw_text(page, x1 + 0.20, y2 - 0.52, x2 - 0.20, y2 - 0.12, title, 16, bold=True, align="left")
    return panel


def draw_module(
    page,
    x1: float,
    y1: float,
    x2: float,
    y2: float,
    title: str,
    subtitle: str,
    *,
    fill: str,
    title_size: float = 13.5,
    subtitle_size: float = 11.0,
):
    rect = draw_round_rect(page, x1, y1, x2, y2, fill=fill, weight_pt=1.25)
    h = y2 - y1
    draw_text(page, x1 + 0.06, y1 + h * 0.54, x2 - 0.06, y2 - 0.06, title, title_size, bold=True)
    draw_text(page, x1 + 0.06, y1 + 0.06, x2 - 0.06, y1 + h * 0.53, subtitle, subtitle_size)
    return rect


def draw_section_label(page, x1: float, y1: float, x2: float, y2: float, text: str):
    box = draw_round_rect(page, x1, y1, x2, y2, fill=COLORS["note"], line="RGB(148, 163, 184)", weight_pt=0.9)
    draw_text(page, x1 + 0.05, y1 + 0.02, x2 - 0.05, y2 - 0.02, text, 10.5, bold=True)
    return box


def draw_segment(
    page,
    p1: tuple[float, float],
    p2: tuple[float, float],
    *,
    color: str,
    weight_pt: float,
    dashed: bool,
    arrow: bool,
):
    line = page.DrawLine(p1[0], p1[1], p2[0], p2[1])
    cell(line, "LineColor", color)
    cell(line, "LineWeight", f"{weight_pt} pt")
    cell(line, "LinePattern", "2" if dashed else "1")
    cell(line, "BeginArrow", "0")
    cell(line, "EndArrow", "4" if arrow else "0")
    cell(line, "EndArrowSize", "2")
    return line


def draw_arrow(
    page,
    points: Sequence[tuple[float, float]],
    *,
    dashed: bool = False,
    weight_pt: float = 1.35,
    color: str | None = None,
):
    line_color = color or (COLORS["muted_line"] if dashed else COLORS["line"])
    for i, (p1, p2) in enumerate(zip(points, points[1:])):
        draw_segment(
            page,
            p1,
            p2,
            color=line_color,
            weight_pt=weight_pt,
            dashed=dashed,
            arrow=i == len(points) - 2,
        )


def close_prior_generated_documents(visio) -> None:
    for idx in range(visio.Documents.Count, 0, -1):
        doc = visio.Documents.Item(idx)
        full_name = ""
        try:
            full_name = str(doc.FullName)
        except Exception:
            pass

        is_output = full_name.lower() == str(VSDX_PATH).lower()
        is_unsaved_generated = False
        try:
            is_unsaved_generated = not full_name and doc.Pages.Item(1).Name == "Figure 2-1"
        except Exception:
            pass

        if is_output or is_unsaved_generated:
            try:
                doc.Saved = True
            except Exception:
                pass
            doc.Close()


def build_diagram() -> None:
    OUT_DIR.mkdir(parents=True, exist_ok=True)

    visio = win32.gencache.EnsureDispatch("Visio.Application")
    visio.Visible = True
    visio.AlertResponse = 7
    close_prior_generated_documents(visio)

    doc = visio.Documents.Add("")
    page = visio.ActivePage
    page.Name = "Figure 2-1"
    cell(page.PageSheet, "PageWidth", f"{PAGE_W} in")
    cell(page.PageSheet, "PageHeight", f"{PAGE_H} in")
    cell(page.PageSheet, "DrawingScale", "1 in")
    cell(page.PageSheet, "PageScale", "1 in")

    bg = page.DrawRectangle(0, 0, PAGE_W, PAGE_H)
    cell(bg, "FillPattern", "1")
    cell(bg, "FillForegnd", COLORS["bg"])
    cell(bg, "LinePattern", "0")
    bg.SendToBack()

    draw_panel(page, 0.35, 0.70, 3.30, 8.45, "GO2W 移动采集平台", COLORS["left_group"])
    draw_panel(page, 3.55, 0.70, 12.45, 8.45, "Orin NX 边缘计算中心", COLORS["middle_group"])
    draw_panel(page, 12.70, 0.70, 15.65, 8.45, "弱网远端", COLORS["right_group"])

    # Left compact input stack.
    draw_section_label(page, 0.62, 7.25, 3.04, 7.62, "多源采集输入")
    draw_module(page, 0.62, 6.42, 3.04, 7.12, "TI 毫米波雷达", "ADC 回波", fill=COLORS["sensor"], title_size=12.5, subtitle_size=10.5)
    draw_module(page, 0.62, 5.58, 3.04, 6.28, "光学摄像头", "图像帧 / 语义线索", fill=COLORS["sensor"], title_size=12.5, subtitle_size=10.5)
    draw_module(page, 0.62, 4.74, 3.04, 5.44, "惯导 / IMU", "速度 / 姿态", fill=COLORS["sensor"], title_size=12.5, subtitle_size=10.5)
    draw_module(page, 0.62, 3.90, 3.04, 4.60, "位姿 / 里程计", "SLAM 状态", fill=COLORS["sensor"], title_size=12.5, subtitle_size=10.5)
    draw_section_label(page, 0.62, 2.88, 3.04, 3.25, "采集执行边界")
    draw_module(page, 0.62, 2.00, 3.04, 2.76, "GO2W 导航接口", "点位到达 / 路线执行", fill=COLORS["sensor"], title_size=12.5, subtitle_size=10.4)
    draw_text(page, 0.68, 1.28, 2.98, 1.78, "不作为实时底盘控制器", 10.5, fill=None)

    # Middle compact matrix.
    draw_section_label(page, 3.92, 7.50, 6.30, 7.82, "接入层")
    draw_section_label(page, 6.60, 7.50, 12.08, 7.82, "状态层")
    draw_module(page, 3.92, 6.56, 6.30, 7.36, "数据接入与时间同步", "时间戳 / 外参 / 本地缓存", fill=COLORS["result"], title_size=13.0, subtitle_size=10.8)
    draw_module(
        page,
        6.60,
        6.56,
        12.08,
        7.36,
        "结构化 WorldState",
        "位姿 / 任务 / 网络 / 传感器新鲜度 / 成像状态",
        fill=COLORS["world"],
        title_size=13.0,
        subtitle_size=10.8,
    )

    draw_section_label(page, 3.92, 5.56, 9.22, 5.88, "成像与融合处理")
    draw_module(page, 3.92, 4.66, 6.30, 5.46, "实时 SAR 成像", "距离向 / 方位向 FFT", fill=COLORS["sar"], title_size=13.0, subtitle_size=10.8)
    draw_module(page, 3.92, 3.72, 6.30, 4.52, "光学语义种子", "目标检测 / 人工修正", fill=COLORS["optical"], title_size=13.0, subtitle_size=10.8)
    draw_module(page, 6.60, 4.06, 9.22, 5.12, "雷光融合标注", "候选搜索 / 方位映射 / 框修正", fill=COLORS["fusion"], title_size=13.2, subtitle_size=10.8)

    draw_section_label(page, 9.50, 5.56, 12.08, 5.88, "闭环支撑")
    draw_module(page, 9.50, 4.48, 12.08, 5.28, "移动采集调度", "本地 LLM / TaskQueue", fill=COLORS["schedule"], title_size=13.0, subtitle_size=10.8)
    draw_module(page, 9.50, 3.48, 12.08, 4.28, "结果管理", "标签 / 证据索引 / 缓存", fill=COLORS["result"], title_size=13.0, subtitle_size=10.8)
    draw_module(page, 6.60, 1.06, 12.08, 1.88, "弱网本地闭环", "原始数据本地留存，远端仅接收摘要、状态与关键证据", fill=COLORS["note"], title_size=12.6, subtitle_size=10.6)

    # Right compact remote stack.
    draw_section_label(page, 13.02, 7.25, 15.33, 7.62, "远端弱依赖")
    draw_module(page, 13.02, 5.84, 15.33, 6.82, "复核与展示界面", "摘要查看 / 人工确认", fill=COLORS["remote"], title_size=13.0, subtitle_size=10.8)
    draw_module(page, 13.02, 3.50, 15.33, 4.42, "数据集导出", "标注文件 / 证据图", fill=COLORS["remote"], title_size=13.0, subtitle_size=10.8)
    draw_module(page, 13.02, 2.12, 15.33, 3.02, "人工复核回传", "确认结果 / 补采建议", fill=COLORS["remote"], title_size=12.8, subtitle_size=10.6)

    # Straight, sparse data flow.
    bus_x = 3.38
    draw_segment(page, (bus_x, 4.25), (bus_x, 6.78), color=COLORS["line"], weight_pt=1.35, dashed=False, arrow=False)
    for y in (6.77, 5.93, 5.09, 4.25):
        draw_segment(page, (3.04, y), (bus_x, y), color=COLORS["line"], weight_pt=1.25, dashed=False, arrow=False)
    draw_arrow(page, [(bus_x, 6.78), (3.92, 6.96)], weight_pt=1.35)

    draw_arrow(page, [(6.30, 6.96), (6.60, 6.96)], weight_pt=1.35)
    draw_arrow(page, [(5.12, 6.56), (5.12, 5.46)], weight_pt=1.35)
    draw_arrow(page, [(4.34, 6.56), (3.70, 6.56), (3.70, 4.12), (3.92, 4.12)], weight_pt=1.25)
    draw_arrow(page, [(6.30, 5.06), (6.46, 5.06), (6.46, 4.76), (6.60, 4.76)], weight_pt=1.35)
    draw_arrow(page, [(6.30, 4.12), (6.46, 4.12), (6.46, 4.42), (6.60, 4.42)], weight_pt=1.35)
    draw_arrow(page, [(9.22, 4.58), (9.36, 4.58), (9.36, 3.88), (9.50, 3.88)], weight_pt=1.35)
    draw_arrow(page, [(12.08, 3.88), (13.02, 3.96)], weight_pt=1.35)

    # Navigation and state feedback, kept orthogonal and sparse.
    draw_arrow(page, [(3.04, 2.38), (9.32, 2.38), (9.32, 4.88), (9.50, 4.88)], weight_pt=1.20)
    draw_arrow(page, [(11.20, 5.28), (11.20, 6.56)], dashed=True, weight_pt=1.15)
    draw_arrow(page, [(10.85, 6.56), (10.85, 5.28)], dashed=True, weight_pt=1.15)

    # Weak-network exchange uses a shared right-side channel.
    channel_x = 12.42
    draw_segment(page, (channel_x, 1.52), (channel_x, 6.96), color=COLORS["muted_line"], weight_pt=1.05, dashed=True, arrow=False)
    draw_arrow(page, [(12.08, 6.96), (channel_x, 6.96), (channel_x, 6.34), (13.02, 6.34)], dashed=True, weight_pt=1.15)
    draw_arrow(page, [(12.08, 3.88), (channel_x, 3.88), (channel_x, 6.08), (13.02, 6.08)], dashed=True, weight_pt=1.15)
    draw_arrow(page, [(13.02, 2.58), (channel_x, 2.58), (channel_x, 1.52), (12.08, 1.52)], dashed=True, weight_pt=1.15)

    if VSDX_PATH.exists():
        VSDX_PATH.unlink()
    if PNG_PATH.exists():
        PNG_PATH.unlink()

    doc.SaveAs(str(VSDX_PATH))
    page.Export(str(PNG_PATH))
    print(f"saved_vsdx={VSDX_PATH}")
    print(f"saved_png={PNG_PATH}")


if __name__ == "__main__":
    build_diagram()
