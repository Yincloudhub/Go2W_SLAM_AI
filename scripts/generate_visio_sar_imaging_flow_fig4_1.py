# -*- coding: utf-8 -*-
"""Generate an editable portrait Visio flowchart for real-time SAR imaging."""

from __future__ import annotations

from pathlib import Path
from typing import Sequence

import win32com.client as win32


ROOT = Path(__file__).resolve().parents[1]
OUT_DIR = ROOT / "docs" / "assets"
VSDX_PATH = OUT_DIR / "fig4_1_realtime_sar_imaging_flow.vsdx"
PNG_PATH = OUT_DIR / "fig4_1_realtime_sar_imaging_flow.png"

PAGE_W = 9.0
PAGE_H = 16.0

COLORS = {
    "bg": "RGB(250, 252, 253)",
    "line": "RGB(71, 85, 105)",
    "muted": "RGB(100, 116, 139)",
    "text": "RGB(30, 41, 59)",
    "band": "RGB(246, 248, 250)",
    "input": "RGB(234, 242, 248)",
    "pre": "RGB(232, 246, 243)",
    "fft": "RGB(234, 243, 255)",
    "motion": "RGB(255, 243, 205)",
    "output": "RGB(232, 245, 233)",
    "remote": "RGB(242, 240, 248)",
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
    align: str = "center",
    valign: str = "middle",
    color: str = COLORS["text"],
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
    set_text_style(shape, size_pt, bold=bold, align=align)
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
    weight_pt: float = 1.2,
    rounding_in: float = 0.055,
):
    shape = page.DrawRectangle(x1, y1, x2, y2)
    cell(shape, "FillPattern", "1")
    cell(shape, "FillForegnd", fill)
    cell(shape, "LineColor", line)
    cell(shape, "LineWeight", f"{weight_pt} pt")
    cell(shape, "Rounding", f"{rounding_in} in")
    return shape


def draw_band(page, x1: float, y1: float, x2: float, y2: float, label: str):
    band = draw_round_rect(
        page,
        x1,
        y1,
        x2,
        y2,
        fill=COLORS["band"],
        line="RGB(203, 213, 225)",
        weight_pt=0.8,
        rounding_in=0.04,
    )
    draw_text(page, x1 + 0.10, y2 - 0.34, x1 + 1.75, y2 - 0.08, label, 10.5, bold=True, align="left")
    return band


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
    subtitle_size: float = 10.5,
):
    rect = draw_round_rect(page, x1, y1, x2, y2, fill=fill, weight_pt=1.2)
    h = y2 - y1
    draw_text(page, x1 + 0.06, y1 + h * 0.52, x2 - 0.06, y2 - 0.06, title, title_size, bold=True)
    draw_text(page, x1 + 0.06, y1 + 0.06, x2 - 0.06, y1 + h * 0.52, subtitle, subtitle_size)
    return rect


def draw_segment(
    page,
    p1: tuple[float, float],
    p2: tuple[float, float],
    *,
    dashed: bool,
    arrow: bool,
    weight_pt: float,
    color: str,
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
    weight_pt: float = 1.5,
    color: str | None = None,
):
    line_color = color or (COLORS["muted"] if dashed else COLORS["line"])
    for i, (p1, p2) in enumerate(zip(points, points[1:])):
        draw_segment(
            page,
            p1,
            p2,
            dashed=dashed,
            arrow=i == len(points) - 2,
            weight_pt=weight_pt,
            color=line_color,
        )


def draw_label(page, x: float, y: float, text: str, *, w: float = 0.96, h: float = 0.25):
    return draw_text(page, x - w / 2, y - h / 2, x + w / 2, y + h / 2, text, 9.2, fill=None)


def draw_fft_icon(page, x: float, y: float, w: float = 0.34, h: float = 0.22):
    base_y = y - h / 2
    for i, height in enumerate((0.08, 0.16, 0.11, 0.19, 0.10)):
        px = x - w / 2 + 0.05 + i * 0.06
        draw_segment(
            page,
            (px, base_y + 0.035),
            (px, base_y + 0.035 + height),
            dashed=False,
            arrow=False,
            weight_pt=0.8,
            color="RGB(59, 130, 246)",
        )
    draw_segment(
        page,
        (x - w / 2 + 0.03, base_y + 0.035),
        (x + w / 2 - 0.03, base_y + 0.035),
        dashed=False,
        arrow=False,
        weight_pt=0.7,
        color=COLORS["muted"],
    )


def draw_grid_icon(page, x: float, y: float, w: float = 0.58, h: float = 0.38):
    left, right = x - w / 2, x + w / 2
    bottom, top = y - h / 2, y + h / 2
    for i in range(4):
        px = left + i * w / 3
        py = bottom + i * h / 3
        draw_segment(page, (px, bottom), (px, top), dashed=False, arrow=False, weight_pt=0.50, color="RGB(148, 163, 184)")
        draw_segment(page, (left, py), (right, py), dashed=False, arrow=False, weight_pt=0.50, color="RGB(148, 163, 184)")
    draw_segment(page, (left + 0.06, bottom + 0.07), (right - 0.06, top - 0.06), dashed=False, arrow=True, weight_pt=0.9, color="RGB(14, 116, 144)")


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
            is_unsaved_generated = not full_name and doc.Pages.Item(1).Name == "Figure 4-1"
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
    page.Name = "Figure 4-1"
    cell(page.PageSheet, "PageWidth", f"{PAGE_W} in")
    cell(page.PageSheet, "PageHeight", f"{PAGE_H} in")
    cell(page.PageSheet, "DrawingScale", "1 in")
    cell(page.PageSheet, "PageScale", "1 in")

    bg = page.DrawRectangle(0, 0, PAGE_W, PAGE_H)
    cell(bg, "FillPattern", "1")
    cell(bg, "FillForegnd", COLORS["bg"])
    cell(bg, "LinePattern", "0")
    bg.SendToBack()

    draw_band(page, 0.35, 13.25, 8.65, 15.45, "外部输入与参数")
    draw_band(page, 0.35, 4.35, 8.65, 12.95, "实时 SAR 成像主流程")
    draw_band(page, 0.35, 0.55, 8.65, 4.05, "缓存、状态与后续接口")

    # Upper auxiliary modules.
    draw_module(
        page,
        0.75,
        14.00,
        4.20,
        14.92,
        "成像配置参数",
        "fc / fs / PRF / Rmax / 波束宽度 / 网格分辨率",
        fill=COLORS["input"],
        title_size=13.0,
        subtitle_size=10.3,
    )
    draw_module(
        page,
        4.78,
        14.00,
        8.25,
        14.92,
        "惯导 / IMU / 里程计",
        "速度 v(t) / 姿态 / 时间戳",
        fill=COLORS["motion"],
        title_size=13.0,
        subtitle_size=10.4,
    )

    # Vertical main chain.
    x1, x2 = 0.85, 4.45
    modules = {
        "radar": (11.67, 12.43),
        "parse": (10.68, 11.44),
        "window": (9.69, 10.45),
        "range": (8.70, 9.46),
        "azimuth": (7.71, 8.47),
        "remap": (6.57, 7.43),
        "sar": (5.51, 6.27),
        "candidate": (4.51, 5.27),
    }
    draw_module(page, x1, modules["radar"][0], x2, modules["radar"][1], "TI 毫米波雷达", "raw ADC 回波", fill=COLORS["input"], title_size=13.2)
    draw_module(page, x1, modules["parse"][0], x2, modules["parse"][1], "数据解析与通道选择", "帧 / Chirp / Rx-Tx 通道组织", fill=COLORS["pre"], title_size=13.0)
    draw_module(page, x1, modules["window"][0], x2, modules["window"][1], "滑窗分帧", "窗口长度 L，步长 S", fill=COLORS["pre"], title_size=13.2)
    draw_module(page, x1, modules["range"][0], x2, modules["range"][1], "距离向 FFT", "Range FFT，距离像生成", fill=COLORS["fft"], title_size=13.2)
    draw_module(page, x1, modules["azimuth"][0], x2, modules["azimuth"][1], "方位向 FFT", "Azimuth FFT，孔径积累", fill=COLORS["fft"], title_size=13.2)
    draw_module(page, x1, modules["remap"][0], x2, modules["remap"][1], "速度辅助距离-方位重映射", "v(t) / 姿态 / 时间戳参与几何映射", fill=COLORS["motion"], title_size=12.8)
    draw_module(page, x1, modules["sar"][0], x2, modules["sar"][1], "SAR 图像输出", "归一化 / 伪彩色 / 本地缓存", fill=COLORS["output"], title_size=13.2)
    draw_module(page, x1, modules["candidate"][0], x2, modules["candidate"][1], "候选区域生成", "目标响应区域 / 雷光融合接口", fill=COLORS["remote"], title_size=13.2)

    x_mid = (x1 + x2) / 2
    order = ["radar", "parse", "window", "range", "azimuth", "remap", "sar", "candidate"]
    for upper, lower in zip(order, order[1:]):
        draw_arrow(page, [(x_mid, modules[upper][0]), (x_mid, modules[lower][1])], weight_pt=1.5)

    draw_label(page, 3.75, 11.53, "ADC 数据流")
    draw_label(page, 3.75, 10.55, "数据块", w=0.70)
    draw_label(page, 3.75, 8.58, "频域结果", w=0.82)
    draw_label(page, 3.75, 5.38, "SAR 图像", w=0.78)

    draw_fft_icon(page, 4.10, 8.99)
    draw_fft_icon(page, 4.10, 8.00)
    draw_grid_icon(page, 4.08, 7.01)

    # Right-side organized auxiliary buses.
    param_bus_x = 5.10
    motion_bus_x = 7.72
    draw_segment(page, (param_bus_x, 13.76), (param_bus_x, 7.42), dashed=True, arrow=False, weight_pt=1.05, color=COLORS["muted"])
    draw_segment(page, (4.20, 14.46), (param_bus_x, 14.46), dashed=True, arrow=False, weight_pt=1.05, color=COLORS["muted"])
    draw_segment(page, (param_bus_x, 14.46), (param_bus_x, 13.76), dashed=True, arrow=False, weight_pt=1.05, color=COLORS["muted"])
    for y in (10.07, 9.08, 8.09, 7.23):
        draw_arrow(page, [(param_bus_x, y), (x2, y)], dashed=True, weight_pt=1.15)
    draw_label(page, 5.78, 13.18, "成像参数", w=0.82)

    draw_segment(page, (motion_bus_x, 14.00), (motion_bus_x, 7.30), dashed=True, arrow=False, weight_pt=1.05, color=COLORS["muted"])
    draw_arrow(page, [(motion_bus_x, 6.95), (x2, 6.95)], dashed=True, weight_pt=1.15)
    draw_label(page, 7.22, 7.51, "速度 / 姿态", w=0.88)

    # Compact side explanation blocks fill the portrait layout without adding clutter.
    draw_module(
        page,
        5.48,
        9.80,
        8.25,
        10.73,
        "滑窗处理",
        "连续帧进入局部孔径",
        fill=COLORS["pre"],
        title_size=12.6,
        subtitle_size=10.3,
    )
    draw_module(
        page,
        5.48,
        7.99,
        8.25,
        8.91,
        "频域成像",
        "距离像与方位积累",
        fill=COLORS["fft"],
        title_size=12.6,
        subtitle_size=10.3,
    )
    draw_module(
        page,
        5.48,
        6.03,
        8.25,
        6.99,
        "几何映射",
        "距离-方位域到 SAR 图像坐标",
        fill=COLORS["motion"],
        title_size=12.6,
        subtitle_size=10.1,
    )

    # Lower cache and state/interface modules.
    draw_module(
        page,
        0.75,
        2.72,
        4.18,
        3.56,
        "本地缓存队列",
        "原始回波 / 中间矩阵 / SAR 图像",
        fill=COLORS["output"],
        title_size=13.0,
        subtitle_size=10.4,
    )
    draw_module(
        page,
        4.72,
        2.72,
        8.25,
        3.56,
        "成像状态上报",
        "处理耗时 / 图像索引 / 成像状态",
        fill=COLORS["remote"],
        title_size=13.0,
        subtitle_size=10.4,
    )
    draw_module(
        page,
        4.72,
        1.22,
        8.25,
        2.08,
        "后续融合接口",
        "候选区域 / SAR 证据",
        fill=COLORS["remote"],
        title_size=13.0,
        subtitle_size=10.4,
    )

    draw_arrow(page, [(x1, 11.06), (0.58, 11.06), (0.58, 3.14), (0.75, 3.14)], dashed=True, weight_pt=1.10)
    draw_arrow(page, [(x2, 5.89), (4.52, 5.89), (4.52, 3.14), (4.18, 3.14)], dashed=True, weight_pt=1.10)
    draw_arrow(page, [(x2, 5.89), (4.56, 5.89), (4.56, 3.14), (4.72, 3.14)], dashed=True, weight_pt=1.10)
    draw_arrow(page, [(x2, 4.89), (4.56, 4.89), (4.56, 1.65), (4.72, 1.65)], dashed=True, weight_pt=1.10)

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
