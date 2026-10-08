#!/usr/bin/env python3
"""Render FINAL_REPORT.md as a paginated Chinese PDF with a control comparison chart."""
from __future__ import annotations

import re
from html import escape
from pathlib import Path

from reportlab.lib import colors
from reportlab.lib.enums import TA_LEFT
from reportlab.lib.pagesizes import A4
from reportlab.lib.styles import ParagraphStyle
from reportlab.lib.units import mm
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.cidfonts import UnicodeCIDFont
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.platypus import (
    Flowable,
    KeepTogether,
    PageBreak,
    Paragraph,
    SimpleDocTemplate,
    Spacer,
    Table,
    TableStyle,
)
from svglib.svglib import svg2rlg


HERE = Path(__file__).resolve().parent
SOURCE = HERE / "FINAL_REPORT.md"
OUTPUT = HERE / "步兵C车Yaw轴系统辨识与控制优化测试总结.pdf"
FONT = "STSong-Light"
MATH_FONT = "DejaVuSans"
MATH_FONT_PATH = Path("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf")
TIME_CHART = HERE / "control_analysis_final_kp10" / "control_comparison.svg"
WIDE_TIME_CHART = HERE / "control_analysis_wide_latest" / "control_comparison_trial_3.svg"
RICH_MODEL_CHART = (HERE / "analysis_output" / "20261006_133045_810815"
                    / "filtered_velocity_arx2_one_step_validation.svg")
DELAYED_CHART = (HERE / "analysis_output" / "20261005_093410_444865_improved"
                 / "delayed_arx2_validation.svg")
CASCADE_CHART = (HERE / "analysis_output" / "20261007_cascade_fresh"
                 / "profile_0_validation.svg")


class EquationBlock(Flowable):
    """Center standalone equations in a compact vector panel."""

    def __init__(self, equations: list[str]):
        super().__init__()
        self.equations = equations
        self.width = 0.0
        self.height = 14 + 16 * len(equations)

    def wrap(self, avail_width: float, avail_height: float) -> tuple[float, float]:
        self.width = avail_width
        return self.width, self.height

    def draw(self) -> None:
        c = self.canv
        c.setFillColor(colors.HexColor("#f2f7fa"))
        c.roundRect(0, 0, self.width, self.height, 5, fill=1, stroke=0)
        c.setStrokeColor(colors.HexColor("#00798c"))
        c.setLineWidth(2.4)
        c.line(0, 5, 0, self.height - 5)
        c.setFillColor(colors.HexColor("#17365d"))
        for index, equation in enumerate(self.equations):
            size = 11.4
            while size > 8 and pdfmetrics.stringWidth(equation, MATH_FONT, size) > self.width - 28:
                size -= 0.25
            c.setFont(MATH_FONT, size)
            c.drawCentredString(self.width / 2, self.height - 17 - 16 * index, equation)


class ControlChart(Flowable):
    """Small vector chart comparing measured baseline and tuned RMSE values."""

    rows = [
        ("首轮 Kp=3.66", 0.7653, 0.8473),
        ("首轮 Kp=8", 0.7653, 0.6118),
        ("Kp=10 主测", 0.7262, 0.5310),
        ("Kp=10 复测", 0.7573, 0.5387),
        ("快速工况", 0.7646, 0.5477),
    ]

    def __init__(self, width: float = 0, height: float = 116 * mm):
        super().__init__()
        self.width = width
        self.height = height

    def wrap(self, avail_width: float, avail_height: float) -> tuple[float, float]:
        self.width = avail_width
        return self.width, self.height

    def draw(self) -> None:
        c = self.canv
        w, h = self.width, self.height
        label_w = 32 * mm
        value_w = 13 * mm
        right = w - value_w
        chart_left = label_w
        chart_right = right - value_w
        chart_w = chart_right - chart_left
        vmax = 0.95
        top = h - 12 * mm
        row_gap = 16 * mm
        bar_h = 4.2 * mm
        c.setFont(FONT, 9)
        c.setFillColor(colors.HexColor("#333333"))
        c.drawString(chart_left, h - 5 * mm, "角度 RMSE（°，越低越好）")
        c.setFont(FONT, 8)
        c.setFillColor(colors.HexColor("#666666"))
        for tick in (0, 0.25, 0.5, 0.75, vmax):
            x = chart_left + chart_w * tick / vmax
            c.setStrokeColor(colors.HexColor("#dddddd"))
            c.setLineWidth(0.45)
            c.line(x, 8 * mm, x, top + 1 * mm)
            c.setFillColor(colors.HexColor("#666666"))
            c.drawCentredString(x, 3 * mm, f"{tick:.2g}")
        for i, (label, base, tuned) in enumerate(self.rows):
            y = top - i * row_gap
            c.setFillColor(colors.HexColor("#333333"))
            c.setFont(FONT, 8.5)
            c.drawRightString(chart_left - 2 * mm, y - 1.2 * mm, label)
            for j, (value, color) in enumerate((
                (base, colors.HexColor("#d1495b")),
                (tuned, colors.HexColor("#00798c")),
            )):
                yy = y - j * 5 * mm
                c.setFillColor(color)
                c.rect(chart_left, yy - bar_h / 2, chart_w * value / vmax, bar_h, fill=1, stroke=0)
                c.setFillColor(colors.HexColor("#333333"))
                c.setFont("Helvetica", 7.5)
                c.drawString(chart_right + 1.5 * mm, yy - 1.2 * mm, f"{value:.3f}")
        legend_y = h - 5 * mm
        c.setFillColor(colors.HexColor("#d1495b"))
        c.rect(w - 37 * mm, legend_y - 1 * mm, 3 * mm, 3 * mm, fill=1, stroke=0)
        c.setFillColor(colors.HexColor("#333333"))
        c.setFont(FONT, 8)
        c.drawString(w - 33 * mm, legend_y, "基准")
        c.setFillColor(colors.HexColor("#00798c"))
        c.rect(w - 22 * mm, legend_y - 1 * mm, 3 * mm, 3 * mm, fill=1, stroke=0)
        c.setFillColor(colors.HexColor("#333333"))
        c.drawString(w - 18 * mm, legend_y, "优化")


def markdown_markup(value: str) -> str:
    # STSong-Light renders Chinese; the embedded TrueType font renders Greek math.
    # Use plain digits/operators where combining marks and subscripts vary by viewer.
    value = value.translate(str.maketrans({
        "̇": "_dot", "₀": "0", "₁": "1", "₂": "2", "₃": "3",
        "₄": "4", "₅": "5", "₆": "6", "₇": "7", "₈": "8", "₉": "9",
        "−": "-", "·": "*", "×": "*", "≈": "~",
    }))
    value = escape(value, quote=False)
    value = re.sub(r"==(.+?)==",
                   lambda match: '<onDraw name="yellow_marker" label="'
                   + escape(match.group(1), quote=True) + '"/>' + match.group(1), value)
    value = re.sub(r"\*\*(.+?)\*\*", r"<b>\1</b>", value)
    def code_span(match: re.Match[str]) -> str:
        content = match.group(1)
        font = FONT if any('\u3400' <= char <= '\u9fff' for char in content) else MATH_FONT
        return f'<font name="{font}">{content}</font>'
    value = re.sub(r"`(.+?)`", code_span, value)
    # Greek outside formula spans also needs the embedded font.
    value = re.sub(r"[\u0370-\u03ff]+",
                   lambda match: f'<font name="{MATH_FONT}">{match.group()}</font>', value)
    value = re.sub(
        r"https://github\.com/[A-Za-z0-9_./-]+",
        lambda match: f'<link href="{match.group()}" color="#006b9a">{match.group()}</link>',
        value,
    )
    return value


def paragraph(text: str, style: ParagraphStyle) -> Paragraph:
    return Paragraph(markdown_markup(text), style)


def parse_table(lines: list[str], styles: dict[str, ParagraphStyle], width: float) -> Table:
    parsed = []
    for row_index, line in enumerate(lines):
        cells = [part.strip() for part in line.strip().strip("|").split("|")]
        if cells and all(re.fullmatch(r":?-{3,}:?", x.replace(" ", "")) for x in cells):
            continue
        style = styles["table_header"] if row_index == 0 else styles["table"]
        parsed.append([Paragraph(markdown_markup(cell), style) for cell in cells])
    columns = max(map(len, parsed))
    for row in parsed:
        row.extend([Paragraph("", styles["table"])] * (columns - len(row)))
    widths = [width / columns] * columns
    table = Table(parsed, colWidths=widths, repeatRows=1, hAlign="LEFT", splitByRow=1)
    table.setStyle(TableStyle([
        ("BACKGROUND", (0, 0), (-1, 0), colors.HexColor("#17365d")),
        ("TEXTCOLOR", (0, 0), (-1, 0), colors.white),
        ("FONTNAME", (0, 0), (-1, -1), FONT),
        ("VALIGN", (0, 0), (-1, -1), "TOP"),
        ("GRID", (0, 0), (-1, -1), 0.35, colors.HexColor("#cbd5e1")),
        ("ROWBACKGROUNDS", (0, 1), (-1, -1), [colors.white, colors.HexColor("#f2f6fa")]),
        ("LEFTPADDING", (0, 0), (-1, -1), 5),
        ("RIGHTPADDING", (0, 0), (-1, -1), 5),
        ("TOPPADDING", (0, 0), (-1, -1), 4),
        ("BOTTOMPADDING", (0, 0), (-1, -1), 4),
    ]))
    return table


def parse_markdown(text: str, styles: dict[str, ParagraphStyle], width: float) -> list[Flowable]:
    lines = text.splitlines()
    story: list[Flowable] = []
    i = 0
    first_title = True
    while i < len(lines):
        line = lines[i].strip()
        if not line:
            i += 1
            continue
        if line == "[[PAGEBREAK]]":
            story.append(PageBreak())
            i += 1
            continue
        if line == "[[CONTROL_CHART]]":
            story.extend([ControlChart(), Spacer(1, 4 * mm)])
            i += 1
            continue
        if line in ("[[TIME_CHART]]", "[[WIDE_TIME_CHART]]", "[[RICH_MODEL_CHART]]",
                    "[[DELAYED_CHART]]", "[[CASCADE_CHART]]"):
            chart_path = {"[[TIME_CHART]]": TIME_CHART,
                          "[[WIDE_TIME_CHART]]": WIDE_TIME_CHART,
                          "[[RICH_MODEL_CHART]]": RICH_MODEL_CHART,
                          "[[DELAYED_CHART]]": DELAYED_CHART,
                          "[[CASCADE_CHART]]": CASCADE_CHART}[line]
            drawing = svg2rlg(str(chart_path))
            if drawing is None:
                raise RuntimeError(f"Could not load time-series chart: {chart_path}")
            chart_width = width * .85 if line == "[[CASCADE_CHART]]" else width
            scale = chart_width / drawing.width
            drawing.width *= scale
            drawing.height *= scale
            drawing.scale(scale, scale)
            if line == "[[CASCADE_CHART]]":
                drawing.hAlign = "CENTER"
            story.extend([drawing, Spacer(1, 4 * mm)])
            i += 1
            continue
        if line.startswith("|"):
            block = []
            while i < len(lines) and lines[i].strip().startswith("|"):
                block.append(lines[i].strip())
                i += 1
            story.extend([KeepTogether([parse_table(block, styles, width)]), Spacer(1, 4 * mm)])
            continue
        if line.startswith("# "):
            story.append(paragraph(line[2:], styles["title"] if first_title else styles["h1"]))
            first_title = False
            story.append(Spacer(1, 4 * mm))
        elif line.startswith("## "):
            story.extend([Spacer(1, 2 * mm), paragraph(line[3:], styles["h1"])])
        elif line.startswith("### "):
            story.extend([Spacer(1, 1.5 * mm), paragraph(line[4:], styles["h2"])])
        elif line.startswith("- "):
            story.append(Paragraph(markdown_markup(line[2:]), styles["body"], bulletText="•"))
        elif re.match(r"^\d+\. ", line):
            story.append(paragraph(line, styles["body"]))
        elif re.fullmatch(r"`[^`]+`", line):
            formulae = []
            while i < len(lines) and re.fullmatch(r"`[^`]+`", lines[i].strip()):
                formulae.append(lines[i].strip()[1:-1])
                i += 1
            story.extend([EquationBlock(formulae), Spacer(1, 2.1 * mm)])
            continue
        else:
            story.append(paragraph(line, styles["lead"] if line.endswith("：") else styles["body"]))
        i += 1
    return story


def yellow_marker(canvas, kind, label) -> None:
    """Draw a flattened vector marker stroke behind a short inline phrase."""
    info = canvas._curr_tx_info
    size = info["xs"].style.fontSize
    x, y = info["cur_x"], info["cur_y"]
    width = pdfmetrics.stringWidth(label, FONT, size)
    canvas.saveState()
    canvas.setFillColor(colors.HexColor("#ffeb67"))
    path = canvas.beginPath()
    path.moveTo(x - 1.5, y - 1.0)
    path.lineTo(x + width * .33, y - 1.5)
    path.lineTo(x + width * .72, y - .8)
    path.lineTo(x + width + 1, y - 1.3)
    path.lineTo(x + width + 2.5, y + size * .82)
    path.lineTo(x + width * .65, y + size * .88)
    path.lineTo(x + width * .25, y + size * .79)
    path.lineTo(x, y + size * .86)
    path.close()
    canvas.drawPath(path, fill=1, stroke=0)
    canvas.restoreState()


def decorate(canvas, doc) -> None:
    canvas.setNamedCB("yellow_marker", yellow_marker)
    canvas.saveState()
    page_w, page_h = A4
    if doc.page > 1:
        canvas.setStrokeColor(colors.HexColor("#d6e0ec"))
        canvas.setLineWidth(0.6)
        canvas.line(17 * mm, page_h - 13 * mm, page_w - 17 * mm, page_h - 13 * mm)
        canvas.setFont(FONT, 8)
        canvas.setFillColor(colors.HexColor("#53657d"))
        canvas.drawString(17 * mm, page_h - 10 * mm, "算法大作业")
    canvas.setStrokeColor(colors.HexColor("#d6e0ec"))
    canvas.line(17 * mm, 12 * mm, page_w - 17 * mm, 12 * mm)
    canvas.setFont(FONT, 8)
    canvas.setFillColor(colors.HexColor("#53657d"))
    canvas.drawString(17 * mm, 7 * mm, "算法大作业｜测试数据与报告")
    canvas.drawRightString(page_w - 17 * mm, 7 * mm, str(doc.page))
    canvas.restoreState()


def main() -> None:
    pdfmetrics.registerFont(UnicodeCIDFont(FONT))
    pdfmetrics.registerFont(TTFont(MATH_FONT, str(MATH_FONT_PATH)))
    page_w, _ = A4
    left = right = 17 * mm
    top, bottom = 20 * mm, 17 * mm
    width = page_w - left - right
    styles = {
        "title": ParagraphStyle("TitleCN", fontName=FONT, fontSize=23, leading=31,
                                textColor=colors.HexColor("#17365d"), alignment=TA_LEFT,
                                spaceAfter=4 * mm, wordWrap="CJK"),
        "h1": ParagraphStyle("H1CN", fontName=FONT, fontSize=15, leading=21,
                             textColor=colors.HexColor("#17365d"), spaceBefore=3 * mm,
                             spaceAfter=2 * mm, keepWithNext=True, wordWrap="CJK"),
        "h2": ParagraphStyle("H2CN", fontName=FONT, fontSize=11.5, leading=16,
                             textColor=colors.HexColor("#00798c"), spaceBefore=2 * mm,
                             spaceAfter=1 * mm, keepWithNext=True, wordWrap="CJK"),
        "body": ParagraphStyle("BodyCN", fontName=FONT, fontSize=9.3, leading=14.2,
                                textColor=colors.HexColor("#202733"), alignment=TA_LEFT,
                                spaceAfter=2.1 * mm, wordWrap="CJK", allowWidows=0, allowOrphans=0),
        "table": ParagraphStyle("TableCN", fontName=FONT, fontSize=7.7, leading=10.5,
                                 textColor=colors.HexColor("#202733"), wordWrap="CJK"),
        "table_header": ParagraphStyle("TableHeaderCN", fontName=FONT, fontSize=7.7,
                                        leading=10.5, textColor=colors.white, wordWrap="CJK"),
    }
    styles["lead"] = ParagraphStyle("LeadCN", parent=styles["body"], keepWithNext=True)
    doc = SimpleDocTemplate(str(OUTPUT), pagesize=A4, rightMargin=right, leftMargin=left,
                            topMargin=top, bottomMargin=bottom,
                            title="算法大作业",
                            author="RMCS 控制作业测试记录")
    story = parse_markdown(SOURCE.read_text(encoding="utf-8"), styles, width)
    doc.build(story, onFirstPage=decorate, onLaterPages=decorate)
    print(f"Created {OUTPUT} ({OUTPUT.stat().st_size} bytes)")


if __name__ == "__main__":
    main()
