#!/usr/bin/env python3
"""把 perception/track1_sim/README.md 排成《基础组仿真教程》PDF。

版式与赛题细则同一套：A4、思源黑体、页眉页脚。

用法:
    python3 design/build_track1_tutorial_pdf.py [输出路径]
"""
import os
import re
import sys

from reportlab.lib import colors
from reportlab.lib.enums import TA_CENTER, TA_LEFT
from reportlab.lib.pagesizes import A4
from reportlab.lib.styles import ParagraphStyle
from reportlab.lib.units import mm
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.platypus import (BaseDocTemplate, Frame, KeepTogether,
                                PageTemplate, Paragraph, Spacer, Table, TableStyle)
from reportlab.platypus.flowables import HRFlowable

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _fonts  # noqa: E402

import reportlab.lib.textsplit as _textsplit
import reportlab.platypus.paragraph as _paragraph
# 全角逗号、分号、冒号不放到行首。reportlab 自带的禁则只有顿号和句号。
_extra = "，；：！？"
_textsplit.ALL_CANNOT_START += "".join(ch for ch in _extra if ch not in _textsplit.ALL_CANNOT_START)
_paragraph.ALL_CANNOT_START = _textsplit.ALL_CANNOT_START

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
SRC = os.path.join(ROOT, "perception", "track1_sim", "README.md")
TITLE = "基础组仿真教程"
RUNNING = "JAKA Mini 2 机器人应用挑战赛 · 基础组仿真教程"

NAVY = colors.HexColor("#1f3f66")
BLUE = colors.HexColor("#2f5d8a")
LIGHT = colors.HexColor("#eef3fa")
CODE_BG = colors.HexColor("#f4f7fb")
GRAY = colors.HexColor("#5a5a5a")
LINE = colors.HexColor("#9db3cc")
INK = colors.HexColor("#1a1a1a")

FONT, FONT_B = _fonts.ensure_fonts()
pdfmetrics.registerFont(TTFont("NSSC", FONT))
pdfmetrics.registerFont(TTFont("NSSC-B", FONT_B))
pdfmetrics.registerFontFamily("NSSC", normal="NSSC", bold="NSSC-B",
                              italic="NSSC", boldItalic="NSSC-B")

BODY = ParagraphStyle(
    "body", fontName="NSSC", fontSize=10.2, leading=16.2,
    alignment=TA_LEFT, firstLineIndent=0, spaceAfter=4,
    wordWrap="CJK", splitLongWords=1, textColor=INK)
H2 = ParagraphStyle(
    "h2", fontName="NSSC-B", fontSize=12.5, leading=18,
    textColor=NAVY, spaceBefore=11, spaceAfter=6, wordWrap="CJK")
BULLET = ParagraphStyle(
    "bullet", parent=BODY, leftIndent=16, bulletIndent=2, spaceAfter=2)
NUM = ParagraphStyle(
    "num", parent=BODY, leftIndent=18, firstLineIndent=-18, spaceAfter=2)
CELL = ParagraphStyle(
    "cell", fontName="NSSC", fontSize=8.6, leading=12.4, wordWrap="CJK",
    splitLongWords=1, textColor=INK)
CELL_H = ParagraphStyle(
    "cellh", parent=CELL, fontName="NSSC-B", textColor=colors.white)
CODE = ParagraphStyle(
    "code", fontName="NSSC", fontSize=8.3, leading=12.0,
    textColor=colors.HexColor("#1c2833"), wordWrap="CJK", splitLongWords=1)
CAP = ParagraphStyle(
    "cap", fontName="NSSC", fontSize=9, leading=13, alignment=TA_CENTER,
    textColor=GRAY, spaceBefore=2, spaceAfter=2)


def esc(text):
    return (text.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;"))


def breakable(text):
    return text


def inline(text):
    parts = re.split(r"(`[^`]+`)", text)
    out = []
    for part in parts:
        if len(part) >= 2 and part.startswith("`") and part.endswith("`"):
            code = breakable(esc(part[1:-1]))
            out.append('<font color="#1f4e79">%s</font>' % code)
        else:
            plain = esc(part)
            plain = re.sub(r"\*\*(.+?)\*\*", r"<b>\1</b>", plain)
            out.append(plain)
    return "".join(out)


def parse(md):
    lines = md.replace("\r\n", "\n").split("\n")
    i = 0
    blocks = []
    title = TITLE
    while i < len(lines):
        line = lines[i]
        if not line.strip():
            i += 1
            continue
        if line.startswith("# "):
            title = line[2:].strip()
            i += 1
            continue
        if line.startswith("## "):
            blocks.append(("h2", line[3:].strip()))
            i += 1
            continue
        if line.strip() == "---":
            blocks.append(("hr",))
            i += 1
            continue
        if line.startswith("```"):
            i += 1
            code = []
            while i < len(lines) and not lines[i].startswith("```"):
                code.append(lines[i])
                i += 1
            if i < len(lines):
                i += 1
            blocks.append(("code", code))
            continue
        if line.startswith("|"):
            rows = []
            while i < len(lines) and lines[i].startswith("|"):
                raw = lines[i].strip().strip("|")
                cells = [c.strip() for c in raw.split("|")]
                if not all(re.fullmatch(r":?-{3,}:?", c.replace(" ", "")) or c == "" for c in cells):
                    if not re.fullmatch(r"[\s|:-]+", lines[i].strip()):
                        rows.append(cells)
                i += 1
            if rows:
                blocks.append(("table", rows[0], rows[1:]))
            continue
        if re.match(r"^\d+\.\s", line):
            items = []
            while i < len(lines) and re.match(r"^\d+\.\s", lines[i]):
                num, rest = lines[i].split(".", 1)
                items.append((num.strip(), rest.strip()))
                i += 1
            blocks.append(("ol", items))
            continue
        if line.startswith("- "):
            items = []
            while i < len(lines) and lines[i].startswith("- "):
                items.append(lines[i][2:].strip())
                i += 1
            blocks.append(("ul", items))
            continue
        para = [line.strip()]
        i += 1
        while i < len(lines) and lines[i].strip() and not lines[i].startswith(("#", "|", "-", "```")) \
                and lines[i].strip() != "---" and not re.match(r"^\d+\.\s", lines[i]):
            para.append(lines[i].strip())
            i += 1
        blocks.append(("p", " ".join(para)))
    return title, blocks


def hrule():
    return HRFlowable(width="100%", thickness=0.8, color=LINE, spaceBefore=1, spaceAfter=6)


def code_block(lines):
    rendered = []
    for line in lines:
        if not line:
            rendered.append("&nbsp;")
        else:
            rendered.append(breakable(esc(line)))
    body = "<br/>".join(rendered)
    data = [[Paragraph(body, CODE)]]
    table = Table(data, colWidths=[170 * mm])
    table.setStyle(TableStyle([
        ("BACKGROUND", (0, 0), (-1, -1), CODE_BG),
        ("BOX", (0, 0), (-1, -1), 0.4, LINE),
        ("LEFTPADDING", (0, 0), (-1, -1), 8),
        ("RIGHTPADDING", (0, 0), (-1, -1), 6),
        ("TOPPADDING", (0, 0), (-1, -1), 5),
        ("BOTTOMPADDING", (0, 0), (-1, -1), 5),
        ("VALIGN", (0, 0), (-1, -1), "TOP"),
    ]))
    return KeepTogether([Spacer(1, 1), table, Spacer(1, 6)])


def data_table(header, rows):
    cols = max(len(header), max((len(r) for r in rows), default=0))
    header = header + [""] * (cols - len(header))
    rows = [r + [""] * (cols - len(r)) for r in rows]
    if cols >= 2 and not header[0].strip():
        header[0] = "项目"
    weights = []
    for c in range(cols):
        longest = max([len(header[c])] + [len(r[c]) for r in rows])
        weights.append(max(longest, 4))
    total = sum(weights) or 1
    # 短列不能被长句子挤没，否则「认不认形状」会折成三行。
    floor = 0.16 if cols <= 3 else 0.10
    weights = [max(w, floor * total) for w in weights]
    total = sum(weights)
    col_widths = [w / total * (170 * mm) for w in weights]
    data = [[Paragraph(inline(c) or "&nbsp;", CELL_H) for c in header]]
    for row in rows:
        data.append([Paragraph(inline(c) or "&nbsp;", CELL) for c in row])
    table = Table(data, colWidths=col_widths, repeatRows=1, hAlign="CENTER")
    style = [
        ("BACKGROUND", (0, 0), (-1, 0), BLUE),
        ("TEXTCOLOR", (0, 0), (-1, 0), colors.white),
        ("GRID", (0, 0), (-1, -1), 0.4, LINE),
        ("VALIGN", (0, 0), (-1, -1), "MIDDLE"),
        ("TOPPADDING", (0, 0), (-1, -1), 3.5),
        ("BOTTOMPADDING", (0, 0), (-1, -1), 3.5),
        ("LEFTPADDING", (0, 0), (-1, -1), 4),
        ("RIGHTPADDING", (0, 0), (-1, -1), 4),
    ]
    for r in range(1, len(data)):
        if r % 2 == 0:
            style.append(("BACKGROUND", (0, r), (-1, r), LIGHT))
    table.setStyle(TableStyle(style))
    return KeepTogether([Spacer(1, 2), table, Spacer(1, 7)])


def story_from(title, blocks):
    story = [
        Paragraph("JAKA Mini 2 机器人应用挑战赛", ParagraphStyle(
            "kicker", fontName="NSSC", fontSize=10, leading=14,
            alignment=TA_CENTER, textColor=GRAY)),
        Spacer(1, 3),
        Paragraph(TITLE, ParagraphStyle(
            "title", fontName="NSSC-B", fontSize=22, leading=30,
            alignment=TA_CENTER, textColor=NAVY)),
        Spacer(1, 2),
        Paragraph(title.replace("基础组：", ""), ParagraphStyle(
            "sub", fontName="NSSC", fontSize=12, leading=18,
            alignment=TA_CENTER, textColor=BLUE)),
        Spacer(1, 3),
        HRFlowable(width="100%", thickness=1.2, color=NAVY, spaceBefore=2, spaceAfter=8),
    ]
    for block in blocks:
        kind = block[0]
        if kind == "h2":
            story.append(Paragraph(inline(block[1]), H2))
        elif kind == "p":
            story.append(Paragraph(inline(block[1]), BODY))
        elif kind == "ul":
            for item in block[1]:
                story.append(Paragraph(inline(item), BULLET, bulletText="●"))
            story.append(Spacer(1, 2))
        elif kind == "ol":
            for num, item in block[1]:
                story.append(Paragraph("%s.　%s" % (num, inline(item)), NUM))
            story.append(Spacer(1, 2))
        elif kind == "code":
            story.append(code_block(block[1]))
        elif kind == "table":
            story.append(data_table(block[1], block[2]))
        elif kind == "hr":
            story.append(hrule())
        else:
            raise ValueError(kind)
    return story


def decorate(canvas, doc):
    canvas.saveState()
    w, h = A4
    canvas.setFont("NSSC", 8)
    canvas.setFillColor(GRAY)
    canvas.drawCentredString(w / 2, h - 12 * mm, RUNNING)
    canvas.setStrokeColor(LINE)
    canvas.setLineWidth(0.6)
    canvas.line(20 * mm, h - 14.2 * mm, w - 20 * mm, h - 14.2 * mm)
    canvas.line(20 * mm, 14 * mm, w - 20 * mm, 14 * mm)
    canvas.setFont("NSSC", 9)
    canvas.drawCentredString(w / 2, 8 * mm, "— %d —" % doc.page)
    canvas.restoreState()


def main():
    out = sys.argv[1] if len(sys.argv) > 1 else os.path.join(ROOT, "基础组仿真教程.pdf")
    with open(SRC, encoding="utf-8") as fh:
        title, blocks = parse(fh.read())
    frame = Frame(18 * mm, 18 * mm, A4[0] - 36 * mm, A4[1] - 18 * mm - 18 * mm,
                  id="body", showBoundary=0)
    doc = BaseDocTemplate(
        out, pagesize=A4, title=TITLE, author="JAKA Mini 2 机器人应用挑战赛",
        subject="基础组仿真教程：没有设备时怎么学习、采集、训练和推理",
        leftMargin=18 * mm, rightMargin=18 * mm, topMargin=18 * mm, bottomMargin=18 * mm)
    doc.addPageTemplates([PageTemplate(id="all", frames=[frame], onPage=decorate)])
    doc.build(story_from(title, blocks))
    print("written:", out)


if __name__ == "__main__":
    main()
