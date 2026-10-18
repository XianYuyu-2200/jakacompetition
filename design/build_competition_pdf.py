#!/usr/bin/env python3
"""生成《JAKA Mini 2 机器人应用挑战赛·赛题细则》PDF。

内容取自 competition.md(定稿版)、jaka_competition_kit/config 与 design/ 实测报告,
版式仿照《大模型任务挑战赛》赛题细则。

用法:
    python3 design/build_competition_pdf.py [输出路径]
"""
import os
import sys

from reportlab.lib import colors
from reportlab.lib.enums import TA_CENTER, TA_JUSTIFY, TA_LEFT
from reportlab.lib.pagesizes import A4
from reportlab.lib.styles import ParagraphStyle
from reportlab.lib.units import mm
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.platypus import (BaseDocTemplate, Frame, Image, KeepTogether,
                                NextPageTemplate, PageBreak, PageTemplate,
                                Paragraph, Spacer, Table, TableStyle)
from reportlab.platypus.flowables import HRFlowable
from reportlab.platypus.paragraph import Paragraph as P

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _fonts  # noqa: E402
import rulebook_content as rc  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
FIG = lambda name: os.path.join(HERE, name)  # noqa: E731

DOC_TITLE = rc.DOC_TITLE
RUNNING = rc.RUNNING

NAVY = colors.HexColor("#1f3f66")
BLUE = colors.HexColor("#2f5d8a")
LIGHT = colors.HexColor("#eef3fa")
GREEN = colors.HexColor("#cfe8d6")
GRAY = colors.HexColor("#5a5a5a")
LINE = colors.HexColor("#9db3cc")

FONT, FONT_B = _fonts.ensure_fonts()
pdfmetrics.registerFont(TTFont("NSSC", FONT))
pdfmetrics.registerFont(TTFont("NSSC-B", FONT_B))
pdfmetrics.registerFontFamily("NSSC", normal="NSSC", bold="NSSC-B",
                              italic="NSSC", boldItalic="NSSC-B")

# ---------------------------------------------------------------- styles ----
BODY = ParagraphStyle("body", fontName="NSSC", fontSize=10.2, leading=16.5,
                      alignment=TA_JUSTIFY, firstLineIndent=20.4, spaceAfter=3,
                      wordWrap="CJK", textColor=colors.HexColor("#1a1a1a"))
BODY_NI = ParagraphStyle("body_ni", parent=BODY, firstLineIndent=0)
H1 = ParagraphStyle("h1", fontName="NSSC-B", fontSize=14, leading=20,
                    textColor=NAVY, spaceBefore=12, spaceAfter=7, wordWrap="CJK")
H2 = ParagraphStyle("h2", fontName="NSSC-B", fontSize=11.6, leading=17,
                    textColor=BLUE, spaceBefore=8, spaceAfter=5)
H3 = ParagraphStyle("h3", fontName="NSSC-B", fontSize=10.4, leading=16,
                    textColor=colors.HexColor("#2b2b2b"), spaceBefore=6,
                    spaceAfter=3)
BULLET = ParagraphStyle("bullet", parent=BODY_NI, leftIndent=14, bulletIndent=2,
                        spaceAfter=2)
CELL = ParagraphStyle("cell", fontName="NSSC", fontSize=8.8, leading=12.6,
                      wordWrap="CJK")
CELL_B = ParagraphStyle("cellb", parent=CELL, fontName="NSSC-B")
CELL_H = ParagraphStyle("cellh", parent=CELL, fontName="NSSC-B",
                        textColor=colors.white)
NOTE = ParagraphStyle("note", parent=BODY_NI, fontSize=9.3, leading=14,
                      textColor=GRAY, leftIndent=8, spaceBefore=2, spaceAfter=4)
CAP = ParagraphStyle("cap", fontName="NSSC", fontSize=9, leading=13,
                     alignment=TA_CENTER, textColor=GRAY, spaceBefore=3,
                     spaceAfter=8)

CODE = lambda s: '<font color="#1f4e79">%s</font>' % s  # noqa: E731


def h1(t, num=None):
    return Paragraph((f"{num}　{t}" if num else t), H1)


def h2(t):
    return Paragraph(t, H2)


def h3(t):
    return Paragraph(t, H3)


def p(t):
    return Paragraph(t, BODY)


def pn(t):
    return Paragraph(t, BODY_NI)


def note(t):
    return Paragraph(t, NOTE)


def li(t):
    return Paragraph(t, BULLET, bulletText="●")


def numi(t):
    return Paragraph(t, ParagraphStyle("numi", parent=BODY_NI, leftIndent=16,
                                       firstLineIndent=-16, spaceAfter=2))


def cell(t, header=False, bold=False):
    if header:
        return Paragraph(t, CELL_H)
    return Paragraph(t, CELL_B if bold else CELL)


def table(header, rows, widths, align_center=()):
    data = [[cell(c, header=True) for c in header]]
    for row in rows:
        data.append([cell(c) for c in row])
    total = sum(widths)
    col = [w / total * (170 * mm) for w in widths]
    t = Table(data, colWidths=col, repeatRows=1, hAlign="CENTER")
    style = [
        ("BACKGROUND", (0, 0), (-1, 0), BLUE),
        ("TEXTCOLOR", (0, 0), (-1, 0), colors.white),
        ("GRID", (0, 0), (-1, -1), 0.5, LINE),
        ("VALIGN", (0, 0), (-1, -1), "MIDDLE"),
        ("TOPPADDING", (0, 0), (-1, -1), 3.5),
        ("BOTTOMPADDING", (0, 0), (-1, -1), 3.5),
        ("LEFTPADDING", (0, 0), (-1, -1), 5),
        ("RIGHTPADDING", (0, 0), (-1, -1), 5),
    ]
    for r in range(1, len(data)):
        if r % 2 == 0:
            style.append(("BACKGROUND", (0, r), (-1, r), LIGHT))
    for c in align_center:
        style.append(("ALIGN", (c, 0), (c, -1), "CENTER"))
    t.setStyle(TableStyle(style))
    return t


def figure(name, width_mm, caption):
    img = Image(FIG(name))
    iw, ih = img.imageWidth, img.imageHeight
    w = width_mm * mm
    img.drawWidth = w
    img.drawHeight = w * ih / iw
    img.hAlign = "CENTER"
    return KeepTogether([Spacer(1, 3), img, Paragraph(caption, CAP)])


def hrule():
    return HRFlowable(width="100%", thickness=0.8, color=LINE,
                      spaceBefore=2, spaceAfter=6)


# ---------------------------------------------------------------- content ---


SUBLI = ParagraphStyle("subli", parent=BODY_NI, leftIndent=14, spaceAfter=2)


def subli(t):
    return Paragraph(t, SUBLI)


# ------------------------------------------------------------- render blocks
def cover_flowables():
    return [
        Spacer(1, 6),
        Paragraph(rc.EVENT_NAME, ParagraphStyle(
            "ev", fontName="NSSC", fontSize=11, leading=15, alignment=TA_CENTER,
            textColor=GRAY)),
        Spacer(1, 4),
        Paragraph(DOC_TITLE, ParagraphStyle(
            "t", fontName="NSSC-B", fontSize=22, leading=30, alignment=TA_CENTER,
            textColor=NAVY)),
        Spacer(1, 4),
        Paragraph(rc.SUBTITLE, ParagraphStyle(
            "st", fontName="NSSC-B", fontSize=15, leading=22, alignment=TA_CENTER,
            textColor=BLUE)),
        Spacer(1, 6),
        HRFlowable(width="100%", thickness=1.2, color=NAVY, spaceAfter=6),
        Paragraph(rc.VERSION_LINE, ParagraphStyle(
            "ver", fontName="NSSC", fontSize=9, leading=13, alignment=TA_CENTER,
            textColor=GRAY)),
        HRFlowable(width="100%", thickness=0.6, color=LINE, spaceAfter=8),
    ]


def render_blocks():
    story = []
    for b in rc.build():
        kind = b[0]
        if kind == "cover":
            story += cover_flowables()
        elif kind == "h1":
            story.append(h1(b[2], b[1]))
        elif kind == "h2":
            story.append(h2(b[1]))
        elif kind == "h3":
            story.append(h3(b[1]))
        elif kind == "p":
            story.append(p(b[1]))
        elif kind == "pn":
            story.append(pn(b[1]))
        elif kind == "note":
            story.append(note(b[1]))
        elif kind == "li":
            story.append(li(b[1]))
        elif kind == "subli":
            story.append(subli(b[1]))
        elif kind == "numi":
            story.append(numi(b[1]))
        elif kind == "space":
            story.append(Spacer(1, b[1]))
        elif kind == "pagebreak":
            story.append(PageBreak())
        elif kind == "hrule":
            story.append(hrule())
        elif kind == "table":
            story.append(table(b[1], b[2], list(b[3]), b[4]))
        elif kind == "figure":
            story.append(figure(b[1], b[2], b[3]))
        else:
            raise ValueError("unknown block: %r" % (kind,))
    return story


# ---------------------------------------------------------------- render ----
def decorate(canvas, doc):
    canvas.saveState()
    w, h = A4
    canvas.setFont("NSSC", 7.8)
    canvas.setFillColor(GRAY)
    canvas.drawCentredString(w / 2, h - 12 * mm, RUNNING)
    canvas.setStrokeColor(LINE)
    canvas.setLineWidth(0.6)
    canvas.line(20 * mm, h - 14.2 * mm, w - 20 * mm, h - 14.2 * mm)
    canvas.line(20 * mm, 15 * mm, w - 20 * mm, 15 * mm)
    canvas.setFont("NSSC", 9)
    canvas.drawCentredString(w / 2, 10.5 * mm, "— %d —" % doc.page)
    canvas.restoreState()


def main():
    out = sys.argv[1] if len(sys.argv) > 1 else os.path.join(
        ROOT, "JAKA Mini 2 机器人应用挑战赛·赛题细则.pdf")
    frame = Frame(20 * mm, 17 * mm, A4[0] - 40 * mm, A4[1] - 17 * mm - 17 * mm,
                  id="body", showBoundary=0)
    doc = BaseDocTemplate(out, pagesize=A4,
                          title=DOC_TITLE + "·赛题细则",
                          author="全球校园人工智能算法精英大赛·算法应用赛",
                          subject="赛题细则", leftMargin=20 * mm,
                          rightMargin=20 * mm, topMargin=17 * mm,
                          bottomMargin=17 * mm)
    doc.addPageTemplates([PageTemplate(id="all", frames=[frame], onPage=decorate)])
    doc.build(render_blocks())
    print("written:", out)


if __name__ == "__main__":
    main()
