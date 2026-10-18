#!/usr/bin/env python3
"""生成《JAKA Mini 2 机器人应用挑战赛·赛题细则》Word(.docx) 版。

内容取自 rulebook_content.py(与 PDF 版同一份内容源),版式仿照 PDF 版:
A4、页眉赛事全称、页脚页码、黑体标题 + 宋体正文、三线表格、内嵌配图。

用法:
    python3 design/build_competition_docx.py [输出路径]
"""
import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _docx_lib  # noqa: E402
import rulebook_content as rc  # noqa: E402

_docx_lib.ensure()
from docx import Document  # noqa: E402
from docx.enum.section import WD_SECTION  # noqa: E402
from docx.enum.table import WD_TABLE_ALIGNMENT  # noqa: E402
from docx.enum.text import WD_ALIGN_PARAGRAPH  # noqa: E402
from docx.oxml import OxmlElement  # noqa: E402
from docx.oxml.ns import qn  # noqa: E402
from docx.shared import Mm, Pt, RGBColor  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
FIG = lambda name: os.path.join(HERE, name)  # noqa: E731

HEAD_FONT = "黑体"
BODY_FONT = "宋体"

NAVY = RGBColor(0x1F, 0x3F, 0x66)
BLUE = RGBColor(0x2F, 0x5D, 0x8A)
GRAY = RGBColor(0x5A, 0x5A, 0x5A)
INK = RGBColor(0x1A, 0x1A, 0x1A)

USABLE_MM = 170.0
CENTER = WD_ALIGN_PARAGRAPH.CENTER
JUSTIFY = WD_ALIGN_PARAGRAPH.JUSTIFY
LEFT = WD_ALIGN_PARAGRAPH.LEFT


# ------------------------------------------------------------------ fonts ---
def set_run(run, font, size, bold=False, color=None, italic=False):
    run.font.name = font
    run.font.size = Pt(size)
    run.font.bold = bold
    run.font.italic = italic
    if color is not None:
        run.font.color.rgb = color
    rPr = run._element.get_or_add_rPr()
    rFonts = rPr.get_or_add_rFonts()
    for attr in ("w:ascii", "w:hAnsi", "w:eastAsia", "w:cs"):
        rFonts.set(qn(attr), font)


_TOK = re.compile(r'<b>|</b>|<font color="([^"]+)">|</font>')


def _tokens(seg):
    pos, bold, color = 0, False, None
    for m in _TOK.finditer(seg):
        if m.start() > pos:
            yield seg[pos:m.start()], bold, color
        tok = m.group(0)
        if tok == "<b>":
            bold = True
        elif tok == "</b>":
            bold = False
        elif tok.startswith("<font"):
            color = m.group(1)
        else:
            color = None
        pos = m.end()
    if pos < len(seg):
        yield seg[pos:], bold, color


def add_markup(par, text, font, size, bold=False, color=None):
    """把 <b>/<font>/<br/> 标记解析成 Word run。"""
    for i, seg in enumerate(re.split(r"<br\s*/?>", text)):
        if i:
            par.add_run().add_break()
        for chunk, b, c in _tokens(seg):
            if not chunk:
                continue
            run = par.add_run(chunk)
            hexcolor = c or None
            col = RGBColor.from_string(hexcolor.lstrip("#").upper()) if hexcolor else color
            set_run(run, font, size, bold=bold or b, color=col)


# ------------------------------------------------------------- paragraphs ---
def _set_ind_chars(par, chars, size_pt):
    pPr = par._p.get_or_add_pPr()
    ind = pPr.find(qn("w:ind"))
    if ind is None:
        ind = OxmlElement("w:ind")
        pPr.append(ind)
    ind.set(qn("w:firstLineChars"), str(int(chars * 100)))
    ind.set(qn("w:firstLine"), str(int(chars * size_pt * 20)))


def _set_line(par, multiple):
    spacing = OxmlElement("w:spacing")
    spacing.set(qn("w:line"), str(int(240 * multiple)))
    spacing.set(qn("w:lineRule"), "auto")
    par._p.get_or_add_pPr().append(spacing)


def _bottom_border(par, color="9DB3CC", size=6):
    pPr = par._p.get_or_add_pPr()
    borders = OxmlElement("w:pBdr")
    bottom = OxmlElement("w:bottom")
    bottom.set(qn("w:val"), "single")
    bottom.set(qn("w:sz"), str(size))
    bottom.set(qn("w:space"), "2")
    bottom.set(qn("w:color"), color)
    borders.append(bottom)
    pPr.append(borders)


def para(doc, text, font, size, bold=False, color=INK, align=JUSTIFY,
         indent_chars=0.0, before=0, after=3, line=1.4, left_mm=0.0,
         hanging_mm=0.0):
    p = doc.add_paragraph()
    p.alignment = align
    pf = p.paragraph_format
    pf.space_before = Pt(before)
    pf.space_after = Pt(after)
    _set_line(p, line)
    if indent_chars:
        _set_ind_chars(p, indent_chars, size)
    if left_mm:
        pf.left_indent = Mm(left_mm)
    if hanging_mm:
        pf.first_line_indent = Mm(-hanging_mm)
    add_markup(p, text, font, size, bold=bold, color=color)
    return p


# ----------------------------------------------------------------- tables ---
def _shade(cell, hexcolor):
    tcPr = cell._tc.get_or_add_tcPr()
    shd = OxmlElement("w:shd")
    shd.set(qn("w:val"), "clear")
    shd.set(qn("w:fill"), hexcolor)
    tcPr.append(shd)


def _repeat_header(row):
    trPr = row._tr.get_or_add_trPr()
    trPr.append(OxmlElement("w:tblHeader"))


def _cell_text(cell, text, size, bold=False, color=INK, align=LEFT):
    cell.text = ""
    p = cell.paragraphs[0]
    p.alignment = align
    pf = p.paragraph_format
    pf.space_before = Pt(1)
    pf.space_after = Pt(1)
    _set_line(p, 1.15)
    add_markup(p, text, BODY_FONT, size, bold=bold, color=color)


def add_table(doc, header, rows, widths, align_center=()):
    total = sum(widths)
    cols = [w / total * USABLE_MM for w in widths]
    t = doc.add_table(rows=1, cols=len(header))
    t.style = "Table Grid"
    t.alignment = WD_TABLE_ALIGNMENT.CENTER
    t.autofit = False
    for j, htxt in enumerate(header):
        _cell_text(t.rows[0].cells[j], htxt, 9, bold=True,
                   color=RGBColor(0xFF, 0xFF, 0xFF),
                   align=CENTER if j in align_center else LEFT)
        _shade(t.rows[0].cells[j], "2F5D8A")
    _repeat_header(t.rows[0])
    for i, row in enumerate(rows, start=1):
        cells = t.add_row().cells
        for j, txt in enumerate(row):
            _cell_text(cells[j], txt, 9,
                       align=CENTER if j in align_center else LEFT)
            if i % 2 == 1:
                _shade(cells[j], "EEF3FA")
    for row in t.rows:
        for j, c in enumerate(row.cells):
            c.width = Mm(cols[j])
    for j, c in enumerate(t.columns):
        c.width = Mm(cols[j])
    doc.add_paragraph().paragraph_format.space_after = Pt(2)
    return t


# ----------------------------------------------------------------- figures ---
def add_figure(doc, name, width_mm, caption):
    p = doc.add_paragraph()
    p.alignment = CENTER
    p.paragraph_format.space_before = Pt(4)
    p.paragraph_format.space_after = Pt(2)
    p.add_run().add_picture(FIG(name), width=Mm(width_mm))
    cap = doc.add_paragraph()
    cap.alignment = CENTER
    cap.paragraph_format.space_after = Pt(8)
    add_markup(cap, caption, BODY_FONT, 9, color=GRAY)


# --------------------------------------------------------- header/footer ---
def _field(par, instr):
    r = par.add_run()
    begin = OxmlElement("w:fldChar")
    begin.set(qn("w:fldCharType"), "begin")
    instrEl = OxmlElement("w:instrText")
    instrEl.set(qn("xml:space"), "preserve")
    instrEl.text = instr
    end = OxmlElement("w:fldChar")
    end.set(qn("w:fldCharType"), "end")
    r._r.append(begin)
    r._r.append(instrEl)
    r._r.append(end)
    return r


def setup_header_footer(section):
    header = section.header
    header.is_linked_to_previous = False
    hp = header.paragraphs[0]
    hp.alignment = CENTER
    hp.text = ""
    add_markup(hp, rc.RUNNING, BODY_FONT, 8, color=GRAY)
    _bottom_border(hp, "9DB3CC", 6)

    footer = section.footer
    footer.is_linked_to_previous = False
    fp = footer.paragraphs[0]
    fp.alignment = CENTER
    fp.text = ""
    r1 = fp.add_run("—　")
    set_run(r1, BODY_FONT, 9, color=GRAY)
    r2 = _field(fp, "PAGE")
    set_run(r2, BODY_FONT, 9, color=GRAY)
    r3 = fp.add_run("　—")
    set_run(r3, BODY_FONT, 9, color=GRAY)


# ------------------------------------------------------------- structure ---
def cover(doc):
    para(doc, rc.EVENT_NAME, BODY_FONT, 11, align=CENTER, color=GRAY,
         after=4, line=1.2)
    para(doc, rc.DOC_TITLE, HEAD_FONT, 22, bold=True, align=CENTER, color=NAVY,
         before=6, after=6, line=1.2)
    para(doc, rc.SUBTITLE, HEAD_FONT, 15, bold=True, align=CENTER, color=BLUE,
         after=8, line=1.2)
    rule = doc.add_paragraph()
    rule.paragraph_format.space_after = Pt(6)
    _bottom_border(rule, "1F3F66", 12)
    para(doc, rc.VERSION_LINE, BODY_FONT, 9, align=CENTER, color=GRAY,
         after=4, line=1.2)
    rule2 = doc.add_paragraph()
    rule2.paragraph_format.space_after = Pt(10)
    _bottom_border(rule2, "9DB3CC", 6)


def build_document():
    doc = Document()
    sec = doc.sections[0]
    sec.page_width = Mm(210)
    sec.page_height = Mm(297)
    sec.top_margin = Mm(20)
    sec.bottom_margin = Mm(18)
    sec.left_margin = Mm(20)
    sec.right_margin = Mm(20)
    sec.header_distance = Mm(10)
    sec.footer_distance = Mm(9)
    setup_header_footer(sec)

    for b in rc.build():
        kind = b[0]
        if kind == "cover":
            cover(doc)
        elif kind == "h1":
            title = (b[2] if not b[1] else b[1] + "　" + b[2])
            para(doc, title, HEAD_FONT, 14, bold=True, color=NAVY, align=LEFT,
                 before=12, after=6, line=1.25)
        elif kind == "h2":
            para(doc, b[1], HEAD_FONT, 12, bold=True, color=BLUE, align=LEFT,
                 before=9, after=4, line=1.25)
        elif kind == "h3":
            para(doc, b[1], HEAD_FONT, 10.5, bold=True, align=LEFT,
                 before=7, after=3, line=1.25)
        elif kind == "p":
            para(doc, b[1], BODY_FONT, 10.5, indent_chars=2.0, after=4, line=1.4)
        elif kind == "pn":
            para(doc, b[1], BODY_FONT, 10.5, after=4, line=1.4)
        elif kind == "note":
            para(doc, b[1], BODY_FONT, 9, color=GRAY, left_mm=8, after=5,
                 line=1.3)
        elif kind == "li":
            para(doc, "●　" + b[1], BODY_FONT, 10.5, align=LEFT, left_mm=9,
                 after=2, line=1.35)
        elif kind == "subli":
            para(doc, b[1], BODY_FONT, 10.5, align=LEFT, left_mm=9, after=2,
                 line=1.35)
        elif kind == "numi":
            para(doc, b[1], BODY_FONT, 10.5, align=LEFT, left_mm=9,
                 hanging_mm=9, after=2, line=1.4)
        elif kind == "space":
            sp = doc.add_paragraph()
            sp.paragraph_format.space_after = Pt(b[1])
        elif kind == "pagebreak":
            doc.add_page_break()
        elif kind == "hrule":
            rule = doc.add_paragraph()
            rule.paragraph_format.space_after = Pt(6)
            _bottom_border(rule, "9DB3CC", 6)
        elif kind == "table":
            add_table(doc, b[1], b[2], list(b[3]), b[4])
        elif kind == "figure":
            add_figure(doc, b[1], b[2], b[3])
        else:
            raise ValueError("unknown block: %r" % (kind,))
    return doc


def main():
    out = sys.argv[1] if len(sys.argv) > 1 else os.path.join(
        ROOT, "JAKA Mini 2 机器人应用挑战赛·赛题细则.docx")
    build_document().save(out)
    print("written:", out)


if __name__ == "__main__":
    main()
