"""把系统的 Noto Sans CJK(OTF/CFF 轮廓)转成 TrueType,
供 reportlab / matplotlib 嵌入。转换结果缓存在 design/.fontcache/。

系统里 reportlab 不支持 CFF 轮廓的 TTC,matplotlib 自带的 Droid Sans Fallback
又缺拉丁字形,所以这里统一做一次转换并复用。
"""
import os

_CACHE = os.path.join(os.path.dirname(os.path.abspath(__file__)), ".fontcache")
_NOTO = {
    "regular": ("/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc", 2),
    "bold": ("/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc", 2),
}


def _glyphs_to_quadratic(glyphs, max_err=1.0):
    from fontTools.pens.cu2quPen import Cu2QuPen
    from fontTools.pens.ttGlyphPen import TTGlyphPen
    quad = {}
    for gname in glyphs.keys():
        pen = TTGlyphPen(glyphs)
        glyphs[gname].draw(Cu2QuPen(pen, max_err, reverse_direction=True))
        quad[gname] = pen.glyph()
    return quad


def _convert(src, idx, dst):
    from fontTools.ttLib import TTCollection, newTable
    font = TTCollection(src).fonts[idx]
    glyph_order = font.getGlyphOrder()
    font["loca"] = newTable("loca")
    font["glyf"] = glyf = newTable("glyf")
    glyf.glyphOrder = glyph_order
    glyf.glyphs = _glyphs_to_quadratic(font.getGlyphSet())
    for tag in ("CFF ", "CFF2", "VORG"):
        if tag in font:
            del font[tag]
    glyf.compile(font)
    maxp = newTable("maxp")
    maxp.tableVersion = 0x00010000
    maxp.maxZones = 1
    maxp.maxTwilightPoints = 0
    maxp.maxStorage = 0
    maxp.maxFunctionDefs = 0
    maxp.maxInstructionDefs = 0
    maxp.maxStackElements = 0
    maxp.maxSizeOfInstructions = 0
    maxp.maxComponentElements = max(
        (len(g.components) if getattr(g, "components", None) else 0)
        for g in glyf.glyphs.values())
    font["maxp"] = maxp
    maxp.compile(font)
    post = font["post"]
    post.formatType = 3.0
    font.sfntVersion = "\000\001\000\000"
    font.save(dst)


def ensure_fonts(verbose=True):
    """返回 (regular_ttf, bold_ttf) 路径,必要时先转换。"""
    os.makedirs(_CACHE, exist_ok=True)
    paths = {}
    for style, (src, idx) in _NOTO.items():
        dst = os.path.join(_CACHE, f"NotoSansSC-{style}.ttf")
        if not os.path.exists(dst):
            if verbose:
                print(f"[fonts] converting {src} -> {dst} ...")
            _convert(src, idx, dst)
        paths[style] = dst
    return paths["regular"], paths["bold"]


if __name__ == "__main__":
    print(ensure_fonts())
