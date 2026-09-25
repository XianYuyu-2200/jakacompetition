#!/usr/bin/env python3
"""把 xwd 抓屏转成 PNG(不依赖 ImageMagick, 用 PIL 手写解析)。

用法:
  xwd -root -display :0 -silent > /tmp/shot.xwd
  python3 xwd2png.py /tmp/shot.xwd /tmp/shot.png

装了 ImageMagick 的话 `import -window <wid> shot.png` 更省事, 本脚本留给
没装 ImageMagick 的机器。

历史坑(2026-09 修): 早期版本拿 ``bytes_per_line`` 去猜"每像素几字节"
(``bpl >= w*4`` 就当 4 字节), 而 xwd 写出来的这两个字段是可以不一致的 ——
本机抓 1000x845 的窗口时是 ``bits_per_pixel=24`` 但 ``bytes_per_line=4000``,
于是按 4 字节去读一份 3 字节的数据, 整张图被横向拉伸错位, 抓出来的画面带
周期约 3px 的彩色竖条纹。这个假象曾被误判成"软渲染的画面瑕疵", 其实同一份
.xwd 用 ImageMagick 解出来是干净的。现在改成按 ``bits_per_pixel`` 取宽度,
并且从文件尾倒推像素起点(跳过窗口名段与调色板段)。
"""
import struct
import sys

import numpy as np
from PIL import Image


def read_xwd(path):
    data = open(path, "rb").read()
    for endian in ("<", ">"):
        try:
            header = struct.unpack(endian + "25I", data[:100])
            (header_size, _ver, _fmt, depth, w, h, _xoff, _byte_order,
             _bitmap_unit, _bitmap_bit_order, _bitmap_pad, bpp, bpl,
             _vclass, _rmask, _gmask, _bmask, _bits_rgb, _cmap_entries,
             ncolors, *_rest) = header
            if not (100 <= w <= 20000 and 100 <= h <= 20000 and 0 < bpl <= w * 8):
                continue
            nbytes = bpp // 8
            if nbytes not in (3, 4) or bpl < w * nbytes:
                continue
            # 像素起点: 头部之后还可能跟着 XWD v7 的窗口名段和调色板段。
            # 它们的长度都写在头部里, 但从文件尾倒推 h*bpl 字节更省事。
            off = len(data) - h * bpl
            if off < header_size:
                off = header_size + ncolors * 12
            if off < 0 or off + h * bpl > len(data):
                continue
            arr = (np.frombuffer(data, dtype=np.uint8, count=h * bpl, offset=off)
                   .reshape(h, bpl)[:, :w * nbytes]
                   .reshape(h, w, nbytes)[:, :, :3])
            print(f"{'LSB' if endian == '<' else 'MSB'} depth={depth} "
                  f"{w}x{h} bpl={bpl} bpp={bpp}")
            # xwd 里通道是 BGR 序, PIL 要 RGB
            return Image.fromarray(arr[:, :, ::-1].copy())
        except Exception:  # noqa: BLE001
            continue
    raise SystemExit("xwd 解析失败")


if __name__ == "__main__":
    image = read_xwd(sys.argv[1])
    image.save(sys.argv[2])
    print("saved", sys.argv[2], image.size)
