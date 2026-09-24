#!/usr/bin/env python3
"""把 xwd 抓屏转成 PNG(本机没有 ImageMagick, 用 PIL 手写解析)。

用法:
  xwd -root -display :0 -silent > /tmp/shot.xwd
  python3 xwd2png.py /tmp/shot.xwd /tmp/shot.png
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
             _bitmap_unit, _bitmap_bit_order, _bitmap_pad, _bpp, bpl, *_rest) = header
            if not (100 <= w <= 20000 and 100 <= h <= 20000 and 0 < bpl <= w * 8):
                continue
            pixels = data[header_size:header_size + w * h * bpl]
            if bpl >= w * 4:
                arr = (np.frombuffer(pixels, dtype=np.uint8)[:h * bpl]
                       .reshape(h, bpl)[:, :w * 4].reshape(h, w, 4)[:, :, :3])
            elif bpl >= w * 3:
                arr = (np.frombuffer(pixels, dtype=np.uint8)[:h * bpl]
                       .reshape(h, bpl)[:, :w * 3].reshape(h, w, 3))
            else:
                continue
            print(f"{'LSB' if endian == '<' else 'MSB'} depth={depth} {w}x{h} bpl={bpl}")
            return Image.fromarray(arr[:, :, ::-1].copy())
        except Exception:  # noqa: BLE001
            continue
    raise SystemExit("xwd 解析失败")


if __name__ == "__main__":
    image = read_xwd(sys.argv[1])
    image.save(sys.argv[2])
    print("saved", sys.argv[2], image.size)
