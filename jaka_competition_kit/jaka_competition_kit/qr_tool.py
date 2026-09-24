"""二维码工具: 生成 / 解析 / 识别图片。

  ros2 run jaka_competition_kit qr_tool make            # 随机生成一张
  ros2 run jaka_competition_kit qr_tool make 246135     # 指定顺序
  ros2 run jaka_competition_kit qr_tool decode a.png    # 从图片识别
"""
from __future__ import annotations

import os
import sys

from .qr import make_order, parse, payload, decode_image, render_png


def main(args=None):
    argv = sys.argv[1:] if args is None else args
    cmd = argv[0] if argv else "make"
    out_dir = os.path.expanduser("~/.ros/jaka_competition")
    os.makedirs(out_dir, exist_ok=True)

    if cmd == "make":
        if len(argv) > 1:
            order = [int(c) for c in argv[1]]
        else:
            order = make_order(require_derangement=True)
        text = payload(order)
        path = os.path.join(out_dir, f"qr_{''.join(map(str, order))}.png")
        render_png(text, path)
        print(f"抓取顺序: {order}")
        print(f"二维码内容: {text}")
        print(f"图片: {path}")
    elif cmd == "decode":
        if len(argv) < 2:
            print("用法: qr_tool decode <图片路径>")
            sys.exit(2)
        text = decode_image(argv[1])
        print(f"原始内容: {text!r}")
        print(f"抓取顺序: {parse(text)}")
    else:
        print(__doc__)
        sys.exit(2)


if __name__ == "__main__":
    main()
