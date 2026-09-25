#!/usr/bin/env python3
"""把另一个 X display 上的窗口画面搬到当前桌面来看(实时, 只读不介入)。

为什么要它: 这台机器上 Gazebo 的 3D 视口在远程桌面 `:1001`(NoMachine 虚拟
X server)里**只画一帧就不重绘**, 而 `:0`(setup/xorg-gpu.sh 起的独显 Xorg)
里正常。偏偏 `:0` 是 NoMachine 看不到的。这个脚本在 `:1001` 上开一个窗口,
每秒几次去 `:0` 抓 Gazebo 的窗口贴进来, 于是就能在远程桌面里看 Gazebo 的
比赛过程了。

用法:
  # 1) 独显 Xorg 上跑 Gazebo 线(另一个终端)
  bash setup/xorg-gpu.sh start :0
  DISPLAY=:0 ros2 launch jaka_competition_kit round.launch.py world:=gazebo

  # 2) 在远程桌面里看(本脚本跑在 :1001, 去抓 :0)
  python3 setup/gazebo-viewer.py --from :0 --title Gazebo

`:0` 上没有窗口管理器, 所以脚本每次抓图前会先把目标窗口置顶(不然 RViz 会盖住
它, 抓到的是 RViz)。不想让它动你的窗口叠放顺序就加 `--no-raise`。

常用参数:
  --from    被抓的 display, 默认 :0
  --title   窗口标题匹配(子串), 默认 "Gazebo"
  --id      直接给窗口 id, 跳过搜索
  --list    只列出 :from 上能抓的窗口, 不显示
  --fps     刷新率上限, 默认 5(RViz 线可以调到 10)
  --scale   显示缩放, 默认 0.9
  --crop    抓完之后裁一刀, 格式 WxH+X+Y(把 Gazebo 右侧的属性面板裁掉,
            只留 3D 视口)。窗口尺寸变了要跟着调
  --no-raise  不把被抓窗口置顶(:0 上没有窗口管理器, 不置顶会被别的窗口盖住)
  --once    只抓一帧存成 PNG 后退出(不需要 GUI)
"""
import argparse
import os
import re
import struct
import subprocess
import sys
import time

import numpy as np
from PIL import Image


def _run(argv, display):
    env = dict(os.environ, DISPLAY=display)
    return subprocess.run(argv, capture_output=True, env=env)


def list_windows(display):
    """返回 [(wid, title, w, h)], 只列有标题的正常窗口。"""
    out = _run(["xwininfo", "-root", "-tree"], display).stdout.decode(
        "utf-8", "replace")
    wins = []
    for line in out.splitlines():
        m = re.search(r'(0x[0-9a-f]+)\s+"([^"]*)":.*?(\d+)x(\d+)\+', line)
        if not m:
            continue
        wid, title, w, h = m.group(1), m.group(2), int(m.group(3)), int(m.group(4))
        if w >= 200 and h >= 200:
            wins.append((wid, title, w, h))
    return wins


def find_window(display, title, want_id=None):
    if want_id:
        return want_id
    for wid, name, _w, _h in list_windows(display):
        if title.lower() in name.lower():
            return wid
    raise SystemExit(f"在 {display} 上找不到标题含 {title!r} 的窗口, "
                     f"用 --list 看看有哪些")


def raise_window(display, wid):
    """把窗口置顶。

    `:0` 上没有窗口管理器, 窗口叠放顺序没人管, RViz 之类后起来的窗口会盖住
    Gazebo。X 不会保存被遮挡的像素, xwd 抓到的是"屏幕上那块区域", 所以抓之前
    必须先把它抬上来(`import -window` 同理)。有 WM 的桌面(比如 `:1001`)不需要。
    """
    _run(["xdotool", "windowraise", wid], display)


def grab_xwd(display, wid):
    """抓一帧, 解析 xwd 成 PIL RGBA 图。

    不落盘、不走 ImageMagick: xwd 的原始数据直接进内存解析, 1000x845 的
    窗口一帧大约 3 MB, 管道够快。
    """
    raw = _run(["xwd", "-display", display, "-id", wid, "-silent"], display).stdout
    if len(raw) < 100:
        raise SystemExit("xwd 没有抓到内容, 窗口 id 可能已经失效")
    for endian in ("<", ">"):
        (header_size, _ver, _fmt, _depth, w, h, _x, _bo, _bu, _bb, _bp,
         bpp, bpl, *_rest) = struct.unpack(endian + "25I", raw[:100])
        if not (10 <= w <= 20000 and 10 <= h <= 20000):
            continue
        nbytes = bpp // 8
        if nbytes not in (3, 4) or bpl < w * nbytes or len(raw) < h * bpl:
            continue
        off = len(raw) - h * bpl
        if off < header_size:
            continue
        arr = (np.frombuffer(raw, dtype=np.uint8, count=h * bpl, offset=off)
               .reshape(h, bpl)[:, :w * nbytes]
               .reshape(h, w, nbytes)[:, :, :3])
        return Image.fromarray(arr[:, :, ::-1].copy())
    raise SystemExit("xwd 解析失败")


def parse_crop(text):
    """解析 WxH+X+Y, 返回 (w, h, x, y)。"""
    m = re.fullmatch(r"(\d+)x(\d+)\+(\d+)\+(\d+)", text.strip())
    if not m:
        raise SystemExit(f"--crop 格式应为 WxH+X+Y(如 578x797+0+48), 收到 {text!r}")
    w, h, x, y = (int(v) for v in m.groups())
    return w, h, x, y


def apply_crop(img, crop):
    if not crop:
        return img
    w, h, x, y = crop
    return img.crop((x, y, x + w, y + h))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--from", dest="src", default=":0")
    ap.add_argument("--title", default="Gazebo")
    ap.add_argument("--id", dest="wid", default=None)
    ap.add_argument("--fps", type=float, default=5.0)
    ap.add_argument("--scale", type=float, default=0.9)
    ap.add_argument("--list", action="store_true")
    ap.add_argument("--once", default=None)
    ap.add_argument("--no-raise", dest="raise_win", action="store_false")
    ap.add_argument("--crop", default=None)
    args = ap.parse_args()

    if args.list:
        for wid, name, w, h in list_windows(args.src):
            print(f"{wid}  {w}x{h}  {name}")
        return

    wid = find_window(args.src, args.title, args.wid)
    crop = parse_crop(args.crop) if args.crop else None
    print(f"抓 {args.src} 的窗口 {wid}, 显示在本机 DISPLAY="
          f"{os.environ.get('DISPLAY', '<unset>')}")

    if args.once:
        if args.raise_win:
            raise_window(args.src, wid)
            time.sleep(0.5)
        img = apply_crop(grab_xwd(args.src, wid), crop)
        img.save(args.once)
        print("已保存", args.once)
        return

    import tkinter as tk
    from PIL import ImageTk

    root = tk.Tk()
    root.title(f"{args.src} {args.title} (实时转发, 只读)")
    label = tk.Label(root, bg="black")
    label.pack(fill="both", expand=True)

    state = {"photo": None, "frames": 0, "t0": time.time(), "stop": False}
    period = 1.0 / max(args.fps, 0.2)

    def tick():
        if state["stop"]:
            return
        started = time.time()
        try:
            if args.raise_win:
                raise_window(args.src, wid)
            img = grab_xwd(args.src, wid)
        except SystemExit as exc:
            print(exc, file=sys.stderr)
            root.destroy()
            return
        img = apply_crop(img, crop)
        if args.scale != 1.0:
            img = img.resize((int(img.width * args.scale),
                              int(img.height * args.scale)))
        state["photo"] = ImageTk.PhotoImage(img)
        label.configure(image=state["photo"])
        state["frames"] += 1
        if state["frames"] % 20 == 0:
            dt = time.time() - state["t0"]
            print(f"{state['frames']} 帧 / {dt:.1f}s = "
                  f"{state['frames'] / dt:.1f} fps", flush=True)
        wait = max(int((period - (time.time() - started)) * 1000), 1)
        root.after(wait, tick)

    def on_close():
        state["stop"] = True
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_close)
    root.bind("<Escape>", lambda _e: on_close())
    root.after(0, tick)
    root.mainloop()


if __name__ == "__main__":
    main()
