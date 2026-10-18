#!/usr/bin/env python3
"""JAKA Mini 2 in MuJoCo, one slider per joint.

Angles are degrees. Limits match the Mini 2 / minicobo URDF.
The flange carries the competition WHEELTEC two-finger gripper.
Gripper slider: 0 is open, 35.9 is closed (left jaw negative, right positive).

This machine's Tk build cannot load CJK fonts, so Chinese is drawn
with Pillow (Noto Sans CJK SC) and shown as images.
"""
from __future__ import annotations

import multiprocessing as mp
import os
import shutil
import subprocess
import time
import tkinter as tk
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont, ImageTk

XML = Path(__file__).with_name("jaka_mini2.xml")
FONT_FILE = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"
FONT_BOLD = "/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc"
FONT_INDEX = 2  # Simplified Chinese in the Noto CJK collection

HOME_DEG = (20.0, -35.0, 70.0, 0.0, -45.0, 20.0)
DEMO = (
    HOME_DEG,
    (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    (55.0, -50.0, 95.0, 35.0, -60.0, 70.0),
    (-40.0, -25.0, 55.0, -80.0, -30.0, -50.0),
    (25.0, -60.0, 110.0, 15.0, -45.0, 30.0),
    HOME_DEG,
)
JOINTS = (
    ("J1", "基座旋转", -6.28, 6.28),
    ("J2", "肩关节", -2.09, 2.09),
    ("J3", "肘关节", -2.27, 2.27),
    ("J4", "腕旋转", -6.28, 6.28),
    ("J5", "腕俯仰", -2.09, 2.09),
    ("J6", "法兰旋转", -6.28, 6.28),
)
DEG = 3.141592653589793 / 180.0
BG = (243, 244, 246, 255)
BG_HEX = "#f3f4f6"


def _viewer(shared) -> None:
    import mujoco
    import mujoco.viewer

    model = mujoco.MjModel.from_xml_path(str(XML))
    data = mujoco.MjData(model)

    def _read_shared():
        with shared.get_lock():
            arm = [float(shared[i]) for i in range(6)]
            grip = float(shared[6])
        return arm, grip

    def _write_ctrl():
        arm, grip = _read_shared()
        data.ctrl[:6] = arm
        data.ctrl[6] = -grip
        data.ctrl[7] = grip

    arm, grip = _read_shared()
    data.qpos[:6] = arm
    data.qpos[6] = -grip
    data.qpos[7] = grip
    _write_ctrl()
    mujoco.mj_forward(model, data)

    with mujoco.viewer.launch_passive(
        model, data, show_left_ui=True, show_right_ui=True
    ) as viewer:
        def _frame_camera() -> None:
            with viewer.lock():
                viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
                viewer.cam.lookat[:] = (-0.02, 0.30, 0.0)
                viewer.cam.distance = 1.85
                viewer.cam.azimuth = -90
                viewer.cam.elevation = -56

        _frame_camera()
        frame_until = time.perf_counter() + 0.8
        last = time.perf_counter()
        while viewer.is_running():
            now = time.perf_counter()
            if now < frame_until:
                _frame_camera()
            steps = max(1, min(40, int((now - last) / model.opt.timestep)))
            _write_ctrl()
            for _ in range(steps):
                mujoco.mj_step(model, data)
            viewer.sync()
            last = now
            time.sleep(0.008)


def _place_windows() -> None:
    if not shutil.which("xdotool"):
        return
    env = os.environ.copy()
    deadline = time.time() + 8.0
    mujoco_id = tk_id = None
    while time.time() < deadline and (mujoco_id is None or tk_id is None):
        try:
            names = subprocess.check_output(
                ["xdotool", "search", "--name", "jaka"], env=env, text=True
            ).split()
        except subprocess.CalledProcessError:
            names = []
        for wid in names:
            try:
                title = subprocess.check_output(
                    ["xdotool", "getwindowname", wid], env=env, text=True
                ).strip()
            except subprocess.CalledProcessError:
                continue
            if title.startswith("MuJoCo"):
                mujoco_id = wid  # later matches are newer windows
            elif "关节" in title or title.startswith("JAKA Mini"):
                tk_id = wid
        time.sleep(0.2)
    if mujoco_id:
        subprocess.call(["xdotool", "windowsize", mujoco_id, "3400", "2100"], env=env)
        subprocess.call(["xdotool", "windowmove", mujoco_id, "1280", "120"], env=env)
        subprocess.call(["xdotool", "windowraise", mujoco_id], env=env)
    if tk_id:
        subprocess.call(["xdotool", "windowmove", tk_id, "40", "80"], env=env)
        subprocess.call(["xdotool", "windowraise", tk_id], env=env)


class TextImages:
    def __init__(self) -> None:
        self.keep: list[ImageTk.PhotoImage] = []
        self.regular = ImageFont.truetype(FONT_FILE, 26, index=FONT_INDEX)
        self.small = ImageFont.truetype(FONT_FILE, 20, index=FONT_INDEX)
        self.bold = ImageFont.truetype(FONT_BOLD, 36, index=FONT_INDEX)
        self.button_font = ImageFont.truetype(FONT_FILE, 24, index=FONT_INDEX)

    def _text(self, text: str, font: ImageFont.FreeTypeFont, fill: tuple[int, int, int]) -> ImageTk.PhotoImage:
        probe = ImageDraw.Draw(Image.new("RGB", (4, 4)))
        box = probe.textbbox((0, 0), text, font=font)
        w, h = box[2] - box[0], box[3] - box[1]
        img = Image.new("RGBA", (w + 4, h + 8), BG)
        draw = ImageDraw.Draw(img)
        draw.text((2 - box[0], 4 - box[1]), text, font=font, fill=fill + (255,))
        photo = ImageTk.PhotoImage(img)
        self.keep.append(photo)
        return photo

    def label(self, parent: tk.Misc, text: str, kind: str = "regular", **pack) -> tk.Label:
        font = {"regular": self.regular, "small": self.small, "bold": self.bold}[kind]
        fill = (26, 26, 26) if kind != "small" else (90, 90, 90)
        widget = tk.Label(parent, image=self._text(text, font, fill), bg=BG_HEX, borderwidth=0)
        widget.pack(**pack)
        return widget

    def button(self, parent: tk.Misc, text: str, command) -> tk.Button:
        font = self.button_font
        probe = ImageDraw.Draw(Image.new("RGB", (4, 4)))
        box = probe.textbbox((0, 0), text, font=font)
        tw, th = box[2] - box[0], box[3] - box[1]
        w, h = tw + 36, th + 18
        img = Image.new("RGBA", (w, h), BG)
        draw = ImageDraw.Draw(img)
        draw.rounded_rectangle((1, 1, w - 2, h - 2), radius=8, fill=(226, 228, 232, 255))
        draw.text(((w - tw) / 2 - box[0], (h - th) / 2 - box[1]), text, font=font, fill=(30, 30, 30, 255))
        photo = ImageTk.PhotoImage(img)
        self.keep.append(photo)
        widget = tk.Button(
            parent, image=photo, command=command, bg=BG_HEX,
            activebackground=BG_HEX, relief="flat", borderwidth=0,
            highlightthickness=0, cursor="hand2",
        )
        widget.pack(side="left", padx=(0, 10))
        return widget

    def set_label(self, widget: tk.Label, text: str, kind: str = "small") -> None:
        font = {"regular": self.regular, "small": self.small, "bold": self.bold}[kind]
        fill = (26, 26, 26) if kind != "small" else (80, 80, 80)
        widget.configure(image=self._text(text, font, fill))


def main() -> None:
    shared = mp.Array("d", [d * DEG for d in HOME_DEG] + [0.0])
    proc = mp.Process(target=_viewer, args=(shared,), daemon=True)
    proc.start()

    root = tk.Tk()
    root.title("JAKA Mini 2 关节")
    root.configure(bg=BG_HEX)
    images = TextImages()
    writing = {"on": False}
    anim = {"job": None, "on": False}

    images.label(root, "JAKA Mini 2", "bold", anchor="w", padx=18, pady=(16, 0))
    status = images.label(root, "拖动滑条控制对应关节，单位是度。", "small", anchor="w", padx=18, pady=(4, 12))

    vars_: list[tk.DoubleVar] = []
    readouts: list[tk.StringVar] = []

    grip_var = tk.DoubleVar(value=0.0)
    grip_read = tk.StringVar(value="0.0°")

    def push() -> None:
        with shared.get_lock():
            for i, var in enumerate(vars_):
                shared[i] = float(var.get()) * DEG
            shared[6] = float(grip_var.get()) * DEG

    def cancel_demo(_event=None) -> None:
        if not anim["on"]:
            return
        anim["on"] = False
        if anim["job"] is not None:
            root.after_cancel(anim["job"])
            anim["job"] = None
        images.set_label(status, "拖动滑条控制对应关节，单位是度。")

    def set_all(degs, from_user: bool = True) -> None:
        if from_user:
            cancel_demo()
        writing["on"] = True
        for var, deg in zip(vars_, degs):
            var.set(deg)
        for txt, deg in zip(readouts, degs):
            txt.set(f"{float(deg):.1f}°")
        writing["on"] = False
        push()

    def on_scale(_v=None) -> None:
        if writing["on"]:
            return
        cancel_demo()
        for var, txt in zip(vars_, readouts):
            txt.set(f"{float(var.get()):.1f}°")
        push()

    for i, (name, label, lo, hi) in enumerate(JOINTS):
        row = tk.Frame(root, bg=BG_HEX)
        row.pack(fill="x", padx=18, pady=3)
        images.label(row, f"{name}   {label}", "regular", side="left")
        var = tk.DoubleVar(value=HOME_DEG[i])
        vars_.append(var)
        scale = tk.Scale(
            row, variable=var, from_=lo / DEG, to=hi / DEG, orient="horizontal",
            resolution=0.1, showvalue=False, length=340, command=on_scale,
            bg=BG_HEX, highlightthickness=0, troughcolor="#d5d5db", sliderrelief="flat",
        )
        scale.pack(side="left", padx=8)
        scale.bind("<ButtonPress-1>", cancel_demo)
        txt = tk.StringVar(value=f"{HOME_DEG[i]:.1f}°")
        readouts.append(txt)
        tk.Label(row, textvariable=txt, width=7, anchor="e", bg=BG_HEX, fg="#222",
                 font=("DejaVu Sans", 12)).pack(side="left")

    grow = tk.Frame(root, bg=BG_HEX)
    grow.pack(fill="x", padx=18, pady=3)
    images.label(grow, "夹爪   开合", "regular", side="left")

    def on_grip(_v=None) -> None:
        if writing["on"]:
            return
        cancel_demo()
        grip_read.set(f"{float(grip_var.get()):.1f}°")
        push()

    gscale = tk.Scale(
        grow, variable=grip_var, from_=0.0, to=35.9, orient="horizontal",
        resolution=0.1, showvalue=False, length=340, command=on_grip,
        bg=BG_HEX, highlightthickness=0, troughcolor="#d5d5db", sliderrelief="flat",
    )
    gscale.pack(side="left", padx=8)
    gscale.bind("<ButtonPress-1>", cancel_demo)
    tk.Label(grow, textvariable=grip_read, width=7, anchor="e", bg=BG_HEX, fg="#222",
             font=("DejaVu Sans", 12)).pack(side="left")

    bar = tk.Frame(root, bg=BG_HEX)
    bar.pack(fill="x", padx=18, pady=(14, 6))

    def demo() -> None:
        cancel_demo()
        anim["on"] = True
        images.set_label(status, "正在演示各关节，拖动滑条可以打断。")
        state = {"seg": 0, "t0": time.perf_counter()}
        span = 1.7

        def tick() -> None:
            if not anim["on"]:
                return
            if state["seg"] >= len(DEMO) - 1:
                anim["on"] = False
                images.set_label(status, "拖动滑条控制对应关节，单位是度。")
                return
            u = (time.perf_counter() - state["t0"]) / span
            if u >= 1.0:
                state["seg"] += 1
                state["t0"] = time.perf_counter()
                if state["seg"] >= len(DEMO) - 1:
                    set_all(DEMO[-1], from_user=False)
                    anim["on"] = False
                    images.set_label(status, "拖动滑条控制对应关节，单位是度。")
                    return
                u = 0.0
            s = u * u * (3.0 - 2.0 * u)
            a, b = DEMO[state["seg"]], DEMO[state["seg"] + 1]
            set_all(tuple((1.0 - s) * x + s * y for x, y in zip(a, b)), from_user=False)
            anim["job"] = root.after(30, tick)

        tick()

    def set_grip(deg: float, from_user: bool = True) -> None:
        if from_user:
            cancel_demo()
        writing["on"] = True
        grip_var.set(deg)
        grip_read.set(f"{float(deg):.1f}°")
        writing["on"] = False
        push()

    images.button(bar, "回零", lambda: set_all((0, 0, 0, 0, 0, 0)))
    images.button(bar, "展示姿态", lambda: set_all(HOME_DEG))
    images.button(bar, "张开", lambda: set_grip(0.0))
    images.button(bar, "闭合", lambda: set_grip(35.9))
    images.button(bar, "演示动作", demo)
    images.label(
        root,
        "末端是比赛同款 WHEELTEC 二指爪。夹爪 0° 张开，35.9° 闭合。三维窗口：左键旋转，右键平移，滚轮缩放。",
        "small", anchor="w", padx=18, pady=(4, 16),
    )

    root.update_idletasks()
    root.geometry(f"{max(root.winfo_reqwidth(), 640)}x{root.winfo_reqheight()}+40+80")
    root.after(700, demo)

    import threading
    threading.Thread(target=_place_windows, daemon=True).start()

    def on_close() -> None:
        cancel_demo()
        proc.terminate()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_close)
    root.mainloop()
    proc.terminate()


if __name__ == "__main__":
    mp.set_start_method("spawn")
    main()
