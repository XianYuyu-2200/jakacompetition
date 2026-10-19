#!/usr/bin/env python3
"""实时显示腕部相机，并用 best.pt 在画面上更新检测框。

跑在 YOLO 的 Python 3.12 虚拟环境里。画面来自 grasp.py 写的
/tmp/track1_live.bin，不依赖 rclpy。
"""
from __future__ import annotations

import argparse
import struct
import threading
import time
from pathlib import Path

import cv2
import numpy as np

from shapes import SHAPES

NAMES = {item.id: item.name for item in SHAPES}
FRAME_PATH = Path("/tmp/track1_live.bin")
STATUS_PATH = Path("/tmp/track1_live_status.txt")
FONT = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"
WINDOW = "基础组识别"
HEADER = struct.Struct("<QII")


def read_frame():
    try:
        blob = FRAME_PATH.read_bytes()
    except OSError:
        return None
    if len(blob) < HEADER.size:
        return None
    seq, width, height = HEADER.unpack_from(blob, 0)
    need = HEADER.size + width * height * 3
    if width < 16 or height < 16 or width > 2000 or height > 2000 or len(blob) < need:
        return None
    image = np.frombuffer(blob, dtype=np.uint8, offset=HEADER.size, count=width * height * 3)
    image = image.reshape(height, width, 3).copy()
    return int(seq), image


def read_status() -> str:
    try:
        return STATUS_PATH.read_text(encoding="utf-8").strip()
    except OSError:
        return ""


def detect(model, image, conf: float) -> list[dict]:
    result = model.predict(source=image, conf=conf, verbose=False)[0]
    names = result.names
    best: dict[str, dict] = {}
    boxes = result.boxes
    if boxes is None:
        return []
    for box in boxes:
        sid = str(names[int(box.cls)])
        if sid not in NAMES:
            continue
        score = float(box.conf)
        if sid in best and score <= best[sid]["conf"]:
            continue
        best[sid] = {"id": sid, "conf": score, "xyxy": [float(v) for v in box.xyxy[0].tolist()]}
    return list(best.values())


_FONTS = None


def _fonts():
    global _FONTS
    if _FONTS is None:
        from PIL import ImageFont
        _FONTS = (ImageFont.truetype(FONT, 18), ImageFont.truetype(FONT, 16))
    return _FONTS


def compose(image, dets, status: str, fps: float, infer_ms: float, ready: bool) -> np.ndarray:
    from PIL import Image, ImageDraw

    canvas = image.copy()
    for item in dets:
        x1, y1, x2, y2 = (int(v) for v in item["xyxy"])
        cv2.rectangle(canvas, (x1, y1), (x2, y2), (40, 170, 60), 2)
        label = f"{item['conf'] * 100:.0f}%"
        cv2.rectangle(canvas, (x1, max(0, y1 - 18)), (x1 + 54, y1), (40, 170, 60), -1)
        cv2.putText(canvas, label, (x1 + 4, max(14, y1 - 4)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
    rows = max(len(dets), 1)
    bar_h = 52 + 24 * rows
    board = np.full((canvas.shape[0] + bar_h, canvas.shape[1], 3), 248, np.uint8)
    board[:canvas.shape[0]] = canvas
    pil = Image.fromarray(cv2.cvtColor(board, cv2.COLOR_BGR2RGB))
    draw = ImageDraw.Draw(pil)
    font, small = _fonts()
    clock = time.strftime("%H:%M:%S")
    head = status or "实时识别"
    extra = f"    {clock}"
    extra += f"    {fps:.0f} fps" if ready else "    模型加载中"
    if ready:
        extra += f"    推理 {infer_ms:.0f} ms"
    draw.text((8, canvas.shape[0] + 4), head + extra, font=font, fill=(20, 20, 20))
    if not dets:
        hint = "画面里还没有检出目标" if ready else "先把相机画面放出来"
        draw.text((8, canvas.shape[0] + 30), hint, font=small, fill=(80, 80, 80))
    for index, item in enumerate(dets):
        y = canvas.shape[0] + 30 + 24 * index
        percent = item["conf"] * 100.0
        color = (20, 120, 40) if percent >= 70 else (160, 90, 10)
        draw.text((8, y), f"{index + 1}. {NAMES[item['id']]}", font=small, fill=(20, 20, 20))
        draw.text((150, y), f"{percent:.0f}%", font=small, fill=color)
    return cv2.cvtColor(np.asarray(pil), cv2.COLOR_RGB2BGR)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--weights", required=True)
    parser.add_argument("--conf", type=float, default=0.35)
    args = parser.parse_args()

    state = {
        "frame": None,
        "seq": -1,
        "dets": [],
        "infer_ms": 0.0,
        "ready": False,
    }
    lock = threading.Lock()
    stop = threading.Event()

    def infer_loop() -> None:
        from ultralytics import YOLO

        model = YOLO(args.weights)
        with lock:
            state["ready"] = True
        seen = -1
        while not stop.is_set():
            with lock:
                frame = None if state["frame"] is None else state["frame"].copy()
                seq = state["seq"]
            if frame is None or seq == seen:
                time.sleep(0.02)
                continue
            seen = seq
            started = time.perf_counter()
            try:
                dets = detect(model, frame, args.conf)
            except Exception:
                time.sleep(0.1)
                continue
            elapsed = (time.perf_counter() - started) * 1000.0
            with lock:
                state["dets"] = dets
                state["infer_ms"] = elapsed

    threading.Thread(target=infer_loop, daemon=True).start()

    cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
    cv2.moveWindow(WINDOW, 40, 40)
    shown = False
    frames = 0
    fps_t = time.perf_counter()
    fps = 0.0
    while True:
        got = read_frame()
        if got is not None:
            seq, image = got
            with lock:
                if seq != state["seq"]:
                    state["frame"] = image
                    state["seq"] = seq
                    frames += 1
        now = time.perf_counter()
        if now - fps_t >= 0.5:
            fps = frames / (now - fps_t)
            frames = 0
            fps_t = now
        with lock:
            frame = None if state["frame"] is None else state["frame"]
            dets = list(state["dets"])
            infer_ms = state["infer_ms"]
            ready = state["ready"]
        if frame is None:
            frame = np.full((400, 640, 3), 230, np.uint8)
            dets = []
        view = compose(frame, dets, read_status(), fps, infer_ms, ready)
        if not shown:
            cv2.resizeWindow(WINDOW, int(view.shape[1]), int(view.shape[0]))
        cv2.imshow(WINDOW, view)
        key = cv2.waitKey(15) & 0xFF
        visible = cv2.getWindowProperty(WINDOW, cv2.WND_PROP_VISIBLE)
        if visible >= 1:
            shown = True
        if key in (ord("q"), 27):
            break
        if shown and visible < 1:
            break
    stop.set()


if __name__ == "__main__":
    main()
