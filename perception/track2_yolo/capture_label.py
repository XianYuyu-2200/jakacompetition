#!/usr/bin/env python3
"""腕部相机取图, 当场框出 20 类工件。

每次抓取前机械臂都会回到同一个关节角再识别。采集也要在那个位姿做,
不要换机位、不要手持乱拍。

按键:
  ] 或 Tab     下一类
  [            上一类
  Space        冻结当前帧
  鼠标拖拽     在冻结帧上画框。框记下的是松手那一刻选中的类
  Backspace    撤销最近一个框
  Enter        保存图片和 YOLO 标注, 回到实时画面
  Esc          丢掉这一帧
  q            退出

混合场景要在同一帧里切换类别再画下一个框。--id 把类别锁死, 用来单类连拍。
"""
from __future__ import annotations

import argparse
import json
import shutil
import subprocess
import time
from pathlib import Path

import cv2
import numpy as np

from boxes import normalize_box, yolo_line
from classes import by_id, load_catalog, package_dir
from vis import BIN_BGR, blit_labels, status_bar


WINDOW = "track2-label"
IMAGE_EXTS = {".jpg", ".jpeg", ".png", ".bmp"}


def list_cameras():
    tool = shutil.which("v4l2-ctl")
    if tool:
        subprocess.run([tool, "--list-devices"], check=False)
        return
    print("没有 v4l2-ctl。安装 v4l-utils 后能列出相机名字。")
    print("现在只看 /dev/video* , 不逐个打开(有的节点一读就卡住)。")
    for node in sorted(Path("/dev").glob("video*"), key=lambda p: p.name):
        print(node)


def open_camera(index, width, height):
    capture = cv2.VideoCapture(index, cv2.CAP_V4L2)
    if not capture.isOpened():
        capture.release()
        capture = cv2.VideoCapture(index)
    if not capture.isOpened():
        raise SystemExit(
            f"打不开相机 {index}。先跑: python capture_label.py --list-cameras"
        )
    capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    for _ in range(3):
        capture.read()
    return capture


def next_seq(session: Path) -> int:
    used = []
    for path in session.iterdir():
        if path.suffix.lower() in IMAGE_EXTS and path.stem.isdigit():
            used.append(int(path.stem))
    return (max(used) + 1) if used else 1


def save_pair(session: Path, seq: int, frame, boxes):
    image_path = session / f"{seq:04d}.jpg"
    label_path = session / f"{seq:04d}.txt"
    if not cv2.imwrite(str(image_path), frame, [int(cv2.IMWRITE_JPEG_QUALITY), 95]):
        raise SystemExit(f"写图片失败: {image_path}")
    height, width = frame.shape[:2]
    lines = [yolo_line(class_index, box, width, height) for class_index, box in boxes]
    label_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return image_path


def draw_scene(frame, boxes, drag, catalog, current):
    view = frame.copy()
    labels = []
    pending = list(boxes)
    if drag is not None:
        pending.append(drag)
    for class_index, box in pending:
        sku = catalog[class_index]
        color = BIN_BGR.get(sku.bin_id, (200, 200, 200))
        x1, y1, x2, y2 = box
        cv2.rectangle(view, (x1, y1), (x2, y2), color, 2)
        labels.append((f"{sku.id} {sku.name_zh}", x1, max(0, y1 - 26), color, 18, (0, 0, 0)))
    return blit_labels(view, labels), current


def handle_key(key, state, locked):
    """Mutate state. Return 'save', 'quit', or None."""
    if key in (-1, 255):
        return None
    low = key & 0xFF
    if low in (ord("q"), ord("Q")):
        return "quit"
    if key in (93, 9, 65289) or low == ord("]"):  # ] or Tab
        if locked is None:
            state["index"] = (state["index"] + 1) % state["n"]
        return None
    if key in (91,) or low == ord("["):
        if locked is None:
            state["index"] = (state["index"] - 1) % state["n"]
        return None
    if low == ord(" "):
        if state["frozen"] is None and state["live"] is not None:
            state["frozen"] = state["live"].copy()
            state["boxes"] = []
            state["drag"] = None
        return None
    if key in (8, 127, 65288):
        if state["boxes"]:
            state["boxes"].pop()
        return None
    if key in (13, 10):
        return "save"
    if key == 27:
        state["frozen"] = None
        state["boxes"] = []
        state["drag"] = None
        return None
    return None


def main():
    parser = argparse.ArgumentParser(description="采集并标注赛道二的 20 类工件")
    parser.add_argument("--camera", type=int, default=0)
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--id", default=None, help="锁死一个 sku, 例如 bottle")
    parser.add_argument("--session", default=None, help="批次名, 默认用时间戳")
    parser.add_argument(
        "--out",
        type=Path,
        default=package_dir() / "dataset" / "incoming",
    )
    parser.add_argument("--list-cameras", action="store_true")
    args = parser.parse_args()
    if args.list_cameras:
        list_cameras()
        return

    catalog = load_catalog()
    locked = None
    if args.id:
        found = by_id(catalog).get(args.id)
        if found is None:
            known = ", ".join(item.id for item in catalog)
            raise SystemExit(f"没有这个 id: {args.id}\n可选: {known}")
        locked = found

    session_name = args.session or time.strftime("%Y%m%d-%H%M%S")
    session = args.out / session_name
    session.mkdir(parents=True, exist_ok=True)
    (session / "classes.json").write_text(
        json.dumps(
            [
                {
                    "index": item.index,
                    "id": item.id,
                    "name_zh": item.name_zh,
                    "bin_id": item.bin_id,
                }
                for item in catalog
            ],
            ensure_ascii=False,
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )
    print(f"这一批写到 {session}")
    print("输入法切到英文, 否则 Tab / [ / ] 会被吃掉。")

    capture = open_camera(args.camera, args.width, args.height)
    state = {
        "index": locked.index if locked else 0,
        "n": len(catalog),
        "frozen": None,
        "live": None,
        "boxes": [],
        "drag": None,
        "anchor": None,
    }
    seq = next_seq(session)

    def on_mouse(event, x, y, _flags, _param):
        frame = state["frozen"]
        if frame is None:
            return
        height, width = frame.shape[:2]
        if y >= height:
            return
        x = max(0, min(width - 1, x))
        y = max(0, min(height - 1, y))
        if event == cv2.EVENT_LBUTTONDOWN:
            state["anchor"] = (x, y)
            state["drag"] = None
        elif event == cv2.EVENT_MOUSEMOVE and state["anchor"] is not None:
            box = normalize_box(*state["anchor"], x, y, width, height, min_px=1)
            if box is not None:
                state["drag"] = (state["index"], box)
        elif event == cv2.EVENT_LBUTTONUP and state["anchor"] is not None:
            box = normalize_box(*state["anchor"], x, y, width, height)
            state["anchor"] = None
            state["drag"] = None
            if box is not None:
                state["boxes"].append((state["index"], box))

    try:
        cv2.namedWindow(WINDOW, cv2.WINDOW_AUTOSIZE)
    except cv2.error as exc:
        raise SystemExit(f"打不开窗口。确认有显示器, 并且 OpenCV 带 highgui。{exc}") from exc
    cv2.setMouseCallback(WINDOW, on_mouse)

    try:
        while True:
            if state["frozen"] is None:
                ok, frame = capture.read()
                if not ok or frame is None:
                    print("读相机失败, 退出。")
                    break
                state["live"] = frame
                shown = frame
            else:
                shown = state["frozen"]
            view, _ = draw_scene(
                shown,
                state["boxes"],
                state["drag"] if state["frozen"] is not None else None,
                catalog,
                state["index"],
            )
            sku = catalog[state["index"]]
            mode = "冻结" if state["frozen"] is not None else "实时"
            lock = " 锁定" if locked else ""
            bar = status_bar(
                view.shape[1],
                [
                    f"{mode}{lock}  [{sku.index}] {sku.id} {sku.name_zh} -> {sku.bin_id} {sku.bin_name_zh}  已框 {len(state['boxes'])}",
                    "] / [ 换类   Space 冻结   拖拽画框   Backspace 撤销   Enter 保存   Esc 丢弃   q 退出",
                ],
            )
            canvas = np.vstack([view, bar])
            cv2.imshow(WINDOW, canvas)
            action = handle_key(cv2.waitKeyEx(15), state, locked)
            if action == "quit":
                break
            if action == "save":
                if state["frozen"] is None:
                    print("先按 Space 冻结, 再画框。")
                    continue
                if not state["boxes"]:
                    print("这一帧还没有框。")
                    continue
                path = save_pair(session, seq, state["frozen"], state["boxes"])
                print(f"saved {path.name}  boxes={len(state['boxes'])}")
                seq += 1
                state["frozen"] = None
                state["boxes"] = []
                state["drag"] = None
                state["anchor"] = None
    finally:
        capture.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
