"""基础组练习的一轮出题。

6 个几何体随机放到 6 个固定工位上。二维码内容是 ``JAKA1-<6 位数字>``，
数字是工位号 1..6 的一个排列，靠近机械臂的一排从 -x 到 +x 为 1、2、3，
远排为 4、5、6。抓取顺序只认腕部画面里解出来的码，不认生成时的那份文本。

工位坐标是这套腕部相机预览世界里够得到的位置，不是比赛场地上那套
150 mm 间距。编号方式和码的格式跟基础组一样。正式比赛的卡片在
(0, 110)，靠俯拍相机读。这里没有俯拍相机，卡片放在远排垫板外侧、
识别画面的下沿，腕部相机才看得全。
"""
from __future__ import annotations

import subprocess
import time
from pathlib import Path

import cv2
import numpy as np

from jaka_competition_kit.qr import decode_image, make_order, parse, payload
from shapes import SHAPES

WORLD = "gemini305_preview"
# 近排 y=0.255，远排 y=0.338。和预览世界里的垫板中心一致。
STATIONS: tuple[tuple[int, float, float], ...] = (
    (1, -0.120, 0.255),
    (2, 0.000, 0.255),
    (3, 0.120, 0.255),
    (4, -0.120, 0.338),
    (5, 0.000, 0.338),
    (6, 0.120, 0.338),
)
STATION_XY = {sid: (x, y) for sid, x, y in STATIONS}
# 远排垫板下沿约 y=0.383。0.10 m 的卡片中心放在 y=0.448，
# 和垫板留出约 15 mm，投影后落在 640×400 画面下沿以内。
QR_XYZ = (0.000, 0.448, 0.004)
QR_SIZE = 0.100
MATCH_RADIUS = 0.045
CARD_NAME = "qr_card"


def _ign(service: str, reqtype: str, reptype: str, req: str, timeout_ms: int = 2000) -> bool:
    result = subprocess.run(
        ["ign", "service", "-s", f"/world/{WORLD}/{service}",
         "--reqtype", reqtype, "--reptype", reptype,
         "--timeout", str(timeout_ms), "--req", req],
        check=False, capture_output=True, text=True,
    )
    text = (result.stdout or "") + (result.stderr or "")
    return result.returncode == 0 and "true" in text.lower()


def qr_modules(text: str) -> list[list[bool]]:
    import qrcode

    # 12 个字符用版本 1 就够。Q 档比 M 更能扛仿真里的糊边。
    code = qrcode.QRCode(
        version=None, border=2,
        error_correction=qrcode.constants.ERROR_CORRECT_Q,
    )
    code.add_data(text)
    code.make(fit=True)
    return code.get_matrix()


def _material(rgb: str, emissive: str) -> str:
    return (
        "<material>"
        f"<ambient>{rgb}</ambient><diffuse>{rgb}</diffuse>"
        "<specular>0 0 0 1</specular>"
        f"<emissive>{emissive}</emissive>"
        "</material>"
    )


def _card_sdf(text: str) -> str:
    """黑白小方块拼出二维码。不靠贴图，Gazebo 里每个模块都是实色。"""
    matrix = qr_modules(text)
    count = len(matrix)
    module = QR_SIZE / count
    visuals = [
        "        <visual name='board'>",
        "          <pose>0 0 0.0005 0 0 0</pose>",
        f"          <geometry><box><size>{QR_SIZE + 0.006:.4f} {QR_SIZE + 0.006:.4f} 0.001</size></box></geometry>",
        f"          {_material('0.96 0.96 0.96 1', '0.55 0.55 0.55 1')}",
        "        </visual>",
    ]
    half = QR_SIZE / 2.0
    # 黑块略大于一格，避免模块之间露出白缝，解码时被当成断线。
    black = module * 1.04
    for row, bits in enumerate(matrix):
        for col, bit in enumerate(bits):
            if not bit:
                continue
            # 矩阵第 0 行放在世界 -Y，识别画面里靠近上沿。
            x = (col + 0.5) * module - half
            y = half - (row + 0.5) * module
            visuals.append(
                f"        <visual name='m{row}_{col}'>"
                f"<pose>{x:.5f} {y:.5f} 0.0012 0 0 0</pose>"
                f"<geometry><box><size>{black:.5f} {black:.5f} 0.001</size></box></geometry>"
                f"{_material('0.02 0.02 0.02 1', '0 0 0 1')}"
                "</visual>"
            )
    body = "\n".join(visuals)
    x, y, z = QR_XYZ
    return (
        "<sdf version='1.6'><model name='qr_card'><static>true</static>"
        f"<pose>{x:.3f} {y:.3f} {z:.3f} 0 0 0</pose><link name='link'>\n"
        f"{body}\n"
        "</link></model></sdf>\n"
    )


def show_qr(text: str, directory: Path) -> Path:
    """撤掉占位白卡片，按这一轮的码重新摆一块。"""
    directory.mkdir(parents=True, exist_ok=True)
    path = directory / "qr_card.sdf"
    path.write_text(_card_sdf(text), encoding="utf-8")
    ok = False
    for _attempt in range(3):
        _ign("remove", "ignition.msgs.Entity", "ignition.msgs.Boolean",
             f'name: "{CARD_NAME}", type: 2', timeout_ms=1500)
        ok = _ign(
            "create", "ignition.msgs.EntityFactory", "ignition.msgs.Boolean",
            f'sdf_filename: "{path}"', timeout_ms=8000,
        )
        if ok:
            break
        time.sleep(0.4)
    if not ok:
        raise RuntimeError(f"二维码卡片没有放进仿真。看 {path}")
    return path


def deal(set_pose) -> tuple[list[tuple[str, int]], list[int], str]:
    """随机把 6 个几何体放到 6 个工位，并生成一组抓取顺序。"""
    import random

    shapes = list(SHAPES)
    random.shuffle(shapes)
    assignment = []
    for shape, (sid, x, y) in zip(shapes, STATIONS):
        if not set_pose(f"shape_{shape.id}", (x, y, shape.center_z), (0.0, 0.0, 0.0, 1.0)):
            raise RuntimeError(f"没能把 {shape.name} 放到工位 {sid}")
        assignment.append((shape.name, sid))
    order = make_order(require_derangement=True)
    return assignment, order, payload(order)


def read_order(image: np.ndarray, info, cam_xyz: tuple[float, float, float]) -> tuple[list[int], str]:
    """从腕部这一帧解二维码。解不到就返回空顺序。"""
    samples = []
    crop = _crop_card(image, info, cam_xyz)
    if crop is not None:
        samples.append(crop)
    samples.append(image)
    for sample in samples:
        text = _decode_views(sample)
        if not text:
            continue
        try:
            return parse(text), text
        except ValueError:
            continue
    return [], ""


def _decode_views(sample: np.ndarray) -> str:
    scale = 4 if max(sample.shape[:2]) < 900 else 2
    views = []
    for interpolation in (cv2.INTER_NEAREST, cv2.INTER_CUBIC):
        big = cv2.resize(sample, None, fx=scale, fy=scale, interpolation=interpolation)
        gray = cv2.cvtColor(big, cv2.COLOR_BGR2GRAY) if big.ndim == 3 else big
        _thr, bw = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        _mid, hard = cv2.threshold(gray, 140, 255, cv2.THRESH_BINARY)
        views.extend((bw, hard, gray))
    for view in views:
        text = decode_image(view)
        if text:
            return text
    return ""


def _crop_card(image: np.ndarray, info, cam_xyz):
    if info is None:
        return None
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    if fx <= 1 or fy <= 1:
        return None
    x, y, z = QR_XYZ
    z_opt = cam_xyz[2] - z
    if z_opt <= 0.05:
        return None
    # 和 grasp.pixels_to_world 同一套外参：光学 +X 朝世界 −X，+Y 朝世界 +Y。
    x_opt = cam_xyz[0] - x
    y_opt = y - cam_xyz[1]
    u = fx * x_opt / z_opt + cx
    v = fy * y_opt / z_opt + cy
    half = int(max(48, 0.62 * QR_SIZE / z_opt * max(fx, fy)))
    height, width = image.shape[:2]
    x1 = max(0, int(u) - half)
    y1 = max(0, int(v) - half)
    x2 = min(width, int(u) + half)
    y2 = min(height, int(v) + half)
    if x2 - x1 < 24 or y2 - y1 < 24:
        return None
    return image[y1:y2, x1:x2]


def order_picks(picks: list[dict], order: list[int]) -> tuple[list[dict], list[int]]:
    """按二维码里的工位号往下排。对不上的工位放进 missing。"""
    used: set[int] = set()
    ordered = []
    missing = []
    for station in order:
        sx, sy = STATION_XY[station]
        best = None
        best_d = 1e9
        for index, item in enumerate(picks):
            if index in used:
                continue
            x, y, _z = item["world"]
            dist = float(np.hypot(x - sx, y - sy))
            if dist < best_d:
                best_d = dist
                best = index
        if best is None or best_d > MATCH_RADIUS:
            missing.append(station)
            continue
        used.add(best)
        item = dict(picks[best])
        item["station"] = station
        ordered.append(item)
    return ordered, missing


def verify_roundtrip(text: str | None = None) -> str:
    """把模块矩阵画成约 5 像素一格的小图，确认能解回同一串。"""
    text = text or payload(make_order(seed=1, require_derangement=True))
    matrix = qr_modules(text)
    ppm = 5
    count = len(matrix)
    image = np.full((count * ppm, count * ppm), 255, np.uint8)
    for row, bits in enumerate(matrix):
        for col, bit in enumerate(bits):
            if bit:
                image[row * ppm:(row + 1) * ppm, col * ppm:(col + 1) * ppm] = 0
    blurred = cv2.GaussianBlur(image, (0, 0), 1.0)
    color = cv2.cvtColor(blurred, cv2.COLOR_GRAY2BGR)
    got = _decode_views(color)
    if got != text:
        raise RuntimeError(f"二维码回读失败: 写入 {text} 解出 {got!r}")
    if parse(got) != parse(text):
        raise RuntimeError(f"二维码顺序不一致: {text} -> {got}")
    return got


if __name__ == "__main__":
    seen = verify_roundtrip()
    picks = [
        {"world": (0.000, 0.255, 0.05), "name": "近中"},
        {"world": (-0.120, 0.338, 0.05), "name": "远左"},
        {"world": (0.121, 0.254, 0.05), "name": "近右"},
    ]
    ordered, missing = order_picks(picks, [4, 3, 2, 1, 5, 6])
    stations = [item["station"] for item in ordered]
    if stations != [4, 3, 2] or missing != [1, 5, 6]:
        raise SystemExit(f"工位排序不对: {stations} missing={missing}")
    print(f"二维码回读 {seen}，工位排序 {stations}")
