#!/usr/bin/env python3
"""基础组 6 个几何体的外形，以及只靠轮廓和深度的分类。

不看颜色。仿真里给它们上了不同的颜色，只是为了让人眼能分开，
分类器拿到的是深度图里高于台面的那一块轮廓。

尺寸是腕部相机预览用的可视模型，不是 arena.yaml 里那个 50 mm
抓取包络。抓取高度仍然按 arena.yaml。
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import cv2
import numpy as np


@dataclass(frozen=True)
class Shape:
    id: str
    name: str
    # 俯视外包络，米。圆台给的是底面直径。
    footprint: tuple[float, float]
    height: float


SHAPES: tuple[Shape, ...] = (
    Shape("cube", "正方体", (0.050, 0.050), 0.050),
    Shape("rod", "长方体", (0.078, 0.032), 0.040),
    Shape("cylinder", "圆柱体", (0.050, 0.050), 0.050),
    Shape("hex", "六角柱", (0.050, 0.050), 0.050),
    Shape("tri", "三角柱", (0.056, 0.048), 0.046),
    Shape("cone", "圆台", (0.052, 0.052), 0.042),
)

# 放在当前识别位姿看得到、又不挨着手指的两排。不是 6 个工位坐标：
# 工位近排 y=205 mm 会顶到画面上沿的手指。
PLACEMENTS: tuple[tuple[str, float, float, float], ...] = (
    ("cube", -0.120, 0.255, 0.35),
    ("rod", 0.000, 0.255, 0.55),
    ("cylinder", 0.120, 0.255, 0.0),
    ("hex", -0.120, 0.338, 0.20),
    ("tri", 0.000, 0.338, 0.40),
    ("cone", 0.120, 0.338, 0.0),
)

SHAPE_BY_ID = {s.id: s for s in SHAPES}


def _circularity(contour) -> float:
    peri = cv2.arcLength(contour, True)
    if peri <= 1e-6:
        return 0.0
    area = abs(cv2.contourArea(contour))
    return float(4.0 * math.pi * area / (peri * peri))


def _aspect(contour) -> float:
    (_, (w, h), _angle) = cv2.minAreaRect(contour)
    short = max(min(w, h), 1e-6)
    return float(max(w, h) / short)


def _vertices(contour) -> int:
    peri = cv2.arcLength(contour, True)
    # 0.04 对这几种凸外形比较稳：三角 3、四边 4、六边 6、圆更多。
    approx = cv2.approxPolyDP(contour, 0.04 * peri, True)
    return int(len(approx))


def _depth_span(depth: np.ndarray, contour) -> float:
    mask = np.zeros(depth.shape[:2], np.uint8)
    cv2.drawContours(mask, [contour], -1, 1, thickness=-1)
    inside = cv2.distanceTransform(mask, cv2.DIST_L2, 3)
    core = inside >= 2.5
    values = depth[core] if np.count_nonzero(core) >= 20 else depth[mask > 0]
    values = values[np.isfinite(values)]
    if values.size < 10:
        return 0.0
    return float(np.percentile(values, 90) - np.percentile(values, 10))


def _top_contour(contour, depth: np.ndarray):
    """只留离相机最近的那一层，也就是顶面。侧面会把方块看成多边形。"""
    mask = np.zeros(depth.shape[:2], np.uint8)
    cv2.drawContours(mask, [contour], -1, 1, thickness=-1)
    values = depth[mask > 0]
    values = values[np.isfinite(values)]
    if values.size < 10:
        return contour, 1.0
    top = float(np.percentile(values, 8))
    band = ((mask > 0) & np.isfinite(depth) & (depth <= top + 0.005)).astype(np.uint8)
    found, _ = cv2.findContours(band, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not found:
        return contour, 1.0
    top_contour = max(found, key=cv2.contourArea)
    full_area = max(abs(cv2.contourArea(contour)), 1.0)
    top_area = max(abs(cv2.contourArea(top_contour)), 1.0)
    return top_contour, full_area / top_area


def classify_contour(contour, depth: np.ndarray) -> tuple[str, dict]:
    """返回形状 id 和用来判的数。depth 是整幅图，单位米。

    先取顶面轮廓。圆柱和圆台的顶面都是圆，圆台顶面比整个外轮廓小一圈。
    """
    span = _depth_span(depth, contour)
    top, ratio = _top_contour(contour, depth)
    circ = _circularity(top)
    aspect = _aspect(top)
    verts = _vertices(top)
    features = {
        "circularity": round(circ, 3),
        "aspect": round(aspect, 3),
        "vertices": verts,
        "depth_span_mm": round(span * 1000.0, 1),
        "top_ratio": round(ratio, 2),
    }
    if ratio >= 1.8 and circ >= 0.78 and aspect < 1.35:
        kind = "cone"
    elif circ >= 0.86 and verts >= 7:
        kind = "cylinder"
    elif aspect >= 1.55 and verts <= 6:
        kind = "rod"
    elif verts == 3 or (circ < 0.68 and aspect < 1.45):
        kind = "tri"
    elif verts == 6 or (0.78 <= circ < 0.88 and 5 <= verts <= 7 and aspect < 1.40):
        kind = "hex"
    elif verts == 4 and aspect < 1.45:
        kind = "cube"
    elif circ >= 0.86:
        kind = "cylinder"
    elif aspect >= 1.50:
        kind = "rod"
    elif circ < 0.70:
        kind = "tri"
    elif circ >= 0.78:
        kind = "hex"
    else:
        kind = "cube"
    return kind, features


def height_mask(depth: np.ndarray, low_m: float = 0.012, high_m: float = 0.070) -> np.ndarray:
    """台面以上、手指以下的像素。depth 是光轴距离，台面几乎是一个常数。"""
    valid = np.isfinite(depth) & (depth > 0.05) & (depth < 1.2)
    if not np.any(valid):
        return np.zeros(depth.shape[:2], np.uint8)
    table = float(np.percentile(depth[valid], 92))
    above = table - depth
    mask = valid & (above >= low_m) & (above <= high_m)
    out = mask.astype(np.uint8)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    out = cv2.morphologyEx(out, cv2.MORPH_OPEN, kernel)
    out = cv2.morphologyEx(out, cv2.MORPH_CLOSE, kernel)
    return out


def _synthetic_depth(kind: str, size: int = 160) -> tuple[np.ndarray, np.ndarray]:
    """造一块和腕部相机差不多大的俯视深度，用来锁分类规则。"""
    canvas = np.zeros((size, size), np.uint8)
    cx = cy = size // 2
    if kind == "cube":
        cv2.fillPoly(canvas, [np.array([
            [cx - 28, cy - 28], [cx + 28, cy - 28],
            [cx + 28, cy + 28], [cx - 28, cy + 28],
        ])], 1)
        depth = np.full((size, size), 0.289, np.float32)
        depth[canvas > 0] = 0.239
    elif kind == "rod":
        rect = cv2.boxPoints(((cx, cy), (90, 34), 28))
        cv2.fillPoly(canvas, [np.int32(rect)], 1)
        depth = np.full((size, size), 0.289, np.float32)
        depth[canvas > 0] = 0.249
    elif kind == "cylinder":
        cv2.circle(canvas, (cx, cy), 30, 1, -1)
        depth = np.full((size, size), 0.289, np.float32)
        depth[canvas > 0] = 0.239
    elif kind == "hex":
        pts = []
        for i in range(6):
            a = math.radians(15 + 60 * i)
            pts.append([cx + 32 * math.cos(a), cy + 32 * math.sin(a)])
        cv2.fillPoly(canvas, [np.int32(pts)], 1)
        depth = np.full((size, size), 0.289, np.float32)
        depth[canvas > 0] = 0.239
    elif kind == "tri":
        pts = []
        for i in range(3):
            a = math.radians(-90 + 120 * i + 18)
            pts.append([cx + 36 * math.cos(a), cy + 36 * math.sin(a)])
        cv2.fillPoly(canvas, [np.int32(pts)], 1)
        depth = np.full((size, size), 0.289, np.float32)
        depth[canvas > 0] = 0.243
    elif kind == "cone":
        cv2.circle(canvas, (cx, cy), 30, 1, -1)
        yy, xx = np.mgrid[0:size, 0:size]
        radius = np.sqrt((xx - cx) ** 2 + (yy - cy) ** 2) / 30.0
        depth = np.full((size, size), 0.289, np.float32)
        # 中心是小顶面，边缘落到台面附近。10 mm 阈值会切掉最外圈。
        depth[canvas > 0] = 0.289 - 0.042 * np.clip(1.0 - radius[canvas > 0], 0, 1)
    else:
        raise ValueError(kind)
    rng = np.random.default_rng(0)
    depth += rng.normal(0, 0.0004, depth.shape).astype(np.float32)
    return depth, canvas


def self_check() -> None:
    failures = []
    for shape in SHAPES:
        depth, _canvas = _synthetic_depth(shape.id)
        mask = height_mask(depth)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            failures.append(f"{shape.id}: 没有轮廓")
            continue
        contour = max(contours, key=cv2.contourArea)
        kind, features = classify_contour(contour, depth)
        if kind != shape.id:
            failures.append(f"{shape.id} 被认成 {kind} {features}")
    if failures:
        raise SystemExit("基础组外形自检失败: " + "; ".join(failures))


if __name__ == "__main__":
    self_check()
    print("self_check ok")
