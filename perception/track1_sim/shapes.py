"""基础组 6 个几何体，给仿真采集做标注用。

尺寸是预览世界里的可视模型，单位米。bounds 是相对模型原点的
(xmin, xmax, ymin, ymax, zmin, zmax)，和网格或基本体的外包络一致。
编号顺序就是 YOLO 的 class id，不要改。
"""
from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class Shape:
    id: str
    name: str
    bounds: tuple[float, float, float, float, float, float]
    # 放在台面上时，模型原点的高度。和 gemini305_preview.sdf 一致。
    center_z: float


SHAPES: tuple[Shape, ...] = (
    Shape("cube", "正方体", (-0.025, 0.025, -0.025, 0.025, -0.025, 0.025), 0.029),
    Shape("rod", "长方体", (-0.039, 0.039, -0.016, 0.016, -0.020, 0.020), 0.024),
    Shape("cylinder", "圆柱体", (-0.025, 0.025, -0.025, 0.025, -0.025, 0.025), 0.029),
    Shape("hex", "六角柱", (-0.025, 0.025, -0.028868, 0.028868, -0.025, 0.025), 0.029),
    Shape("tri", "三角柱", (-0.028, 0.028, -0.016166, 0.032332, -0.023, 0.023), 0.027),
    Shape("cone", "圆台", (-0.026, 0.026, -0.026, 0.026, -0.021, 0.021), 0.025),
)

# 识别演示用的那一摆。采集结束要放回去，别把正在看的预览留乱。
HOME: tuple[tuple[str, float, float, float], ...] = (
    ("cube", -0.120, 0.255, 0.35),
    ("rod", 0.000, 0.255, 0.55),
    ("cylinder", 0.120, 0.255, 0.00),
    ("hex", -0.120, 0.338, 0.20),
    ("tri", 0.000, 0.338, 0.40),
    ("cone", 0.120, 0.338, 0.00),
)
