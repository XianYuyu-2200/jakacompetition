"""OpenCV 画面上写中文。putText 没有中文字形。"""
from __future__ import annotations

from functools import lru_cache
from pathlib import Path

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont


FONT_CANDIDATES = (
    "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
    "/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc",
    "/usr/share/fonts/truetype/droid/DroidSansFallbackFull.ttf",
    "/usr/share/fonts/truetype/arphic/uming.ttc",
)

BIN_BGR = {
    "life": (70, 170, 60),
    "tool": (30, 140, 230),
    "toy": (200, 90, 200),
    "fruit": (40, 50, 220),
}


@lru_cache(maxsize=1)
def font_path() -> str | None:
    for candidate in FONT_CANDIDATES:
        if Path(candidate).is_file():
            return candidate
    return None


@lru_cache(maxsize=8)
def get_font(size: int):
    path = font_path()
    if path is None:
        return ImageFont.load_default()
    return ImageFont.truetype(path, size=size)


def blit_labels(bgr, labels):
    """Draw labels in one RGB conversion.

    labels: iterable of (text, x, y, color_bgr, size, bg_bgr or None)
    """
    labels = list(labels)
    if not labels:
        return bgr
    image = Image.fromarray(cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB))
    draw = ImageDraw.Draw(image)
    for text, x, y, color, size, bg in labels:
        font = get_font(int(size))
        bbox = font.getbbox(text)
        width = bbox[2] - bbox[0]
        height = bbox[3] - bbox[1]
        x = int(x)
        y = int(y)
        if bg is not None:
            draw.rectangle(
                [x, y, x + width + 8, y + height + 8],
                fill=(int(bg[2]), int(bg[1]), int(bg[0])),
            )
        draw.text(
            (x + 4, y + 2),
            text,
            font=font,
            fill=(int(color[2]), int(color[1]), int(color[0])),
        )
    return cv2.cvtColor(np.asarray(image), cv2.COLOR_RGB2BGR)


def status_bar(width, lines, height=64):
    bar = np.zeros((height, int(width), 3), dtype=np.uint8)
    bar[:] = (24, 24, 24)
    return blit_labels(
        bar,
        [
            (text, 8, 6 + i * 28, (240, 240, 240), 20, None)
            for i, text in enumerate(lines[:2])
        ],
    )
