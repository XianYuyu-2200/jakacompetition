"""Pixel rectangles to YOLO detection lines.

YOLO label format is one object per line:

    class cx cy w h

with every number except the class id normalized to the image size.
"""
from __future__ import annotations


def normalize_box(x1, y1, x2, y2, width, height, min_px=6):
    """Return an inclusive-exclusive box clipped to the frame, or None if it is tiny."""
    if width <= 1 or height <= 1:
        return None
    ax, bx = sorted((int(x1), int(x2)))
    ay, by = sorted((int(y1), int(y2)))
    ax = max(0, min(width - 1, ax))
    ay = max(0, min(height - 1, ay))
    bx = max(0, min(width, bx))
    by = max(0, min(height, by))
    if bx - ax < min_px or by - ay < min_px:
        return None
    return ax, ay, bx, by


def yolo_line(class_index, box, width, height):
    x1, y1, x2, y2 = box
    bw = (x2 - x1) / float(width)
    bh = (y2 - y1) / float(height)
    cx = (x1 + x2) / 2.0 / float(width)
    cy = (y1 + y2) / 2.0 / float(height)

    def clip(value):
        return min(1.0, max(0.0, value))

    return (
        f"{int(class_index)} {clip(cx):.6f} {clip(cy):.6f} "
        f"{clip(bw):.6f} {clip(bh):.6f}"
    )


def _self_test():
    assert normalize_box(10, 10, 4, 40, 100, 100) == (4, 10, 10, 40)
    assert normalize_box(0, 0, 2, 2, 100, 100) is None
    assert normalize_box(-5, -5, 20, 20, 100, 100) == (0, 0, 20, 20)
    line = yolo_line(3, (0, 0, 100, 50), 100, 100)
    assert line == "3 0.500000 0.250000 1.000000 0.500000", line
    print("boxes ok")


if __name__ == "__main__":
    _self_test()
