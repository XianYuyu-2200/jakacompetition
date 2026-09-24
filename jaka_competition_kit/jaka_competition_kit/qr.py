"""二维码生成与解析。

载荷格式: ``JAKA1-<6 位数字>``, 例如 ``JAKA1-246135`` 表示抓取顺序 2→4→6→1→3→5。
数字必须恰好是 1..6 的一个排列。
"""
from __future__ import annotations

import random
from typing import List, Sequence

PREFIX = "JAKA1-"
ORDER_LEN = 6


def make_order(seed: int | None = None, require_derangement: bool = False) -> List[int]:
    """生成一个 1..6 的随机排列。

    require_derangement=True 时保证不是递增序(避免出现 123456 这种无聊的局面)。
    """
    rng = random.Random(seed)
    while True:
        order = list(range(1, ORDER_LEN + 1))
        rng.shuffle(order)
        if not require_derangement or order != list(range(1, ORDER_LEN + 1)):
            return order


def payload(order: Sequence[int]) -> str:
    order = list(order)
    if sorted(order) != list(range(1, ORDER_LEN + 1)):
        raise ValueError(f"抓取顺序必须是 1..6 的排列, 收到 {order}")
    return PREFIX + "".join(str(d) for d in order)


def parse(text: str) -> List[int]:
    """解析二维码文本;格式不合法直接抛 ValueError(裁判判定为无效)。"""
    text = (text or "").strip()
    if not text.upper().startswith(PREFIX):
        raise ValueError(f"二维码前缀不是 {PREFIX!r}: {text!r}")
    digits = text[len(PREFIX):]
    if len(digits) != ORDER_LEN or not digits.isdigit():
        raise ValueError(f"二维码内容必须是 {ORDER_LEN} 位数字: {text!r}")
    order = [int(c) for c in digits]
    if sorted(order) != list(range(1, ORDER_LEN + 1)):
        raise ValueError(f"二维码数字不是 1..{ORDER_LEN} 的排列: {text!r}")
    return order


def render_png(text: str, path: str, box_size: int = 10, border: int = 4) -> str:
    """把二维码写成 PNG, 返回路径。"""
    import qrcode
    qr = qrcode.QRCode(version=None, box_size=box_size, border=border,
                       error_correction=qrcode.constants.ERROR_CORRECT_M)
    qr.add_data(text)
    qr.make(fit=True)
    qr.make_image(fill_color="black", back_color="white").save(path)
    return path


def decode_image(image) -> str:
    """从图像(numpy 数组或路径)解码二维码文本;识别不到返回空串。

    依赖 pyzbar;OpenCV 自带的 QRCodeDetector 在很多发行版里没有链接 QUIRC,
    实际不能解码, 所以这里默认用 pyzbar。
    """
    try:
        from pyzbar.pyzbar import decode as _decode
    except ImportError as exc:                       # pragma: no cover
        raise RuntimeError("需要 python3-pyzbar: sudo apt install python3-pyzbar libzbar0") from exc
    if isinstance(image, str):
        import cv2
        image = cv2.imread(image, cv2.IMREAD_GRAYSCALE)
    results = _decode(image)
    return results[0].data.decode("utf-8") if results else ""
