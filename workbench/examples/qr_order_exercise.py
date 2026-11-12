#!/usr/bin/env python3
"""纯 Python 顺序练习，不启动 ROS、不控制机械臂、不识别图片。

输入的是已经识读的二维码文本；相机/图片解码请使用 student_track1.py。
用法：python3 qr_order_exercise.py JAKA1-246135
"""
import argparse


def parse_order(text):
    value = text.strip()
    if not value.upper().startswith("JAKA1-"):
        raise ValueError("前缀应为 JAKA1-")
    digits = value[6:]
    if len(digits) != 6 or sorted(digits) != list("123456"):
        raise ValueError("顺序必须恰好是 1～6 各一次，例如 JAKA1-246135")
    return [int(digit) for digit in digits]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("payload", nargs="?", default="JAKA1-246135")
    args = parser.parse_args()
    try:
        order = parse_order(args.payload)
    except ValueError as exc:
        parser.error(str(exc))
    print("抓取顺序：" + " → ".join(map(str, order)))


if __name__ == "__main__":
    main()
