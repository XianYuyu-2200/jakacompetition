#!/usr/bin/env python3
"""生成《JAKA Mini 2 机器人应用挑战赛·赛题细则》配图。

输出(与本脚本同目录):
  fig_arena_track1.png   赛道一赛场布局(俯视)
  fig_arena_track2.png   赛道二赛场布局(俯视)
  fig_workspace.png      竖直抓取可达作业带
  fig_flow.png           竞赛流程图
"""
import math
import os
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.patches import Circle, Rectangle, Wedge, FancyBboxPatch, FancyArrowPatch

import _fonts
_reg, _bold = _fonts.ensure_fonts(verbose=False)
font_manager.fontManager.addfont(_reg)
font_manager.fontManager.addfont(_bold)
plt.rcParams["font.sans-serif"] = ["Noto Sans CJK SC"]
plt.rcParams["axes.unicode_minus"] = False

HERE = os.path.dirname(os.path.abspath(__file__))
R_MIN, R_MAX = 180.0, 420.0
GREEN, BLUE, RED = "#2e8b57", "#3a6ea5", "#c0392b"


def base_arena(ax, xlim, ylim, band_label=(302, 302)):
    ax.add_patch(Wedge((0, 0), R_MAX, 0, 180, width=R_MAX - R_MIN,
                       fc="#d9f0e0", ec="none", zorder=0))
    for r in (R_MIN, R_MAX):
        ax.add_patch(Circle((0, 0), r, fill=False, ec=GREEN, lw=1.4, ls="--", zorder=1))
    ax.text(band_label[0], band_label[1], "可达作业带\n180–420 mm",
            color=GREEN, fontsize=9, ha="center", va="center", zorder=3)
    ax.add_patch(Circle((0, 0), 46, color="0.25", zorder=4))
    ax.text(0, 0, "JAKA\nMini 2", ha="center", va="center", color="w",
            fontsize=8, zorder=5)
    ax.annotate("", xy=(720, 0), xytext=(0, 0),
                arrowprops=dict(arrowstyle="->", color="0.3", lw=1.2))
    ax.text(660, -55, "+x", color="0.3", fontsize=9)
    ax.set_aspect("equal")
    ax.set_xlim(*xlim)
    ax.set_ylim(*ylim)
    ax.grid(alpha=0.18, zorder=0)
    ax.set_xlabel("x (mm)")
    ax.set_ylabel("y (mm)  距基座正前方")


def camera(ax, y):
    ax.add_patch(Circle((0, y), 30, fill=False, ec="#7d3c98", lw=1.5, zorder=4))
    ax.text(0, y, "俯视\n相机", ha="center", va="center", fontsize=7.5,
            color="#7d3c98", zorder=5)


def fig_track1():
    fig, ax = plt.subplots(figsize=(9, 7.2))
    base_arena(ax, (-540, 800), (-160, 640))
    ax.add_patch(Rectangle((-425, -110), 850, 690, fill=False, ec="0.6",
                           ls=":", lw=1))
    ax.text(-420, 590, "工作台面  900 × 700 mm", fontsize=8, color="0.45")
    camera(ax, 590)

    stations = {1: (-150, 205), 2: (0, 205), 3: (150, 205),
                4: (-150, 355), 5: (0, 355),
                6: (150, 355)}
    for sid, (x, y) in stations.items():
        r = math.hypot(x, y)
        ax.add_patch(Rectangle((x - 50, y - 50), 100, 100, fc="#bfe3cb",
                               ec=GREEN, lw=1.4, zorder=2))
        ax.text(x, y + 12, f"工位 {sid}", ha="center", va="center", fontsize=9,
                zorder=3)
        ax.text(x, y - 16, f"r={r:.0f}", ha="center", va="center", fontsize=7.5,
                color="0.35", zorder=3)

    ax.add_patch(Rectangle((215, 105), 200, 150, fc="#cfe0f5", ec=BLUE,
                           lw=1.4, zorder=2))
    ax.text(315, 180, "料盒\n200×150×40", ha="center", va="center", fontsize=8.5,
            color="#1f3f66", zorder=3)

    ax.add_patch(Rectangle((-30, 80), 60, 60, fc="#f6e6c8", ec="#b8860b",
                           lw=1.4, zorder=2))
    ax.text(0, 110, "二维码", ha="center", va="center", fontsize=8, zorder=3)
    ax.text(44, 100, "60×60", ha="left", fontsize=7, color="0.4")

    ax.set_title("图 1  赛道一 · 二维码序令定点抓取赛  赛场布局(俯视)", fontsize=12,
                 pad=12)
    fig.tight_layout()
    fig.savefig(os.path.join(HERE, "fig_arena_track1.png"), dpi=200)
    plt.close(fig)


def fig_track2():
    fig, ax = plt.subplots(figsize=(9.2, 7.4))
    base_arena(ax, (-620, 720), (-160, 660), band_label=(500, 160))
    ax.add_patch(Rectangle((-425, -110), 850, 690, fill=False, ec="0.6",
                           ls=":", lw=1))
    ax.text(-420, 590, "工作台面  900 × 700 mm", fontsize=8, color="0.45")
    camera(ax, 600)

    # 散放区 x[-190,190] y[190,365]
    ax.add_patch(Rectangle((-190, 190), 380, 175, fc="#e7ecf5", ec=BLUE,
                           lw=1.6, ls="-", zorder=2))
    ax.text(0, 278, "随机散放区\n380 (x) × 175 (y)\n每轮 16 件，每类 4 件",
            ha="center", va="center", fontsize=9.5, color="#1f3f66", zorder=3)
    ax.annotate("最近点 (0, 190)\nr = 190 mm", xy=(0, 190), xytext=(40, 95),
                fontsize=8.5, color=RED, ha="left",
                arrowprops=dict(arrowstyle="->", color=RED, lw=1.1))
    ax.annotate("最远角 (190, 365)   r = 411 mm", xy=(190, 365), xytext=(250, 430),
                fontsize=8.5, color=RED,
                arrowprops=dict(arrowstyle="->", color=RED, lw=1.1))

    bins = [
        (270, 125, "#d6e4f7", "#1f4e79", "生活\n130×110"),
        (-270, 125, "#fde3cc", "#a85b10", "工具\n130×110"),
        (270, 245, "#f8d5e6", "#9a3368", "玩具\n130×110"),
        (-270, 245, "#d7efd4", "#2d7a34", "水果\n130×110"),
    ]
    for cx, cy, fc, ec, label in bins:
        ax.add_patch(Rectangle((cx - 65, cy - 55), 130, 110, fc=fc, ec=ec,
                               lw=1.4, zorder=2))
        ax.text(cx, cy, label, ha="center", va="center", fontsize=8.5,
                color=ec, zorder=3)

    ax.set_title("图 2  赛道二 · 泛化目标分类抓取赛  赛场布局(俯视)", fontsize=12,
                 pad=12)
    fig.tight_layout()
    fig.savefig(os.path.join(HERE, "fig_arena_track2.png"), dpi=200)
    plt.close(fig)


def fig_workspace():
    z = [30, 50, 80, 120, 160, 200]
    rmin = [180, 180, 170, 155, 120, 85]
    rmax = [420, 420, 415, 406, 397, 380]
    fig, ax = plt.subplots(figsize=(8.2, 5.4))
    ax.fill_betweenx(z, rmin, rmax, color="#d9f0e0", alpha=0.9,
                     label="竖直抓取可达区间")
    ax.plot(rmin, z, "o-", color="#2e8b57", lw=1.6, ms=4, label="可达下界 r_min")
    ax.plot(rmax, z, "o-", color="#c0392b", lw=1.6, ms=4, label="可达上界 r_max")
    for xi, zi, v in zip(rmax, z, rmax):
        ax.annotate(f"{v}", (xi, zi), textcoords="offset points", xytext=(4, 4),
                    fontsize=7.5, color="#c0392b")
    for xi, zi, v in zip(rmin, z, rmin):
        ax.annotate(f"{v}", (xi, zi), textcoords="offset points", xytext=(-20, -3),
                    fontsize=7.5, color="#2e8b57")
    ax.axvspan(180, 420, color="#f2c14e", alpha=0.12)
    ax.text(300, 20, "通用作业带\n180–420 mm", color="#8a6d1a", fontsize=9,
            ha="center")
    ax.set_xlabel("抓取点至基座水平距离 r (mm)")
    ax.set_ylabel("抓取高度 z (mm, 相对基座安装面)")
    ax.set_title("图 3  工具轴竖直向下时的可达作业带(MoveIt 实测)", fontsize=12)
    ax.set_xlim(0, 470)
    ax.set_ylim(0, 220)
    ax.grid(alpha=0.25)
    ax.legend(loc="upper right", fontsize=8)
    fig.tight_layout()
    fig.savefig(os.path.join(HERE, "fig_workspace.png"), dpi=200)
    plt.close(fig)


def fig_flow():
    steps = ["裁判出题\n随机生成二维码/工件位姿", "系统识别\n视觉/大模型解析任务",
             "运动规划\nMoveIt 2 求无碰撞轨迹", "抓取搬运\n夹爪抓取并放入料盒",
             "自动判分\n计时/计分/生成成绩单"]
    fig, ax = plt.subplots(figsize=(11, 2.2))
    w, h, gap = 1.75, 1.0, 0.5
    for i, s in enumerate(steps):
        x = i * (w + gap)
        ax.add_patch(FancyBboxPatch((x, 0), w, h, boxstyle="round,pad=0.06,rounding_size=0.12",
                                    fc="#eef3fa", ec=BLUE, lw=1.4))
        ax.text(x + w / 2, h / 2, s, ha="center", va="center", fontsize=9)
        if i < len(steps) - 1:
            ax.add_patch(FancyArrowPatch((x + w + 0.04, h / 2), (x + w + gap - 0.04, h / 2),
                                         arrowstyle="-|>", mutation_scale=14,
                                         color="0.4", lw=1.3))
    ax.set_xlim(-0.15, len(steps) * (w + gap) - gap + 0.15)
    ax.set_ylim(-0.2, h + 0.2)
    ax.axis("off")
    fig.tight_layout()
    fig.savefig(os.path.join(HERE, "fig_flow.png"), dpi=200)
    plt.close(fig)


if __name__ == "__main__":
    fig_track1()
    fig_track2()
    fig_workspace()
    fig_flow()
    print("figures written to", HERE)
