#!/usr/bin/env python3
"""生成赛场布局图 + 可达性边界图, 用于赛题细则配图。"""
import math, os
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Rectangle, FancyArrow
import numpy as np

plt.rcParams["font.sans-serif"] = ["Noto Sans CJK JP", "Droid Sans Fallback"]
plt.rcParams["axes.unicode_minus"] = False
HERE = os.path.dirname(os.path.abspath(__file__))

STRICT_R = 420.0    # 工具严格竖直向下时的可达半径上限 (z=50mm, MoveIt 实测)
TILT_R = 459.0      # 允许倾角<=15度 / 腕心最大伸展
PITCH = 150.0


def stations(R):
    return [((c-1)*PITCH, R + (rw-0.5)*PITCH) for rw in (0,1) for c in (0,1,2)]


fig, axes = plt.subplots(1, 2, figsize=(16, 8))

for ax, R, title in ((axes[0], 350.0, "原稿方案: 工位阵中心距 350mm"),
                     (axes[1], 280.0, "建议方案: 工位阵中心距 280mm")):
    ax.add_patch(Circle((0,0), STRICT_R, fill=False, ec="tab:red", lw=2, ls="-",
                        label=f"严格竖直下抓可达上限 {STRICT_R:.0f}mm"))
    ax.add_patch(Circle((0,0), TILT_R, fill=False, ec="tab:orange", lw=2, ls="--",
                        label=f"允许倾角15°上限 {TILT_R:.0f}mm"))
    ax.add_patch(Circle((0,0), 62, color="0.3"))
    ax.text(0, -18, "JAKA\nMini 2", ha="center", va="center", color="w", fontsize=8)
    # 工作台
    ax.add_patch(Rectangle((-450, 60), 900, 560, fill=False, ec="0.6", ls=":", lw=1))
    ax.text(-445, 625, "工作台面", fontsize=8, color="0.4")
    ax.add_patch(Circle((0, 640), 26, fill=False, ec="tab:purple", lw=1.5))
    ax.text(0, 640, "俯视\n相机", ha="center", va="center", fontsize=7, color="tab:purple")
    # 工位
    maxr = 0
    for i, (x, y) in enumerate(stations(R), start=1):
        rr = math.hypot(x, y)
        maxr = max(maxr, rr)
        bad = rr > STRICT_R
        ax.add_patch(Rectangle((x-50, y-50), 100, 100,
                               fill=True, alpha=0.25,
                               fc="tab:red" if bad else "tab:green",
                               ec="tab:red" if bad else "tab:green"))
        ax.text(x, y, f"{i}\n${rr:.0f}$", ha="center", va="center", fontsize=8,
                color="darkred" if bad else "darkgreen")
    # 料盒
    bx, by = 280, 180
    ax.add_patch(Rectangle((bx-75, by-50), 150, 100, fill=True, alpha=0.3,
                           fc="tab:blue", ec="tab:blue"))
    ax.text(bx, by, "料盒", ha="center", va="center", fontsize=9)
    ax.plot([0], [0], "k.")
    ax.set_aspect("equal"); ax.grid(alpha=0.25)
    ax.set_xlim(-520, 520); ax.set_ylim(-120, 680)
    ax.set_xlabel("x (mm)   横向"); ax.set_ylabel("y (mm)   距基座方向")
    verdict = "最外工位 %.0fmm > %.0fmm : 竖直抓取不可达!" % (maxr, STRICT_R) if maxr > STRICT_R \
              else "最外工位 %.0fmm, 余量 %.0fmm : 全部可行" % (maxr, STRICT_R - maxr)
    ax.set_title(f"{title}\n{verdict}", fontsize=11)
    ax.legend(loc="lower left", fontsize=8)

plt.tight_layout()
out = os.path.join(HERE, "arena_layout.png")
plt.savefig(out, dpi=140)
print("saved", out)

# 第二张: r-z 剖面
fig2, ax2 = plt.subplots(figsize=(9, 7))
zz = [30, 50, 80, 120, 160, 200]
rr = [420, 420, 415, 406, 397, 380]
ax2.plot(rr, zz, "o-", label="严格竖直下抓可达上限 (MoveIt 实测)")
ax2.fill(np.r_[rr, 380, 420, 420], np.r_[zz, 200, 30, 0], alpha=0.12, color="tab:green")
ax2.axvline(300, color="tab:blue", ls="--", lw=1)
ax2.axvline(500, color="tab:purple", ls="--", lw=1)
ax2.axvspan(300, 500, alpha=0.08, color="tab:purple")
ax2.text(400, 190, "原稿声称的\n\"300-500mm 最优区间\"", ha="center", fontsize=9, color="tab:purple")
ax2.text(275, 30, "建议工位区\n~200-400mm", ha="right", fontsize=9, color="tab:blue")
ax2.set_xlabel("工位至基座水平距离 r (mm)")
ax2.set_ylabel("抓取高度 z (mm, 相对基座安装面)")
ax2.set_title("JAKA Mini 2 竖直抓取可达域 (基于 jaka_minicobo URDF + MoveIt 实测)")
ax2.grid(alpha=0.3); ax2.legend(fontsize=9)
ax2.set_xlim(0, 620); ax2.set_ylim(0, 240)
plt.tight_layout()
out2 = os.path.join(HERE, "reach_envelope.png")
plt.savefig(out2, dpi=140)
print("saved", out2)
