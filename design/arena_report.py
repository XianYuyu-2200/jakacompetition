#!/usr/bin/env python3
"""汇总: 给出推荐的赛场尺寸依据。"""
import sys, math, numpy as np
sys.path.insert(0, ".")
import table_clearance as tc

tc.LIB = tc.build_library()
J2, J3, J5, pos, app = tc.LIB
r = np.hypot(pos[:,0], pos[:,1]); z = pos[:,2]
ang = np.degrees(np.arccos(np.clip(-app[:,2], -1, 1)))

print("== 垂直下抓(倾角<=15°) 在不同高度的可用半径带 + 解密度 ==")
print(f"{'抓取高度z':>9s} {'可用半径带 r(mm)':>20s} {'带宽':>7s} {'解密度(个/M样本)':>18s}")
for zt in (30, 50, 80, 120, 180, 250, 320, 380):
    sel = (np.abs(z - zt) <= 5) & (ang <= 15)
    if sel.sum() < 50:
        print(f"{zt:9d} {'无解':>20s}")
        continue
    rs = np.sort(r[sel])
    # 找连续主带: 用 20mm 分箱, 找连续非空箱
    bins = np.arange(0, 620, 20)
    cnt, _ = np.histogram(rs, bins=bins)
    nz = cnt > 0
    # 最长连续段
    best_i = best_len = cur = 0
    for i, v in enumerate(nz):
        cur = cur + 1 if v else 0
        if cur > best_len: best_len, best_i = cur, i - cur + 1
    lo, hi = bins[best_i], bins[best_i + best_len]
    print(f"{zt:9d} {f'{lo} .. {hi}':>20s} {hi-lo:7d} {sel.sum()/len(r)*1e6:18.0f}")

print("\n== 候选布局方案打分 ==")
print("两条判据: A=工具严格竖直(可达带 180-420mm) B=允许倾角<=15度(130-459mm)")
print("比赛默认按 A 判(夹爪固定、动作可复现); B 仅作参考, 因为边界附近解很少")
pitch = 150.0
print(f"\n{'方案(2x3, 中心间距150)':>26s} {'工位r范围':>14s} {'A余量':>8s} {'B余量':>8s} {'判定':>8s}")
for R in (250, 280, 300, 320, 350, 380, 400):
    rr = [math.hypot((c-1)*pitch, R + (rw-0.5)*pitch)
          for rw in (0,1) for c in (0,1,2)]
    inner, outer = min(rr), max(rr)
    mA = min(inner - 180, 420 - outer)
    mB = min(inner - 130, 459 - outer)
    v = "推荐" if mA >= 30 else ("可用" if mA >= 0 else "超出")
    print(f"{f'中心距 {R}mm':>26s} {f'{inner:.0f}..{outer:.0f}':>14s} "
          f"{mA:8.0f} {mB:8.0f} {v:>8s}")

print("\n== 结论 ==")
print("  competition.md 的 350mm 中心 -> 最外工位 r=451mm")
print("    按严格竖直判据 A(上限420mm): 超出 31mm, 远排 3 个工位 IK 无解")
print("    这正是 MoveIt 实测结果(见 verification/check_arena_layout.py)")
print("  建议 280mm 中心 -> 工位 r=205..385mm, A 余量 25mm, 实测 6/6 成功")
