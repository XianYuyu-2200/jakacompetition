#!/usr/bin/env python3
"""赛场布局可达性校核(向量化版)。

先用 verification/README.md 里已交叉验证过的 4 个位姿自检 FK, 再扫工作空间。
输出: (r,z) 剖面、可垂直下抓的可行域、competition.md 中布局的可行台面高度。
"""
import math, os
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))

# joint_1..joint_6 的固定变换 (xyz, rpy), 从 jaka_minicobo.urdf 摘录
CHAIN = [
    ([0.0, 0.0, 0.187], [0.0, 0.0, 0.0]),
    ([0.0, -0.006, 0.0], [1.5708, -1.5708, 0.0]),
    ([0.21, 0.0, 0.0], [0.0, 0.0, -1.5708]),
    ([0.0, 0.2105, 0.0], [-1.5708, 0.0, 0.0]),
    ([0.0, 0.0, 0.0], [1.5708, 0.0, 0.0]),
    ([0.0, 0.1593, 0.0], [-1.5708, 0.0, 0.0]),
]
LIM = (6.28, 2.09, 2.27, 6.28, 2.09, 6.28)


def rpy_to_matrix(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return (np.array([[cy,-sy,0],[sy,cy,0],[0,0,1]]) @
            np.array([[cp,0,sp],[0,1,0],[-sp,0,cp]]) @
            np.array([[1,0,0],[0,cr,-sr],[0,sr,cr]]))


def fk(q):
    T = np.eye(4)
    for (xyz, rpy), a in zip(CHAIN, q):
        M = np.eye(4); M[:3,:3] = rpy_to_matrix(*rpy); M[:3,3] = xyz
        c, s = math.cos(a), math.sin(a)
        Rz = np.eye(4); Rz[:3,:3] = np.array([[c,-s,0],[s,c,0],[0,0,1]])
        T = T @ M @ Rz
    return T


def selfcheck():
    known = {
        (0,0,0,0,0,0): [0.00, -6.00, 766.80],
        (0, 60, -30, 0, 0, 0): [-366.77, -6.00, 612.26],
        (0, -45, 80, 0, 45, 0): [-129.13, -6.00, 535.59],
        (30, 30, 20, 90, -40, 120): [-359.74, -96.39, 582.61],
    }
    print("== FK 自检(对比 verification/README.md 的实测值) ==")
    worst = 0.0
    for qd, exp in known.items():
        got = fk([math.radians(v) for v in qd])[:3,3] * 1000.0
        err = float(np.linalg.norm(got - np.array(exp)))
        worst = max(worst, err)
        print(f"  q={list(qd)!s:34s} got=[{got[0]:8.2f},{got[1]:8.2f},{got[2]:8.2f}]  err={err:.3f} mm")
    print(f"  -> 最大误差 {worst:.3f} mm  {'OK' if worst < 0.05 else 'FAIL'}\n")
    return worst < 0.05


def sweep(n2=140, n3=140, n5=90):
    j2 = np.linspace(-LIM[1], LIM[1], n2)
    j3 = np.linspace(-LIM[2], LIM[2], n3)
    j5 = np.linspace(-LIM[4], LIM[4], n5)
    J2, J3, J5 = np.meshgrid(j2, j3, j5, indexing="ij")
    J2, J3, J5 = J2.ravel(), J3.ravel(), J5.ravel()
    N = J2.size
    T = np.tile(np.eye(4), (N,1,1))
    for i, ((xyz, rpy), ang) in enumerate(zip(CHAIN, [None]*6), start=1):
        M = np.tile(np.eye(4), (N,1,1))
        M[:,:3,:3] = rpy_to_matrix(*rpy)
        M[:,:3,3] = xyz
        if i == 1: a = np.zeros(N)
        elif i == 2: a = J2
        elif i == 3: a = J3
        elif i == 5: a = J5
        else: a = np.zeros(N)
        Rz = np.tile(np.eye(4), (N,1,1))
        c, s = np.cos(a), np.sin(a)
        Rz[:,0,0], Rz[:,0,1] = c, -s
        Rz[:,1,0], Rz[:,1,1] = s, c
        T = T @ M @ Rz
    pos = T[:,:3,3] * 1000.0          # mm
    app = T[:,:3,2]                   # 工具 z 轴单位向量
    return pos, app


def main():
    ok = selfcheck()
    pos, app = sweep()
    r = np.hypot(pos[:,0], pos[:,1]); z = pos[:,2]
    # 工具轴与"竖直向下"(0,0,-1) 的夹角
    cosang = np.clip(-app[:,2], -1, 1)
    ang = np.degrees(np.arccos(cosang))
    print(f"== 工作空间采样 (j1=0 平面, 绕基座轴对称), {len(pos)} 点 ==")
    print(f"  z  范围 : {z.min():7.0f} .. {z.max():7.0f} mm (相对基座安装面)")
    print(f"  r  范围 : {r.min():7.0f} .. {r.max():7.0f} mm")

    down = ang <= 15.0
    print(f"\n== 可垂直下抓(工具轴与竖直向下夹角 <=15°) ==")
    print(f"  占全部采样 {down.mean()*100:.1f}%")
    print(f"  z : {z[down].min():7.0f} .. {z[down].max():7.0f} mm")
    print(f"  r : {r[down].min():7.0f} .. {r[down].max():7.0f} mm")

    print(f"\n== 不同台面高度的可达性(工位放在 z=台面) ==")
    print(f"  {'台面 z(mm)':>10s} {'可达 r(mm)':>18s} {'可垂直下抓 r(mm)':>20s}")
    for zt in (0, 50, 100, 150, 187, 200, 250, 300):
        sel = np.abs(z - zt) <= 12
        dsel = sel & down
        if sel.sum() < 5:
            print(f"  {zt:10d} {'完全不可达':>18s}")
            continue
        a = f"{r[sel].min():.0f}..{r[sel].max():.0f}"
        b = f"{r[dsel].min():.0f}..{r[dsel].max():.0f}" if dsel.sum() > 5 else "不可行"
        print(f"  {zt:10d} {a:>18s} {b:>20s}")

    print(f"\n== 垂直下抓可行域边界(每 40mm 高度带) ==")
    for zc in range(120, 620, 40):
        sel = down & (np.abs(z - zc) <= 20)
        if sel.sum() > 20:
            print(f"  z={zc:4d}mm : r = {r[sel].min():4.0f} .. {r[sel].max():4.0f} mm")
        else:
            print(f"  z={zc:4d}mm : 无可行解")

    # 赛题布局: 2x3, 工位 100x100, 间距 50 → 中心间距 150
    print("\n== competition.md 工位阵列(2 行 x 3 列, 间距 150mm) ==")
    print("   工位中心坐标(px 为横向, py 为离基座径向), 单位 mm")
    pitch = 150.0
    for R in (300, 350, 400, 450, 500):
        coords = [( (c-1)*pitch, R + (rw-0.5)*pitch )
                  for rw in (0,1) for c in (0,1,2)]
        rr = [math.hypot(a,b) for a,b in coords]
        print(f"   阵列中心 R={R:3.0f}: 工位半径 {min(rr):.0f}..{max(rr):.0f} mm | "
              f"最外边缘 {max(rr)+70.7:.0f} mm")
    print("\n   注: 每个工位 100x100, 角点比中心再远 70.7mm")


if __name__ == "__main__":
    main()
