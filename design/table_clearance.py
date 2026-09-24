#!/usr/bin/env python3
"""桌面干涉校核(全局搜索版)。

思路: 先把 (j2,j3,j5) 全空间扫一遍存成"位姿库", 查询时按 (r,z,工具倾角)
筛出候选构型, 再用真实 STL 网格算每个连杆的最低点 z, 取"最安全"的那个。
基座安装面 = 台面 = z=0; 连杆最低点 < 0 即判定撞台面。
"""
import math, os, struct
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
MESH_DIR = os.path.join(HERE, "..", "jaka_ros2", "src", "jaka_description",
                        "meshes", "jaka_minicobo_meshes")
CHAIN = [
    ([0.0, 0.0, 0.187], [0.0, 0.0, 0.0]),
    ([0.0, -0.006, 0.0], [1.5708, -1.5708, 0.0]),
    ([0.21, 0.0, 0.0], [0.0, 0.0, -1.5708]),
    ([0.0, 0.2105, 0.0], [-1.5708, 0.0, 0.0]),
    ([0.0, 0.0, 0.0], [1.5708, 0.0, 0.0]),
    ([0.0, 0.1593, 0.0], [-1.5708, 0.0, 0.0]),
]
LIM = (6.28, 2.09, 2.27, 6.28, 2.09, 6.28)
LINKS = [f"Link_{i}" for i in range(7)]


def read_stl(path):
    with open(path, "rb") as f:
        f.read(80); n = struct.unpack("<I", f.read(4))[0]
        raw = np.frombuffer(f.read(n * 50), dtype=np.uint8).reshape(n, 50)
    return raw[:, 12:48].copy().view("<f4").reshape(-1, 3).astype(np.float64)


MESHES = [read_stl(os.path.join(MESH_DIR, f"Link{i}.STL")) for i in range(7)]


def rpy_to_matrix(r, p, y):
    cr, sr, cp, sp, cy, sy = (math.cos(r), math.sin(r), math.cos(p),
                              math.sin(p), math.cos(y), math.sin(y))
    return (np.array([[cy,-sy,0],[sy,cy,0],[0,0,1]]) @
            np.array([[cp,0,sp],[0,1,0],[-sp,0,cp]]) @
            np.array([[1,0,0],[0,cr,-sr],[0,sr,cr]]))


def link_transforms(q):
    Ts = [np.eye(4)]; T = np.eye(4)
    for (xyz, rpy), a in zip(CHAIN, q):
        M = np.eye(4); M[:3,:3] = rpy_to_matrix(*rpy); M[:3,3] = xyz
        c, s = math.cos(a), math.sin(a)
        Rz = np.eye(4); Rz[:3,:3] = np.array([[c,-s,0],[s,c,0],[0,0,1]])
        T = T @ M @ Rz; Ts.append(T.copy())
    return Ts


def min_z_all(q, skip_base=True):
    """活动连杆(Link_1..Link_6)几何网格的最低点 z(mm)。Link_0 固定, 跳过。"""
    worst = (1e9, None)
    for i, T in enumerate(link_transforms(q)):
        if skip_base and i == 0:
            continue
        z = (MESHES[i] @ T[:3,:3].T + T[:3,3])[:, 2] * 1000.0
        if z.min() < worst[0]:
            worst = (float(z.min()), LINKS[i])
    return worst


def build_library(n2=180, n3=180, n5=120):
    j2 = np.linspace(-LIM[1], LIM[1], n2)
    j3 = np.linspace(-LIM[2], LIM[2], n3)
    j5 = np.linspace(-LIM[4], LIM[4], n5)
    J2, J3, J5 = np.meshgrid(j2, j3, j5, indexing="ij")
    J2, J3, J5 = J2.ravel(), J3.ravel(), J5.ravel()
    N = J2.size
    T = np.tile(np.eye(4), (N,1,1))
    for i, ((xyz, rpy), _) in enumerate(zip(CHAIN, range(6)), start=1):
        M = np.tile(np.eye(4), (N,1,1)); M[:,:3,:3] = rpy_to_matrix(*rpy); M[:,:3,3] = xyz
        a = {2: J2, 3: J3, 5: J5}.get(i, np.zeros(N))
        c, s = np.cos(a), np.sin(a)
        Rz = np.tile(np.eye(4), (N,1,1))
        Rz[:,0,0], Rz[:,0,1], Rz[:,1,0], Rz[:,1,1] = c, -s, s, c
        T = T @ M @ Rz
    return J2, J3, J5, T[:,:3,3]*1000.0, T[:,:3,2]


LIB = None


def query(px, py, pz, tilt_max=15.0, rtol=6.0, ztol=6.0):
    r_s = math.hypot(px, py); j1 = math.atan2(py, px)
    J2, J3, J5, pos, app = LIB
    r = np.hypot(pos[:,0], pos[:,1]); z = pos[:,2]
    ang = np.degrees(np.arccos(np.clip(-app[:,2], -1, 1)))
    sel = (np.abs(r - r_s) < rtol) & (np.abs(z - pz) < ztol) & (ang < tilt_max)
    idx = np.nonzero(sel)[0]
    if idx.size == 0:
        return None
    # 只取 j1=0 那一支, 转成实际方位
    idx = idx[::5]
    if idx.size > 4000:
        idx = idx[np.linspace(0, idx.size-1, 4000).astype(int)]
    best, worst = (-1e9, None), (1e9, None)
    for k in idx:
        q = [j1, J2[k], J3[k], 0.0, J5[k], 0.0]
        mz, link = min_z_all(q)
        if mz > best[0]:
            best = (mz, link)
        if mz < worst[0]:
            worst = (mz, link)
    return best, worst


def main():
    global LIB
    print("建立位姿库 ...", flush=True)
    LIB = build_library()
    print(f"  样本数 {LIB[0].size}\n")
    pitch = 150.0
    print(f"{'工位阵中心R':>10s} {'工位':>8s} {'抓取z':>6s} {'径向r':>7s} "
          f"{'最优连杆':>8s} {'最低z':>9s} {'最差情形':>16s} {'判定':>8s}")
    for R in (300, 350, 400, 450):
        for row in (0, 1):
            for col in (0, 1, 2):
                px = (col - 1) * pitch
                py = R + (row - 0.5) * pitch
                for pz in (50.0,):
                    res = query(px, py, pz)
                    nm = f"{'近' if row==0 else '远'}{col+1}"
                    if res is None:
                        print(f"{R:10.0f} {nm:>8s} {pz:6.0f} {math.hypot(px,py):7.0f} "
                              f"{'-':>8s} {'-':>10s} {'不可达/无法下抓':>10s}")
                        continue
                    (bmz, blink), (wmz, wlink) = res
                    verdict = "可行" if bmz >= 10 else ("勉强" if bmz >= 0 else "撞台面!")
                    print(f"{R:10.0f} {nm:>8s} {pz:6.0f} {math.hypot(px,py):7.0f} "
                          f"{blink:>8s} {bmz:10.1f}  最差 {wlink:>7s} {wmz:8.1f}  {verdict:>7s}")


if __name__ == "__main__":
    main()
