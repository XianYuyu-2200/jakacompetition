#!/usr/bin/env python3
"""gripper.STEP -> 机械臂末端能用的网格 + 实测几何参数。

**为什么要转**: Gazebo 和 RViz/MoveIt 都不认 STEP(它是 B-rep 实体, 不是三角网格),
必须先用 CAD 内核(OCC)把每个实体细分(tessellate)成 STL/DAE 才能当 <mesh> 用。

**怎么跑**(需要一个带 OCC 的 Python, 系统 Python 没有 CAD 内核):

    uv venv ~/.venvs/stepmesh --python 3.10
    uv pip install --python ~/.venvs/stepmesh/bin/python \
        trimesh cascadio numpy scipy fast-simplification
    ~/.venvs/stepmesh/bin/python tools/step_to_mesh.py

无网/不想装: 仓库里已经带了生成好的 meshes/*.stl, 直接用即可, 不必重跑。

输出
----
meshes/gripper_base.stl / gripper_jaw_l.stl / gripper_jaw_r.stl
    视觉网格, 坐标系 = **dummy_tcp**(单位 m, -z 为工具轴向下, 即夹爪指向)
config/gripper.yaml
    实测几何: 指尖深度 / 转轴 / 张合曲线 / 惯量 / 碰撞盒(URDF 与工具包共用这一份)

坐标约定(STEP 装配 -> dummy_tcp)
--------------------------------
    TCP.x =  step.x - 49.35mm          装配对称轴 -> 夹爪中心
    TCP.y = -(step.z - 49.90mm)        刀片厚度方向(镜像, 与 x/z 构成右手系)
    TCP.z =  step.y + 16.8mm           装配轴(+y=夹爪朝向) -> 工具轴(+z, 朝外/朝下)

**工具轴到底是 +z 还是 -z**: 是 **+z**。dummy_tcp 与 Link_6 同原点, 而 Link_6
的网格是 z ∈ [-39, 0] —— 臂的材料整个在 -z 一侧, 也就是说 **z=0 就是法兰面,
工具只能往 +z 装**。换上 arena 的 tool_down_quat([1,0,0,0], 按 x,y,z,w 解释 =
绕 X 转 180°), TCP 的 +z 正好指向世界下方, J5 落在 TCP 的 -z 侧 159.3mm(即
"J5 在 TCP 正上方", 可达性公式就是按这个推的)。之前这里按"工具轴向下 = -z"
写, 又在 -z 上多减了 40mm(误把 Link_6 的 39mm 当成法兰下方的实体), 结果是整只
夹爪被装进了小臂里。

所以本脚本输出的网格/参数统一用这套约定: **法兰贴合面 = z 0, 指尖 = +z**。
夹爪的 63x63 转接板就贴在 z=0 这个面上, `tip_depth` 也就是"法兰面到刀尖"的
真实长度 164.6mm(不是加上 40mm 的 204.6mm)。
"""
from __future__ import annotations

import os
import sys

import numpy as np
import trimesh
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)

# --- 装配坐标系的常量 (m, cascadio 已经把 STEP 里的 mm 换算成 m) ---
AXIS_X = 0.04935     # 两指的对称轴
AXIS_Z = 0.04990     # 刀片厚度方向的中心
MOUNT_Y = -0.01680   # 转接板背面 = 机械臂法兰贴合面
FLANGE_Z = 0.0       # 法兰贴合面在 dummy_tcp 坐标系里的位置(= Link_6 网格的 +z 端)
# 碰撞体之间的最小留白(m): 手指的转动包络和底座要靠这个数隔开, 见 base_boxes()
COL_MARGIN = 0.001

# --- 零件 -> 角色。GBK 名字见 tools/ 里的注释; 用 SolidWorks 装配的 NAUO 编号定位 ---
PARTS = {
    "NAUO1": ("base", "42步进电机-40.5"),
    "NAUO2": ("base", "FAE2二指固定机架V2电动"),
    "NAUO5": ("base", "软垫座(横梁)"),
    "NAUO6": ("base", "指根夹块"),
    "NAUO7": ("base", "指根夹块"),
    "NAUO8": ("base", "法兰转接板63x63"),
    "NAUO3": ("jaw_l", "柔性指(+x 侧, 1-FAEFR86组件)"),
    "NAUO4": ("jaw_r", "柔性指(-x 侧, 1-FAEFR86组件)"),
}
MASS = {"base": 0.300, "jaw_l": 0.035, "jaw_r": 0.035}   # 厂家标称整爪 370g
REF_DEPTH = 0.120           # 张合量参考深度: 距法兰面 120mm 处(工件被抓的位置)
SLAB = 0.020                # 碰撞盒沿深度的切片厚度


def to_tcp() -> np.ndarray:
    """STEP 装配坐标 -> dummy_tcp 的 4x4 变换。"""
    return np.array([
        [1.0, 0.0, 0.0, -AXIS_X],
        [0.0, 0.0, -1.0, AXIS_Z],
        [0.0, 1.0, 0.0, -MOUNT_Y - FLANGE_Z],
        [0.0, 0.0, 0.0, 1.0],
    ])


def load_parts(step_path: str) -> dict:
    scene = trimesh.load(step_path, force="scene")
    out = {}
    for node in scene.graph.nodes_geometry:
        transform, geom_name = scene.graph.get(node)
        if node not in PARTS:
            raise RuntimeError(f"STEP 里出现了没登记的零件 {node!r}, 请更新 PARTS")
        mesh = scene.geometry[geom_name].copy()
        mesh.apply_transform(transform)
        mesh.apply_transform(to_tcp())
        out[node] = mesh
    return out


def merge(parts: dict, role: str) -> trimesh.Trimesh:
    meshes = [m for node, m in parts.items() if PARTS[node][0] == role]
    mesh = trimesh.util.concatenate(meshes)
    mesh.merge_vertices()
    return mesh


def inner_profile(mesh: trimesh.Trimesh, dz: float = 0.005):
    """沿深度 z 的 (z, 内侧面 x, 外侧面 x) 表 —— 内侧面 = 面向夹爪中心的那一面。"""
    rows = []
    z = mesh.bounds[0][2] + dz / 2
    while z < mesh.bounds[1][2]:
        sel = np.abs(mesh.vertices[:, 2] - z) < dz
        v = mesh.vertices[sel]
        if len(v):
            rows.append((float(z), float(v[:, 0].min()), float(v[:, 0].max())))
        z += dz
    return rows


def bbox_box(mesh: trimesh.Trimesh):
    """整个网格用一个盒子包住 —— 给"远离工件"的底座用。"""
    lo, hi = mesh.bounds
    return [[round(float(v), 5) for v in
             ((lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, (lo[2] + hi[2]) / 2,
              hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2])]]


def boxes_to_mesh(boxes):
    """把 [cx,cy,cz,sx,sy,sz] 列表变成**一个**网格(几个互不相连的盒子)。

    FCL 对碰撞网格只做三角面求交, 不要求封闭, 所以盒子叠在一起也能直接用 ——
    这比在 URDF 里写一堆 <collision> 干净, 也比拿 15k 面的实体网格做碰撞快得多。
    """
    parts = []
    for cx, cy, cz, sx, sy, sz in boxes:
        box = trimesh.creation.box(extents=(sx, sy, sz))
        box.apply_translation((cx, cy, cz))
        parts.append(box)
    return trimesh.util.concatenate(parts)


def to_link_frame(mesh, pivot):
    """把网格从 dummy_tcp 坐标系搬到"以指根转轴为原点"的指 link 坐标系。"""
    m = mesh.copy()
    m.apply_translation((-pivot[0], -pivot[1], -pivot[2]))
    return m


def collision_boxes(mesh: trimesh.Trimesh, slab: float = SLAB):
    """把网格切成若干层, 每层用一个盒子(不含内部凹腔)近似。

    为什么不用原网格当碰撞体: MoveIt/FCL 每次规划要拿它做上千次碰撞检测,
    15k 面的网格会让规划时间翻好几倍; 而这里真正要紧的只有"内侧面的锥度"
    (指是往下张开的), 切成 20mm 一层就能把误差压到几毫米以内。
    每层的盒子取该层内的**最小**内侧 x(保守), 深度按层切开。
    """
    boxes = []
    z_lo, z_hi = mesh.bounds[0][2], mesh.bounds[1][2]
    n = max(1, int(round((z_hi - z_lo) / slab)))
    edges = np.linspace(z_lo, z_hi, n + 1)
    for i in range(n):
        lo, hi = edges[i], edges[i + 1]
        sel = (mesh.vertices[:, 2] >= lo) & (mesh.vertices[:, 2] <= hi)
        v = mesh.vertices[sel]
        if not len(v):
            continue
        cx = (v[:, 0].min() + v[:, 0].max()) / 2
        cy = (v[:, 1].min() + v[:, 1].max()) / 2
        boxes.append([
            round(float(cx), 5), round(float(cy), 5), round(float((lo + hi) / 2), 5),
            round(float(v[:, 0].max() - v[:, 0].min()), 5),
            round(float(v[:, 1].max() - v[:, 1].min()), 5),
            round(float(hi - lo), 5),
        ])
    return boxes


def _box(lo, hi):
    """(x_lo,y_lo,z_lo),(x_hi,y_hi,z_hi) -> 生成的盒子格式 [cx,cy,cz,sx,sy,sz]。"""
    return [round(float((a + b) / 2), 5) for a, b in zip(lo, hi)] + \
           [round(float(b - a), 5) for a, b in zip(lo, hi)]


def jaw_sweep(jaw_boxes, pivot, limit_rad):
    """手指碰撞盒在整个张合行程上的扫掠, 逐 θ 返回每个盒子的 (x_min, z_min, z_max)。

    指根是插在底座机架里的(STEP 里本来就互相穿模), 所以底座必须把这一块让出来,
    否则 Gazebo 会把"指根和机架互相嵌入"当成硬接触, 手指直接被顶死 —— 实测现象是
    给 /gripper_controller/commands 发 0.45rad, 两个关节纹丝不动。
    """
    px, pz = pivot[0], pivot[2]
    out = []
    for t in np.linspace(0.0, limit_rad, 37):
        c, s = np.cos(t), np.sin(t)
        step = []
        for cx, cy, cz, sx, sy, sz in jaw_boxes:
            xs, zs = [], []
            for dx in (-0.5, 0.5):
                for dz in (-0.5, 0.5):
                    u, w = cx + dx * sx - px, cz + dz * sz - pz
                    xs.append(px + u * c - w * s)   # 绕 -y 转 = 指尖朝中心合
                    zs.append(pz + u * s + w * c)
            step.append((min(xs), min(zs), max(zs)))
        out.append(step)
    return out


def base_boxes(mesh: trimesh.Trimesh, jaw_boxes, pivot, limit_rad,
               margin: float = COL_MARGIN):
    """底座的碰撞体: 整块包围盒会让指根嵌进去, 所以拆成两块。

    ① 扫掠区下沿以下(转接板 + 电机 + 机架下半): 还是完整包围盒;
    ② 扫掠区高度上: 只保留两指之间的中央机架(|x| 收到手指扫掠的内侧面以内)。

    ② 丢掉的那部分(两侧机架耳)其实被手指自己的碰撞体盖住了 —— 手指永远在那
    附近, 工件不可能从那里钻到底座上; 换来的是手指能自由转动。
    """
    lo, hi = mesh.bounds
    sweep = jaw_sweep(jaw_boxes, pivot, limit_rad)
    z_clear = min(b[1] for step in sweep for b in step) - margin
    # 中央机架的宽度: 只看"某个 θ 下确实伸进底座高度范围"的那些手指盒子
    x_clear = min(b[0] for step in sweep for b in step
                  if b[1] <= hi[2] and b[2] >= z_clear) - margin
    band = mesh.vertices[mesh.vertices[:, 2] >= z_clear]
    boxes = [_box((lo[0], lo[1], lo[2]), (hi[0], hi[1], z_clear))]
    if len(band) and hi[2] > z_clear:
        boxes.append(_box((-x_clear, band[:, 1].min(), z_clear),
                          (x_clear, band[:, 1].max(), hi[2])))
    return boxes


def inertia_of(mesh: trimesh.Trimesh, mass: float, name: str) -> dict:
    """按凸包估惯量(原网格不封闭, 直接算体积会得到 nan)。

    夹爪是"实心外壳 + 少量凹槽"的结构, 凸包体积比实体略大, 惯量因此略保守。
    """
    hull = mesh.convex_hull
    hull.density = mass / hull.volume
    com = hull.center_mass
    I = hull.moment_inertia
    diag = {"ixx": round(float(I[0][0]), 9), "ixy": round(float(I[0][1]), 9),
            "ixz": round(float(I[0][2]), 9), "iyy": round(float(I[1][1]), 9),
            "iyz": round(float(I[1][2]), 9), "izz": round(float(I[2][2]), 9)}
    return {"name": name, "mass": mass,
            "origin": [round(float(v), 5) for v in com],
            "hull_volume": round(float(hull.volume), 8), **diag}


def main() -> int:
    step = sys.argv[1] if len(sys.argv) > 1 else os.path.join(PKG, "step", "gripper.STEP")
    parts = load_parts(step)
    print(f"读入 {len(parts)} 个零件 <- {step}")

    out = {}
    for role in ("base", "jaw_l", "jaw_r"):
        out[role] = merge(parts, role)

    # 指根转轴 = 离法兰面 115mm 以内那一段(指根)顶点云的中心; 取 +x 侧, 另一侧镜像
    v = out["jaw_l"].vertices
    root = v[v[:, 2] < FLANGE_Z + 0.115]
    pivot = [round(float(abs(root.mean(axis=0)[0])), 5), 0.0,
             round(float(root.mean(axis=0)[2]), 5)]

    # 张合曲线: 参考深度处的间隙 = 2 * 内侧面 x; 转动 1° 间隙变化 = 2*|z_pivot - z_ref| * pi/180
    rows = inner_profile(out["jaw_l"])
    near = min(rows, key=lambda r: abs(r[0] - REF_DEPTH))
    gap_open = 2 * near[1]
    gap_per_rad = 2 * (near[0] - pivot[2])
    gap_per_deg = abs(gap_per_rad) * np.pi / 180

    # 最大闭合角: +x 侧的指绕 **-y** 转 theta 时 x' = dx*cos - dz*sin(指尖朝中心合),
    # 合到指尖越过中心线(x=0)为止
    tip = out["jaw_l"].vertices[out["jaw_l"].vertices[:, 2].argmax()]
    dx, dz = tip[0] - pivot[0], tip[2] - pivot[2]
    lo, hi = 0.0, 89.0
    for _ in range(60):
        mid = (lo + hi) / 2
        t = np.radians(mid)
        x = pivot[0] + dx * np.cos(t) - dz * np.sin(t)
        if x > 0.002:
            lo = mid
        else:
            hi = mid
    limit = lo

    # --- 碰撞体: 底座避开手指的转动包络(见 base_boxes); 手指逐层切片,
    #     保住"往下张开"的锥度 ---
    jaw_col = collision_boxes(out["jaw_l"])
    col = {
        "base": base_boxes(out["base"], jaw_col, pivot, np.radians(limit)),
        "jaw": jaw_col,
    }

    # --- 导出(视觉网格用完整面数, 碰撞网格用盒子) ---
    meshes = {
        "base": out["base"],                                    # dummy_tcp 坐标系
        "base_col": boxes_to_mesh(col["base"]),
        "jaw_l": to_link_frame(out["jaw_l"], pivot),            # 指 link 坐标系
        "jaw_l_col": to_link_frame(boxes_to_mesh(col["jaw"]), pivot),
    }
    # -x 侧的指在 STEP 里本来就是分开的实体, 直接搬坐标系即可;
    # 碰撞盒是从 +x 侧量出来的, 需要镜像到 -x。
    meshes["jaw_r"] = to_link_frame(out["jaw_r"], [-pivot[0], pivot[1], pivot[2]])
    boxes_r = [[-b[0], b[1], b[2], b[3], b[4], b[5]] for b in col["jaw"]]
    meshes["jaw_r_col"] = to_link_frame(boxes_to_mesh(boxes_r), [-pivot[0], pivot[1], pivot[2]])

    for name, mesh in meshes.items():
        path = os.path.join(PKG, "meshes", f"gripper_{name}.stl")
        mesh.export(path)
        print(f"  {name:9s} {len(mesh.faces):6d} 面  -> meshes/gripper_{name}.stl")

    cfg = {
        "frame": "dummy_tcp",
        "tip_depth": round(float(out["jaw_l"].bounds[1][2]), 5),
        "flange_z": round(float(FLANGE_Z), 5),
        "palm_depth": round(float(out["base"].bounds[1][2]), 5),
        "pivot": [round(float(v), 5) for v in pivot],
        # 两片指的 URDF 轴都用 +y(负向轴会被 Gazebo 冻住), 镜像靠命令符号
        "jaw_axis": [0.0, 1.0, 0.0],
        "jaw_limit_deg": round(float(limit), 2),
        "gap_ref_depth": round(float(near[0]), 5),
        "gap_at_open": round(float(gap_open), 5),
        "gap_per_deg": round(float(abs(gap_per_deg)), 6),
        "grip_range": [round(float(abs(gap_open) - limit * abs(gap_per_deg)), 3),
                       round(float(gap_open), 3)],
        "inertia": [inertia_of(out["base"], MASS["base"], "base"),
                    inertia_of(out["jaw_l"], MASS["jaw_l"], "jaw")],
    }
    path = os.path.join(PKG, "config", "gripper.yaml")
    with open(path, "w", encoding="utf-8") as f:
        f.write("# 夹爪几何参数 —— 由 tools/step_to_mesh.py 从 step/gripper.STEP 实测生成\n")
        f.write("# 改 STEP 就重跑那个脚本; 手改这里会在下次生成时被覆盖。\n")
        yaml.safe_dump({k: v for k, v in cfg.items() if k != "# ..."}, f,
                       allow_unicode=True, sort_keys=False)

    print(f"\n指尖深度(TCP->刀尖)  {cfg['tip_depth']*1000:.1f} mm"
          f"   底座最低点 {cfg['palm_depth']*1000:.1f} mm")
    print(f"指根转轴            {[round(v*1000,1) for v in cfg['pivot']]} mm(取 +x 侧, 另一侧镜像)")
    print(f"参考深度 {cfg['gap_ref_depth']*1000:.1f} mm 处: 张开 {cfg['gap_at_open']*1000:.1f} mm,"
          f" 每合 1° = {cfg['gap_per_deg']*1000:.2f} mm, 最大闭合 {cfg['jaw_limit_deg']:.1f}°"
          f" -> 可夹 {cfg['grip_range'][0]*1000:.0f}~{cfg['grip_range'][1]*1000:.0f} mm")
    print(f"写入 {os.path.relpath(path, PKG)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
