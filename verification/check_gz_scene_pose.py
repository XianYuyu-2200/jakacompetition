#!/usr/bin/env python3
"""不启动 Gazebo, 只检查 gz_scene 镜像的位姿形状契约(回归测试)。

为什么需要它: `GazeboSceneMirror._to_world()` 有两条返回路径 —— 帧就是
`world`(场地几何走这条)和要过 TF(工件被 attach 后帧是 `dummy_tcp` 走这条)。
必须返回**同一种**形状 `(x, y, z, (qx, qy, qz, qw))`, 否则 `_same_pose`
会在 `abs(float - tuple)` 上抛 TypeError, 把镜像节点整个打死 —— 表现是
"RViz 里工件抓进料盒了, Gazebo 里的工件一直躺在工位上"。

用法:
  source /opt/ros/humble/setup.bash
  source jaka_ros2/install/setup.bash
  python3 verification/check_gz_scene_pose.py
"""
import sys

import rclpy
from rclpy.time import Time

from jaka_competition_kit.gz_scene import GazeboSceneMirror, _compose

POS = (0.12, -0.34, 0.56)
QUAT = (0.0, 0.0, 0.0, 1.0)          # 单位四元数
FLAT_LEN = 4


class _StubTF:
    """假 TF: 返回一个已知的 world -> <frame> 变换, 不依赖任何 ROS 图。"""

    def __init__(self, translation, rotation=(0.0, 0.0, 0.0, 1.0)):
        self._t = translation
        self._q = rotation

    def lookup_transform(self, target, source, _time):
        class _T:
            pass

        class _V:
            x, y, z = self._t
            w = 1.0

        class _Q:
            x, y, z, w = self._q

        tf = _T()
        tf.transform = _T()
        tf.transform.translation = _V()
        tf.transform.rotation = _Q()
        return tf


def _fake_mirror(translation):
    m = GazeboSceneMirror.__new__(GazeboSceneMirror)   # 不走 Node.__init__
    m._tf = _StubTF(translation)
    return m


def main():
    rclpy.init()
    ok = True

    world_pose = _fake_mirror((0.0, 0.0, 0.0))._to_world("world", POS, QUAT)
    tcp_pose = _fake_mirror((0.1, 0.0, 0.2))._to_world("dummy_tcp", POS, QUAT)

    for label, pose in (("frame=world", world_pose), ("frame=dummy_tcp", tcp_pose)):
        if pose is None:
            print(f"[X] {label}: _to_world 返回 None")
            ok = False
            continue
        if len(pose) != FLAT_LEN or not isinstance(pose[3], tuple) or len(pose[3]) != 4:
            print(f"[X] {label}: 形状不对, 期望 (x, y, z, (qx,qy,qz,qw)), 实际 {pose!r}")
            ok = False
        else:
            print(f"[OK] {label}: 形状正确 {tuple(round(v, 4) for v in pose[:3])}")

    # 关键回归点: 两种帧的位姿放一起比较, 以前这里抛 TypeError
    if world_pose is not None and tcp_pose is not None:
        try:
            same = GazeboSceneMirror._same_pose(world_pose, tcp_pose)
            print(f"[OK] _same_pose(世界帧, 末端帧) 不再抛异常, 结果 same={same}")
        except TypeError as exc:
            print(f"[X] _same_pose 仍然抛 TypeError: {exc}")
            ok = False

    # 数值: 平移 (0.1, 0, 0.2) + 原始位姿 = 预期值
    if tcp_pose is not None:
        want = tuple(POS[i] + (0.1, 0.0, 0.2)[i] for i in range(3))
        got = tcp_pose[:3]
        if max(abs(a - b) for a, b in zip(want, got)) > 1e-9:
            print(f"[X] 平移合成不对: 期望 {want}, 得到 {got}")
            ok = False
        else:
            print("[OK] 平移合成正确")

    # _compose 本身保持"两元组"契约(add() 里就是这么解构的)
    p, q = _compose((0.0, 0.0, 0.0), QUAT, POS, QUAT)
    print(f"[OK] _compose 仍返回 (pos, quat): pos={tuple(round(v, 4) for v in p)}, len(quat)={len(q)}")

    rclpy.shutdown()
    print("\n结论:", "全部通过" if ok else "有失败项")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
