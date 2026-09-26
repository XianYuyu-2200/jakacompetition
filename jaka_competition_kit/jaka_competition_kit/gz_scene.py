"""把 MoveIt 规划场景镜像到 Gazebo(Ignition), 让 Gazebo 里也能看到赛场。

为什么需要它
------------
Gazebo 那条线(`demo_gazebo.launch.py`)跑的是**物理 + 机械臂本体**;
而赛场(工作台 / 工位板 / 料盒 / 二维码 / 工件)只写在 MoveIt 的
**规划场景**里(`/apply_planning_scene`), Gazebo 根本不认识它。
所以直接开 Gazebo 看到的画面是"机械臂悬在一片空地上" —— 不是 Gazebo
不好看, 而是场地从来没被放进去过。

本节点把规划场景里的每个 CollisionObject 变成 Gazebo 模型:
  - 新出现的  ->  ``/world/<world>/create``(带位姿)
  - 位姿变了  ->  ``/world/<world>/set_pose``
  - 消失了的  ->  ``/world/<world>/remove``

工件被 attach 到末端时, 它在规划场景里的位姿是**相对末端坐标系**的,
这里用 TF 换算到 world, 于是 Gazebo 里能看到工件跟着机械臂走。

两个和"看着不对"直接相关的行为:

1. **工件入盒后不删, 冻结在画面里。** 判分侧把入盒的工件从规划场景里移除
   (`Scene.release_into_bin`), 照规矩删掉的话料盒在画面里永远是空的,
   看不出这一轮抓了几件。``keep_removed``(默认 true, 只对 ``wp*``)改成
   "冻结在最后位姿", 下一轮出题时再跟着场景走。
2. **两条位姿路径必须返回同一种形状。** ``_to_world`` 既处理"frame 就是
   world"(场地几何), 也处理"要过 TF"(附着工件)。曾经两者返回形状不一致,
   工件一被 attach 就抛 TypeError 把节点打死 —— 现象是 "RViz 里工件进料盒了,
   Gazebo 里的工件一直躺在工位上"。回归测试见
   ``verification/check_gz_scene_pose.py``。

数据来源是 **``/get_planning_scene`` 服务轮询**, 不是
``/monitored_planning_scene`` 话题 —— 那个是 VOLATILE 的, 晚订阅的节点
收不到已有场景, 实测镜像会一个模型都不生成。

定位(重要)
----------
这是一个**显示层**, 不参与判分。判分仍然只认 MoveIt 规划场景(RViz)。
Gazebo 里的几何默认 **collision=false**(纯视觉), 免得机械臂在 Gazebo 里
被静态几何顶住导致轨迹跟踪变差; 需要物理接触时把 ``collision:=true`` 打开。

用法:
  ros2 launch jaka_competition_kit gazebo_mirror.launch.py
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.time import Time
from moveit_msgs.msg import PlanningScene
from moveit_msgs.srv import GetPlanningScene
from shape_msgs.msg import SolidPrimitive
from ros_gz_interfaces.srv import DeleteEntity, SetEntityPose, SpawnEntity
from tf2_ros import Buffer, TransformListener

Quat = Tuple[float, float, float, float]

# 颜色表 (r, g, b, a) —— 按 id 前缀挑, 让 Gazebo 画面一眼能分清东西
PALETTE: Dict[str, Tuple[float, float, float, float]] = {
    "table":   (0.55, 0.38, 0.24, 1.0),   # 木色台面
    "station": (0.90, 0.91, 0.93, 1.0),   # 工位板: 浅灰
    "bin":     (0.18, 0.29, 0.45, 1.0),   # 料盒: 深蓝
    "qr":      (0.97, 0.97, 0.96, 1.0),   # 二维码卡: 白
    "default": (0.72, 0.72, 0.74, 1.0),
}
# 工件用一组暖色, 按序号轮换, 好区分
WORKPIECE_COLORS: List[Tuple[float, float, float, float]] = [
    (0.91, 0.36, 0.20, 1.0), (0.95, 0.65, 0.15, 1.0),
    (0.35, 0.66, 0.42, 1.0), (0.24, 0.55, 0.82, 1.0),
    (0.66, 0.42, 0.78, 1.0), (0.85, 0.30, 0.52, 1.0),
]


def _quat_mul(a: Quat, b: Quat) -> Quat:
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def _compose(parent_pos: Sequence[float], parent_quat: Quat,
             child_pos: Sequence[float], child_quat: Quat):
    """把子位姿(相对父系)换算到父系所在的坐标系。"""
    cx, cy, cz = child_pos
    px, py, pz = parent_pos
    rx, ry, rz, rw = parent_quat
    # 罗德里格斯: v' = v + rw*t + (r_v x t),  其中 t = 2*(r_v x v)
    tx = 2.0 * (ry * cz - rz * cy)
    ty = 2.0 * (rz * cx - rx * cz)
    tz = 2.0 * (rx * cy - ry * cx)
    ox = cx + rw * tx + (ry * tz - rz * ty)
    oy = cy + rw * ty + (rz * tx - rx * tz)
    oz = cz + rw * tz + (rx * ty - ry * tx)
    return (px + ox, py + oy, pz + oz), _quat_mul(parent_quat, child_quat)


def _color_for(name: str) -> Tuple[float, float, float, float]:
    head = name.split("_")[0]
    if head == "wp":
        try:
            idx = int(name.split("_")[1]) - 1
        except (IndexError, ValueError):
            idx = 0
        return WORKPIECE_COLORS[idx % len(WORKPIECE_COLORS)]
    return PALETTE.get(head, PALETTE["default"])


def _geometry(prim: SolidPrimitive):
    """SolidPrimitive -> (SDF <geometry> 片段, 半高)。不支持的类型返回 (None, 0)。"""
    d = list(prim.dimensions)
    if prim.type == SolidPrimitive.BOX and len(d) >= 3:
        return (f"<box><size>{d[0]} {d[1]} {d[2]}</size></box>", d[2] / 2.0)
    if prim.type == SolidPrimitive.CYLINDER and len(d) >= 2:
        return (f"<cylinder><radius>{d[1]}</radius><length>{d[0]}</length></cylinder>",
                d[0] / 2.0)
    if prim.type == SolidPrimitive.SPHERE and len(d) >= 1:
        return f"<sphere><radius>{d[0]}</radius></sphere>", d[0]
    return None, 0.0


@dataclass
class Item:
    """Gazebo 里应该有的一个模型。"""
    name: str
    geom: str                       # SDF <geometry> 片段
    pose: Tuple[float, float, float, Quat]
    color: Tuple[float, float, float, float]
    half_h: float = 0.0             # 竖直半高, 用来对齐地面

    @property
    def key(self) -> str:
        return self.geom


class GazeboSceneMirror(Node):
    def __init__(self):
        super().__init__("competition_gazebo_mirror")
        self.declare_parameter("world", "empty")
        self.declare_parameter("collision", False)
        self.declare_parameter("update_rate", 10.0)
        self.declare_parameter("entity_prefix", "")
        self.declare_parameter("align_ground", True)
        self.declare_parameter("ground_model", "ground_plane")
        # 工件放进料盒后, 判分侧会把它从规划场景里**移除**(见
        # Scene.release_into_bin: 已入库, 不再参与碰撞与规划)。照规矩删的话
        # 料盒在 Gazebo 画面里永远是空的, 看不出"这一轮抓了几件"。所以默认
        # 把这些实体**冻结在最后位姿**留在画面里, 下一轮出题时再跟着场景走。
        self.declare_parameter("keep_removed", True)
        self.declare_parameter("keep_removed_prefix", "wp")

        self._world = str(self.get_parameter("world").value)
        self._collide = bool(self.get_parameter("collision").value)
        self._prefix = str(self.get_parameter("entity_prefix").value)
        self._align_ground = bool(self.get_parameter("align_ground").value)
        self._ground_model = str(self.get_parameter("ground_model").value)
        self._keep_removed = bool(self.get_parameter("keep_removed").value)
        self._keep_prefix = str(self.get_parameter("keep_removed_prefix").value)
        rate = float(self.get_parameter("update_rate").value)

        cbg = ReentrantCallbackGroup()
        self._create_cli = self.create_client(
            SpawnEntity, f"/world/{self._world}/create", callback_group=cbg)
        self._remove_cli = self.create_client(
            DeleteEntity, f"/world/{self._world}/remove", callback_group=cbg)
        self._pose_cli = self.create_client(
            SetEntityPose, f"/world/{self._world}/set_pose", callback_group=cbg)
        self._scene_cli = self.create_client(
            GetPlanningScene, "/get_planning_scene", callback_group=cbg)

        self._tf = Buffer()
        self._tf_listener = TransformListener(self._tf, self)

        self._target: Dict[str, Item] = {}
        self._live: Dict[str, str] = {}          # name -> 已创建的 geom key
        self._last_pose: Dict[str, Tuple] = {}
        self._frozen: set = set()                # 已"冻结"在画面里的实体名
        self._scene_future = None
        self._ground_z: Optional[float] = None

        self.create_timer(1.0 / max(rate, 0.5), self._tick, callback_group=cbg)

        self.get_logger().info(
            f"Gazebo 场景镜像已启动: world={self._world}, "
            f"collision={'on' if self._collide else 'off(纯视觉)'}, "
            f"入盒后{'保留' if self._keep_removed else '删除'}"
            f"({self._keep_prefix}*), {rate:.0f}Hz")

    # ---------- 规划场景 -> 目标集合 ----------
    def _ingest(self, msg: PlanningScene) -> None:
        target: Dict[str, Item] = {}

        def add(obj_id: str, prim, pose_holder, frame_id: str,
                obj_pose=None) -> None:
            geom, half = _geometry(prim)
            if geom is None:
                return
            p = pose_holder
            # ⚠️ 踩过的坑: ``moveit_msgs/CollisionObject`` 里 ``pose`` 是物体
            # 自身的位姿, ``primitive_poses`` 是**相对它**的。从
            # ``/get_planning_scene`` 读回来时 MoveIt 已经把位姿折叠进
            # ``CollisionObject.pose``、并把 ``primitive_poses`` 清零 ——
            # 只看 ``primitive_poses`` 的话每个场地几何都会落在世界原点:
            # 台面/工位板/料盒/工件全部叠在基座底下, Gazebo 画面里除了机械臂
            # 什么都看不见(而且它们还会把地面一起压下去)。
            base_pos = (0.0, 0.0, 0.0)
            base_quat = (0.0, 0.0, 0.0, 1.0)
            if obj_pose is not None:
                base_pos = (obj_pose.position.x, obj_pose.position.y,
                            obj_pose.position.z)
                base_quat = (obj_pose.orientation.x, obj_pose.orientation.y,
                             obj_pose.orientation.z, obj_pose.orientation.w)
            pos, quat = _compose(base_pos, base_quat,
                                 (p.position.x, p.position.y, p.position.z),
                                 (p.orientation.x, p.orientation.y,
                                  p.orientation.z, p.orientation.w))
            pose = self._to_world(frame_id, pos, quat)
            if pose is None:
                return
            name = self._prefix + obj_id
            target[name] = Item(name, geom, pose, _color_for(obj_id), half)

        for obj in msg.world.collision_objects:
            if obj.primitives and obj.primitive_poses:
                add(obj.id, obj.primitives[0], obj.primitive_poses[0],
                    obj.header.frame_id, obj.pose)

        # 已抓取的工件: 位姿是相对末端连杆的, 要过一遍 TF
        for aco in msg.robot_state.attached_collision_objects:
            obj = aco.object
            if obj.primitives and obj.primitive_poses:
                add(obj.id, obj.primitives[0], obj.primitive_poses[0],
                    aco.link_name, obj.pose)

        self._target = target

    def _to_world(self, frame_id: str, pos, quat):
        """把 frame_id 下的位姿换算到 world; 拿不到 TF 时返回 None。

        返回值**统一**是扁平的 ``(x, y, z, (qx, qy, qz, qw))`` —— ``_spawn``
        和 ``_move`` 都按这个形状解构。

        ⚠️ 踩过的坑(2026-09): 这里以前"frame 就是 world"时返回扁平 4 元组、
        走 TF 时直接返回 ``_compose`` 的**嵌套** ``((x,y,z), quat)``。场地几何
        的 frame 是 ``world``, 一直走扁平分支所以看着没事; 工件一被 attach,
        frame 变成 ``dummy_tcp`` 就走了嵌套分支, ``_same_pose`` 里
        ``abs(float - tuple)`` 抛 TypeError **把镜像节点打死**。表现是
        "RViz 里工件抓进料盒了, Gazebo 里的工件一直躺在工位上"。
        """
        if frame_id in ("world", ""):
            return (pos[0], pos[1], pos[2], quat)
        try:
            tf = self._tf.lookup_transform("world", frame_id, Time())
        except Exception:
            return None
        t = tf.transform.translation
        q = tf.transform.rotation
        p, qq = _compose((t.x, t.y, t.z), (q.x, q.y, q.z, q.w), pos, quat)
        return (p[0], p[1], p[2], qq)

    # ---------- 轮询 + 差分同步 ----------
    def _tick(self) -> None:
        """拉一次最新场景, 再差分同步到 Gazebo。

        同一时刻只允许一个在途请求, 所以 move_group 忙的时候会自然降频,
        不会把请求堆成队列。
        """
        fut = self._scene_future
        if fut is not None:
            if not fut.done():
                return
            self._scene_future = None
            try:
                res = fut.result()
            except Exception as exc:            # 服务端异常不该把镜像搞崩
                self.get_logger().warn(f"get_planning_scene 失败: {exc}")
                res = None
            if res is not None:
                self._ingest(res.scene)
        elif self._scene_cli.service_is_ready():
            self._scene_future = self._scene_cli.call_async(
                GetPlanningScene.Request())
            return
        # 同步出错不能让镜像节点整个死掉 —— 这是个显示层, 死了以后 Gazebo 里
        # 的工件就再也不会跟着机械臂走, 而且只在"某个几何出问题"时才复现。
        try:
            self._sync()
        except Exception as exc:
            self.get_logger().warn(
                f"同步 Gazebo 场景失败(镜像继续跑): {exc}",
                throttle_duration_sec=5.0)

    def _sync(self) -> None:
        for name in [n for n in self._live if n not in self._target]:
            if self._keep_removed and name.startswith(self._keep_prefix):
                # 工件已入库 -> 冻结在最后位姿, 别从画面里删掉
                if name not in self._frozen:
                    self._frozen.add(name)
                    self.get_logger().info(
                        f"{name} 已从规划场景移除(工件入盒), 冻结在料盒里的最后位姿")
                continue
            self._delete(name)

        for name, item in self._target.items():
            self._frozen.discard(name)
            if name not in self._live:
                self._spawn(item)
            elif self._live[name] != item.key:
                # 形状变了(Gazebo 改不了几何) -> 删了重建
                self._delete(name)
            else:
                self._move(name, item.pose)

        if self._align_ground:
            self._drop_ground()

    def _drop_ground(self) -> None:
        """把 Gazebo 自带的地面降到场地最低点, 免得地面从台面中间穿过去。

        赛场坐标里 z=0 是**基座安装面**, 台面顶面在 z=-1mm, 也就是说整个
        台面在地面**以下**。不挪地面的话, Gazebo 里只能看到地面, 看不到台面。
        """
        lows = [it.pose[2] - it.half_h for it in self._target.values()
                if it.half_h > 0.0]
        if not lows:
            return
        z = min(lows)
        if self._ground_z is not None and abs(self._ground_z - z) < 1e-4:
            return
        self._ground_z = z
        self._move(self._ground_model, (0.0, 0.0, z, (0.0, 0.0, 0.0, 1.0)))

    # ---------- Gazebo 操作 ----------
    def _spawn(self, item: Item) -> None:
        if not self._create_cli.service_is_ready():
            return
        px, py, pz, quat = item.pose
        req = SpawnEntity.Request()
        req.entity_factory.name = item.name
        req.entity_factory.sdf = self._sdf(item)
        req.entity_factory.allow_renaming = False
        req.entity_factory.pose.position.x = px
        req.entity_factory.pose.position.y = py
        req.entity_factory.pose.position.z = pz
        req.entity_factory.pose.orientation.x = quat[0]
        req.entity_factory.pose.orientation.y = quat[1]
        req.entity_factory.pose.orientation.z = quat[2]
        req.entity_factory.pose.orientation.w = quat[3]

        self._live[item.name] = item.key
        self._last_pose[item.name] = item.pose
        fut = self._create_cli.call_async(req)
        fut.add_done_callback(
            lambda f, name=item.name, key=item.key: self._on_spawn(name, key, f))

    def _on_spawn(self, name: str, key: str, fut) -> None:
        """创建失败就撤回记录, 下个周期重试(否则会永远缺一个模型)。"""
        try:
            ok = bool(fut.result().success)
        except Exception as exc:
            ok = False
            self.get_logger().warn(f"{name}: 创建 Gazebo 模型异常 {exc}")
        if ok:
            return
        # 只在"记录还是这次尝试"的时候撤回, 避免把后来成功的那次抹掉
        if self._live.get(name) == key:
            self._live.pop(name, None)
            self._last_pose.pop(name, None)
            self.get_logger().warn(f"{name}: Gazebo 创建失败, 下个周期重试")

    def _delete(self, name: str) -> None:
        self._live.pop(name, None)
        self._last_pose.pop(name, None)
        if not self._remove_cli.service_is_ready():
            return
        req = DeleteEntity.Request()
        req.entity.name = name
        req.entity.type = DeleteEntity.Request().entity.MODEL
        self._remove_cli.call_async(req)

    def _move(self, name: str, pose) -> None:
        old = self._last_pose.get(name)
        if old is not None and self._same_pose(old, pose):
            return
        if not self._pose_cli.service_is_ready():
            return
        self._last_pose[name] = pose
        px, py, pz, quat = pose
        req = SetEntityPose.Request()
        req.entity.name = name
        req.entity.type = SetEntityPose.Request().entity.MODEL
        req.pose.position.x, req.pose.position.y, req.pose.position.z = px, py, pz
        req.pose.orientation.x = quat[0]
        req.pose.orientation.y = quat[1]
        req.pose.orientation.z = quat[2]
        req.pose.orientation.w = quat[3]
        self._pose_cli.call_async(req)

    @staticmethod
    def _same_pose(a, b) -> bool:
        if any(abs(x - y) > 1e-4 for x, y in zip(a[:3], b[:3])):
            return False
        return all(abs(x - y) <= 1e-4 for x, y in zip(a[3], b[3]))

    def _sdf(self, item: Item) -> str:
        r, g, b, a = item.color
        collision = (f"<collision name='collision'><geometry>{item.geom}</geometry>"
                     f"</collision>") if self._collide else ""
        return (
            f"<sdf version='1.9'><model name='{item.name}'>"
            f"<static>true</static><link name='link'>"
            f"<visual name='visual'><geometry>{item.geom}</geometry>"
            f"<material><ambient>{r} {g} {b} {a}</ambient>"
            f"<diffuse>{r} {g} {b} {a}</diffuse>"
            f"<specular>0.15 0.15 0.15 1</specular></material></visual>"
            f"{collision}</link></model></sdf>")


def main(args=None):
    rclpy.init(args=args)
    node = GazeboSceneMirror()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
