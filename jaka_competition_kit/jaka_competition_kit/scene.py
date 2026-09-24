"""规划场景操作: 摆放工作台/工位/料盒/工件, 以及工件的抓取/释放附着。

所有函数都通过 ``/apply_planning_scene`` 差分下发, 仿真与真机一致。
"""
from __future__ import annotations

import threading
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import rclpy
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import Pose, PoseStamped, Quaternion
from moveit_msgs.msg import (AttachedCollisionObject, CollisionObject,
                             PlanningScene)
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
from shape_msgs.msg import SolidPrimitive


def box_object(obj_id: str, frame_id: str, center: Sequence[float],
               size: Sequence[float], operation: int = CollisionObject.ADD,
               z_bottom: Optional[float] = None) -> CollisionObject:
    """构造一个立方体碰撞体。z_bottom 给定时, center 的 z 由底面推算。"""
    prim = SolidPrimitive()
    prim.type = SolidPrimitive.BOX
    prim.dimensions = [float(v) for v in size]

    pose = Pose()
    pose.position.x = float(center[0])
    pose.position.y = float(center[1])
    pose.position.z = (float(z_bottom) + size[2] / 2.0) if z_bottom is not None \
        else float(center[2] if len(center) > 2 else 0.0)
    pose.orientation.w = 1.0

    obj = CollisionObject()
    obj.header.frame_id = frame_id
    obj.id = obj_id
    obj.primitives.append(prim)
    obj.primitive_poses.append(pose)
    obj.operation = operation
    return obj


def cylinder_object(obj_id: str, frame_id: str, center: Sequence[float],
                    radius: float, height: float,
                    operation: int = CollisionObject.ADD) -> CollisionObject:
    prim = SolidPrimitive()
    prim.type = SolidPrimitive.CYLINDER
    prim.dimensions = [float(height), float(radius)]
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = (float(v) for v in center)
    pose.orientation.w = 1.0
    obj = CollisionObject()
    obj.header.frame_id = frame_id
    obj.id = obj_id
    obj.primitives.append(prim)
    obj.primitive_poses.append(pose)
    obj.operation = operation
    return obj


def bin_container(prefix: str, frame_id: str, box: Box, height: float,
                  wall: float = 0.010, floor: float = 0.008) -> List[CollisionObject]:
    """把料盒建模成一个"容器"(底板 + 4 面侧墙), 这样工件可以真的落在里面。

    如果建成实心方块, 盒内是实心的, 工件无法放进去。
    """
    x0, x1 = box.x_range
    y0, y1 = box.y_range
    objs = [box_object(f"{prefix}_floor", frame_id,
                       ((x0 + x1) / 2, (y0 + y1) / 2, 0.0),
                       (box.size[0], box.size[1], floor), z_bottom=0.0)]
    objs.append(box_object(f"{prefix}_wall_x0", frame_id,
                           (x0 + wall / 2, (y0 + y1) / 2, 0.0),
                           (wall, box.size[1], height), z_bottom=0.0))
    objs.append(box_object(f"{prefix}_wall_x1", frame_id,
                           (x1 - wall / 2, (y0 + y1) / 2, 0.0),
                           (wall, box.size[1], height), z_bottom=0.0))
    objs.append(box_object(f"{prefix}_wall_y0", frame_id,
                           ((x0 + x1) / 2, y0 + wall / 2, 0.0),
                           (box.size[0], wall, height), z_bottom=0.0))
    objs.append(box_object(f"{prefix}_wall_y1", frame_id,
                           ((x0 + x1) / 2, y1 - wall / 2, 0.0),
                           (box.size[0], wall, height), z_bottom=0.0))
    return objs


class SceneClient:
    """``/apply_planning_scene`` 的薄封装, 同步(阻塞)调用。

    ⚠️ 为什么自带一个 node + 后台线程, 而不是复用调用方的 node:

    ``apply()`` 是阻塞的, 而它经常**在服务回调里**被调用(场景生成器的
    ``/competition/generate`` 就是)。如果在回调里对同一个 node 做
    ``rclpy.spin_until_future_complete``, 就会出现"回调等回调"的自我阻塞 ——
    实测表现为 ``/competition/generate`` 挂住几十秒甚至不返回。

    所以这里用一个**独立的内部 node**, 由后台单线程执行器负责收发,
    调用侧只用 ``threading.Event`` 等结果, 完全不碰调用方的执行器。
    """

    def __init__(self, node=None, callback_group=None):
        self._owns_node = node is None
        if node is None:
            self._node = rclpy.create_node("competition_scene_client")
            self._executor = SingleThreadedExecutor()
            self._executor.add_node(self._node)
            self._thread = threading.Thread(target=self._executor.spin, daemon=True)
            self._thread.start()
        else:                                   # 兼容旧用法: 复用传入的 node(不推荐)
            self._node = node
        self._closed = False

        self._apply = self._node.create_client(
            ApplyPlanningScene, "/apply_planning_scene", callback_group=callback_group)
        self._get = self._node.create_client(
            GetPlanningScene, "/get_planning_scene", callback_group=callback_group)
        if not self._apply.wait_for_service(timeout_sec=30.0):
            raise RuntimeError("/apply_planning_scene 不可用 —— move_group 起了吗?")

    def close(self) -> None:
        """停掉后台执行器并销毁内部 node。

        顺序很重要: **先 shutdown 执行器 -> 再 join 线程 -> 最后销毁 node**。
        先销毁 node 会让还在 spin 的线程踩到已释放的对象,
        进程退出时表现为 `terminate called without an active exception`。
        """
        if not self._owns_node or self._closed:
            return
        self._closed = True
        try:
            self._executor.shutdown(timeout_sec=1.0)
        except Exception:                                # noqa: BLE001
            pass
        self._thread.join(timeout=2.0)
        try:
            self._node.destroy_node()
        except Exception:                                # noqa: BLE001
            pass

    # ---------- 阻塞调用 ----------
    def _wait(self, client, req, timeout: float = 15.0):
        fut = client.call_async(req)
        done = threading.Event()
        fut.add_done_callback(lambda _f: done.set())
        if not done.wait(timeout):
            return None
        try:
            return fut.result()
        except Exception:                       # noqa: BLE001
            return None

    def apply(self, objects: Iterable[CollisionObject],
              attached: Iterable[AttachedCollisionObject] = ()) -> bool:
        scene = PlanningScene()
        scene.is_diff = True
        # ⚠️ 关键(踩过的坑): 附着物体的 REMOVE 只有在 `robot_state.is_diff`
        # 为 True 时才被 MoveIt 处理。moveit_core/planning_scene.cpp 的
        # processRobotStateMsg() 里写着:
        #     "The specified RobotState is not marked as is_diff.
        #      The request to modify the object ... is not supported."
        # 服务会照样返回 success=True, 但摘除被静默忽略 —— 于是幽灵工件
        # 一直挂在末端, 之后每次规划都失败。ADD 不受影响, 所以只有"摘"会踩坑。
        scene.robot_state.is_diff = True
        scene.world.collision_objects.extend(objects)
        scene.robot_state.attached_collision_objects.extend(attached)
        req = ApplyPlanningScene.Request()
        req.scene = scene
        res = self._wait(self._apply, req)
        return bool(res and res.success)

    def snapshot_ids(self, timeout: float = 15.0):
        """读回规划场景, 返回 ``(世界物体 id 集合, 已附着物体 id 集合)``。

        为什么需要它: 对**不存在**的物体下发 REMOVE 时, MoveIt 会为每个 id
        刷一条日志 —— 世界里没有就是
        ``Tried to remove world object 'x', but it does not exist``(WARN),
        没挂在末端就是 ``Attached body 'x' not found``(**ERROR**)。
        出题时每次都要清场(20 个候选工件 + 两个料盒的部件),
        不做过滤就是几十行 ERROR, 现场根本看不出真正的问题。

        读不到(超时/服务没起)时返回 ``None``, 调用方退回"硬删"。
        """
        components = GetPlanningScene.Request().components
        req = GetPlanningScene.Request()
        req.components.components = (
            components.WORLD_OBJECT_NAMES | components.ROBOT_STATE_ATTACHED_OBJECTS)
        res = self._wait(self._get, req, timeout)
        if res is None:
            return None
        world = {o.id for o in res.scene.world.collision_objects}
        attached = {a.object.id for a in res.scene.robot_state.attached_collision_objects}
        return world, attached

    def remove_existing(self, ids: Iterable[str]) -> bool:
        """只删**当前场景里确实存在**的那些物体。"""
        snap = self.snapshot_ids()
        if snap is None:
            return self.remove(list(ids))
        world, _attached = snap
        todo = [i for i in ids if i in world]
        return self.remove(todo) if todo else True

    def detach_existing(self, ids: Iterable[str],
                        links: Sequence[str] = ("dummy_tcp", "Link_6",
                                                "Link_5", "Link_4")) -> bool:
        """只摘**确实还挂在末端上**的那些工件。"""
        snap = self.snapshot_ids()
        if snap is None:
            return self.detach_any(list(ids), links)
        _world, attached = snap
        todo = [i for i in ids if i in attached]
        return self.detach_any(todo, links) if todo else True

    def remove(self, ids: Iterable[str]) -> bool:
        objs = []
        for i in ids:
            o = CollisionObject()
            o.id = i
            o.operation = CollisionObject.REMOVE
            objs.append(o)
        return self.apply(objs)

    def detach_any(self, obj_ids: Iterable[str],
                   links: Sequence[str] = ("dummy_tcp", "Link_6", "Link_5", "Link_4")) -> bool:
        """不问是谁挂的, 挨个 link 试一遍把它摘下来(收尾兜底用)。"""
        ok = True
        for link in links:
            ok = self.detach_all(link, obj_ids) and ok
        return ok

    def detach_all(self, link: str, obj_ids: Iterable[str]) -> bool:
        """把可能还挂在末端上的工件全部摘掉。

        这是**必须**的:工件一旦 attach, 它在世界坐标里就"消失"了,
        再对世界下发 REMOVE 是无效的, 场景里会一直留着一个跟着机械臂跑的
        幽灵工件, 导致之后所有规划都失败。
        """
        acos = []
        for i in obj_ids:
            aco = AttachedCollisionObject()
            aco.link_name = link
            aco.object.id = i
            aco.object.operation = CollisionObject.REMOVE
            acos.append(aco)
        return self.apply([], acos)

    def clear_all(self) -> bool:
        self.apply([], [])
        return self.remove(["table", "bin_t1", "bin_t2", "qr_card"]
                           + [f"station_{i}" for i in range(1, 7)]
                           + [f"wp_{i}" for i in range(12)])

    def attach(self, obj_id: str, link: str,
               touch_links: Sequence[str]) -> bool:
        """按 id 附着(不指定相对位姿)。

        ⚠️ 实测:MoveIt 用这种方式附着时, 会把物体的位姿当成**相对末端坐标系**,
        即物体中心被放到 TCP 原点 —— 抓一个 50mm 的方块就会和法兰重叠,
        之后所有规划都报 INVALID_MOTION_PLAN。请用 ``attach_at``。
        """
        aco = AttachedCollisionObject()
        aco.link_name = link
        aco.object.id = obj_id
        aco.object.operation = CollisionObject.ADD
        aco.touch_links = list(touch_links)
        return self.apply([], [aco])

    def attach_at(self, obj_id: str, link: str, shape: CollisionObject,
                  rel_pose: Pose, touch_links: Sequence[str]) -> bool:
        """附着并把物体放到**相对末端坐标系**的指定位置。

        必须显式给相对位姿, 否则 MoveIt 会把物体放到 TCP 原点上。
        """
        aco = AttachedCollisionObject()
        aco.link_name = link
        aco.object.header.frame_id = link
        aco.object.id = obj_id
        aco.object.primitives = list(shape.primitives)
        aco.object.primitive_poses = [rel_pose]
        aco.object.operation = CollisionObject.ADD
        aco.touch_links = list(touch_links)
        return self.apply([], [aco])

    def put_back(self, obj_id: str, frame_id: str, xy_z_bottom: Sequence[float],
                 size: float) -> bool:
        """把工件从末端摘下来、原样放回台面上(失败重试前的还原)。"""
        self.detach_any([obj_id])
        obj = box_object(obj_id, frame_id, (xy_z_bottom[0], xy_z_bottom[1], 0.0),
                         (size, size, size), z_bottom=float(xy_z_bottom[2]))
        return self.apply([obj])

    def release_into_bin(self, obj_id: str, link: str) -> bool:
        """工件入盒: 从末端摘下并从场景中移除(已"入库", 不再参与碰撞与规划)。

        判分由 scorer 依据释放点几何独立完成, 不依赖本操作。
        """
        aco = AttachedCollisionObject()
        aco.link_name = link
        aco.object.id = obj_id
        aco.object.operation = CollisionObject.REMOVE
        ok = self.apply([], [aco])
        self.remove([obj_id])
        return ok
