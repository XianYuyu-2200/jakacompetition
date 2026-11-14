"""夹爪抽象 —— 末端执行器的统一接口。

    SimGripper      —— 仿真: 把手指角度发给 ``/gripper_controller/commands``
                       (ros2_control 的 JointGroupPositionController, 见
                       jaka_ros2/src/wheeltec_gripper)。
    WheeltecGripper —— 真机: WHEELTEC MS42DC 柔性爪, 走 ``step_motor`` 包的
                       串口协议(话题 ``/motor_control``), 而不是 ros2_control。
    MockGripper     —— 不驱动任何东西, 只保证流程能跑通。
    JakaIOGripper   —— 真机: 工具端两路 NPN。DO_1 张开, DO_2 闭合。

队伍如果自备夹爪, 只要实现 ``open()`` / ``close(object_size)`` 两个方法即可接入。

**闭合角度是按工件尺寸算的**: 夹爪要"夹住"工件, 不能当没看见它 ——
真机靠堵转保护(顶到工件就停), 仿真没有这回事, 只能按几何把指间净距收到
工件宽度 + 一点缝(见 arena.yaml 的 jaw_gap_* 三个参数)。
"""
from __future__ import annotations

import math
import time
from typing import Optional

import rclpy

class Gripper:
    name = "abstract"

    def open(self) -> bool:
        raise NotImplementedError

    def close(self, object_size: float | None = None) -> bool:
        """闭合。``object_size`` 给出工件宽度, 只影响"收到多紧"。"""
        raise NotImplementedError

    def close_ms(self) -> int:
        return 400


class MockGripper(Gripper):
    """仿真用:无硬件,只记录状态并留出与真机相近的闭合时间。"""

    name = "mock"

    def __init__(self, node=None, close_delay: float = 0.4):
        self.node = node
        self.close_delay = close_delay
        self.is_closed = False

    def open(self) -> bool:
        if self.node is not None:
            rclpy.spin_once(self.node, timeout_sec=0.05)
        self.is_closed = False
        return True

    def close(self, object_size: float | None = None) -> bool:
        if self.node is not None:
            rclpy.spin_once(self.node, timeout_sec=self.close_delay)
        else:
            time.sleep(self.close_delay)
        self.is_closed = True
        return True

    def close_ms(self) -> int:
        return int(self.close_delay * 1000)


class JakaIOGripper(Gripper):
    """真机: 工具端两路 NPN 数字输出。

    示教器「I/O 面板 → 固定 I/O → 工具端」,DO_1 / DO_2 的类型都选 NPN
    (NPN/PNP 是输出电气类型, 由示教器保存; 这里只负责把开关打到和点按一样的状态):

      DO_1 (SDK index 0) 置 1 → 张开
      DO_2 (SDK index 1) 置 1 → 闭合

    两路互斥: 先关掉另一路, 再打开这一路, 避免两个线圈同时吸合。
    置 1 之后保持, 和示教器上把开关拨到开是同一件事。

    服务优先 ``/jaka_planner/set_io`` —— 和轨迹共用 ``jaka_planner`` 已经登录的
    那条 SDK 连接。``jaka_driver`` 不能和 planner 同时跑, 只在单独开 driver 时
    才退回 ``/jaka_driver/set_io``。
    """

    name = "jaka_io"

    def __init__(self, node, open_index: int = 0, close_index: int = 1,
                 settle: float = 0.5, io_type: int = 1, service: str = ""):
        from jaka_msgs.srv import SetIO
        self.node = node
        self.open_index = int(open_index)
        self.close_index = int(close_index)
        self.settle = float(settle)
        self.io_type = int(io_type)          # 1 = 工具端 IO_TOOL
        self._SetIO = SetIO
        names = [service] if service else ["/jaka_planner/set_io", "/jaka_driver/set_io"]
        self._cli = None
        self._service = ""
        for name in names:
            cli = node.create_client(SetIO, name)
            if cli.wait_for_service(timeout_sec=1.5):
                self._cli = cli
                self._service = name
                break
            node.destroy_client(cli)
        if self._cli is None:
            self._service = names[0]
            self._cli = node.create_client(SetIO, self._service)
            node.get_logger().warn(
                f"夹爪 IO 服务暂未就绪 ({', '.join(names)})。"
                "先起 jaka_planner, 并确认工具端 DO_1/DO_2 为 NPN。")
        else:
            node.get_logger().info(
                f"夹爪 IO: {self._service}, 工具端 NPN, "
                f"DO_{self.open_index + 1}=张开, DO_{self.close_index + 1}=闭合")

    def _set_do(self, index: int, value: float) -> bool:
        if not self._cli.service_is_ready() and not self._cli.wait_for_service(timeout_sec=2.0):
            self.node.get_logger().error(f"夹爪 IO 服务 {self._service} 不可用")
            return False
        req = self._SetIO.Request()
        req.signal = "digital"
        req.type = self.io_type
        req.index = int(index)
        req.value = float(value)
        fut = self._cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=5.0)
        res = fut.result()
        if res is None or not res.ret:
            msg = "" if res is None else res.message
            self.node.get_logger().error(
                f"工具端 DO_{index + 1}={value:g} 失败: {msg or '超时'}")
            return False
        return True

    def _drive(self, on_index: int, off_index: int) -> bool:
        if not self._set_do(off_index, 0.0):
            return False
        if not self._set_do(on_index, 1.0):
            return False
        end = time.time() + self.settle
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)
        return True

    def open(self) -> bool:
        self.node.get_logger().info(
            f"夹爪张开: DO_{self.open_index + 1}=1, DO_{self.close_index + 1}=0")
        return self._drive(self.open_index, self.close_index)

    def close(self, object_size: float | None = None) -> bool:
        self.node.get_logger().info(
            f"夹爪闭合: DO_{self.close_index + 1}=1, DO_{self.open_index + 1}=0")
        return self._drive(self.close_index, self.open_index)

    def close_ms(self) -> int:
        return int(self.settle * 1000)


class SimGripper(Gripper):
    """仿真: 把两片指的关节角发给 ros2_control 的 ``gripper_controller``。

    关节角 -> 指间净距的换算用 arena.yaml 里实测的三个数:
    ``jaw_gap_open``(张开时净距) / ``jaw_gap_per_deg``(每合 1° 少多少) /
    ``jaw_limit_deg``(两片指不互相干涉的最大角)。

    ``_angle_for()`` 返回的永远是**正的"闭合量"**; 真正下发时左指取负、右指取正 ——
    两个关节的轴向量都是 +y(见 wheeltec_gripper.urdf.xacro 里的说明: 用负向轴会让
    Gazebo 直接把关节冻住), 镜像关系只能靠命令符号表达。
    """

    name = "sim"

    def __init__(self, node, arena=None, topic: str = "/gripper_controller/commands",
                 open_angle: float = 0.0, close_delay: float = 0.35):
        from std_msgs.msg import Float64MultiArray
        self.node = node
        self.arena = arena
        self.open_angle = float(open_angle)
        self.close_delay = float(close_delay)
        self._msg_type = Float64MultiArray
        self.pub = node.create_publisher(Float64MultiArray, topic, 10)
        self.is_closed = False
        # Position acknowledgement is not contact/force feedback. It only
        # prevents a disconnected or stalled simulated controller from being
        # treated as a successful open/close command.
        from sensor_msgs.msg import JointState
        from rclpy.qos import qos_profile_sensor_data
        self._feedback = None
        self._feedback_at = 0.0
        self._feedback_seq = 0
        self.feedback_timeout = 2.0
        self.position_tolerance = 0.015
        self._joint_sub = node.create_subscription(
            JointState, "/joint_states", self._on_joints, qos_profile_sensor_data)
        if not arena or not arena.has_gripper:
            node.get_logger().warn(
                "arena.yaml 里 tool.gripper 不是 wheeltec: 仿真夹爪不会动作")

    def _angle_for(self, object_size: float | None) -> float:
        """工件尺寸 -> 关节角(rad), 让指间净距刚好收在工件两侧。"""
        a = self.arena
        gap_open = a.raw["tool"].get("jaw_gap_open", 0.07033)
        per_deg = a.raw["tool"].get("jaw_gap_per_deg", 0.001303)
        limit = a.raw["tool"].get("jaw_limit_deg", 35.86)
        extra = a.raw["tool"].get("jaw_gap_extra", 0.004)
        if object_size is None:
            deg = limit                       # 不知道自己夹的是什么: 合到底
        else:
            deg = (gap_open - (object_size + extra)) / per_deg
            deg = max(0.0, min(float(limit), float(deg)))
        return math.radians(deg)

    def _on_joints(self, msg) -> None:
        names = ("gripper_jaw_l_joint", "gripper_jaw_r_joint")
        if any(msg.name.count(name) != 1 for name in names):
            return
        try:
            values = tuple(float(msg.position[msg.name.index(name)]) for name in names)
        except (IndexError, TypeError, ValueError):
            return
        if not all(math.isfinite(value) for value in values):
            return
        self._feedback = values
        self._feedback_at = time.monotonic()
        self._feedback_seq += 1

    def _send(self, angle: float) -> bool:
        if not math.isfinite(angle):
            self.node.get_logger().error("仿真夹爪目标角无效")
            return False
        msg = self._msg_type()
        # Both axes point +y, so inward motion has opposite signs.
        msg.data = [-float(angle), float(angle)]
        started = time.monotonic()
        previous_seq = self._feedback_seq
        matched = 0
        self.pub.publish(msg)
        while time.monotonic() - started < self.feedback_timeout:
            rclpy.spin_once(self.node, timeout_sec=0.02)
            now = time.monotonic()
            if self._feedback_seq == previous_seq:
                continue
            previous_seq = self._feedback_seq
            at_target = self._feedback is not None and all(
                abs(actual - target) <= self.position_tolerance
                for actual, target in zip(self._feedback, msg.data))
            # Two distinct valid samples after the settling interval. A cached
            # pre-command pose or a single transient match cannot acknowledge it.
            if now - started >= self.close_delay and now - self._feedback_at < 0.2 and at_target:
                matched += 1
                if matched >= 2:
                    return True
            else:
                matched = 0
        self.node.get_logger().error(
            f"仿真夹爪关节未确认到位（{self.feedback_timeout:.1f}s），停止后续附着/释放；"
            f"目标={msg.data}，反馈={self._feedback}")
        return False

    def open(self) -> bool:
        ok = self._send(self.open_angle)
        self.is_closed = False if ok else None
        return ok

    def close(self, object_size: float | None = None) -> bool:
        ok = self._send(self._angle_for(object_size))
        self.is_closed = True if ok else None
        return ok

    def close_ms(self) -> int:
        return int(self.close_delay * 1000)


class WheeltecGripper(Gripper):
    """真机: WHEELTEC MS42DC 二指柔性爪(驱控一体步进电机, USB 串口)。

    走厂家 ``step_motor`` 包的协议: 往 ``/motor_control`` 发
    ``step_motor/msg/Motor``; 那个节点把帧发给电机(见
    jaka_ros2/src/step_motor/src/motor_node.cpp, 帧格式 7B ... 7D + BCC 异或)。

    协议要点(见《驱控一体步进电机 ROS 控制使用手册》):
      mode=2 相对角度模式, 角度放大 10 倍发送(单位 0.1°), 角度是**增量**;
      转向 0 = 逆时针 = 张开, 1 = 顺时针 = 闭合;
      细分 32 建议值; speed 单位 rad/s, 也放大 10 倍;
      完全闭合需要顺时针 5.2 圈(1872°) —— 夹到工件会堵转自停, 所以
      "发够大的角度"就是"夹紧"的用法。
    """

    name = "wheeltec"

    def __init__(self, node, motor_id: int = 1, speed: int = 200,
                 sub_divide: int = 32, angle_deg: float = 1872.0,
                 settle: float = 1.2, topic: str = "/motor_control"):
        from step_motor.msg import Motor      # 懒加载: 没装 step_motor 也能用别的夹爪
        self.node = node
        self._Motor = Motor
        self.motor_id = int(motor_id)
        self.speed = int(speed)
        self.sub_divide = int(sub_divide)
        self.angle_deg = float(angle_deg)
        self.settle = float(settle)
        self.pub = node.create_publisher(Motor, topic, 10)
        self.is_closed = False
        node.get_logger().info(
            f"WHEELTEC 夹爪: 话题 {topic}, id={motor_id}, speed={speed}×0.1rad/s, "
            f"细分 {sub_divide}, 行程 {angle_deg:.0f}°(需先 ros2 run step_motor motor_node)")

    def _send(self, direction: int) -> bool:
        msg = self._Motor()
        msg.id = self.motor_id
        msg.speed = self.speed
        msg.dir = direction
        msg.mode = 2                       # 相对角度
        msg.angle = int(self.angle_deg * 10)   # 协议规定放大 10 倍
        msg.state = 0                      # 0 = 控制, 1 = 查询状态
        msg.sub_divide = self.sub_divide
        self.pub.publish(msg)
        if self.node is not None:
            end = time.time() + self.settle
            while time.time() < end:
                rclpy.spin_once(self.node, timeout_sec=0.02)
        return True

    def open(self) -> bool:
        self.is_closed = False
        return self._send(0)               # 逆时针 = 张开

    def close(self, object_size: float | None = None) -> bool:
        self.is_closed = True
        return self._send(1)               # 顺时针 = 闭合(顶到工件即堵转自停)

    def close_ms(self) -> int:
        return int(self.settle * 1000)


def make_gripper(mode: str, node=None, arena=None, **kwargs) -> Gripper:
    mode = (mode or "mock").lower()
    if mode == "mock":
        return MockGripper(node, **kwargs)
    if mode in ("sim", "gazebo"):
        return SimGripper(node, arena=arena, **kwargs)
    if mode in ("wheeltec", "step_motor"):
        return WheeltecGripper(node, **kwargs)
    if mode in ("jaka_io",):
        return JakaIOGripper(node, **kwargs)
    if mode == "real":
        # 真机默认用厂家夹爪; 用 jaka_driver 的 IO 夹爪请显式写 gripper:=jaka_io
        return WheeltecGripper(node, **kwargs)
    raise ValueError(
        f"未知夹爪模式: {mode!r} (可选 sim / wheeltec / mock / jaka_io)")
