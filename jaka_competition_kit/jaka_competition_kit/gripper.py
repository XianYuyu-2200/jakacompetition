"""夹爪抽象。

官方仓库**不包含夹爪驱动**,所以这里定义最小接口 + 两个实现:

  MockGripper   —— 不驱动任何硬件, 只在夹爪状态变化时发布事件。仿真用。
  JakaIOGripper —— 通过 jaka_driver 的 ``/jaka_driver/set_io`` 驱动工具端数字 IO。
                   真机用, 需要按你实际夹爪的 IO 编号与电平改 ``open_level``/``closed_level``。

队伍如果自备夹爪, 只要实现 ``open()`` / ``close()`` 两个方法即可接入。
"""
from __future__ import annotations

import time
from typing import Optional

import rclpy
from std_srvs.srv import SetBool


class Gripper:
    name = "abstract"

    def open(self) -> bool:
        raise NotImplementedError

    def close(self) -> bool:
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

    def close(self) -> bool:
        if self.node is not None:
            rclpy.spin_once(self.node, timeout_sec=self.close_delay)
        else:
            time.sleep(self.close_delay)
        self.is_closed = True
        return True

    def close_ms(self) -> int:
        return int(self.close_delay * 1000)


class JakaIOGripper(Gripper):
    """真机用:通过 jaka_driver 的工具端数字 IO 控制夹爪。

    参数按你的实际夹爪改:
      io_index        工具 IO 编号(0..1)
      open_level      张开电平(True=高电平)
      closed_level    闭合电平

    注意:jaka_driver 与 jaka_planner **不能同时运行**。
    本项目真机走 jaka_planner(轨迹执行),所以如果你的夹爪只认 jaka_driver 的
    set_io 服务,需要另起一个 jaka_driver 或用机械臂厂商提供的 IO 旁路。
    这一点在 docs/真机部署.md 里有说明。
    """

    name = "jaka_io"

    def __init__(self, node, io_index: int = 0, open_level: bool = False,
                 closed_level: bool = True, settle: float = 0.4):
        self.node = node
        self.io_index = io_index
        self.open_level = open_level
        self.closed_level = closed_level
        self.settle = settle
        self._cli = node.create_client(SetBool, "/jaka_driver/set_io")
        self._available = self._cli.wait_for_service(timeout_sec=3.0)

    def _set(self, level: bool) -> bool:
        if not self._available:
            return False
        req = SetBool.Request()
        req.data = bool(level)
        fut = self._cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=5.0)
        res = fut.result()
        ok = bool(res and res.success)
        if ok:
            end = time.time() + self.settle
            while time.time() < end:
                rclpy.spin_once(self.node, timeout_sec=0.05)
        return ok

    def open(self) -> bool:
        return self._set(self.open_level)

    def close(self) -> bool:
        return self._set(self.closed_level)

    def close_ms(self) -> int:
        return int(self.settle * 1000)


def make_gripper(mode: str, node=None, **kwargs) -> Gripper:
    mode = (mode or "mock").lower()
    if mode == "mock":
        return MockGripper(node, **kwargs)
    if mode in ("jaka_io", "real"):
        return JakaIOGripper(node, **kwargs)
    raise ValueError(f"未知夹爪模式: {mode!r} (可选 mock / jaka_io)")
