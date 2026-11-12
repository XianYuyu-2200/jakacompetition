"""有界 ROS 操作与只读监视。由工作台在已 source 的独立子进程中调用。

GUI 本身不导入 rclpy；stdout 的 WB_JSON: 行是稳定的进程间协议。
"""
from __future__ import annotations

import argparse
import base64
import json
import math
import sys
import time
from typing import Any

PREFIX = "WB_JSON:"
SERVICES = {
    "generate": "/competition/generate",
    "start": "/competition/start",
    "abort": "/competition/abort",
    "run-reset": "/competition/run/reset",
    "scene-reset": "/competition/scene/reset",
}
ARM_JOINTS = {f"joint_{i}" for i in range(1, 7)}
FINGER_JOINTS = {"gripper_jaw_l_joint", "gripper_jaw_r_joint"}
CONTROLLERS = {"joint_state_broadcaster", "jaka_minicobo_controller", "gripper_controller"}


def emit(payload: dict[str, Any]) -> None:
    print(PREFIX + json.dumps(payload, ensure_ascii=False), flush=True)


def readiness(mode: str, facts: dict[str, Any], camera_required: bool = False) -> list[dict[str, str]]:
    """纯函数；ready 只由实时关节/控制器/服务反馈决定，不由进程存在决定。"""
    if mode == "recognition":
        pairs = [
            ("仿真时钟", facts.get("clock", False)),
            ("彩色与深度图像", facts.get("images", False)),
            ("基础组识别结果", facts.get("detections", False)),
        ]
        return [{"name": name, "level": "ok" if ok else "warning",
                 "detail": "已就绪" if ok else "等待有效反馈"} for name, ok in pairs]
    pairs = [
        ("机械臂关节", facts.get("arm_joints", False)),
        ("夹爪关节", facts.get("finger_joints", False)),
        ("三组控制器", facts.get("controllers", False)),
        ("MoveIt 执行接口", facts.get("moveit", False)),
        ("规划场景服务", facts.get("planning_scene", False)),
    ]
    if mode == "basic":
        pairs.append(("出题与发令服务", facts.get("competition", False)))
    else:
        pairs.append(("仿真时钟", facts.get("clock", False)))
    if mode == "vision" or camera_required:
        pairs.append(("彩色与深度图像", facts.get("images", False)))
    return [{"name": name, "level": "ok" if ok else "warning",
             "detail": "已就绪" if ok else "等待有效反馈"} for name, ok in pairs]



def round_snapshot(current: dict[str, Any]) -> dict[str, Any]:
    """筛掉复位后仍由 transient-local 话题保留的上一轮题目和成绩。

    生成器复位不会发布空 scene，判分器复位也不会发布空 score。
    不改动既有节点：以 run_state 门控展示，并在收到新题目时清成绩。
    score 可以早于 finished 状态到达，因此先缓存、仅在 finished 展示。
    """
    state = current.get("run_state", "idle")
    return {"run_state": state,
            "scene": current.get("scene", {}) if state != "idle" else {},
            "score": current.get("score") if state == "finished" else None}


def _spin_until(node, predicate, timeout):
    import rclpy
    end = time.monotonic() + timeout
    while rclpy.ok() and not predicate() and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=min(0.1, max(0.0, end - time.monotonic())))
    return bool(predicate())


def service(name: str, timeout: float) -> int:
    import rclpy
    from std_srvs.srv import Trigger
    rclpy.init()
    node = rclpy.create_node("workbench_control")
    try:
        client = node.create_client(Trigger, SERVICES[name])
        deadline = time.monotonic() + timeout
        if not _spin_until(node, client.service_is_ready, timeout):
            raise TimeoutError(f"服务未就绪：{SERVICES[name]}")
        future = client.call_async(Trigger.Request())
        if not _spin_until(node, future.done, max(0.0, deadline - time.monotonic())):
            raise TimeoutError(f"服务调用超时：{SERVICES[name]}")
        result = future.result()
        emit({"type": "service", "name": name, "success": bool(result.success),
              "message": result.message})
        return 0 if result.success else 2
    except Exception as exc:
        emit({"type": "service", "name": name, "success": False, "message": str(exc)})
        return 2
    finally:
        node.destroy_node()
        rclpy.shutdown()


def halt(timeout: float) -> int:
    """停止本工作台仿真中的轨迹，再终止计时；不是实体设备安全急停。"""
    import rclpy
    from action_msgs.srv import CancelGoal
    from std_srvs.srv import Trigger
    rclpy.init()
    node = rclpy.create_node("workbench_stop")
    successes, errors = [], []
    try:
        deadline = time.monotonic() + timeout
        for path in ("/move_action/_action/cancel_goal",
                     "/execute_trajectory/_action/cancel_goal",
                     "/jaka_minicobo_controller/follow_joint_trajectory/_action/cancel_goal"):
            client = node.create_client(CancelGoal, path)
            if not client.wait_for_service(timeout_sec=min(1.0, max(0.0, deadline - time.monotonic()))):
                errors.append(f"取消接口不可用：{path}")
                continue
            # 零 goal_id 与零时间表示取消所有当前目标。仅在本工作台环境拥有期间调用。
            future = client.call_async(CancelGoal.Request())
            if _spin_until(node, future.done, max(0.0, min(2.0, deadline - time.monotonic()))):
                future.result()
                successes.append(path)
            else:
                errors.append(f"取消请求超时：{path}")
        client = node.create_client(Trigger, SERVICES["abort"])
        if client.wait_for_service(timeout_sec=min(1.0, max(0.0, deadline - time.monotonic()))):
            future = client.call_async(Trigger.Request())
            if _spin_until(node, future.done, max(0.0, deadline - time.monotonic())):
                response = future.result()
                if response.success:
                    successes.append(SERVICES["abort"])
                else:
                    errors.append(response.message)
            else:
                errors.append("终止本轮请求超时")
        else:
            errors.append("本轮终止服务不可用")
        emit({"type": "service", "name": "halt", "success": not errors,
              "message": "已发送仿真停止请求。" + ("；".join(errors) if errors else "轨迹取消及计时终止已响应。")})
        return 0 if not errors else 2
    finally:
        node.destroy_node()
        rclpy.shutdown()


def monitor(mode: str, camera: bool = False, recognition: bool = False) -> int:
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from controller_manager_msgs.srv import ListControllers
    from moveit_msgs.action import MoveGroup, ExecuteTrajectory
    from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import Image, JointState
    from std_msgs.msg import String
    from std_srvs.srv import Trigger

    rclpy.init()
    node = rclpy.create_node("workbench_monitor")
    # 接收最佳努力发布的传感器，也兼容可靠发布者。
    sensors = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    current = {"joint_at": 0.0, "joints": set(), "joint_positions": {}, "clock_at": 0.0,
               "color_at": 0.0, "depth_at": 0.0, "controller_at": 0.0,
               "controllers": set(), "run_state": "idle", "scene": {}, "score": None,
               "image_jpeg": "", "image_seq": 0, "annotated_jpeg": "",
               "annotated_seq": 0, "detections": None, "detection_at": 0.0}

    def joint(msg):
        positions = {name: float(position) for name, position in zip(msg.name, msg.position)
                     if math.isfinite(position)}
        current.update(joints=set(positions), joint_positions=positions,
                       joint_at=time.monotonic())

    def stamp(key):
        return lambda _msg: current.update({key: time.monotonic()})

    def color_image(msg):
        now = time.monotonic()
        current["color_at"] = now
        if not camera:
            return
        try:
            import cv2
            import numpy as np
            encoding = str(getattr(msg, "encoding", "")).lower()
            width = int(msg.width)
            height = int(msg.height)
            step = int(msg.step)
            raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(height, step)
            if encoding in ("rgb8", "bgr8"):
                pixels = raw[:, :width * 3].reshape(height, width, 3)
                if encoding == "rgb8":
                    pixels = cv2.cvtColor(pixels, cv2.COLOR_RGB2BGR)
            elif encoding in ("rgba8", "bgra8"):
                pixels = raw[:, :width * 4].reshape(height, width, 4)
                if encoding == "rgba8":
                    pixels = cv2.cvtColor(pixels, cv2.COLOR_RGBA2BGR)
                else:
                    pixels = cv2.cvtColor(pixels, cv2.COLOR_BGRA2BGR)
            elif encoding in ("mono8", "8uc1"):
                pixels = raw[:, :width]
            else:
                return
            # Keep the GUI IPC light: this is a preview, not a recording stream.
            if pixels.shape[1] > 320:
                height = max(1, round(pixels.shape[0] * 320 / pixels.shape[1]))
                pixels = cv2.resize(pixels, (320, height), interpolation=cv2.INTER_AREA)
            ok, encoded = cv2.imencode(".jpg", pixels, [cv2.IMWRITE_JPEG_QUALITY, 60])
            if ok:
                current["image_jpeg"] = base64.b64encode(encoded.tobytes()).decode("ascii")
                current["image_seq"] += 1
        except (ImportError, ValueError, TypeError, AttributeError):
            # Image display is optional; readiness still reports the topic.
            pass

    def annotated_image(msg):
        # Only used in basic recognition mode; keep the latest classifier overlay.
        try:
            import cv2
            import numpy as np
            width, height, step = int(msg.width), int(msg.height), int(msg.step)
            raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(height, step)
            pixels = raw[:, :width * 3].reshape(height, width, 3)
            if str(msg.encoding).lower() == "rgb8":
                pixels = cv2.cvtColor(pixels, cv2.COLOR_RGB2BGR)
            if width > 320:
                pixels = cv2.resize(pixels, (320, max(1, round(height * 320 / width))),
                                    interpolation=cv2.INTER_AREA)
            ok, encoded = cv2.imencode(".jpg", pixels, [cv2.IMWRITE_JPEG_QUALITY, 65])
            if ok:
                current["annotated_jpeg"] = base64.b64encode(encoded.tobytes()).decode("ascii")
                current["annotated_seq"] += 1
        except (ImportError, ValueError, TypeError, AttributeError):
            pass

    def detections(msg):
        try:
            value = json.loads(msg.data)
            if isinstance(value, dict):
                current["detections"] = value
                current["detection_at"] = time.monotonic()
        except (ValueError, TypeError, AttributeError):
            pass

    def scene(msg):
        try:
            value = json.loads(msg.data)
            current["scene"] = {"seed": value.get("seed"), "qr_png": value.get("qr_png", ""),
                                "qr_payload": value.get("qr_payload", ""),
                                "qr_order": value.get("qr_order", []),
                                "object_count": len(value.get("objects", []))}
            current["score"] = None
        except (ValueError, TypeError, AttributeError):
            pass

    def score(msg):
        try:
            value = json.loads(msg.data)
            if isinstance(value, dict):
                current["score"] = value
        except ValueError:
            pass

    node.create_subscription(JointState, "/joint_states", joint, sensors)
    node.create_subscription(String, "/competition/run_state",
                             lambda msg: current.update(run_state=msg.data), latched)
    node.create_subscription(String, "/competition/scene", scene, latched)
    node.create_subscription(String, "/competition/score", score, latched)
    camera_enabled = camera or mode in ("vision", "recognition")
    recognition = recognition or mode == "recognition"
    if camera_enabled:
        node.create_subscription(Clock, "/clock", stamp("clock_at"), sensors)
        node.create_subscription(Image, "/gemini305/color/image_raw", color_image, sensors)
        node.create_subscription(Image, "/gemini305/depth/image_raw", stamp("depth_at"), sensors)
        if recognition:
            node.create_subscription(Image, "/track1/annotated_image", annotated_image, sensors)
            node.create_subscription(String, "/track1/detections", detections, 10)
    move = ActionClient(node, MoveGroup, "/move_action")
    execute = ActionClient(node, ExecuteTrajectory, "/execute_trajectory")
    apply_scene = node.create_client(ApplyPlanningScene, "/apply_planning_scene")
    get_scene = node.create_client(GetPlanningScene, "/get_planning_scene")
    ctl = node.create_client(ListControllers, "/controller_manager/list_controllers")
    competition = [node.create_client(Trigger, path) for path in SERVICES.values()]
    pending = None
    requested_at = 0.0
    emitted_at = 0.0
    image_emitted_at = 0.0
    image_sent_token = None
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            now = time.monotonic()
            if pending is not None and pending.done():
                try:
                    current["controllers"] = {c.name for c in pending.result().controller
                                              if c.state == "active"}
                    current["controller_at"] = now
                except Exception:
                    current["controllers"] = set()
                pending = None
            if pending is not None and now - requested_at > 5.0:
                pending.cancel()
                pending = None
            if pending is None and now - requested_at > 2.0 and ctl.service_is_ready():
                pending = ctl.call_async(ListControllers.Request())
                requested_at = now
            annotated = recognition and bool(current["annotated_jpeg"])
            preview_seq = current["annotated_seq"] if annotated else current["image_seq"]
            preview_jpeg = current["annotated_jpeg"] if annotated else current["image_jpeg"]
            preview_token = (annotated, preview_seq)
            if camera_enabled and preview_jpeg and preview_token != image_sent_token \
                    and now - image_emitted_at >= 0.33:
                emit({"type": "image", "jpeg": preview_jpeg,
                      "seq": preview_seq, "annotated": annotated})
                image_sent_token = preview_token
                image_emitted_at = now
            if now - emitted_at < 1.0:
                continue
            emitted_at = now
            fresh = now - current["joint_at"] < 4.0
            detection_fresh = current["detections"] is not None \
                              and now - current["detection_at"] < 6.0
            facts = {
                "arm_joints": fresh and ARM_JOINTS <= current["joints"],
                "finger_joints": fresh and FINGER_JOINTS <= current["joints"],
                "controllers": now - current["controller_at"] < 6.0
                               and CONTROLLERS <= current["controllers"],
                "moveit": move.server_is_ready() and execute.server_is_ready(),
                "planning_scene": apply_scene.service_is_ready() and get_scene.service_is_ready(),
                "competition": all(c.service_is_ready() for c in competition),
                "clock": now - current["clock_at"] < 4.0,
                "images": now - current["color_at"] < 6.0 and now - current["depth_at"] < 6.0,
                "detections": detection_fresh,
            }
            checks = readiness(mode, facts, camera_required=camera_enabled)
            emit({"type": "status", "ready": all(c["level"] == "ok" for c in checks),
                  "checks": checks, "joint_positions":
                  {name: current["joint_positions"].get(name) for name in sorted(ARM_JOINTS)
                   if name in current["joint_positions"]},
                  "recognition": (current["detections"] if detection_fresh else None)
                                 if recognition else None,
                  **round_snapshot(current)})
    except KeyboardInterrupt:
        pass
    finally:
        move.destroy()
        execute.destroy()
        node.destroy_node()
        rclpy.shutdown()
    return 0



def move_joints(values: list[float], velocity: float = 0.25) -> int:
    """工作台受控的单次关节姿态运动，仅连接当前已 source 的仿真/ROS 会话。"""
    import rclpy
    from jaka_competition_kit.backend import MoveGroupBackend
    rclpy.init()
    node = rclpy.create_node("workbench_pose_move")
    try:
        if len(values) != 6 or any(not math.isfinite(float(v)) for v in values):
            raise ValueError("必须提供 6 个有限关节角")
        backend = MoveGroupBackend(node, vel_scale=float(velocity), acc_scale=float(velocity),
                                   planning_attempts=2, allowed_planning_time=2.0)
        result = backend.goto_joints([float(v) for v in values])
        emit({"type": "pose-move", "success": bool(result.ok), "detail": str(result)})
        return 0 if result.ok else 2
    except Exception as exc:
        emit({"type": "pose-move", "success": False, "detail": str(exc)})
        return 2
    finally:
        node.destroy_node()
        rclpy.shutdown()


def close_reference_node(node) -> None:
    """复用既有 SceneClient 的清理顺序，避免后台执行线程先于资源销毁退出。"""
    try:
        node.scene_cli.close()
    finally:
        node.destroy_node()


def reference(velocity: float, fixed_poses: str = "") -> int:
    import rclpy
    from jaka_competition_kit.ref_track1 import Track1Reference
    ros_args = ["--ros-args", "-p", "gripper:=sim", "-p", f"vel_scale:={velocity}"]
    if fixed_poses:
        ros_args += ["-p", f"fixed_poses_config:={fixed_poses}"]
    rclpy.init(args=ros_args)
    node = None
    try:
        node = Track1Reference()
        if not node.wait_for_scene(timeout=15.0):
            raise TimeoutError("参考程序未收到题目，请先生成题目")
        # 只有构造完成、MoveIt 已连接、收到本轮场景后才通知 GUI 发令。
        emit({"type": "program-ready"})
        node.run()
        return 0 if node.done else 2
    except KeyboardInterrupt:
        return 130
    finally:
        try:
            if node is not None:
                close_reference_node(node)
        finally:
            if rclpy.ok():
                rclpy.shutdown()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    mon = sub.add_parser("monitor")
    mon.add_argument("--mode", choices=("basic", "vision"), default="basic")
    mon.add_argument("--camera", action="store_true", help="订阅并转发腕部相机图像")
    mon.add_argument("--recognition", action="store_true", help="显示基础组识别标注图像与结果")
    srv = sub.add_parser("service")
    srv.add_argument("--name", choices=(*SERVICES, "halt"), required=True)
    srv.add_argument("--timeout", type=float, default=15.0)
    ref = sub.add_parser("reference")
    ref.add_argument("--velocity", type=float, default=0.5)
    ref.add_argument("--fixed-poses", default="")
    pose = sub.add_parser("move-joints")
    pose.add_argument("--joints", nargs=6, type=float, required=True)
    pose.add_argument("--velocity", type=float, default=0.25)
    args = parser.parse_args(argv)
    try:
        if args.command == "monitor":
            return monitor(args.mode, camera=args.camera or args.recognition, recognition=args.recognition)
        if args.command == "service":
            return halt(args.timeout) if args.name == "halt" else service(args.name, args.timeout)
        if args.command == "move-joints":
            return move_joints(args.joints, args.velocity)
        return reference(args.velocity, args.fixed_poses)
    except Exception as exc:
        emit({"type": "error", "message": f"{type(exc).__name__}: {exc}"})
        return 2


if __name__ == "__main__":
    sys.exit(main())
