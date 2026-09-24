"""JAKA Mini 2 机器人应用挑战赛官方工具包。

模块划分:
  arena           赛场几何(唯一真源)与合法性校核
  qr              二维码生成 / 解析
  backend         MoveGroup 动作客户端(仿真与真机接口一致)
  gripper         夹爪抽象(Mock / JAKA IO)
  executor        抓取原语
  scene_generator 场景生成器(ROS 2 节点)
  scorer          自动计时判分(ROS 2 节点)
  ref_track1      赛道一参考实现
"""

__version__ = "0.1.0"
