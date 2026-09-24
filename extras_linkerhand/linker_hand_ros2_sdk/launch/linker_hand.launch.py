#!/usr/bin/env python3 
# -*- coding: utf-8 -*-
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='linker_hand_ros2_sdk',
            executable='linker_hand_sdk',
            name='linker_hand_sdk',
            output='screen',
            parameters=[{
                'hand_type': 'left', # 配置Linker Hand灵巧手类型 left | right 字母为小写
                'hand_joint': "O6", # O6\L6P\L6\L7\L10\L20\G20(工业版)\L21 字母为大写
                # The O6 uses Modbus RTU over USB-RS485. The CAN parameter is
                # ignored whenever modbus is not "None".
                'is_touch': False, # 压感扩展寄存器尚未由现场协议/固件确认
                'can': 'can0',
                "modbus": "/dev/ttyUSB0", # 接入后按实际枚举结果修改为 /dev/ttyUSB* 或 /dev/ttyACM*
                'initialize_pose': False, # 仅验证状态和话题时禁止启动后自动改变手型
                'read_device_info': False, # Excel 仅定义 30-35；暂不读取 SDK 的 30-44 扩展区
            }],
        ),
    ])
