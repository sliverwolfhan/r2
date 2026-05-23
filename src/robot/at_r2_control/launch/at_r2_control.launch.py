#!/usr/bin/env python3
"""
AT_R2 键盘控制启动文件
注意：这个launch文件只启动键盘控制节点
桥接配置已经在 spawn_robots.launch.py 中完成
"""
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    # 键盘控制节点
    keyboard_control = Node(
        package='at_r2_control',
        executable='keyboard_control_node',
        parameters=[{'robot_name': 'AT_R2'}],  # 机器人名称参数
        output='screen',
        prefix='xterm -e'  # 在单独的终端运行以支持键盘输入
    )
    
    return LaunchDescription([
        keyboard_control,
    ])

