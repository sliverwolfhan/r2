#!/usr/bin/env python3
# Copyright 2024
# Licensed under the Apache License, Version 2.0

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch file for running the simple BT runner with weapon grasp parameters.

    指定行为树 xml (默认 grasp_head.xml):
      ros2 launch at_r2_bt r2_bt_launch.py bt_xml:=grasp_head_2.xml
    """

    pkg_dir = get_package_share_directory('at_r2_bt')
    weapon_params_file = os.path.join(pkg_dir, 'config', 'weapon_grasp_params.yaml')
    place_kfs_params_file = os.path.join(pkg_dir, 'config', 'place_kfs_params.yaml')

    bt_xml = LaunchConfiguration('bt_xml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'bt_xml',
            default_value='meilin_mission.xml',
            description='行为树 xml 文件名 (位于 at_r2_bt/behavior_trees/)',
        ),
        Node(
            package='at_r2_bt',
            executable='simple_bt_runner',
            name='simple_bt_runner',
            output='screen',
            arguments=[bt_xml],
            parameters=[
                weapon_params_file,
                place_kfs_params_file,
                {
                    'use_sim_time': False,
                },
            ],
            # 在独立的 gnome-terminal 窗口里运行, 让 simple_bt_runner 拥有自己的 stdin TTY,
            # 这样 WaitForEnter 节点可以在该窗口内等待操作员按 Enter。
            # --wait 让 gnome-terminal 进程在子命令结束前不退出, 便于 launch 跟踪生命周期。
            prefix='gnome-terminal --wait --title="AT_R2 BT" --',
        ),
    ])
