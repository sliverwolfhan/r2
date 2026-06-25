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

    bt_xml = LaunchConfiguration('bt_xml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'bt_xml',
            default_value='grasp_head.xml',
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
                {
                    'use_sim_time': False,
                },
            ],
        ),
    ])
