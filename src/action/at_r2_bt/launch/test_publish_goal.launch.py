#!/usr/bin/env python3
# Copyright 2024
# Licensed under the Apache License, Version 2.0

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch file for running the simple BT runner with weapon grasp parameters.
    """

    pkg_dir = get_package_share_directory('at_r2_bt')
    weapon_params_file = os.path.join(pkg_dir, 'config', 'weapon_grasp_params.yaml')

    return LaunchDescription([
        Node(
            package='at_r2_bt',
            executable='simple_bt_runner',
            name='simple_bt_runner',
            output='screen',
            arguments=['grasp_head.xml'],
            parameters=[
                weapon_params_file,
                {
                    'use_sim_time': False,
                },
            ],
        ),
    ])
