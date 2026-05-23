#!/usr/bin/env python3
# Copyright 2024
# Licensed under the Apache License, Version 2.0

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    Launch file for testing the PublishGoal BT node
    
    This launch file starts a simple BT navigator node that can be used
    to test the PublishGoal behavior tree node.
    """
    
    # Get package directories
    pkg_dir = get_package_share_directory('nav2_bt_publish_goal')
    
    # Path to example behavior tree
    bt_xml_file = os.path.join(pkg_dir, 'behavior_trees', 'example_single_goal.xml')
    
    return LaunchDescription([
        # Simple test node that loads and runs the behavior tree
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator_test',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'default_nav_to_pose_bt_xml': bt_xml_file,
                'plugin_lib_names': [
                    'nav2_bt_publish_goal',  # Our custom plugin
                ],
                'bt_loop_duration': 10,
                'default_server_timeout': 20,
            }]
        ),
    ])
