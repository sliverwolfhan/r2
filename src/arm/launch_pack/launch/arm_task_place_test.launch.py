from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    launch_pack_share = get_package_share_directory("launch_pack")
    
    # Include arm_task_sim launch file
    arm_task_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_pack_share, "launch", "arm_task_sim.launch.py")
        ),
        launch_arguments={
            'show_rviz': LaunchConfiguration('show_rviz')
        }.items()
    )
    
    show_rviz_arg = DeclareLaunchArgument(
        "show_rviz",
        default_value="true",
        description="Whether to start RViz2 together with simulation",
    )
    
    # Static transform for test target shelf
    # Position: x=0.5, y=0.3, z=0.4
    # Orientation: (0, 0, 0) which is identity quaternion (0, 0, 0, 1)
    # This simulates a target shelf location
    static_tf_target_shelf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "0.8", "-0.3", "0.7",  # x, y, z translation
            "0.0", "0.0", "0.0", "1.0",  # quaternion (x, y, z, w) - identity (no rotation)
            "base_link",
            "target_shelf"
        ],
        output="screen",
    )

    place_kfs_test = TimerAction(
        period=3.0,
        actions=[
            Node(
                package="arm_task",
                executable="place_kfs_test",
                output="screen",
            )
        ],
    )
    
    return LaunchDescription([
        show_rviz_arg,
        arm_task_sim_launch,
        static_tf_target_shelf,
        place_kfs_test,
    ])
