from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, RegisterEventHandler, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    launch_pack_share = get_package_share_directory("launch_pack")
    
    # Include arm_mujoco_sim launch file
    arm_mujoco_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_pack_share, "launch", "arm_mujoco_sim.launch.py")
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
    
    static_tf_camera = Node(
    package="tf2_ros",
    executable="static_transform_publisher",
    arguments=[
        "0.1", "0.09", "-0.03",  # x, y, z
        "0.7071068", "0.0", "0.7071068", "0.0",
        "link4",
        "camera_link"
    ],
    output="screen",
)
    
    # arm_task node
    # arm_task = Node(
    #     package="arm_task",
    #     executable="arm_task",
    #     output="screen",
    # )
    
    return LaunchDescription([
        show_rviz_arg,
        arm_mujoco_sim_launch,
        static_tf_camera,
        # arm_task,
    ])
