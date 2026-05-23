from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    arm_share = get_package_share_directory("arm")
    launch_pack_share = get_package_share_directory("launch_pack")

    urdf_path = os.path.join(arm_share, "model", "robotic_arm.urdf")
    rviz_path = os.path.join(launch_pack_share, "rviz", "display_config.rviz")

    with open(urdf_path, "r", encoding="utf-8") as inf:
        robot_desc = inf.read()

    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_desc}],
        output="screen",
    )

    arm_calc = Node(
        package="arm_calc",
        executable="arm_calc",
        output="screen",
    )

    arm_driver = Node(
        package="robot_driver",
        executable="robot_driver",
        output="screen",
    )

    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_path],
    )

    vision=Node(package="vision",
        executable="vision_node",
        output="screen",
    )

    static_tf_camera = Node(
    package="tf2_ros",
    executable="static_transform_publisher",
    arguments=[
        "0.1", "0.09", "-0.03",
        "0.0", "0.7071068", "0.0", "0.7071068",
        "link4",
        "camera_link"
    ],
    output="screen",
)

    return LaunchDescription([
        arm_driver,
        robot_state_pub,
        arm_calc,
        rviz2,
        static_tf_camera,
        vision
    ])
