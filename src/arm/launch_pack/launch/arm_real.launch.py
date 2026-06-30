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

    arm_task = Node(
        package="arm_task",
        executable="arm_task",
        output="screen",
        parameters=[{
            # 估算"最快移动时间"用的上限（duration<=0 时生效）。
            # 关节空间: 最快时间 = 最大关节角度差 / max_joint_velocity，再 clamp 到 [min, max]。
            "max_joint_velocity": 1.5,        # rad/s，调大→动作更快，别超过下位机实际能跟的速度
            "min_trajectory_duration": 0.1,   # s，时间下限
            "max_trajectory_duration": 10.0,  # s，时间上限
            # 笛卡尔估算速度（当前未接入，预留）
            "max_linear_velocity": 0.1,       # m/s
            "max_angular_velocity": 0.5,      # rad/s
        }],
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
        "0.0", "0.09625", "0.11225",
        "0.5", "0.5", "0.5", "-0.5",
        "link1",
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
        vision,
        arm_task
    ])
