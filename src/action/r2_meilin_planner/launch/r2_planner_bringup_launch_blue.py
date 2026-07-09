"""R2 梅林规划【蓝区】

启动 kfs_subscriber_node: 订阅下位机 /AT_R2/kfs_positions → 解析 KFS 布局 →
A* 路径规划 → 发布 /r2_planner/plan

"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("r2_meilin_planner")
    default_params = os.path.join(pkg_share, "config", "planner_params_blue.yaml")

    zone = LaunchConfiguration("zone")
    ignore_height = LaunchConfiguration("ignore_height")
    can_climb_400 = LaunchConfiguration("can_climb_400")
    params_file = LaunchConfiguration("params_file")

    declare_zone = DeclareLaunchArgument(
        "zone",
        default_value="blue",
        description="地图选择",
    )
    declare_ignore_height = DeclareLaunchArgument(
        "ignore_height",
        default_value="false",
        description="高度开关",
    )
    declare_can_climb_400 = DeclareLaunchArgument(
        "can_climb_400",
        default_value="true",
        description="能否上 400 台阶",
    )
    declare_params = DeclareLaunchArgument(
        "params_file",
        default_value=default_params,
        description="yaml",
    )

    planner_node = Node(
        package="r2_meilin_planner",
        executable="kfs_subscriber_node",
        name="kfs_subscriber_node",
        output="screen",
        # 先读 yaml，再用启动参数覆盖 zone / ignore_height / can_climb_400（后者优先）
        parameters=[
            params_file,
            {
                "zone": zone,
                "ignore_height": ignore_height,
                "can_climb_400": can_climb_400,
            },
        ],
    )

    return LaunchDescription([
        declare_zone,
        declare_ignore_height,
        declare_can_climb_400,
        declare_params,
        planner_node,
    ])
