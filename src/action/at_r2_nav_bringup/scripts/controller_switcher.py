#!/usr/bin/env python3
# Copyright 2025 Lihan Chen
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
控制器切换节点：根据机器人与目标点的距离，自动在 MPPI 和 PID Pursuit 之间切换。
- 距离 > switch_distance: 使用 FollowPath (MPPI)
- 距离 <= switch_distance: 使用 PreciseApproach (PID Pursuit)

使用 TF 获取机器人在 map 坐标系下的位置，与 plan 终点（也在 map 坐标系）比较距离。
通过发布 std_msgs/String 到 controller_selector topic 实现切换（QoS: TRANSIENT_LOCAL）。
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import Path
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener, TransformException


class ControllerSwitcher(Node):
    def __init__(self):
        super().__init__("controller_switcher")

        self.declare_parameter("switch_distance", 1.0)
        self.declare_parameter("far_controller", "FollowPath")
        self.declare_parameter("near_controller", "PreciseApproach")
        self.declare_parameter("plan_topic", "plan")
        self.declare_parameter("goal_change_threshold", 0.3)
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("robot_base_frame", "chassis")
        self.declare_parameter("check_hz", 10.0)

        self.switch_distance = self.get_parameter("switch_distance").value
        self.far_controller = self.get_parameter("far_controller").value
        self.near_controller = self.get_parameter("near_controller").value
        plan_topic = self.get_parameter("plan_topic").value
        self.goal_change_threshold = self.get_parameter("goal_change_threshold").value
        self.map_frame = self.get_parameter("map_frame").value
        self.robot_base_frame = self.get_parameter("robot_base_frame").value
        check_hz = self.get_parameter("check_hz").value

        self.current_controller = self.far_controller
        self.goal_x = None
        self.goal_y = None

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 发布者 QoS 必须匹配 BT ControllerSelector 订阅端: TRANSIENT_LOCAL
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.controller_pub = self.create_publisher(
            String, "controller_selector", qos
        )

        # 订阅全局路径
        self.create_subscription(Path, plan_topic, self.plan_callback, 10)

        # 定时器检查距离（不依赖 odom topic）
        self.create_timer(1.0 / check_hz, self.check_distance)

        self.get_logger().info(
            f"Controller switcher started: "
            f"{self.far_controller} (>{self.switch_distance}m) <-> "
            f"{self.near_controller} (<={self.switch_distance}m) | "
            f"TF: {self.map_frame} -> {self.robot_base_frame}"
        )

    def plan_callback(self, msg: Path):
        if not msg.poses:
            return
        end_pose = msg.poses[-1].pose
        new_x = end_pose.position.x
        new_y = end_pose.position.y

        # 仅当目标点发生显著变化时才重置控制器
        if self.goal_x is not None:
            goal_shift = math.hypot(new_x - self.goal_x, new_y - self.goal_y)
            if goal_shift < self.goal_change_threshold:
                self.goal_x = new_x
                self.goal_y = new_y
                return

        self.goal_x = new_x
        self.goal_y = new_y
        self.get_logger().info(
            f"New goal: ({self.goal_x:.2f}, {self.goal_y:.2f})"
        )
        self.publish_controller(self.far_controller)

    def check_distance(self):
        if self.goal_x is None:
            return

        try:
            t = self.tf_buffer.lookup_transform(
                self.map_frame, self.robot_base_frame, rclpy.time.Time()
            )
        except TransformException:
            return

        robot_x = t.transform.translation.x
        robot_y = t.transform.translation.y
        dist = math.hypot(self.goal_x - robot_x, self.goal_y - robot_y)

        if dist <= self.switch_distance:
            desired = self.near_controller
        else:
            desired = self.far_controller

        if desired != self.current_controller:
            self.get_logger().info(
                f"Distance to goal: {dist:.2f}m -> switching to {desired}"
            )
            self.publish_controller(desired)

    def publish_controller(self, controller_name: str):
        msg = String()
        msg.data = controller_name
        self.controller_pub.publish(msg)
        self.current_controller = controller_name


def main(args=None):
    rclpy.init(args=args)
    node = ControllerSwitcher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
