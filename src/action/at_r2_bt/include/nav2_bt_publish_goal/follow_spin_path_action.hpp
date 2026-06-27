// Copyright 2024
// Licensed under the Apache License, Version 2.0

#ifndef NAV2_BT_PUBLISH_GOAL__FOLLOW_SPIN_PATH_ACTION_HPP_
#define NAV2_BT_PUBLISH_GOAL__FOLLOW_SPIN_PATH_ACTION_HPP_

#include <string>
#include <memory>

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace nav2_bt_publish_goal
{

// 自己生成一条 dense Path 直接喂 controller_server 的 FollowPath action,
// 完全绕开 global planner。
//
// xy: 起点(当前 TF) -> 目标点(goal_x, goal_y) 直线插值
// yaw: 三段插值
//   - 距离起点 < rotation_start_distance: yaw 保持 start_yaw
//   - 距离目标 < rotation_end_distance:   yaw 保持 goal_yaw
//   - 中间段: 按弧长在 start_yaw -> goal_yaw 之间平滑插值
// rotation_direction: "cw" 顺时针 / "ccw" 逆时针 (用于消除 yaw wrap 歧义)
//
// 注: 要让 MPPI 在执行中真的边走边转, 必须在 controller_server 配置里开
//   FollowPath.PathAlignCritic.use_path_orientations: true
// 并给足权重, 否则 robot 只跟踪 xy, yaw 只在终点被 GoalAngleCritic 约束。
class FollowSpinPathAction : public BT::StatefulActionNode
{
public:
  using FollowPath = nav2_msgs::action::FollowPath;
  using GoalHandleFollowPath = rclcpp_action::ClientGoalHandle<FollowPath>;

  FollowSpinPathAction(
    const std::string & name,
    const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  bool ensureInitialized();

  // 读 map -> base 当前位姿, 写入 sx, sy, syaw
  bool readCurrentPose(
    const std::string & map_frame,
    const std::string & base_frame,
    double tf_timeout_s,
    double & sx, double & sy, double & syaw);

  // 生成 dense Path
  nav_msgs::msg::Path buildPath(
    double sx, double sy, double syaw,
    double gx, double gy, double gyaw,
    double rot_start_d, double rot_end_d,
    const std::string & direction,
    int num_points,
    const std::string & frame_id);

  // 把 yaw 归一到 [-pi, pi]
  static double normalizeAngle(double a);

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  rclcpp::Node::SharedPtr tf_node_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp_action::Client<FollowPath>::SharedPtr action_client_;
  GoalHandleFollowPath::SharedPtr goal_handle_;
  bool goal_result_available_;
  rclcpp_action::ClientGoalHandle<FollowPath>::WrappedResult result_;

  // 把目标点 latched 出去, 与 PublishGoal 一致
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr current_goal_pub_;

  bool initialized_{false};
};

}  // namespace nav2_bt_publish_goal

#endif  // NAV2_BT_PUBLISH_GOAL__FOLLOW_SPIN_PATH_ACTION_HPP_
