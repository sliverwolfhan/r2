// Copyright 2024
// Licensed under the Apache License, Version 2.0

#ifndef NAV2_BT_PUBLISH_GOAL__GRASP_READY_BY_POSE_DISTANCE_CONDITION_HPP_
#define NAV2_BT_PUBLISH_GOAL__GRASP_READY_BY_POSE_DISTANCE_CONDITION_HPP_

#include <memory>
#include <mutex>
#include <string>

#include "behaviortree_cpp/condition_node.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace nav2_bt_publish_goal
{

// 条件节点：导航过程中同时检查当前 y、yaw 和测距值。
// 三个条件都满足时返回 SUCCESS，用于提前中止导航并进入抓取流程。
class GraspReadyByPoseDistanceCondition : public BT::ConditionNode
{
public:
  GraspReadyByPoseDistanceCondition(
    const std::string & name,
    const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  bool ensureInitialized();
  static double normalizeAngle(double a);

  void distanceCallback(const std_msgs::msg::Float64::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

  // 仅在黑板没有共享 buffer 时使用的兜底
  rclcpp::Node::SharedPtr tf_node_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr distance_sub_;

  std::mutex distance_mutex_;
  double latest_distance_{0.0};
  rclcpp::Time latest_distance_time_;
  bool has_distance_{false};

  int stable_count_{0};
  bool initialized_{false};
};

}  // namespace nav2_bt_publish_goal

#endif  // NAV2_BT_PUBLISH_GOAL__GRASP_READY_BY_POSE_DISTANCE_CONDITION_HPP_
