// Copyright 2024
// Licensed under the Apache License, Version 2.0

#ifndef NAV2_BT_PUBLISH_GOAL__GRASP_PRESSED_BY_DISTANCE_CONDITION_HPP_
#define NAV2_BT_PUBLISH_GOAL__GRASP_PRESSED_BY_DISTANCE_CONDITION_HPP_

#include <deque>
#include <memory>
#include <mutex>
#include <string>

#include "behaviortree_cpp/condition_node.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

namespace nav2_bt_publish_goal
{

// 条件节点：只用爪子侧激光测距(/AT_R2/distance_grasp)判断是否已"靠住"柜台。
// 原始测距先做中值+EMA滤波去尖刺/抖动，滤波后的缩放距离落入
// [target_distance - distance_tolerance, target_distance + distance_tolerance]
// 且连续 stable_count 次满足，即返回 SUCCESS(说明已贴到位)。
// 不依赖 TF/位姿，纯测距判断。
class GraspPressedByDistanceCondition : public BT::ConditionNode
{
public:
  GraspPressedByDistanceCondition(
    const std::string & name,
    const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  bool ensureInitialized();
  void distanceCallback(const std_msgs::msg::Float64::SharedPtr msg);
  double filterDistance(double raw, const rclcpp::Time & stamp);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr distance_sub_;

  std::mutex distance_mutex_;
  double latest_distance_{0.0};
  rclcpp::Time latest_distance_time_;
  bool has_distance_{false};

  // 滤波状态(仅在 distanceCallback 内、持锁访问)
  bool filter_enable_{true};
  int median_window_{3};
  double ema_tau_{0.1};
  std::deque<double> median_buf_;
  bool ema_initialized_{false};
  double ema_value_{0.0};
  rclcpp::Time last_filter_time_;

  int stable_count_{0};
  bool initialized_{false};
};

}  // namespace nav2_bt_publish_goal

#endif  // NAV2_BT_PUBLISH_GOAL__GRASP_PRESSED_BY_DISTANCE_CONDITION_HPP_
