// Copyright 2024
// Licensed under the Apache License, Version 2.0

#ifndef NAV2_BT_PUBLISH_GOAL__CLIMB_STAIR_ACTION_HPP_
#define NAV2_BT_PUBLISH_GOAL__CLIMB_STAIR_ACTION_HPP_

#include <string>
#include <memory>

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/int32.hpp"

namespace nav2_bt_publish_goal
{

class ClimbStairAction : public BT::StatefulActionNode
{
public:
  ClimbStairAction(
    const std::string & name,
    const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

  // StatefulActionNode interface
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  // ROS2 components
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr climb_pub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr status_sub_;
  
  // State management
  int32_t current_status_;
  bool status_received_;
  
  // Status constants
  static constexpr int32_t STATUS_CLIMBING = 1;
  static constexpr int32_t STATUS_SUCCESS = 2;
  static constexpr int32_t STATUS_FAILURE = 3;
  
  // Callback function
  void statusCallback(const std_msgs::msg::Int32::SharedPtr msg);
};

}  // namespace nav2_bt_publish_goal

#endif  // NAV2_BT_PUBLISH_GOAL__CLIMB_STAIR_ACTION_HPP_
