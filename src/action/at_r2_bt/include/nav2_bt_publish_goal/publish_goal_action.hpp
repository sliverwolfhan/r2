// Copyright 2024
// Licensed under the Apache License, Version 2.0

#ifndef NAV2_BT_PUBLISH_GOAL__PUBLISH_GOAL_ACTION_HPP_
#define NAV2_BT_PUBLISH_GOAL__PUBLISH_GOAL_ACTION_HPP_

#include <string>
#include <memory>

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

namespace nav2_bt_publish_goal
{

class PublishGoalAction : public BT::StatefulActionNode
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  PublishGoalAction(
    const std::string & name,
    const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

  // StatefulActionNode 需要实现这三个方法
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;
  GoalHandleNavigateToPose::SharedPtr goal_handle_;
  bool goal_result_available_;
  rclcpp_action::ClientGoalHandle<NavigateToPose>::WrappedResult result_;

  // 把当前导航目标点（latched）发出去，供 BackUpFreeSpace 等行为读取
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr current_goal_pub_;

  geometry_msgs::msg::PoseStamped createGoalMessage(
    double x, double y, double yaw, const std::string & frame_id);
};

}  // namespace nav2_bt_publish_goal

#endif  // NAV2_BT_PUBLISH_GOAL__PUBLISH_GOAL_ACTION_HPP_
