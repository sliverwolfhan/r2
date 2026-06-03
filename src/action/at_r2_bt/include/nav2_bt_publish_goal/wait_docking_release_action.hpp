// Copyright 2024
// Licensed under the Apache License, Version 2.0

#ifndef NAV2_BT_PUBLISH_GOAL__WAIT_DOCKING_RELEASE_ACTION_HPP_
#define NAV2_BT_PUBLISH_GOAL__WAIT_DOCKING_RELEASE_ACTION_HPP_

#include <memory>
#include <string>
#include <mutex>

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

namespace nav2_bt_publish_goal
{

/**
 * @brief 等待对接状态变为目标值，然后发布松开爪子指令
 *
 * 订阅 /AT_R2/docking_status (Int32)，当值变为 target_status (默认1) 时，
 * 发布 /AT_R2/head_gripper_cmd = release_cmd (默认5)，然后返回 SUCCESS。
 * 超时则返回 FAILURE。
 */
class WaitDockingReleaseAction : public BT::StatefulActionNode
{
public:
  WaitDockingReleaseAction(
    const std::string & name,
    const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  void statusCallback(const std_msgs::msg::Int32::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr status_sub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr gripper_pub_;

  std::mutex mutex_;
  bool docking_confirmed_{false};
  rclcpp::Time start_time_;
  double timeout_{30.0};

  std::string status_topic_;
  std::string gripper_topic_;
  int32_t release_cmd_{5};
  int32_t target_status_{1};
};

}  // namespace nav2_bt_publish_goal

#endif  // NAV2_BT_PUBLISH_GOAL__WAIT_DOCKING_RELEASE_ACTION_HPP_
