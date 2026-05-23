// Copyright 2024
// Licensed under the Apache License, Version 2.0

#ifndef NAV2_BT_PUBLISH_GOAL__PRE_STEER_ALIGN_ACTION_HPP_
#define NAV2_BT_PUBLISH_GOAL__PRE_STEER_ALIGN_ACTION_HPP_

#include <string>
#include <memory>
#include <mutex>

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace nav2_bt_publish_goal
{

// 在上下台阶之前，向 nav 速度话题按固定频率发布一个小的 Twist，
// 让舵轮（swerve）提前朝指定方向打舵。动作结束时会发一帧零速度
// 以确保底盘停稳。
class PreSteerAlignAction : public BT::StatefulActionNode
{
public:
  PreSteerAlignAction(
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
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

  // 缓存的本次执行参数
  geometry_msgs::msg::Twist cmd_;
  rclcpp::Time start_time_;
  rclcpp::Duration duration_{0, 0};
  std::string topic_name_;

  void publishZero();
  void stopPublishTimer();
  void publishCmdTimerCallback();

  rclcpp::TimerBase::SharedPtr publish_timer_;
  std::mutex publish_timer_mutex_;
  bool publish_timer_armed_{false};
};

}  // namespace nav2_bt_publish_goal

#endif  // NAV2_BT_PUBLISH_GOAL__PRE_STEER_ALIGN_ACTION_HPP_
