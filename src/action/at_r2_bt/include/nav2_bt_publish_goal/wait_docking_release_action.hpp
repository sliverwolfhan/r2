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
 * @brief 等待对接完成信号，然后发布松开爪子指令
 *
 * 订阅 /AT_R2/docking_status (Int32)，当收到 1、2、3 任一值时视为"对接完成"，
 * 发布 /AT_R2/head_gripper_cmd = release_cmd (默认7)，然后返回 SUCCESS。
 * 若收到的值是 3（写死的"结束"信号），额外把输出端口 finished_all 置 1，
 * 供外层循环判断"不再继续抓取，结束整棵行为树"。
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
  bool stdinReady() const;
  bool pollEnter();

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr status_sub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr gripper_pub_;

  std::mutex mutex_;
  bool docking_confirmed_{false};
  bool finish_received_{false};   // 收到的对接完成信号是否为"结束值"(3)
  rclcpp::Time start_time_;
  double timeout_{30.0};

  std::string status_topic_;
  std::string gripper_topic_;
  int32_t release_cmd_{5};
  int32_t target_status_{1};

  // 写死: 收到 1/2/3 任一视为对接完成; 收到 3 额外触发"结束整棵树"
  static constexpr int32_t kFinishStatus = 3;

  bool allow_enter_{true};
  bool enter_enabled_{false};
};

}  // namespace nav2_bt_publish_goal

#endif  // NAV2_BT_PUBLISH_GOAL__WAIT_DOCKING_RELEASE_ACTION_HPP_
