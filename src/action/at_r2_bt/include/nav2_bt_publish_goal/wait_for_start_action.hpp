// Copyright 2024
// Licensed under the Apache License, Version 2.0

#ifndef NAV2_BT_PUBLISH_GOAL__WAIT_FOR_START_ACTION_HPP_
#define NAV2_BT_PUBLISH_GOAL__WAIT_FOR_START_ACTION_HPP_

#include <atomic>
#include <chrono>
#include <memory>
#include <string>

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

namespace nav2_bt_publish_goal
{

/**
 * @brief 一键启动门控: 操作员按 Enter 或收到启动话题任一即放行。
 *
 * 与 WaitForEnter 相同的 pre-start 用途, 但增加了一路话题启动:
 * 订阅 start_topic (默认 /AT_R2/start_cmd, std_msgs/Int32), 当收到的值等于
 * start_value (默认 1) 时立即返回 SUCCESS; 同时仍监听 stdin, 操作员按 Enter
 * 也立即返回 SUCCESS。任一路触发即放行, 二者互为冗余。
 *
 * 无 TTY 时(如 ros2 launch 下)自动只保留话题一路; require_tty=true 时若
 * 既无 TTY 也需要 Enter, 会退化为只依赖话题(不会因缺 TTY 直接失败, 因为
 * 话题仍可启动)。
 */
class WaitForStartAction : public BT::StatefulActionNode
{
public:
  WaitForStartAction(
    const std::string & name,
    const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  void startCmdCallback(const std_msgs::msg::Int32::SharedPtr msg);

  bool stdinReady() const;
  bool readPendingInput(bool & got_enter, bool fail_on_eof);
  void drainPendingInput();

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr start_sub_;

  std::atomic<bool> start_received_{false};

  std::string prompt_{"参数已设置完成，按 Enter 或等待启动话题开始"};
  std::string start_topic_{"/AT_R2/start_cmd"};
  int32_t start_value_{1};
  double timeout_{0.0};
  bool clear_buffer_{true};
  bool enable_topic_{true};

  bool enter_enabled_{false};
  std::chrono::steady_clock::time_point start_time_;
};

}  // namespace nav2_bt_publish_goal

#endif  // NAV2_BT_PUBLISH_GOAL__WAIT_FOR_START_ACTION_HPP_
