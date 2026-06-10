// Copyright 2024
// Licensed under the Apache License, Version 2.0

#ifndef NAV2_BT_PUBLISH_GOAL__WAIT_FOR_ENTER_ACTION_HPP_
#define NAV2_BT_PUBLISH_GOAL__WAIT_FOR_ENTER_ACTION_HPP_

#include <chrono>
#include <memory>
#include <string>

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"

namespace nav2_bt_publish_goal
{

/**
 * @brief Wait for operator to press Enter before continuing the behavior tree.
 *
 * This node is intended for competition pre-start flow: start the BT early,
 * finish parameter setup, then wait here. When Enter is pressed, the next node
 * can immediately send the navigation goal.
 */
class WaitForEnterAction : public BT::StatefulActionNode
{
public:
  WaitForEnterAction(
    const std::string & name,
    const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  bool stdinReady() const;
  bool readPendingInput(bool & got_enter, bool fail_on_eof);
  void drainPendingInput();

  rclcpp::Node::SharedPtr node_;
  std::string prompt_{"参数已设置完成，按 Enter 开始导航"};
  double timeout_{0.0};
  bool require_tty_{true};
  bool clear_buffer_{true};
  std::chrono::steady_clock::time_point start_time_;
};

}  // namespace nav2_bt_publish_goal

#endif  // NAV2_BT_PUBLISH_GOAL__WAIT_FOR_ENTER_ACTION_HPP_
