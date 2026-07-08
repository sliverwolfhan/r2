// Copyright 2024
// Licensed under the Apache License, Version 2.0

#ifndef NAV2_BT_PUBLISH_GOAL__WAIT_SLOT_SELECT_ACTION_HPP_
#define NAV2_BT_PUBLISH_GOAL__WAIT_SLOT_SELECT_ACTION_HPP_

#include <atomic>
#include <chrono>
#include <string>

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

namespace nav2_bt_publish_goal
{

// 等待"放格子选择"话题 (std_msgs/Int32) 的动作节点。
//   - 阻塞等待, 收到值 v 后输出到黑板 {slot_sel}=v。
//   - v ∈ {1,2,3}: 把黑板里 slot_<v>_x/y/yaw 拷到无前缀工作键 slot_x/y/yaw
//                  (供后续 PublishGoal 直接用), 值直接当 slot 编号。
//   - v == 4:      大胜信号, 只输出 slot_sel=4, 不选 slot。
//   - 其它值:      忽略, 继续等待。
// 也支持操作员按 Enter 手动放行 (调试用): Enter 时 slot_sel 输出 enter_value。
class WaitSlotSelectAction : public BT::StatefulActionNode
{
public:
  WaitSlotSelectAction(const std::string & name, const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  void slotSelectCallback(const std_msgs::msg::Int32::SharedPtr msg);
  bool copyDouble(const std::string & prefix, const std::string & suffix);
  bool selectSlotParams(int v);   // 按值 v 把 slot_<v>_* 拷到 slot_*

  // stdin (Enter) 辅助
  bool stdinReady() const;
  bool readPendingInput(bool & got_enter, bool fail_on_eof);
  void drainPendingInput();

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_;

  std::string topic_;
  std::string prompt_;
  double timeout_{0.0};
  bool clear_buffer_{true};
  bool enable_enter_{true};
  int enter_value_{1};        // 按 Enter 时视作收到的值

  std::atomic<bool> received_{false};
  std::atomic<int> received_value_{0};
  bool enter_enabled_{false};
  std::chrono::steady_clock::time_point start_time_;
};

}  // namespace nav2_bt_publish_goal

#endif  // NAV2_BT_PUBLISH_GOAL__WAIT_SLOT_SELECT_ACTION_HPP_
