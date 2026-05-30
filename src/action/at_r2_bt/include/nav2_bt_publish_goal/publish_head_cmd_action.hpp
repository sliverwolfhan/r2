// Copyright 2024
// Licensed under the Apache License, Version 2.0

#ifndef NAV2_BT_PUBLISH_GOAL__PUBLISH_HEAD_CMD_ACTION_HPP_
#define NAV2_BT_PUBLISH_GOAL__PUBLISH_HEAD_CMD_ACTION_HPP_

#include <string>
#include <memory>

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

namespace nav2_bt_publish_goal
{

/**
 * PublishHeadCmd: 向武器头爪子(与机械臂相互独立的另一套爪子)发布一个
 * 整数指令，告诉它当前要做什么动作。
 *
 *   command = 1  -> 准备抓取
 *   command = 2  -> 抓取
 *   command = 3  -> 抬起武器头
 *   command = 4  -> 对接
 *
 * 指令话题默认 /AT_R2/head_gripper_cmd (std_msgs/Int32)，使用
 * transient_local(latched) QoS，确保爪子节点稍晚连接也能收到最新指令。
 *
 * 同步模式：
 *   - wait_done = false (默认)：发完指令立即返回 SUCCESS (fire-and-forget)，
 *     由行为树自行用 DelayMs / 导航等动作来留出执行时间。
 *   - wait_done = true：订阅 done_topic (std_msgs/Int32)，直到收到的反馈值
 *     等于本次 command 才返回 SUCCESS；超过 timeout 秒仍未收到则返回 FAILURE
 *     (timeout <= 0 表示无限等待)。
 *
 * 输入端口:
 *   - node       (ROS node, 来自黑板)
 *   - command    (int)    要发布的指令: 1/2/3
 *   - topic      (string, 默认 "/AT_R2/head_gripper_cmd")
 *   - wait_done  (bool,   默认 false)
 *   - done_topic (string, 默认 "/AT_R2/head_gripper_done")
 *   - timeout    (double, 默认 0.0 = 无限等待) 等待反馈的超时秒数
 */
class PublishHeadCmdAction : public BT::StatefulActionNode
{
public:
  PublishHeadCmdAction(
    const std::string & name,
    const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

  // StatefulActionNode interface
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  void doneCallback(const std_msgs::msg::Int32::SharedPtr msg);

  // ROS2 components
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr cmd_pub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr done_sub_;

  // 当前已创建的话题名(用于检测端口变更后重建 pub/sub)
  std::string cmd_topic_;
  std::string done_topic_;

  // 本次执行状态
  int32_t command_{0};
  bool wait_done_{false};
  double timeout_{0.0};
  bool done_received_{false};
  int32_t done_value_{0};
  rclcpp::Time start_time_;
};

}  // namespace nav2_bt_publish_goal

#endif  // NAV2_BT_PUBLISH_GOAL__PUBLISH_HEAD_CMD_ACTION_HPP_
