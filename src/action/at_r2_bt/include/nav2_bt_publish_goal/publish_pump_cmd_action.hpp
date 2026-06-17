// Copyright 2024
// Licensed under the Apache License, Version 2.0

#ifndef NAV2_BT_PUBLISH_GOAL__PUBLISH_PUMP_CMD_ACTION_HPP_
#define NAV2_BT_PUBLISH_GOAL__PUBLISH_PUMP_CMD_ACTION_HPP_

#include <string>
#include <memory>

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

namespace nav2_bt_publish_goal
{

/**
 * PublishPumpCmd: 向气泵发布一个整数指令，控制吸气 / 放气。
 *
 *   command = 1  -> 吸气 (开)
 *   command = 0  -> 放气 (关)
 *
 * 指令话题默认 /AT_R2/pump_cmd (std_msgs/Int32)，使用 transient_local(latched)
 * QoS，确保气泵节点稍晚连接也能收到最新指令。
 *
 * fire-and-forget：发完指令立即返回 SUCCESS，由行为树自行用 DelayMs / 导航等
 * 动作来留出吸气/放气的执行时间。
 *
 * 输入端口:
 *   - node     (ROS node, 来自黑板)
 *   - command  (int)    要发布的指令: 1=吸气, 0=放气
 *   - topic    (string, 默认 "/AT_R2/pump_cmd")
 */
class PublishPumpCmdAction : public BT::SyncActionNode
{
public:
  PublishPumpCmdAction(
    const std::string & name,
    const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  // ROS2 components
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr cmd_pub_;

  // 当前已创建的话题名(用于检测端口变更后重建 publisher)
  std::string cmd_topic_;
};

}  // namespace nav2_bt_publish_goal

#endif  // NAV2_BT_PUBLISH_GOAL__PUBLISH_PUMP_CMD_ACTION_HPP_
