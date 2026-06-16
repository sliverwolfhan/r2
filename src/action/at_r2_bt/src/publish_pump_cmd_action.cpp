// Copyright 2024
// Licensed under the Apache License, Version 2.0

#include "nav2_bt_publish_goal/publish_pump_cmd_action.hpp"

namespace nav2_bt_publish_goal
{

PublishPumpCmdAction::PublishPumpCmdAction(
  const std::string & name,
  const BT::NodeConfig & config)
: BT::SyncActionNode(name, config)
{
  // node 在 tick 中从黑板获取
}

BT::PortsList PublishPumpCmdAction::providedPorts()
{
  return {
    BT::InputPort<rclcpp::Node::SharedPtr>("node", "ROS node"),
    BT::InputPort<int>("command", "整数指令: 1=吸气, 0=放气"),
    BT::InputPort<std::string>(
      "topic", "/AT_R2/pump_cmd", "指令发布话题 (std_msgs/Int32)")
  };
}

BT::NodeStatus PublishPumpCmdAction::tick()
{
  // 第一次获取 ROS node
  if (!node_) {
    if (!getInput("node", node_) || !node_) {
      RCLCPP_ERROR(
        rclcpp::get_logger("PublishPumpCmdAction"), "Missing required input [node]");
      return BT::NodeStatus::FAILURE;
    }
  }

  // 读取指令
  int command = 0;
  if (!getInput("command", command)) {
    RCLCPP_ERROR(node_->get_logger(), "Missing required input [command]");
    return BT::NodeStatus::FAILURE;
  }
  if (command != 0 && command != 1) {
    RCLCPP_WARN(
      node_->get_logger(),
      "pump command=%d 不在预期范围 [0,1] (1=吸气/0=放气)", command);
  }

  // 读取话题
  std::string topic = "/AT_R2/pump_cmd";
  getInput("topic", topic);

  // 按需(首次或话题变更)创建 latched publisher
  if (!cmd_pub_ || topic != cmd_topic_) {
    auto qos = rclcpp::QoS(1).transient_local().reliable();
    cmd_pub_ = node_->create_publisher<std_msgs::msg::Int32>(topic, qos);
    cmd_topic_ = topic;
    RCLCPP_INFO(node_->get_logger(), "pump 指令话题: %s", cmd_topic_.c_str());
  }

  // 发布指令
  std_msgs::msg::Int32 msg;
  msg.data = command;
  cmd_pub_->publish(msg);
  RCLCPP_INFO(
    node_->get_logger(), "已发布 pump 指令: %d (%s)", command,
    command == 1 ? "吸气" : command == 0 ? "放气" : "未知");

  return BT::NodeStatus::SUCCESS;
}

}  // namespace nav2_bt_publish_goal

// Register the node with BehaviorTree.CPP v4
#include "behaviortree_cpp/bt_factory.h"

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_bt_publish_goal::PublishPumpCmdAction>("PublishPumpCmd");
}
