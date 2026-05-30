// Copyright 2024
// Licensed under the Apache License, Version 2.0

#include "nav2_bt_publish_goal/set_zone_mode_action.hpp"

namespace nav2_bt_publish_goal
{

SetZoneModeAction::SetZoneModeAction(
  const std::string & name,
  const BT::NodeConfig & config)
: BT::SyncActionNode(name, config)
{
  // node 在 tick 中从黑板获取
}

BT::PortsList SetZoneModeAction::providedPorts()
{
  return {
    BT::InputPort<rclcpp::Node::SharedPtr>("node", "ROS node"),
    BT::InputPort<int>("mode", "区模式: 1=一区(抓头/对接), 2=二区(台阶/抓块), 3=三区(预留)"),
    BT::InputPort<std::string>(
      "topic", "/AT_R2/zone_mode", "模式发布话题 (std_msgs/Int32)")
  };
}

BT::NodeStatus SetZoneModeAction::tick()
{
  // 第一次获取 ROS node
  if (!node_) {
    if (!getInput("node", node_) || !node_) {
      RCLCPP_ERROR(
        rclcpp::get_logger("SetZoneModeAction"), "Missing required input [node]");
      return BT::NodeStatus::FAILURE;
    }
  }

  // 读取模式
  int mode = 0;
  if (!getInput("mode", mode)) {
    RCLCPP_ERROR(node_->get_logger(), "Missing required input [mode]");
    return BT::NodeStatus::FAILURE;
  }
  if (mode < 1 || mode > 3) {
    RCLCPP_WARN(
      node_->get_logger(),
      "zone_mode=%d 不在预期范围 [1,3] (1一区/2二区/3三区)", mode);
  }

  // 按需(首次或话题变更)创建 latched publisher
  std::string topic = "/AT_R2/zone_mode";
  getInput("topic", topic);
  if (!mode_pub_ || topic != topic_) {
    auto qos = rclcpp::QoS(1).transient_local().reliable();
    mode_pub_ = node_->create_publisher<std_msgs::msg::Int32>(topic, qos);
    topic_ = topic;
    RCLCPP_INFO(node_->get_logger(), "zone_mode 话题: %s", topic_.c_str());
  }

  // 发布模式
  std_msgs::msg::Int32 msg;
  msg.data = mode;
  mode_pub_->publish(msg);
  RCLCPP_INFO(
    node_->get_logger(), "已切换区模式: %d (%s)", mode,
    mode == 1 ? "一区: 抓武器头/对接"
    : mode == 2 ? "二区: 上下台阶/抓块"
    : mode == 3 ? "三区: 预留" : "未知");

  return BT::NodeStatus::SUCCESS;
}

}  // namespace nav2_bt_publish_goal

// Register the node with BehaviorTree.CPP v4
#include "behaviortree_cpp/bt_factory.h"

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_bt_publish_goal::SetZoneModeAction>("SetZoneMode");
}
