// Copyright 2024
// Licensed under the Apache License, Version 2.0

#include "nav2_bt_publish_goal/set_arm_payload_action.hpp"

#include "behaviortree_cpp/bt_factory.h"

namespace nav2_bt_publish_goal
{

SetArmPayloadAction::SetArmPayloadAction(
  const std::string & name,
  const BT::NodeConfig & config)
: BT::StatefulActionNode(name, config)
{
}

BT::PortsList SetArmPayloadAction::providedPorts()
{
  return {
    BT::InputPort<rclcpp::Node::SharedPtr>("node", "ROS node"),
    BT::InputPort<std::string>(
      "arm_node", "arm_calc_node",
      "目标 arm_calc 节点名 (含命名空间, 如 /AT_R2/arm_calc_node)"),
    BT::InputPort<bool>(
      "payload", false,
      "true=带块模型(payload), false=不带块模型(nominal)"),
    BT::InputPort<double>("service_timeout", 2.0, "等待 set_parameters 服务的超时秒数"),
    BT::InputPort<double>("wait_after_set", 0.2, "设置成功后额外等待秒数, 让切换生效"),
  };
}

BT::NodeStatus SetArmPayloadAction::onStart()
{
  if (!node_) {
    if (!getInput("node", node_) || !node_) {
      RCLCPP_ERROR(
        rclcpp::get_logger("SetArmPayloadAction"), "Missing/null required input [node]");
      return BT::NodeStatus::FAILURE;
    }
  }

  arm_node_name_ = "arm_calc_node";
  payload_ = false;
  getInput("arm_node", arm_node_name_);
  getInput("payload", payload_);

  client_ = std::make_shared<rclcpp::AsyncParametersClient>(node_, arm_node_name_);

  double service_timeout_s = 2.0;
  getInput("service_timeout", service_timeout_s);

  request_sent_ = false;
  finished_ = false;

  const auto timeout = std::chrono::milliseconds(static_cast<int>(service_timeout_s * 1000));
  if (!client_->service_is_ready() && !client_->wait_for_service(timeout)) {
    RCLCPP_WARN(
      node_->get_logger(),
      "SetArmPayload: set_parameters service of '%s' not available within %.1fs, skipping",
      arm_node_name_.c_str(), service_timeout_s);
    finished_ = true;  // 服务不可用: 跳过但不失败
  } else {
    future_ = client_->set_parameters({rclcpp::Parameter("use_payload", payload_)});
    request_sent_ = true;
  }

  double wait_after_set_s = 0.2;
  getInput("wait_after_set", wait_after_set_s);
  wait_after_set_ = rclcpp::Duration::from_seconds(wait_after_set_s);
  start_time_ = node_->now();

  RCLCPP_INFO(
    node_->get_logger(), "SetArmPayload: 请求将 '%s' 的 use_payload 设为 %s (%s模型)",
    arm_node_name_.c_str(), payload_ ? "true" : "false", payload_ ? "带块" : "不带块");

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus SetArmPayloadAction::onRunning()
{
  if (request_sent_ && !finished_) {
    if (future_.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
      return BT::NodeStatus::RUNNING;
    }
    const auto results = future_.get();
    if (results.empty() || !results.front().successful) {
      RCLCPP_WARN(
        node_->get_logger(), "SetArmPayload: 设置 use_payload 失败: %s",
        results.empty() ? "no result" : results.front().reason.c_str());
    }
    finished_ = true;
  }

  if ((node_->now() - start_time_) < wait_after_set_) {
    return BT::NodeStatus::RUNNING;
  }

  RCLCPP_INFO(node_->get_logger(), "SetArmPayload: done");
  return BT::NodeStatus::SUCCESS;
}

void SetArmPayloadAction::onHalted()
{
  RCLCPP_INFO(node_->get_logger(), "SetArmPayload halted");
}

}  // namespace nav2_bt_publish_goal

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_bt_publish_goal::SetArmPayloadAction>("SetArmPayload");
}
