// Copyright 2024
// Licensed under the Apache License, Version 2.0

#include "nav2_bt_publish_goal/set_costmap_inflation_action.hpp"

#include <chrono>

namespace nav2_bt_publish_goal
{

namespace
{
constexpr const char * kInflationRadiusParam = "inflation_layer.inflation_radius";
constexpr const char * kCostScalingFactorParam = "inflation_layer.cost_scaling_factor";
}  // namespace

SetCostmapInflationAction::SetCostmapInflationAction(
  const std::string & name,
  const BT::NodeConfig & config)
: BT::StatefulActionNode(name, config),
  start_time_(0, 0, RCL_ROS_TIME),
  wait_after_set_(rclcpp::Duration::from_seconds(0.0))
{
}

BT::PortsList SetCostmapInflationAction::providedPorts()
{
  return {
    BT::InputPort<rclcpp::Node::SharedPtr>("node", "ROS node"),
    BT::InputPort<double>("local_radius", "Local costmap inflation_radius (optional)"),
    BT::InputPort<double>("global_radius", "Global costmap inflation_radius (optional)"),
    BT::InputPort<double>(
      "local_cost_scaling_factor", "Local costmap cost_scaling_factor (optional)"),
    BT::InputPort<double>(
      "global_cost_scaling_factor", "Global costmap cost_scaling_factor (optional)"),
    BT::InputPort<std::string>(
      "local_node", "/AT_R2/local_costmap/local_costmap", "Local costmap node fully-qualified name"),
    BT::InputPort<std::string>(
      "global_node", "/AT_R2/global_costmap/global_costmap",
      "Global costmap node fully-qualified name"),
    BT::InputPort<double>(
      "wait_after_set", 0.3, "Seconds to wait after set_parameters before SUCCESS"),
    BT::InputPort<double>("service_timeout", 2.0, "Seconds to wait for set_parameters service")
  };
}

void SetCostmapInflationAction::buildParams(
  Target & target,
  const std::string & port_radius_key,
  const std::string & port_scaling_key)
{
  target.params.clear();

  double radius = 0.0;
  if (getInput(port_radius_key, radius)) {
    target.params.emplace_back(kInflationRadiusParam, radius);
  }
  double scaling = 0.0;
  if (getInput(port_scaling_key, scaling)) {
    target.params.emplace_back(kCostScalingFactorParam, scaling);
  }

  target.valid = !target.params.empty();
}

void SetCostmapInflationAction::sendIfReady(Target & target, double service_timeout_s)
{
  if (!target.valid) {
    target.finished = true;
    return;
  }

  // 等待服务可用；如果超时，按"WARN + SUCCESS 继续走"策略放弃这个节点
  const auto timeout = std::chrono::milliseconds(
    static_cast<int>(service_timeout_s * 1000));
  if (!target.client->service_is_ready()) {
    if (!target.client->wait_for_service(timeout)) {
      RCLCPP_WARN(
        node_->get_logger(),
        "SetCostmapInflation: set_parameters service of '%s' not available within %.1fs, skipping",
        target.node_name.c_str(), service_timeout_s);
      target.valid = false;
      target.finished = true;
      return;
    }
  }

  target.future = target.client->set_parameters(target.params);
  target.finished = false;
}

void SetCostmapInflationAction::evaluate(Target & target)
{
  if (target.finished || !target.valid) {
    return;
  }

  // 非阻塞 check
  if (target.future.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
    return;
  }

  try {
    auto results = target.future.get();
    for (size_t i = 0; i < results.size() && i < target.params.size(); ++i) {
      if (!results[i].successful) {
        RCLCPP_WARN(
          node_->get_logger(),
          "SetCostmapInflation: failed to set '%s' on '%s': %s",
          target.params[i].get_name().c_str(),
          target.node_name.c_str(),
          results[i].reason.c_str());
      }
    }
  } catch (const std::exception & e) {
    RCLCPP_WARN(
      node_->get_logger(),
      "SetCostmapInflation: exception while setting params on '%s': %s",
      target.node_name.c_str(), e.what());
  }

  target.finished = true;
}

BT::NodeStatus SetCostmapInflationAction::onStart()
{
  // 第一次拉 ROS node
  if (!node_) {
    if (!getInput("node", node_)) {
      RCLCPP_ERROR(rclcpp::get_logger("SetCostmapInflationAction"),
        "Missing required input [node]");
      return BT::NodeStatus::FAILURE;
    }
    if (!node_) {
      RCLCPP_ERROR(rclcpp::get_logger("SetCostmapInflationAction"),
        "Node pointer is null");
      return BT::NodeStatus::FAILURE;
    }
  }

  // 节点名（必填，但有默认）
  std::string local_node_name = "/AT_R2/local_costmap/local_costmap";
  std::string global_node_name = "/AT_R2/global_costmap/global_costmap";
  getInput("local_node", local_node_name);
  getInput("global_node", global_node_name);

  // 节点名变更或第一次：重建客户端
  if (!local_.client || local_.node_name != local_node_name) {
    local_.client = std::make_shared<rclcpp::AsyncParametersClient>(node_, local_node_name);
    local_.node_name = local_node_name;
  }
  if (!global_.client || global_.node_name != global_node_name) {
    global_.client = std::make_shared<rclcpp::AsyncParametersClient>(node_, global_node_name);
    global_.node_name = global_node_name;
  }

  buildParams(local_, "local_radius", "local_cost_scaling_factor");
  buildParams(global_, "global_radius", "global_cost_scaling_factor");

  if (!local_.valid && !global_.valid) {
    RCLCPP_WARN(node_->get_logger(),
      "SetCostmapInflation: no parameters provided in any port, nothing to do");
    return BT::NodeStatus::SUCCESS;
  }

  double service_timeout_s = 2.0;
  getInput("service_timeout", service_timeout_s);

  sendIfReady(local_, service_timeout_s);
  sendIfReady(global_, service_timeout_s);

  double wait_after_set_s = 0.3;
  getInput("wait_after_set", wait_after_set_s);
  wait_after_set_ = rclcpp::Duration::from_seconds(wait_after_set_s);
  start_time_ = node_->now();

  RCLCPP_INFO(node_->get_logger(),
    "SetCostmapInflation: requested %zu local + %zu global params, waiting %.2fs after set",
    local_.valid ? local_.params.size() : 0,
    global_.valid ? global_.params.size() : 0,
    wait_after_set_s);

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus SetCostmapInflationAction::onRunning()
{
  evaluate(local_);
  evaluate(global_);

  // future 都处理完才考虑等待 wait_after_set
  if (!local_.finished || !global_.finished) {
    return BT::NodeStatus::RUNNING;
  }

  if ((node_->now() - start_time_) < wait_after_set_) {
    return BT::NodeStatus::RUNNING;
  }

  RCLCPP_INFO(node_->get_logger(), "SetCostmapInflation: done");
  return BT::NodeStatus::SUCCESS;
}

void SetCostmapInflationAction::onHalted()
{
  // 参数请求已发出，没必要也没法取消；只清状态
  local_.finished = true;
  global_.finished = true;
}

}  // namespace nav2_bt_publish_goal

#include "behaviortree_cpp/bt_factory.h"

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_bt_publish_goal::SetCostmapInflationAction>("SetCostmapInflation");
}
