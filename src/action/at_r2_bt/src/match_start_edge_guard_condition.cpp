// Copyright 2024
// Licensed under the Apache License, Version 2.0

#include "nav2_bt_publish_goal/match_start_edge_guard_condition.hpp"

#include "behaviortree_cpp/bt_factory.h"

namespace nav2_bt_publish_goal
{

MatchStartEdgeGuardCondition::MatchStartEdgeGuardCondition(
  const std::string & name, const BT::NodeConfig & config)
: BT::ConditionNode(name, config),
  red_latch_time_(0, 0, RCL_ROS_TIME),
  blue_latch_time_(0, 0, RCL_ROS_TIME)
{
}

BT::PortsList MatchStartEdgeGuardCondition::providedPorts()
{
  return {
    BT::InputPort<rclcpp::Node::SharedPtr>("node", "ROS node"),
    BT::InputPort<std::string>("red_topic", "/AT_R2/red_start_cmd",
      "红方启动话题 (std_msgs/Int32): 跳到 red_trigger_value 的下降沿参与触发"),
    BT::InputPort<std::string>("blue_topic", "/AT_R2/blue_start_cmd",
      "蓝方启动话题 (std_msgs/Int32): 跳到 blue_trigger_value 的上升沿参与触发"),
    BT::InputPort<int>("red_trigger_value", 0,
      "red_topic 跳到该值算触发边沿 (默认 0)"),
    BT::InputPort<int>("blue_trigger_value", 1,
      "blue_topic 跳到该值算触发边沿 (默认 1)"),
    BT::InputPort<double>("edge_window", 1.0,
      "两话题边沿必须在此秒数内先后到达才算联合触发; 先到的边沿超时后作废 (默认 1.0s)"),
  };
}

bool MatchStartEdgeGuardCondition::ensureInitialized()
{
  if (initialized_) {
    return true;
  }
  if (!getInput("node", node_) || !node_) {
    RCLCPP_ERROR(rclcpp::get_logger("MatchStartEdgeGuard"),
      "Missing/null required input [node]");
    return false;
  }

  std::string red_topic = "/AT_R2/red_start_cmd";
  std::string blue_topic = "/AT_R2/blue_start_cmd";
  getInput("red_topic", red_topic);
  getInput("blue_topic", blue_topic);
  getInput("red_trigger_value", red_trig_);
  getInput("blue_trigger_value", blue_trig_);
  getInput("edge_window", edge_window_);
  if (edge_window_ < 0.0) {
    edge_window_ = 0.0;
  }

  red_sub_ = node_->create_subscription<std_msgs::msg::Int32>(
    red_topic, rclcpp::QoS(10).reliable(),
    std::bind(&MatchStartEdgeGuardCondition::redCallback, this, std::placeholders::_1));
  blue_sub_ = node_->create_subscription<std_msgs::msg::Int32>(
    blue_topic, rclcpp::QoS(10).reliable(),
    std::bind(&MatchStartEdgeGuardCondition::blueCallback, this, std::placeholders::_1));

  initialized_ = true;

  RCLCPP_INFO(node_->get_logger(),
    "MatchStartEdgeGuard ready (red %s->%d 下降沿 && blue %s->%d 上升沿 触发重来)",
    red_topic.c_str(), red_trig_, blue_topic.c_str(), blue_trig_);
  return true;
}

BT::NodeStatus MatchStartEdgeGuardCondition::tick()
{
  if (!ensureInitialized()) {
    return BT::NodeStatus::FAILURE;
  }

  const rclcpp::Time now = node_->now();

  // red 边沿判定: 首值只记基线不判沿 (防上电误触发)。
  const int r = red_latest_.load();
  if (r != INT_MIN) {
    if (!have_prev_red_) {
      prev_red_ = r;
      have_prev_red_ = true;
    } else {
      if (prev_red_ != red_trig_ && r == red_trig_ && !red_edge_latched_) {
        red_edge_latched_ = true;
        red_latch_time_ = now;   // 记锁存时刻, 供时间窗判定
      }
      prev_red_ = r;
    }
  }

  // blue 边沿判定: 同理。
  const int b = blue_latest_.load();
  if (b != INT_MIN) {
    if (!have_prev_blue_) {
      prev_blue_ = b;
      have_prev_blue_ = true;
    } else {
      if (prev_blue_ != blue_trig_ && b == blue_trig_ && !blue_edge_latched_) {
        blue_edge_latched_ = true;
        blue_latch_time_ = now;
      }
      prev_blue_ = b;
    }
  }

  // 时间窗: 只有一个 latch 成立时, 若它已超过 edge_window_ 秒仍等不到另一个, 作废重来。
  // (两个都成立时下面立即触发, 不走这里; edge_window_<=0 时视为无限窗, 不作废。)
  if (edge_window_ > 0.0) {
    if (red_edge_latched_ && !blue_edge_latched_ &&
        (now - red_latch_time_).seconds() > edge_window_) {
      red_edge_latched_ = false;
      RCLCPP_INFO(node_->get_logger(),
        "MatchStartEdgeGuard: red 边沿等待 blue 超过 %.2fs 未到, 作废该 red 边沿", edge_window_);
    }
    if (blue_edge_latched_ && !red_edge_latched_ &&
        (now - blue_latch_time_).seconds() > edge_window_) {
      blue_edge_latched_ = false;
      RCLCPP_INFO(node_->get_logger(),
        "MatchStartEdgeGuard: blue 边沿等待 red 超过 %.2fs 未到, 作废该 blue 边沿", edge_window_);
    }
  }

  // 两 latch 同时成立 -> 中断; 触发后清 latch 防重复。
  if (red_edge_latched_ && blue_edge_latched_) {
    red_edge_latched_ = false;
    blue_edge_latched_ = false;
    RCLCPP_WARN(node_->get_logger(),
      "MatchStartEdgeGuard: red->%d && blue->%d 联合跳变沿, 中断当前流程, 触发重来",
      red_trig_, blue_trig_);
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::SUCCESS;
}

void MatchStartEdgeGuardCondition::redCallback(const std_msgs::msg::Int32::SharedPtr msg)
{
  red_latest_.store(msg->data);
}

void MatchStartEdgeGuardCondition::blueCallback(const std_msgs::msg::Int32::SharedPtr msg)
{
  blue_latest_.store(msg->data);
}

}  // namespace nav2_bt_publish_goal

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_bt_publish_goal::MatchStartEdgeGuardCondition>(
    "MatchStartEdgeGuard");
}
