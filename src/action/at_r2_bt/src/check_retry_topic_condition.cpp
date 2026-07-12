// Copyright 2024
// Licensed under the Apache License, Version 2.0

#include "nav2_bt_publish_goal/check_retry_topic_condition.hpp"

#include "behaviortree_cpp/bt_factory.h"

namespace nav2_bt_publish_goal
{

// 两次 tick 间隔超过此值 (s) 视为"重新进入顶墙", 首个 tick 先排空陈旧值。
// 主循环 20Hz -> 正常连续 tick 间隔 50ms; 150ms 足以区分连续与重入。
static constexpr double kReentryGapSec = 0.15;

CheckRetryTopicCondition::CheckRetryTopicCondition(
  const std::string & name, const BT::NodeConfig & config)
: BT::ConditionNode(name, config),
  last_tick_time_(0, 0, RCL_ROS_TIME)
{
}

BT::PortsList CheckRetryTopicCondition::providedPorts()
{
  return {
    BT::InputPort<rclcpp::Node::SharedPtr>("node", "ROS node"),
    BT::InputPort<std::string>("topic", "/AT_R2/place_retry",
      "放块顶墙中断话题 (std_msgs/Int32): 1/2/3/4 触发退回重试"),
    BT::OutputPort<int>("retry_val",
      "中断触发时锁存的值 (1/2/3/4), 供恢复段分流"),
  };
}

bool CheckRetryTopicCondition::ensureInitialized()
{
  if (initialized_) {
    return true;
  }
  if (!getInput("node", node_) || !node_) {
    RCLCPP_ERROR(rclcpp::get_logger("CheckRetryTopic"),
      "Missing/null required input [node]");
    return false;
  }

  std::string topic = "/AT_R2/place_retry";
  getInput("topic", topic);

  sub_ = node_->create_subscription<std_msgs::msg::Int32>(
    topic, rclcpp::QoS(10).reliable(),
    std::bind(&CheckRetryTopicCondition::retryCallback, this, std::placeholders::_1));

  latest_.store(0);   // 丢弃订阅建立前可能到达的残留
  initialized_ = true;

  RCLCPP_INFO(node_->get_logger(),
    "CheckRetryTopic ready (topic %s)", topic.c_str());
  return true;
}

BT::NodeStatus CheckRetryTopicCondition::tick()
{
  if (!ensureInitialized()) {
    return BT::NodeStatus::FAILURE;
  }

  const rclcpp::Time now = node_->now();

  // 判断是否"重新进入": 与上一 tick 不连续 (间隔过大, 说明本节点上一 tick 未被激活)。
  const bool reentered =
    !have_last_tick_ || (now - last_tick_time_).seconds() > kReentryGapSec;
  last_tick_time_ = now;
  have_last_tick_ = true;

  if (reentered) {
    // 重新进入顶墙的首个 tick: 排空陈旧值, 放行本轮顶墙。
    latest_.store(0);
    return BT::NodeStatus::SUCCESS;
  }

  // 消费即清: 一次发布 = 一次中断。
  const int v = latest_.exchange(0);
  if (v >= 1 && v <= 4) {
    setOutput<int>("retry_val", v);
    RCLCPP_INFO(node_->get_logger(),
      "CheckRetryTopic: 收到中断值 %d, 打断顶墙, 退回重试", v);
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::SUCCESS;
}

void CheckRetryTopicCondition::retryCallback(const std_msgs::msg::Int32::SharedPtr msg)
{
  const int v = msg->data;
  // 只接受 1/2/3/4, 其它 (含 0) 忽略。
  if (v >= 1 && v <= 4) {
    latest_.store(v);
  }
}

}  // namespace nav2_bt_publish_goal

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_bt_publish_goal::CheckRetryTopicCondition>("CheckRetryTopic");
}
