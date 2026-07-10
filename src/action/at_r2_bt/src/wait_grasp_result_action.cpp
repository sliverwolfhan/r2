// Copyright 2024
// Licensed under the Apache License, Version 2.0

#include "nav2_bt_publish_goal/wait_grasp_result_action.hpp"

#include <functional>

#include "behaviortree_cpp/bt_factory.h"

namespace nav2_bt_publish_goal
{

WaitGraspResultAction::WaitGraspResultAction(
  const std::string & name,
  const BT::NodeConfig & config)
: BT::StatefulActionNode(name, config)
{
}

BT::PortsList WaitGraspResultAction::providedPorts()
{
  return {
    BT::InputPort<rclcpp::Node::SharedPtr>("node", "ROS node"),
    BT::InputPort<std::string>("topic_a", "/AT_R2/meilin_mission_start",
      "反馈话题 A (std_msgs/Int32)"),
    BT::InputPort<std::string>("topic_b", "/AT_R2/zone3_cmd",
      "反馈话题 B (std_msgs/Int32)"),
    BT::InputPort<int32_t>("success_a", 1, "成功组合中 topic_a 的取值"),
    BT::InputPort<int32_t>("success_b", 0, "成功组合中 topic_b 的取值"),
    BT::InputPort<int32_t>("fail_a", 0, "失败组合中 topic_a 的取值"),
    BT::InputPort<int32_t>("fail_b", 1, "失败组合中 topic_b 的取值"),
    BT::InputPort<double>("timeout", 0.0, "超时秒数, <=0 表示无限等待"),
    BT::InputPort<bool>("timeout_result", false,
      "超时返回值: true=SUCCESS, false=FAILURE(默认, 交外层重试)"),
  };
}

BT::NodeStatus WaitGraspResultAction::onStart()
{
  // 获取 node
  if (!node_) {
    if (!getInput("node", node_) || !node_) {
      RCLCPP_ERROR(rclcpp::get_logger("WaitGraspResult"),
        "Missing/null required input [node]");
      return BT::NodeStatus::FAILURE;
    }
  }

  // 读取参数(默认值)
  topic_a_ = "/AT_R2/meilin_mission_start";
  topic_b_ = "/AT_R2/zone3_cmd";
  success_a_ = 1;
  success_b_ = 0;
  fail_a_ = 0;
  fail_b_ = 1;
  timeout_ = 0.0;
  timeout_result_ = false;

  getInput("topic_a", topic_a_);
  getInput("topic_b", topic_b_);
  getInput("success_a", success_a_);
  getInput("success_b", success_b_);
  getInput("fail_a", fail_a_);
  getInput("fail_b", fail_b_);
  getInput("timeout", timeout_);
  getInput("timeout_result", timeout_result_);

  // 初始化状态(清空上一轮的最新值)
  has_a_ = false;
  has_b_ = false;
  last_a_ = 0;
  last_b_ = 0;
  start_time_ = node_->now();

  // 创建订阅者(每次进入都重建, 保证只接收本轮启动后到达的消息)
  sub_a_ = node_->create_subscription<std_msgs::msg::Int32>(
    topic_a_, rclcpp::QoS(10).reliable(),
    std::bind(&WaitGraspResultAction::topicACallback, this, std::placeholders::_1));
  sub_b_ = node_->create_subscription<std_msgs::msg::Int32>(
    topic_b_, rclcpp::QoS(10).reliable(),
    std::bind(&WaitGraspResultAction::topicBCallback, this, std::placeholders::_1));

  RCLCPP_INFO(node_->get_logger(),
    "WaitGraspResult started: 成功=(%s==%d 且 %s==%d) / 失败=(%s==%d 且 %s==%d) (timeout=%.1f)",
    topic_a_.c_str(), success_a_, topic_b_.c_str(), success_b_,
    topic_a_.c_str(), fail_a_, topic_b_.c_str(), fail_b_, timeout_);

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus WaitGraspResultAction::onRunning()
{
  std::lock_guard<std::mutex> lock(mutex_);

  // 需两话题都收到过, 才用最新值组合判定
  if (has_a_ && has_b_) {
    // 成功组合
    if (last_a_ == success_a_ && last_b_ == success_b_) {
      sub_a_.reset();
      sub_b_.reset();
      RCLCPP_INFO(node_->get_logger(),
        "✓ WaitGraspResult: 抓取成功 (%s=%d, %s=%d)",
        topic_a_.c_str(), last_a_, topic_b_.c_str(), last_b_);
      return BT::NodeStatus::SUCCESS;
    }
    // 失败组合
    if (last_a_ == fail_a_ && last_b_ == fail_b_) {
      sub_a_.reset();
      sub_b_.reset();
      RCLCPP_WARN(node_->get_logger(),
        "✗ WaitGraspResult: 抓取失败 (%s=%d, %s=%d)",
        topic_a_.c_str(), last_a_, topic_b_.c_str(), last_b_);
      return BT::NodeStatus::FAILURE;
    }
    // 其它组合 -> 继续等待
  }

  // 超时
  if (timeout_ > 0.0) {
    const double elapsed = (node_->now() - start_time_).seconds();
    if (elapsed >= timeout_) {
      sub_a_.reset();
      sub_b_.reset();
      RCLCPP_WARN(node_->get_logger(),
        "WaitGraspResult 超时 %.1fs, 按 %s 处理",
        timeout_, timeout_result_ ? "SUCCESS" : "FAILURE");
      return timeout_result_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }
  }

  return BT::NodeStatus::RUNNING;
}

void WaitGraspResultAction::onHalted()
{
  sub_a_.reset();
  sub_b_.reset();
  RCLCPP_INFO(node_->get_logger(), "WaitGraspResult halted");
}

void WaitGraspResultAction::topicACallback(const std_msgs::msg::Int32::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  last_a_ = msg->data;
  has_a_ = true;
}

void WaitGraspResultAction::topicBCallback(const std_msgs::msg::Int32::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  last_b_ = msg->data;
  has_b_ = true;
}

}  // namespace nav2_bt_publish_goal

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_bt_publish_goal::WaitGraspResultAction>("WaitGraspResult");
}
