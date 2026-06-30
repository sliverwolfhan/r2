// Copyright 2024
// Licensed under the Apache License, Version 2.0

#include "nav2_bt_publish_goal/publish_lift_cmd_action.hpp"

namespace nav2_bt_publish_goal
{

PublishLiftCmdAction::PublishLiftCmdAction(
  const std::string & name,
  const BT::NodeConfig & config)
: BT::StatefulActionNode(name, config)
{
  // node 在 onStart 中从黑板获取
}

BT::PortsList PublishLiftCmdAction::providedPorts()
{
  return {
    BT::InputPort<rclcpp::Node::SharedPtr>("node", "ROS node"),
    BT::InputPort<int>(
      "command", "整数指令: 1=抬升, 2=降到架机, 3=抬腿, 4=伸腿, 5=降下去"),
    BT::InputPort<std::string>(
      "topic", "/AT_R2/lift_cmd", "指令发布话题 (std_msgs/Int32)"),
    BT::InputPort<bool>(
      "wait_done", false, "是否等待 done_topic 反馈本次 command 后才成功"),
    BT::InputPort<std::string>(
      "done_topic", "/AT_R2/lift_done", "完成反馈话题 (std_msgs/Int32)"),
    BT::InputPort<double>(
      "timeout", 0.0, "等待反馈的超时秒数, <=0 表示无限等待")
  };
}

BT::NodeStatus PublishLiftCmdAction::onStart()
{
  // 第一次获取 ROS node
  if (!node_) {
    if (!getInput("node", node_) || !node_) {
      RCLCPP_ERROR(
        rclcpp::get_logger("PublishLiftCmdAction"), "Missing required input [node]");
      return BT::NodeStatus::FAILURE;
    }
  }

  // 读取指令
  if (!getInput("command", command_)) {
    RCLCPP_ERROR(node_->get_logger(), "Missing required input [command]");
    return BT::NodeStatus::FAILURE;
  }
  if (command_ < 1 || command_ > 5) {
    RCLCPP_WARN(
      node_->get_logger(),
      "lift command=%d 不在预期范围 [1,5] (1抬升/2降到架机/3抬腿/4伸腿/5降下去)", command_);
  }

  // 读取话题与同步参数
  std::string topic = "/AT_R2/lift_cmd";
  getInput("topic", topic);
  wait_done_ = false;
  getInput("wait_done", wait_done_);
  timeout_ = 0.0;
  getInput("timeout", timeout_);
  std::string done_topic = "/AT_R2/lift_done";
  getInput("done_topic", done_topic);

  // 按需(首次或话题变更)创建 latched publisher
  if (!cmd_pub_ || topic != cmd_topic_) {
    auto qos = rclcpp::QoS(1).transient_local().reliable();
    cmd_pub_ = node_->create_publisher<std_msgs::msg::Int32>(topic, qos);
    cmd_topic_ = topic;
    RCLCPP_INFO(node_->get_logger(), "lift 指令话题: %s", cmd_topic_.c_str());
  }

  // 需要等待反馈时, 按需创建订阅
  done_received_ = false;
  done_value_ = 0;
  if (wait_done_) {
    if (!done_sub_ || done_topic != done_topic_) {
      done_sub_ = node_->create_subscription<std_msgs::msg::Int32>(
        done_topic, 10,
        std::bind(&PublishLiftCmdAction::doneCallback, this, std::placeholders::_1));
      done_topic_ = done_topic;
      RCLCPP_INFO(node_->get_logger(), "lift 反馈话题: %s", done_topic_.c_str());
    }
  }

  // 发布指令
  std_msgs::msg::Int32 msg;
  msg.data = command_;
  cmd_pub_->publish(msg);
  RCLCPP_INFO(
    node_->get_logger(), "已发布 lift 指令: %d (%s)", command_,
    command_ == 1 ? "抬升" : command_ == 2 ? "降到架机" : command_ == 3 ? "抬腿" :
    command_ == 4 ? "伸腿" : command_ == 5 ? "降下去" : "未知");

  if (!wait_done_) {
    return BT::NodeStatus::SUCCESS;
  }

  start_time_ = node_->now();
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus PublishLiftCmdAction::onRunning()
{
  // 收到与本次指令匹配的反馈即成功
  if (done_received_ && done_value_ == command_) {
    RCLCPP_INFO(node_->get_logger(), "✓ lift 动作 %d 完成", command_);
    return BT::NodeStatus::SUCCESS;
  }

  // 超时检查
  if (timeout_ > 0.0) {
    const double elapsed = (node_->now() - start_time_).seconds();
    if (elapsed >= timeout_) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "✗ 等待 lift 动作 %d 反馈超时 (%.1fs)", command_, timeout_);
      return BT::NodeStatus::FAILURE;
    }
  }

  return BT::NodeStatus::RUNNING;
}

void PublishLiftCmdAction::onHalted()
{
  RCLCPP_INFO(node_->get_logger(), "PublishLiftCmd 被中断 (command=%d)", command_);
  done_received_ = false;
  done_value_ = 0;
}

void PublishLiftCmdAction::doneCallback(const std_msgs::msg::Int32::SharedPtr msg)
{
  done_value_ = msg->data;
  done_received_ = true;
  RCLCPP_DEBUG(node_->get_logger(), "收到 lift 反馈: %d", done_value_);
}

}  // namespace nav2_bt_publish_goal

// Register the node with BehaviorTree.CPP v4
#include "behaviortree_cpp/bt_factory.h"

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_bt_publish_goal::PublishLiftCmdAction>("PublishLiftCmd");
}
