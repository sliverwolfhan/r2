// Copyright 2024
// Licensed under the Apache License, Version 2.0

#include "nav2_bt_publish_goal/wait_docking_release_action.hpp"

#include <functional>

#include "behaviortree_cpp/bt_factory.h"

namespace nav2_bt_publish_goal
{

WaitDockingReleaseAction::WaitDockingReleaseAction(
  const std::string & name,
  const BT::NodeConfig & config)
: BT::StatefulActionNode(name, config)
{
}

BT::PortsList WaitDockingReleaseAction::providedPorts()
{
  return {
    BT::InputPort<rclcpp::Node::SharedPtr>("node", "ROS node"),
    BT::InputPort<std::string>("status_topic", "/AT_R2/docking_status",
      "Topic to subscribe for docking status (Int32)"),
    BT::InputPort<std::string>("gripper_topic", "/AT_R2/head_gripper_cmd",
      "Topic to publish gripper release command (Int32)"),
    BT::InputPort<int32_t>("target_status", 1,
      "(已废弃) 兼容旧端口, 不再使用; 现固定匹配 1/2/3 任一值"),
    BT::InputPort<int32_t>("release_cmd", 5,
      "Command value to publish for release"),
    BT::InputPort<double>("timeout", 30.0, "Timeout in seconds"),
    BT::OutputPort<int32_t>("finished_all",
      "收到结束信号(docking_status==3)时置 1, 否则 0; 供外层判断是否结束循环"),
  };
}

BT::NodeStatus WaitDockingReleaseAction::onStart()
{
  // 获取 node
  if (!node_) {
    if (!getInput("node", node_) || !node_) {
      RCLCPP_ERROR(rclcpp::get_logger("WaitDockingRelease"),
        "Missing/null required input [node]");
      return BT::NodeStatus::FAILURE;
    }
  }

  // 读取参数
  status_topic_ = "/AT_R2/docking_status";
  gripper_topic_ = "/AT_R2/head_gripper_cmd";
  target_status_ = 1;
  release_cmd_ = 5;
  timeout_ = 30.0;

  getInput("status_topic", status_topic_);
  getInput("gripper_topic", gripper_topic_);
  getInput("target_status", target_status_);
  getInput("release_cmd", release_cmd_);
  getInput("timeout", timeout_);

  // 初始化状态
  docking_confirmed_ = false;
  finish_received_ = false;
  setOutput("finished_all", static_cast<int32_t>(0));
  start_time_ = node_->now();

  // 创建订阅者
  status_sub_ = node_->create_subscription<std_msgs::msg::Int32>(
    status_topic_, rclcpp::QoS(10).reliable(),
    std::bind(&WaitDockingReleaseAction::statusCallback, this, std::placeholders::_1));

  // 创建发布者 (transient_local 确保后启动的节点也能收到)
  gripper_pub_ = node_->create_publisher<std_msgs::msg::Int32>(
    gripper_topic_, rclcpp::QoS(1).transient_local().reliable());

  RCLCPP_INFO(node_->get_logger(),
    "WaitDockingRelease started: waiting for %s in {1,2,3}, will publish %s = %d "
    "(收到 %d 则结束整棵树)",
    status_topic_.c_str(), gripper_topic_.c_str(), release_cmd_, kFinishStatus);

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus WaitDockingReleaseAction::onRunning()
{
  std::lock_guard<std::mutex> lock(mutex_);

  auto now = node_->now();

  // 超时检查
  if ((now - start_time_).seconds() > timeout_) {
    status_sub_.reset();
    RCLCPP_WARN(node_->get_logger(),
      "WaitDockingRelease timed out after %.1fs", timeout_);
    return BT::NodeStatus::FAILURE;
  }

  // 检查是否收到对接完成信号(1/2/3 任一)
  if (docking_confirmed_) {
    // 发布松开指令
    auto msg = std_msgs::msg::Int32();
    msg.data = release_cmd_;
    gripper_pub_->publish(msg);

    // 收到 3 -> 结束整棵树; 否则继续下一轮
    setOutput("finished_all", static_cast<int32_t>(finish_received_ ? 1 : 0));

    status_sub_.reset();
    RCLCPP_INFO(node_->get_logger(),
      "WaitDockingRelease SUCCESS: docking done, published release cmd=%d, finished_all=%d",
      release_cmd_, finish_received_ ? 1 : 0);
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::RUNNING;
}

void WaitDockingReleaseAction::onHalted()
{
  if (status_sub_) {
    status_sub_.reset();
  }
  RCLCPP_INFO(node_->get_logger(), "WaitDockingRelease halted");
}

void WaitDockingReleaseAction::statusCallback(const std_msgs::msg::Int32::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);

  // 收到 1/2/3 任一值即视为对接完成; 3 额外标记为"结束"信号
  if (msg->data == 1 || msg->data == 2 || msg->data == kFinishStatus) {
    docking_confirmed_ = true;
    if (msg->data == kFinishStatus) {
      finish_received_ = true;
    }
    RCLCPP_DEBUG(node_->get_logger(),
      "WaitDockingRelease: received docking status %d", msg->data);
  }
}

}  // namespace nav2_bt_publish_goal

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_bt_publish_goal::WaitDockingReleaseAction>("WaitDockingRelease");
}
