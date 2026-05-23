// Copyright 2024
// Licensed under the Apache License, Version 2.0

#include <chrono>
#include <functional>

#include "nav2_bt_publish_goal/pre_steer_align_action.hpp"

namespace nav2_bt_publish_goal
{

void PreSteerAlignAction::stopPublishTimer()
{
  std::lock_guard<std::mutex> lock(publish_timer_mutex_);
  publish_timer_armed_ = false;
  if (publish_timer_) {
    publish_timer_->cancel();
    publish_timer_.reset();
  }
}

void PreSteerAlignAction::publishCmdTimerCallback()
{
  std::lock_guard<std::mutex> lock(publish_timer_mutex_);
  if (!publish_timer_armed_ || !cmd_vel_pub_) {
    return;
  }
  cmd_vel_pub_->publish(cmd_);
}

PreSteerAlignAction::PreSteerAlignAction(
  const std::string & name,
  const BT::NodeConfig & config)
: BT::StatefulActionNode(name, config),
  start_time_(0, 0, RCL_ROS_TIME),
  duration_(rclcpp::Duration::from_seconds(0.0))
{
  // 不在构造函数中获取 node，而是在 onStart 中获取
}

BT::PortsList PreSteerAlignAction::providedPorts()
{
  return {
    BT::InputPort<rclcpp::Node::SharedPtr>("node", "ROS node"),
    BT::InputPort<double>("vx", 0.05, "Linear velocity x (m/s)"),
    BT::InputPort<double>("vy", 0.0, "Linear velocity y (m/s)"),
    BT::InputPort<double>("wz", 0.0, "Angular velocity z (rad/s)"),
    BT::InputPort<double>("duration", 0.2, "How long to keep publishing the velocity (s)"),
    BT::InputPort<double>("publish_rate_hz", 50.0, "Fixed cmd_vel publish rate (Hz) while aligning"),
    BT::InputPort<std::string>("topic", "/AT_R2/cmd_vel_nav2_result", "Velocity topic to publish on")
  };
}

BT::NodeStatus PreSteerAlignAction::onStart()
{
  // 第一次进入时初始化 ROS 组件
  if (!node_) {
    if (!getInput("node", node_)) {
      RCLCPP_ERROR(rclcpp::get_logger("PreSteerAlignAction"),
        "Missing required input [node]");
      return BT::NodeStatus::FAILURE;
    }
    if (!node_) {
      RCLCPP_ERROR(rclcpp::get_logger("PreSteerAlignAction"),
        "Node pointer is null");
      return BT::NodeStatus::FAILURE;
    }
  }

  // 读取参数
  double vx = 0.05;
  double vy = 0.0;
  double wz = 0.0;
  double duration_s = 0.2;
  std::string topic = "/AT_R2/cmd_vel_nav2_result";

  getInput("vx", vx);
  getInput("vy", vy);
  getInput("wz", wz);
  getInput("duration", duration_s);
  getInput("topic", topic);

  double publish_rate_hz = 50.0;
  getInput("publish_rate_hz", publish_rate_hz);
  if (publish_rate_hz < 1.0) {
    publish_rate_hz = 1.0;
  }

  if (duration_s < 0.0) {
    RCLCPP_ERROR(node_->get_logger(),
      "Invalid duration: %.3f (must be >= 0)", duration_s);
    return BT::NodeStatus::FAILURE;
  }

  stopPublishTimer();

  // 话题变更时重建 publisher
  if (!cmd_vel_pub_ || topic != topic_name_) {
    cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(topic, 10);
    topic_name_ = topic;
    RCLCPP_INFO(node_->get_logger(),
      "PreSteerAlignAction publishing on topic: %s", topic_name_.c_str());
  }

  // 缓存本次的速度命令
  cmd_ = geometry_msgs::msg::Twist();
  cmd_.linear.x = vx;
  cmd_.linear.y = vy;
  cmd_.angular.z = wz;

  // 立即发一帧，随后由定时器按固定频率刷新
  cmd_vel_pub_->publish(cmd_);

  const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz);
  const auto period_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(period);
  publish_timer_ = node_->create_wall_timer(
    period_ns,
    std::bind(&PreSteerAlignAction::publishCmdTimerCallback, this));

  {
    std::lock_guard<std::mutex> lock(publish_timer_mutex_);
    publish_timer_armed_ = true;
  }

  start_time_ = node_->now();
  duration_ = rclcpp::Duration::from_seconds(duration_s);

  RCLCPP_INFO(node_->get_logger(),
    "PreSteerAlign started: vx=%.3f vy=%.3f wz=%.3f duration=%.2fs rate=%.1fHz",
    vx, vy, wz, duration_s, publish_rate_hz);

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus PreSteerAlignAction::onRunning()
{
  // 时间到了：停定时器并发一帧零速度兜底，返回 SUCCESS
  if ((node_->now() - start_time_) >= duration_) {
    stopPublishTimer();
    publishZero();
    RCLCPP_INFO(node_->get_logger(),
      "PreSteerAlign finished, sent zero velocity");
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::RUNNING;
}

void PreSteerAlignAction::onHalted()
{
  stopPublishTimer();
  // 中断时也要把车停稳
  publishZero();
  RCLCPP_INFO(node_->get_logger(), "PreSteerAlign halted, sent zero velocity");
}

void PreSteerAlignAction::publishZero()
{
  if (!cmd_vel_pub_) {
    return;
  }
  geometry_msgs::msg::Twist zero;
  cmd_vel_pub_->publish(zero);
}

}  // namespace nav2_bt_publish_goal

// Register the node with BehaviorTree.CPP v4
#include "behaviortree_cpp/bt_factory.h"

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_bt_publish_goal::PreSteerAlignAction>("PreSteerAlign");
}
