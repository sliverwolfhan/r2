// Copyright 2024
// Licensed under the Apache License, Version 2.0

#include "nav2_bt_publish_goal/dock_to_wall_action.hpp"

#include <chrono>
#include <cmath>
#include <functional>

#include "behaviortree_cpp/bt_factory.h"

namespace nav2_bt_publish_goal
{

using namespace std::chrono_literals;

DockToWallAction::DockToWallAction(
  const std::string & name,
  const BT::NodeConfig & config)
: BT::StatefulActionNode(name, config),
  start_time_(0, 0, RCL_ROS_TIME),
  stall_start_time_(0, 0, RCL_ROS_TIME),
  last_odom_time_(0, 0, RCL_ROS_TIME)
{
}

BT::PortsList DockToWallAction::providedPorts()
{
  return {
    BT::InputPort<rclcpp::Node::SharedPtr>("node", "ROS node"),
    BT::InputPort<double>("vy", 0.1, "Lateral velocity y (m/s), positive=left"),
    BT::InputPort<double>("vx", 0.0, "Forward velocity x (m/s)"),
    BT::InputPort<double>("wz", 0.0, "Angular velocity z (rad/s)"),
    BT::InputPort<double>("stall_threshold", 0.005,
      "Position change threshold (m) below which we consider stalled"),
    BT::InputPort<double>("stall_duration", 0.5,
      "Time (s) position must remain unchanged to confirm docked"),
    BT::InputPort<double>("timeout", 10.0, "Max time (s) before giving up"),
    BT::InputPort<double>("publish_rate_hz", 50.0, "cmd_vel publish rate (Hz)"),
    BT::InputPort<std::string>("cmd_vel_topic", "/AT_R2/cmd_vel_nav2_result",
      "Velocity command topic"),
    BT::InputPort<std::string>("odom_topic", "/AT_R2/odometry",
      "Odometry topic for position feedback"),
  };
}

BT::NodeStatus DockToWallAction::onStart()
{
  if (!node_) {
    if (!getInput("node", node_) || !node_) {
      RCLCPP_ERROR(rclcpp::get_logger("DockToWall"),
        "Missing/null required input [node]");
      return BT::NodeStatus::FAILURE;
    }
  }

  // 读取参数
  vy_ = 0.1;
  vx_ = 0.0;
  wz_ = 0.0;
  stall_threshold_ = 0.005;
  stall_duration_ = 0.5;
  timeout_ = 10.0;
  double publish_rate_hz = 50.0;
  cmd_vel_topic_ = "/AT_R2/cmd_vel_nav2_result";
  odom_topic_ = "/AT_R2/odometry";

  getInput("vy", vy_);
  getInput("vx", vx_);
  getInput("wz", wz_);
  getInput("stall_threshold", stall_threshold_);
  getInput("stall_duration", stall_duration_);
  getInput("timeout", timeout_);
  getInput("publish_rate_hz", publish_rate_hz);
  getInput("cmd_vel_topic", cmd_vel_topic_);
  getInput("odom_topic", odom_topic_);

  if (publish_rate_hz < 1.0) {
    publish_rate_hz = 1.0;
  }

  // 初始化状态
  is_stalling_ = false;
  odom_received_ = false;

  // 创建 publisher
  if (!cmd_vel_pub_) {
    cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
  }

  // 创建 odom 订阅
  odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
    odom_topic_, rclcpp::SensorDataQoS(),
    std::bind(&DockToWallAction::odomCallback, this, std::placeholders::_1));

  // 创建定时器发布速度
  const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz);
  const auto period_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(period);
  publish_timer_ = node_->create_wall_timer(
    period_ns,
    std::bind(&DockToWallAction::publishCmdTimerCallback, this));

  start_time_ = node_->now();

  RCLCPP_INFO(node_->get_logger(),
    "DockToWall started: vy=%.3f vx=%.3f wz=%.3f "
    "stall_thresh=%.4fm stall_dur=%.2fs timeout=%.1fs",
    vy_, vx_, wz_, stall_threshold_, stall_duration_, timeout_);

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus DockToWallAction::onRunning()
{
  std::lock_guard<std::mutex> lock(mutex_);

  auto now = node_->now();

  // 超时检查
  if ((now - start_time_).seconds() > timeout_) {
    stopAll();
    RCLCPP_WARN(node_->get_logger(),
      "DockToWall timed out after %.1fs without confirming dock", timeout_);
    return BT::NodeStatus::FAILURE;
  }

  // 还没收到odom，继续等待
  if (!odom_received_) {
    return BT::NodeStatus::RUNNING;
  }

  // 检查是否已经持续stall足够长时间
  if (is_stalling_) {
    double stall_elapsed = (now - stall_start_time_).seconds();
    if (stall_elapsed >= stall_duration_) {
      stopAll();
      RCLCPP_INFO(node_->get_logger(),
        "DockToWall SUCCESS: position stalled for %.2fs, docked!",
        stall_elapsed);
      return BT::NodeStatus::SUCCESS;
    }
  }

  return BT::NodeStatus::RUNNING;
}

void DockToWallAction::onHalted()
{
  stopAll();
  RCLCPP_INFO(node_->get_logger(), "DockToWall halted");
}

void DockToWallAction::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);

  double cur_x = msg->pose.pose.position.x;
  double cur_y = msg->pose.pose.position.y;
  auto now = node_->now();

  if (!odom_received_) {
    // 第一帧，初始化
    last_x_ = cur_x;
    last_y_ = cur_y;
    last_odom_time_ = now;
    odom_received_ = true;
    return;
  }

  // 计算与上次检查的位移
  double dt = (now - last_odom_time_).seconds();
  if (dt < 0.1) {
    // 至少 100ms 采样一次，避免噪声
    return;
  }

  double dx = cur_x - last_x_;
  double dy = cur_y - last_y_;
  double displacement = std::sqrt(dx * dx + dy * dy);

  // 更新记录
  last_x_ = cur_x;
  last_y_ = cur_y;
  last_odom_time_ = now;

  // 判断是否stall (位移小于阈值)
  if (displacement < stall_threshold_) {
    if (!is_stalling_) {
      is_stalling_ = true;
      stall_start_time_ = now;
      RCLCPP_DEBUG(node_->get_logger(),
        "DockToWall: stall detected, displacement=%.4fm", displacement);
    }
    // 已经在stall中，继续等待stall_duration
  } else {
    // 还在移动，重置stall状态
    if (is_stalling_) {
      RCLCPP_DEBUG(node_->get_logger(),
        "DockToWall: stall reset, displacement=%.4fm", displacement);
    }
    is_stalling_ = false;
  }
}

void DockToWallAction::publishCmdTimerCallback()
{
  if (!cmd_vel_pub_) {
    return;
  }
  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = vx_;
  cmd.linear.y = vy_;
  cmd.angular.z = wz_;
  cmd_vel_pub_->publish(cmd);
}

void DockToWallAction::publishZero()
{
  if (!cmd_vel_pub_) {
    return;
  }
  geometry_msgs::msg::Twist zero;
  cmd_vel_pub_->publish(zero);
}

void DockToWallAction::stopAll()
{
  if (publish_timer_) {
    publish_timer_->cancel();
    publish_timer_.reset();
  }
  publishZero();
  if (odom_sub_) {
    odom_sub_.reset();
  }
}

}  // namespace nav2_bt_publish_goal

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_bt_publish_goal::DockToWallAction>("DockToWall");
}
