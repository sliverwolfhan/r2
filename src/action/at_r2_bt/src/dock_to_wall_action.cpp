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
  swing_last_flip_time_(0, 0, RCL_ROS_TIME),
  last_odom_time_(0, 0, RCL_ROS_TIME)
{
}

BT::PortsList DockToWallAction::providedPorts()
{
  return {
    BT::InputPort<rclcpp::Node::SharedPtr>("node", "ROS node"),
    BT::InputPort<double>("vy", 0.1, "Base velocity y (m/s), body frame, constant press"),
    BT::InputPort<double>("vx", 0.0, "Base velocity x (m/s), body frame, constant press"),
    BT::InputPort<double>("wz", 0.0, "Angular velocity z (rad/s)"),
    BT::InputPort<double>("swing_vx", 0.0,
      "Swing velocity x (m/s), body frame; set parallel to wall, sign flips every swing_period"),
    BT::InputPort<double>("swing_vy", 0.0,
      "Swing velocity y (m/s), body frame; set parallel to wall, sign flips every swing_period"),
    BT::InputPort<double>("swing_period", 0.5,
      "Interval (s) between swing direction reversals; <=0 disables swinging"),
    BT::InputPort<double>("stall_threshold", 0.005,
      "Position change threshold (m) below which we consider stalled"),
    BT::InputPort<double>("stall_duration", 0.5,
      "Time (s) position must remain unchanged to confirm docked"),
    BT::InputPort<double>("timeout", 10.0, "Max time (s) before giving up"),
    BT::InputPort<double>("publish_rate_hz", 50.0, "cmd_vel publish rate (Hz)"),
    BT::InputPort<std::string>("cmd_vel_topic", "/AT_R2/cmd_vel_bt",
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
  swing_vx_ = 0.0;
  swing_vy_ = 0.0;
  swing_period_ = 0.5;
  stall_threshold_ = 0.005;
  stall_duration_ = 0.5;
  timeout_ = 10.0;
  double publish_rate_hz = 50.0;
  cmd_vel_topic_ = "/AT_R2/cmd_vel_bt";
  odom_topic_ = "/AT_R2/odometry";

  getInput("vy", vy_);
  getInput("vx", vx_);
  getInput("wz", wz_);
  getInput("swing_vx", swing_vx_);
  getInput("swing_vy", swing_vy_);
  getInput("swing_period", swing_period_);
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
  swing_sign_ = 1;
  swing_last_flip_time_ = node_->now();

  // 创建 publisher
  if (!cmd_vel_pub_ || active_cmd_vel_topic_ != cmd_vel_topic_) {
    cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
    active_cmd_vel_topic_ = cmd_vel_topic_;
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
    "DockToWall started: vx=%.3f vy=%.3f wz=%.3f swing=(%.3f,%.3f)/%.2fs "
    "stall_thresh=%.4fm stall_dur=%.2fs timeout=%.1fs topic=%s",
    vx_, vy_, wz_, swing_vx_, swing_vy_, swing_period_,
    stall_threshold_, stall_duration_, timeout_, cmd_vel_topic_.c_str());

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

  double displacement;
  const bool swinging = swing_period_ > 0.0 && (swing_vx_ != 0.0 || swing_vy_ != 0.0);
  const double press_norm = std::hypot(vx_, vy_);
  if (swinging && press_norm > 1e-6) {
    // 摇摆时，平行墙面的往复运动会使总位移一直偏大，导致无法判定贴住。
    // 因此只统计沿压墙方向(垂直墙面)的位移分量：把车体系压墙方向(vx_,vy_)
    // 旋转到里程计系后，将本次位移投影到该方向上。
    const auto & q = msg->pose.pose.orientation;
    double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                            1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    double ux = (vx_ * std::cos(yaw) - vy_ * std::sin(yaw)) / press_norm;
    double uy = (vx_ * std::sin(yaw) + vy_ * std::cos(yaw)) / press_norm;
    displacement = std::fabs(dx * ux + dy * uy);
  } else {
    displacement = std::sqrt(dx * dx + dy * dy);
  }

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

  std::lock_guard<std::mutex> lock(mutex_);

  // 到达周期则反转摇摆方向
  if (swing_period_ > 0.0 && (swing_vx_ != 0.0 || swing_vy_ != 0.0)) {
    auto now = node_->now();
    if ((now - swing_last_flip_time_).seconds() >= swing_period_) {
      swing_sign_ = -swing_sign_;
      swing_last_flip_time_ = now;
    }
  }

  geometry_msgs::msg::Twist cmd;
  // 恒定压墙速度 + 平行墙面的往复摇摆速度
  cmd.linear.x = vx_ + swing_sign_ * swing_vx_;
  cmd.linear.y = vy_ + swing_sign_ * swing_vy_;
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
