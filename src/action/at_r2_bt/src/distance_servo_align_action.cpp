// Copyright 2024
// Licensed under the Apache License, Version 2.0

#include "nav2_bt_publish_goal/distance_servo_align_action.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>

#include "behaviortree_cpp/bt_factory.h"

namespace nav2_bt_publish_goal
{

DistanceServoAlignAction::DistanceServoAlignAction(
  const std::string & name,
  const BT::NodeConfig & config)
: BT::StatefulActionNode(name, config),
  latest_distance_time_(0, 0, RCL_ROS_TIME),
  start_time_(0, 0, RCL_ROS_TIME)
{
}

BT::PortsList DistanceServoAlignAction::providedPorts()
{
  return {
    BT::InputPort<rclcpp::Node::SharedPtr>("node", "ROS node"),
    BT::InputPort<std::string>("distance_topic", "/AT_R2/distance",
      "Distance topic (std_msgs/Float64)"),
    BT::InputPort<std::string>("cmd_vel_topic", "/AT_R2/cmd_vel_nav2_result",
      "Velocity command topic"),
    BT::InputPort<double>("distance_scale", 1.0,
      "Scale applied to raw distance before control"),
    BT::InputPort<double>("target_distance", "Target scaled distance (m)"),
    BT::InputPort<double>("tolerance", 0.005, "Success tolerance (m)"),
    BT::InputPort<double>("kp", 0.8, "Proportional gain"),
    BT::InputPort<double>("max_speed", 0.06, "Maximum absolute command speed (m/s)"),
    BT::InputPort<double>("min_speed", 0.015,
      "Minimum absolute command speed when outside tolerance (m/s)"),
    BT::InputPort<int>("stable_count", 3, "Required consecutive successful ticks"),
    BT::InputPort<double>("max_data_age", 0.5,
      "Maximum age of distance data (s), <=0 disables age check"),
    BT::InputPort<double>("timeout", 8.0, "Max action duration (s)"),
    BT::InputPort<double>("publish_rate_hz", 50.0, "cmd_vel publish rate (Hz)"),
    BT::InputPort<std::string>("cmd_axis", "x",
      "Which linear velocity axis to publish: x or y"),
    BT::InputPort<double>("positive_error_direction", 1.0,
      "Velocity sign for positive error; use -1 to invert"),
  };
}

BT::NodeStatus DistanceServoAlignAction::onStart()
{
  if (!node_) {
    if (!getInput("node", node_) || !node_) {
      RCLCPP_ERROR(rclcpp::get_logger("DistanceServoAlign"),
        "Missing/null required input [node]");
      return BT::NodeStatus::FAILURE;
    }
  }

  distance_topic_ = "/AT_R2/distance";
  cmd_vel_topic_ = "/AT_R2/cmd_vel_nav2_result";
  distance_scale_ = 1.0;
  tolerance_ = 0.005;
  kp_ = 0.8;
  max_speed_ = 0.06;
  min_speed_ = 0.015;
  stable_count_required_ = 3;
  max_data_age_ = 0.5;
  timeout_ = 8.0;
  double publish_rate_hz = 50.0;
  cmd_axis_ = "x";
  positive_error_direction_ = 1.0;

  (void)getInput("distance_topic", distance_topic_);
  (void)getInput("cmd_vel_topic", cmd_vel_topic_);
  (void)getInput("distance_scale", distance_scale_);
  if (!getInput("target_distance", target_distance_)) {
    RCLCPP_ERROR(node_->get_logger(),
      "DistanceServoAlign missing required input [target_distance]");
    return BT::NodeStatus::FAILURE;
  }
  (void)getInput("tolerance", tolerance_);
  (void)getInput("kp", kp_);
  (void)getInput("max_speed", max_speed_);
  (void)getInput("min_speed", min_speed_);
  (void)getInput("stable_count", stable_count_required_);
  (void)getInput("max_data_age", max_data_age_);
  (void)getInput("timeout", timeout_);
  (void)getInput("publish_rate_hz", publish_rate_hz);
  (void)getInput("cmd_axis", cmd_axis_);
  (void)getInput("positive_error_direction", positive_error_direction_);

  if (!std::isfinite(distance_scale_) || distance_scale_ == 0.0) {
    RCLCPP_ERROR(node_->get_logger(),
      "DistanceServoAlign invalid distance_scale: %.6f", distance_scale_);
    return BT::NodeStatus::FAILURE;
  }
  if (!std::isfinite(target_distance_)) {
    RCLCPP_ERROR(node_->get_logger(),
      "DistanceServoAlign invalid target_distance: %.6f", target_distance_);
    return BT::NodeStatus::FAILURE;
  }
  if (!std::isfinite(kp_)) {
    RCLCPP_ERROR(node_->get_logger(), "DistanceServoAlign invalid kp: %.6f", kp_);
    return BT::NodeStatus::FAILURE;
  }
  if (!std::isfinite(max_speed_) || max_speed_ <= 0.0) {
    RCLCPP_ERROR(node_->get_logger(),
      "DistanceServoAlign invalid max_speed: %.6f", max_speed_);
    return BT::NodeStatus::FAILURE;
  }
  if (!std::isfinite(min_speed_) || min_speed_ < 0.0) {
    RCLCPP_ERROR(node_->get_logger(),
      "DistanceServoAlign invalid min_speed: %.6f", min_speed_);
    return BT::NodeStatus::FAILURE;
  }
  if (min_speed_ > max_speed_) {
    RCLCPP_WARN(node_->get_logger(),
      "DistanceServoAlign min_speed %.3f > max_speed %.3f, clamp min_speed to max_speed",
      min_speed_, max_speed_);
    min_speed_ = max_speed_;
  }
  if (!std::isfinite(tolerance_)) {
    RCLCPP_ERROR(node_->get_logger(),
      "DistanceServoAlign invalid tolerance: %.6f", tolerance_);
    return BT::NodeStatus::FAILURE;
  }
  tolerance_ = std::max(0.0, tolerance_);
  stable_count_required_ = std::max(1, stable_count_required_);
  if (!std::isfinite(max_data_age_)) {
    max_data_age_ = 0.5;
  }
  if (!std::isfinite(timeout_) || timeout_ <= 0.0) {
    RCLCPP_ERROR(node_->get_logger(),
      "DistanceServoAlign invalid timeout: %.6f", timeout_);
    return BT::NodeStatus::FAILURE;
  }
  if (!std::isfinite(publish_rate_hz) || publish_rate_hz < 1.0) {
    publish_rate_hz = 1.0;
  }
  if (cmd_axis_ != "x" && cmd_axis_ != "y") {
    RCLCPP_ERROR(node_->get_logger(),
      "DistanceServoAlign invalid cmd_axis [%s], expected x or y", cmd_axis_.c_str());
    return BT::NodeStatus::FAILURE;
  }
  positive_error_direction_ = (positive_error_direction_ >= 0.0) ? 1.0 : -1.0;

  stopAll();

  {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_distance_ = 0.0;
    latest_distance_time_ = node_->now();
    has_distance_ = false;
    stable_count_current_ = 0;
    cmd_ = geometry_msgs::msg::Twist();
  }

  cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
  distance_sub_ = node_->create_subscription<std_msgs::msg::Float64>(
    distance_topic_, 10,
    std::bind(&DistanceServoAlignAction::distanceCallback, this, std::placeholders::_1));

  const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz);
  const auto period_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(period);
  publish_timer_ = node_->create_wall_timer(
    period_ns,
    std::bind(&DistanceServoAlignAction::publishCmdTimerCallback, this));

  {
    std::lock_guard<std::mutex> lock(publish_timer_mutex_);
    publish_timer_armed_ = true;
  }

  start_time_ = node_->now();

  RCLCPP_INFO(node_->get_logger(),
    "DistanceServoAlign started: target=%.3fm tol=%.3fm kp=%.3f speed=[%.3f, %.3f] "
    "axis=%s direction=%.0f distance_topic=%s cmd_vel_topic=%s timeout=%.1fs",
    target_distance_, tolerance_, kp_, min_speed_, max_speed_, cmd_axis_.c_str(),
    positive_error_direction_, distance_topic_.c_str(), cmd_vel_topic_.c_str(), timeout_);

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus DistanceServoAlignAction::onRunning()
{
  const auto now = node_->now();
  if ((now - start_time_).seconds() > timeout_) {
    stopAll();
    RCLCPP_WARN(node_->get_logger(),
      "DistanceServoAlign timed out after %.1fs", timeout_);
    return BT::NodeStatus::FAILURE;
  }

  double raw_distance = 0.0;
  rclcpp::Time distance_time;
  bool has_distance = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    raw_distance = latest_distance_;
    distance_time = latest_distance_time_;
    has_distance = has_distance_;
  }

  if (!has_distance) {
    setZeroCommand();
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
      "DistanceServoAlign: no distance data yet");
    return BT::NodeStatus::RUNNING;
  }

  const double data_age = (now - distance_time).seconds();
  if (max_data_age_ > 0.0 && data_age > max_data_age_) {
    setZeroCommand();
    stable_count_current_ = 0;
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
      "DistanceServoAlign: distance data too old (age=%.3fs > %.3fs)",
      data_age, max_data_age_);
    return BT::NodeStatus::RUNNING;
  }

  const double current_distance = raw_distance * distance_scale_;
  if (!std::isfinite(raw_distance) || !std::isfinite(current_distance)) {
    setZeroCommand();
    stable_count_current_ = 0;
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
      "DistanceServoAlign: invalid distance raw=%.3f scaled=%.3f",
      raw_distance, current_distance);
    return BT::NodeStatus::RUNNING;
  }

  const double error = target_distance_ - current_distance;
  const double abs_error = std::fabs(error);

  if (abs_error <= tolerance_) {
    setZeroCommand();
    ++stable_count_current_;
    RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 500,
      "DistanceServoAlign: distance=%.3f target=%.3f error=%.3f stable=%d/%d",
      current_distance, target_distance_, error,
      stable_count_current_, stable_count_required_);

    if (stable_count_current_ >= stable_count_required_) {
      stopAll();
      RCLCPP_INFO(node_->get_logger(),
        "DistanceServoAlign SUCCESS: distance=%.3f target=%.3f error=%.3f",
        current_distance, target_distance_, error);
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::RUNNING;
  }

  stable_count_current_ = 0;
  const double speed = computeSpeed(error);
  geometry_msgs::msg::Twist cmd;
  if (cmd_axis_ == "x") {
    cmd.linear.x = speed;
  } else {
    cmd.linear.y = speed;
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    cmd_ = cmd;
  }

  RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 500,
    "DistanceServoAlign: distance raw=%.3f scaled=%.3f target=%.3f error=%.3f "
    "cmd.%s=%.3f age=%.3fs",
    raw_distance, current_distance, target_distance_, error,
    cmd_axis_.c_str(), speed, data_age);

  return BT::NodeStatus::RUNNING;
}

void DistanceServoAlignAction::onHalted()
{
  stopAll();
  RCLCPP_INFO(node_->get_logger(), "DistanceServoAlign halted, sent zero velocity");
}

void DistanceServoAlignAction::distanceCallback(
  const std_msgs::msg::Float64::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  latest_distance_ = msg->data;
  latest_distance_time_ = node_->now();
  has_distance_ = true;
}

void DistanceServoAlignAction::publishCmdTimerCallback()
{
  geometry_msgs::msg::Twist cmd;
  {
    std::lock_guard<std::mutex> timer_lock(publish_timer_mutex_);
    if (!publish_timer_armed_ || !cmd_vel_pub_) {
      return;
    }
    std::lock_guard<std::mutex> data_lock(mutex_);
    cmd = cmd_;
  }
  cmd_vel_pub_->publish(cmd);
}

void DistanceServoAlignAction::publishZero()
{
  if (!cmd_vel_pub_) {
    return;
  }
  geometry_msgs::msg::Twist zero;
  cmd_vel_pub_->publish(zero);
}

void DistanceServoAlignAction::stopAll()
{
  {
    std::lock_guard<std::mutex> lock(publish_timer_mutex_);
    publish_timer_armed_ = false;
    if (publish_timer_) {
      publish_timer_->cancel();
      publish_timer_.reset();
    }
  }

  setZeroCommand();
  publishZero();

  if (distance_sub_) {
    distance_sub_.reset();
  }
}

void DistanceServoAlignAction::setZeroCommand()
{
  std::lock_guard<std::mutex> lock(mutex_);
  cmd_ = geometry_msgs::msg::Twist();
}

double DistanceServoAlignAction::computeSpeed(double error) const
{
  double speed = kp_ * error * positive_error_direction_;
  if (std::fabs(speed) > max_speed_) {
    speed = std::copysign(max_speed_, speed);
  } else if (std::fabs(speed) < min_speed_) {
    speed = std::copysign(min_speed_, speed);
  }
  return speed;
}

}  // namespace nav2_bt_publish_goal

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_bt_publish_goal::DistanceServoAlignAction>(
    "DistanceServoAlign");
}
