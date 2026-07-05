// Copyright 2024
// Licensed under the Apache License, Version 2.0

#include "nav2_bt_publish_goal/grasp_pressed_by_distance_condition.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

#include "behaviortree_cpp/bt_factory.h"

namespace nav2_bt_publish_goal
{

GraspPressedByDistanceCondition::GraspPressedByDistanceCondition(
  const std::string & name,
  const BT::NodeConfig & config)
: BT::ConditionNode(name, config),
  latest_distance_time_(0, 0, RCL_ROS_TIME),
  last_filter_time_(0, 0, RCL_ROS_TIME)
{
}

BT::PortsList GraspPressedByDistanceCondition::providedPorts()
{
  return {
    BT::InputPort<rclcpp::Node::SharedPtr>("node", "ROS node"),
    BT::InputPort<std::string>("distance_topic", "/AT_R2/distance_grasp",
      "Grasp-side laser distance topic (std_msgs/Float64)"),
    BT::InputPort<double>("distance_scale", 0.001,
      "Scale applied to raw distance before comparing (raw mm -> m)"),
    BT::InputPort<double>("target_distance", "Target scaled distance when pressed (m)"),
    BT::InputPort<double>("distance_tolerance", "Tolerance for scaled distance (m)"),
    BT::InputPort<int>("stable_count", 3, "Required consecutive successful ticks"),
    BT::InputPort<double>("max_data_age", 0.5,
      "Maximum age of distance data (s), <=0 disables age check"),
    BT::InputPort<bool>("filter_enable", true,
      "Enable median+EMA distance filtering before judging"),
    BT::InputPort<int>("median_window", 3,
      "Median filter window size in samples (>=1); rejects laser spikes"),
    BT::InputPort<double>("ema_tau", 0.1,
      "EMA time constant (s); larger=smoother but more lag, <=0 disables EMA"),
  };
}

bool GraspPressedByDistanceCondition::ensureInitialized()
{
  if (initialized_) {
    return true;
  }

  if (!getInput("node", node_) || !node_) {
    RCLCPP_ERROR(rclcpp::get_logger("GraspPressedByDistanceCondition"),
      "Missing or null required input [node]");
    return false;
  }

  filter_enable_ = true;
  median_window_ = 3;
  ema_tau_ = 0.1;
  (void)getInput("filter_enable", filter_enable_);
  (void)getInput("median_window", median_window_);
  (void)getInput("ema_tau", ema_tau_);
  median_window_ = std::max(1, median_window_);
  if (!std::isfinite(ema_tau_) || ema_tau_ < 0.0) {
    ema_tau_ = 0.0;
  }

  {
    std::lock_guard<std::mutex> lock(distance_mutex_);
    latest_distance_ = 0.0;
    latest_distance_time_ = node_->now();
    has_distance_ = false;
    median_buf_.clear();
    ema_initialized_ = false;
    ema_value_ = 0.0;
    last_filter_time_ = node_->now();
  }

  std::string distance_topic = "/AT_R2/distance_grasp";
  (void)getInput("distance_topic", distance_topic);
  distance_sub_ = node_->create_subscription<std_msgs::msg::Float64>(
    distance_topic, 10,
    std::bind(&GraspPressedByDistanceCondition::distanceCallback, this, std::placeholders::_1));

  RCLCPP_INFO(node_->get_logger(),
    "GraspPressedByDistance: subscribed %s, filter=%d median_window=%d ema_tau=%.3fs",
    distance_topic.c_str(), filter_enable_ ? 1 : 0, median_window_, ema_tau_);

  initialized_ = true;
  return true;
}

void GraspPressedByDistanceCondition::distanceCallback(
  const std_msgs::msg::Float64::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(distance_mutex_);
  const rclcpp::Time stamp = node_->now();
  latest_distance_ = filter_enable_ ? filterDistance(msg->data, stamp) : msg->data;
  latest_distance_time_ = stamp;
  has_distance_ = true;
}

double GraspPressedByDistanceCondition::filterDistance(
  double raw, const rclcpp::Time & stamp)
{
  // 非有限值直接透传，交给 tick 的 isfinite 兜底处理，不污染滤波状态。
  if (!std::isfinite(raw)) {
    return raw;
  }

  // 1) 中值：去尖刺/丢点。窗口内排序取中位数。
  median_buf_.push_back(raw);
  while (static_cast<int>(median_buf_.size()) > median_window_) {
    median_buf_.pop_front();
  }
  std::vector<double> sorted(median_buf_.begin(), median_buf_.end());
  std::sort(sorted.begin(), sorted.end());
  const size_t n = sorted.size();
  const double median = (n % 2 == 1)
    ? sorted[n / 2]
    : 0.5 * (sorted[n / 2 - 1] + sorted[n / 2]);

  // 2) 时间常数 EMA：去高斯抖动，平滑程度与话题频率解耦。
  if (ema_tau_ <= 0.0) {
    ema_value_ = median;
    ema_initialized_ = true;
    last_filter_time_ = stamp;
    return median;
  }
  if (!ema_initialized_) {
    ema_value_ = median;
    ema_initialized_ = true;
    last_filter_time_ = stamp;
    return median;
  }
  double dt = (stamp - last_filter_time_).seconds();
  last_filter_time_ = stamp;
  if (!(dt > 0.0)) {
    // 时间戳异常(0 或回退)，本帧跳过 EMA 更新，仅返回当前估计。
    return ema_value_;
  }
  const double alpha = 1.0 - std::exp(-dt / ema_tau_);
  ema_value_ += alpha * (median - ema_value_);
  return ema_value_;
}

BT::NodeStatus GraspPressedByDistanceCondition::tick()
{
  if (!ensureInitialized()) {
    stable_count_ = 0;
    return BT::NodeStatus::FAILURE;
  }

  double target_distance = 0.0;
  double distance_tolerance = 0.0;
  if (!getInput("target_distance", target_distance) ||
      !getInput("distance_tolerance", distance_tolerance))
  {
    RCLCPP_ERROR(node_->get_logger(),
      "GraspPressedByDistance missing required inputs [target_distance / distance_tolerance]");
    stable_count_ = 0;
    return BT::NodeStatus::FAILURE;
  }

  double distance_scale = 0.001;
  double max_data_age = 0.5;
  int required_stable_count = 3;
  (void)getInput("distance_scale", distance_scale);
  (void)getInput("max_data_age", max_data_age);
  (void)getInput("stable_count", required_stable_count);
  required_stable_count = std::max(1, required_stable_count);
  distance_tolerance = std::max(0.0, distance_tolerance);
  const double distance_min = target_distance - distance_tolerance;
  const double distance_max = target_distance + distance_tolerance;

  double raw_distance = 0.0;
  rclcpp::Time distance_time;
  bool has_distance = false;
  {
    std::lock_guard<std::mutex> lock(distance_mutex_);
    raw_distance = latest_distance_;
    distance_time = latest_distance_time_;
    has_distance = has_distance_;
  }

  if (!has_distance) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
      "GraspPressedByDistance: no distance data yet");
    stable_count_ = 0;
    return BT::NodeStatus::FAILURE;
  }

  const double data_age = (node_->now() - distance_time).seconds();
  if (max_data_age > 0.0 && data_age > max_data_age) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
      "GraspPressedByDistance: distance data too old (age=%.3fs > %.3fs)",
      data_age, max_data_age);
    stable_count_ = 0;
    return BT::NodeStatus::FAILURE;
  }

  const double scaled_distance = raw_distance * distance_scale;
  const bool distance_ok =
    std::isfinite(scaled_distance) &&
    (scaled_distance >= distance_min) &&
    (scaled_distance <= distance_max);

  if (distance_ok) {
    ++stable_count_;
  } else {
    stable_count_ = 0;
  }

  const bool pressed = stable_count_ >= required_stable_count;
  RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 500,
    "GraspPressedByDistance: raw=%.3f scaled=%.3f range=[%.3f, %.3f] age=%.3fs %s, "
    "stable=%d/%d -> %s",
    raw_distance, scaled_distance, distance_min, distance_max, data_age,
    distance_ok ? "OK" : "NO",
    stable_count_, required_stable_count,
    pressed ? "PRESSED" : "WAIT");

  return pressed ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace nav2_bt_publish_goal

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_bt_publish_goal::GraspPressedByDistanceCondition>(
    "GraspPressedByDistance");
}
