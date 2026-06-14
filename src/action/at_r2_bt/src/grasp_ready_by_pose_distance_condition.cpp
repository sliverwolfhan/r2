// Copyright 2024
// Licensed under the Apache License, Version 2.0

#include "nav2_bt_publish_goal/grasp_ready_by_pose_distance_condition.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

#include "behaviortree_cpp/bt_factory.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/time.h"

namespace nav2_bt_publish_goal
{

GraspReadyByPoseDistanceCondition::GraspReadyByPoseDistanceCondition(
  const std::string & name,
  const BT::NodeConfig & config)
: BT::ConditionNode(name, config)
{
}

BT::PortsList GraspReadyByPoseDistanceCondition::providedPorts()
{
  return {
    BT::InputPort<rclcpp::Node::SharedPtr>("node", "ROS node"),
    BT::InputPort<double>("target_y", "Target robot y in map frame (m)"),
    BT::InputPort<double>("y_tolerance", "Tolerance for robot y (m)"),
    BT::InputPort<double>("target_yaw", "Target robot yaw in map frame (rad)"),
    BT::InputPort<double>("yaw_tolerance", 0.15, "Tolerance for yaw (rad)"),
    BT::InputPort<std::string>("distance_topic", "/AT_R2/distance", "Distance topic (std_msgs/Float64)"),
    BT::InputPort<double>("target_distance", "Target scaled distance"),
    BT::InputPort<double>("distance_tolerance", "Tolerance for scaled distance"),
    BT::InputPort<double>("distance_scale", 1.0, "Scale applied to raw distance before comparing"),
    BT::InputPort<int>("stable_count", 3, "Required consecutive successful ticks"),
    BT::InputPort<double>("max_data_age", 0.5, "Maximum age of distance data (s), <=0 disables age check"),
    BT::InputPort<std::string>("base_frame", "base_link", "Robot base TF frame"),
    BT::InputPort<std::string>("map_frame", "map", "Map TF frame"),
    BT::InputPort<std::string>("tf_namespace", "AT_R2",
      "Namespace prefix for /tf and /tf_static (only used in fallback path)"),
    BT::InputPort<double>("tf_timeout", 0.2, "TF lookup timeout seconds"),
  };
}

double GraspReadyByPoseDistanceCondition::normalizeAngle(double a)
{
  while (a > M_PI) {
    a -= 2.0 * M_PI;
  }
  while (a < -M_PI) {
    a += 2.0 * M_PI;
  }
  return a;
}

bool GraspReadyByPoseDistanceCondition::ensureInitialized()
{
  if (initialized_) {
    return true;
  }

  if (!getInput("node", node_) || !node_) {
    RCLCPP_ERROR(rclcpp::get_logger("GraspReadyByPoseDistanceCondition"),
      "Missing or null required input [node]");
    return false;
  }

  auto bb = config().blackboard;
  if (bb) {
    std::shared_ptr<tf2_ros::Buffer> shared_buf;
    if (bb->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer", shared_buf) && shared_buf) {
      tf_buffer_ = shared_buf;
      RCLCPP_INFO(node_->get_logger(),
        "GraspReadyByPoseDistanceCondition: using shared tf_buffer from blackboard");
    }
  }

  if (!tf_buffer_) {
    std::string tf_ns = "AT_R2";
    (void)getInput("tf_namespace", tf_ns);

    rclcpp::NodeOptions tf_node_opts;
    bool use_sim_time = false;
    node_->get_parameter_or("use_sim_time", use_sim_time, false);
    tf_node_opts.parameter_overrides({rclcpp::Parameter("use_sim_time", use_sim_time)});

    std::vector<std::string> remap_args;
    if (!tf_ns.empty()) {
      const std::string ns_prefix = (tf_ns.front() == '/') ? tf_ns : ("/" + tf_ns);
      remap_args = {
        "--ros-args",
        "-r", "/tf:=" + ns_prefix + "/tf",
        "-r", "/tf_static:=" + ns_prefix + "/tf_static",
      };
    }
    tf_node_opts.arguments(remap_args);

    tf_node_ = rclcpp::Node::make_shared("grasp_ready_pose_distance_tf", "", tf_node_opts);
    RCLCPP_WARN(node_->get_logger(),
      "GraspReadyByPoseDistanceCondition: blackboard has no tf_buffer, falling back to local "
      "listener (remap /tf -> %s/tf)",
      tf_ns.empty() ? "(global)" : ("/" + tf_ns).c_str());

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, tf_node_, true);
  }

  std::string distance_topic = "/AT_R2/distance";
  (void)getInput("distance_topic", distance_topic);
  distance_sub_ = node_->create_subscription<std_msgs::msg::Float64>(
    distance_topic, 10,
    std::bind(&GraspReadyByPoseDistanceCondition::distanceCallback, this, std::placeholders::_1));

  RCLCPP_INFO(node_->get_logger(),
    "GraspReadyByPoseDistanceCondition: subscribed distance topic: %s", distance_topic.c_str());

  initialized_ = true;
  return true;
}

void GraspReadyByPoseDistanceCondition::distanceCallback(
  const std_msgs::msg::Float64::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(distance_mutex_);
  latest_distance_ = msg->data;
  latest_distance_time_ = node_->now();
  has_distance_ = true;
}

BT::NodeStatus GraspReadyByPoseDistanceCondition::tick()
{
  if (!ensureInitialized()) {
    stable_count_ = 0;
    return BT::NodeStatus::FAILURE;
  }

  double target_y = 0.0;
  double y_tolerance = 0.0;
  double target_yaw = 0.0;
  double target_distance = 0.0;
  double distance_tolerance = 0.0;
  if (!getInput("target_y", target_y) ||
      !getInput("y_tolerance", y_tolerance) ||
      !getInput("target_yaw", target_yaw) ||
      !getInput("target_distance", target_distance) ||
      !getInput("distance_tolerance", distance_tolerance))
  {
    RCLCPP_ERROR(node_->get_logger(),
      "Missing required inputs [target_y / y_tolerance / target_yaw / target_distance / distance_tolerance]");
    stable_count_ = 0;
    return BT::NodeStatus::FAILURE;
  }

  y_tolerance = std::max(0.0, y_tolerance);
  distance_tolerance = std::max(0.0, distance_tolerance);
  const double y_min = target_y - y_tolerance;
  const double y_max = target_y + y_tolerance;
  const double distance_min = target_distance - distance_tolerance;
  const double distance_max = target_distance + distance_tolerance;

  double yaw_tol = 0.15;
  double distance_scale = 1.0;
  double max_data_age = 0.5;
  double tf_timeout_s = 0.2;
  int required_stable_count = 3;
  (void)getInput("yaw_tolerance", yaw_tol);
  (void)getInput("distance_scale", distance_scale);
  (void)getInput("max_data_age", max_data_age);
  (void)getInput("tf_timeout", tf_timeout_s);
  (void)getInput("stable_count", required_stable_count);
  yaw_tol = std::max(0.0, yaw_tol);
  tf_timeout_s = std::max(0.0, tf_timeout_s);
  required_stable_count = std::max(1, required_stable_count);

  std::string base_frame = "base_link";
  std::string map_frame = "map";
  (void)getInput("base_frame", base_frame);
  (void)getInput("map_frame", map_frame);

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
      "GraspReadyByPoseDistance: no distance data yet");
    stable_count_ = 0;
    return BT::NodeStatus::FAILURE;
  }

  const double data_age = (node_->now() - distance_time).seconds();
  if (max_data_age > 0.0 && data_age > max_data_age) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
      "GraspReadyByPoseDistance: distance data too old (age=%.3fs > %.3fs)",
      data_age, max_data_age);
    stable_count_ = 0;
    return BT::NodeStatus::FAILURE;
  }

  geometry_msgs::msg::TransformStamped tf_mb;
  try {
    const tf2::Duration tf_timeout = tf2::durationFromSec(tf_timeout_s);
    if (!tf_buffer_->canTransform(
        map_frame, base_frame, tf2::TimePointZero, tf_timeout))
    {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
        "GraspReadyByPoseDistance: TF unavailable %s -> %s",
        base_frame.c_str(), map_frame.c_str());
      stable_count_ = 0;
      return BT::NodeStatus::FAILURE;
    }
    tf_mb = tf_buffer_->lookupTransform(map_frame, base_frame, tf2::TimePointZero);
  } catch (const std::exception & e) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
      "GraspReadyByPoseDistance: TF lookup failed: %s", e.what());
    stable_count_ = 0;
    return BT::NodeStatus::FAILURE;
  }

  const double cur_y = tf_mb.transform.translation.y;
  tf2::Quaternion q(
    tf_mb.transform.rotation.x,
    tf_mb.transform.rotation.y,
    tf_mb.transform.rotation.z,
    tf_mb.transform.rotation.w);
  double roll = 0.0;
  double pitch = 0.0;
  double cur_yaw = 0.0;
  tf2::Matrix3x3(q).getRPY(roll, pitch, cur_yaw);

  const double yaw_error = std::fabs(normalizeAngle(cur_yaw - target_yaw));
  const double scaled_distance = raw_distance * distance_scale;

  const bool y_ok = (cur_y >= y_min) && (cur_y <= y_max);
  const bool yaw_ok = yaw_error <= yaw_tol;
  const bool distance_ok =
    std::isfinite(scaled_distance) &&
    (scaled_distance >= distance_min) &&
    (scaled_distance <= distance_max);
  const bool ready_this_tick = y_ok && yaw_ok && distance_ok;

  if (ready_this_tick) {
    ++stable_count_;
  } else {
    stable_count_ = 0;
  }

  const bool ready = stable_count_ >= required_stable_count;
  RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 500,
    "GraspReadyByPoseDistance: y=%.3f range=[%.3f, %.3f] %s, "
    "yaw=%.3f target=%.3f err=%.3f tol=%.3f %s, "
    "distance raw=%.3f scaled=%.3f range=[%.3f, %.3f] age=%.3fs %s, "
    "stable=%d/%d -> %s",
    cur_y, y_min, y_max, y_ok ? "OK" : "NO",
    cur_yaw, target_yaw, yaw_error, yaw_tol, yaw_ok ? "OK" : "NO",
    raw_distance, scaled_distance, distance_min, distance_max, data_age,
    distance_ok ? "OK" : "NO",
    stable_count_, required_stable_count,
    ready ? "READY" : "NAVIGATE");

  return ready ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace nav2_bt_publish_goal

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_bt_publish_goal::GraspReadyByPoseDistanceCondition>(
    "GraspReadyByPoseDistance");
}
