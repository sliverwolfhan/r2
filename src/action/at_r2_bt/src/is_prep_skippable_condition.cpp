// Copyright 2024
// Licensed under the Apache License, Version 2.0

#include "nav2_bt_publish_goal/is_prep_skippable_condition.hpp"

#include <cmath>
#include <vector>

#include "behaviortree_cpp/bt_factory.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/time.h"

namespace nav2_bt_publish_goal
{

IsPrepSkippableCondition::IsPrepSkippableCondition(
  const std::string & name,
  const BT::NodeConfig & config)
: BT::ConditionNode(name, config)
{
}

BT::PortsList IsPrepSkippableCondition::providedPorts()
{
  return {
    BT::InputPort<rclcpp::Node::SharedPtr>("node", "ROS node"),
    BT::InputPort<double>("target_x", "Target x in map frame (m)"),
    BT::InputPort<double>("target_y", "Target y in map frame (m)"),
    BT::InputPort<double>("target_yaw", "Target yaw in map frame (rad)"),
    BT::InputPort<double>("x_tolerance", 0.20, "Tolerance for x (m)"),
    BT::InputPort<double>("y_tolerance", 0.20, "Tolerance for y (m)"),
    BT::InputPort<double>("yaw_tolerance", 0.15, "Tolerance for yaw (rad)"),
    BT::InputPort<std::string>("base_frame", "base_link", "Robot base TF frame"),
    BT::InputPort<std::string>("map_frame", "map", "Map TF frame"),
    BT::InputPort<std::string>("tf_namespace", "AT_R2",
      "Namespace prefix for /tf and /tf_static (only used in fallback path)"),
    BT::InputPort<double>("tf_timeout", 0.2, "TF lookup timeout seconds"),
  };
}

double IsPrepSkippableCondition::normalizeAngle(double a)
{
  while (a > M_PI) {
    a -= 2.0 * M_PI;
  }
  while (a < -M_PI) {
    a += 2.0 * M_PI;
  }
  return a;
}

bool IsPrepSkippableCondition::ensureInitialized()
{
  if (initialized_) {
    return true;
  }

  if (!getInput("node", node_) || !node_) {
    RCLCPP_ERROR(rclcpp::get_logger("IsPrepSkippableCondition"),
      "Missing or null required input [node]");
    return false;
  }

  // 1) 优先使用 simple_bt_runner 注入到根黑板的共享 tf_buffer
  auto bb = config().blackboard;
  if (bb) {
    std::shared_ptr<tf2_ros::Buffer> shared_buf;
    if (bb->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer", shared_buf) && shared_buf) {
      tf_buffer_ = shared_buf;
      RCLCPP_INFO(node_->get_logger(),
        "IsPrepSkippableCondition: using shared tf_buffer from blackboard");
    }
  }

  // 2) 兜底：自己创建一个带 namespace remap 的 TF listener
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

    tf_node_ = rclcpp::Node::make_shared("is_prep_skippable_tf", "", tf_node_opts);
    RCLCPP_WARN(node_->get_logger(),
      "IsPrepSkippableCondition: blackboard has no tf_buffer, falling back to local listener "
      "(remap /tf -> %s/tf)",
      tf_ns.empty() ? "(global)" : ("/" + tf_ns).c_str());

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, tf_node_, true);
  }

  initialized_ = true;
  return true;
}

BT::NodeStatus IsPrepSkippableCondition::tick()
{
  if (!ensureInitialized()) {
    return BT::NodeStatus::FAILURE;
  }

  double target_x = 0.0;
  double target_y = 0.0;
  double target_yaw = 0.0;
  if (!getInput("target_x", target_x) ||
      !getInput("target_y", target_y) ||
      !getInput("target_yaw", target_yaw))
  {
    RCLCPP_ERROR(node_->get_logger(),
      "Missing required inputs [target_x / target_y / target_yaw]");
    return BT::NodeStatus::FAILURE;
  }

  double x_tol = 0.20;
  double y_tol = 0.20;
  double yaw_tol = 0.15;
  (void)getInput("x_tolerance", x_tol);
  (void)getInput("y_tolerance", y_tol);
  (void)getInput("yaw_tolerance", yaw_tol);

  std::string base_frame = "base_link";
  std::string map_frame = "map";
  (void)getInput("base_frame", base_frame);
  (void)getInput("map_frame", map_frame);

  double tf_timeout_s = 0.2;
  (void)getInput("tf_timeout", tf_timeout_s);
  if (tf_timeout_s < 0.0) {
    tf_timeout_s = 0.0;
  }

  // 拿 map -> base 的当前位姿
  geometry_msgs::msg::TransformStamped tf_mb;
  try {
    const tf2::Duration tf_timeout = tf2::durationFromSec(tf_timeout_s);
    if (!tf_buffer_->canTransform(
        map_frame, base_frame, tf2::TimePointZero, tf_timeout))
    {
      RCLCPP_WARN(node_->get_logger(),
        "IsPrepSkippable: TF unavailable %s -> %s, treat as not skippable",
        base_frame.c_str(), map_frame.c_str());
      return BT::NodeStatus::FAILURE;
    }
    tf_mb = tf_buffer_->lookupTransform(
      map_frame, base_frame, tf2::TimePointZero);
  } catch (const std::exception & e) {
    RCLCPP_WARN(node_->get_logger(),
      "IsPrepSkippable: TF lookup failed (%s), treat as not skippable", e.what());
    return BT::NodeStatus::FAILURE;
  }

  const double cur_x = tf_mb.transform.translation.x;
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

  const double dx = std::fabs(cur_x - target_x);
  const double dy = std::fabs(cur_y - target_y);
  const double dyaw = std::fabs(normalizeAngle(cur_yaw - target_yaw));

  const bool skippable = (dx <= x_tol) && (dy <= y_tol) && (dyaw <= yaw_tol);

  RCLCPP_INFO(node_->get_logger(),
    "IsPrepSkippable: cur=(%.3f, %.3f, %.3f) target=(%.3f, %.3f, %.3f) "
    "diff=(%.3f, %.3f, %.3f) tol=(%.3f, %.3f, %.3f) -> %s",
    cur_x, cur_y, cur_yaw,
    target_x, target_y, target_yaw,
    dx, dy, dyaw,
    x_tol, y_tol, yaw_tol,
    skippable ? "SKIP" : "NAVIGATE");

  return skippable ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace nav2_bt_publish_goal

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_bt_publish_goal::IsPrepSkippableCondition>("IsPrepSkippable");
}
