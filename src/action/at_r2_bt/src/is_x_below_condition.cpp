// Copyright 2024
// Licensed under the Apache License, Version 2.0

#include "nav2_bt_publish_goal/is_x_below_condition.hpp"

#include <vector>

#include "behaviortree_cpp/bt_factory.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/time.h"

namespace nav2_bt_publish_goal
{

IsXBelowCondition::IsXBelowCondition(
  const std::string & name,
  const BT::NodeConfig & config)
: BT::ConditionNode(name, config)
{
}

BT::PortsList IsXBelowCondition::providedPorts()
{
  return {
    BT::InputPort<rclcpp::Node::SharedPtr>("node", "ROS node"),
    BT::InputPort<double>("x_threshold", 6.0,
      "map 系 x 分界阈值 (m): cur_x < x_threshold -> SUCCESS"),
    BT::InputPort<std::string>("base_frame", "base_footprint", "Robot base TF frame"),
    BT::InputPort<std::string>("map_frame", "map", "Map TF frame"),
    BT::InputPort<std::string>("tf_namespace", "AT_R2",
      "Namespace prefix for /tf and /tf_static (only used in fallback path)"),
    BT::InputPort<double>("tf_timeout", 0.2, "TF lookup timeout seconds"),
  };
}

bool IsXBelowCondition::ensureInitialized()
{
  if (initialized_) {
    return true;
  }

  if (!getInput("node", node_) || !node_) {
    RCLCPP_ERROR(rclcpp::get_logger("IsXBelowCondition"),
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
        "IsXBelowCondition: using shared tf_buffer from blackboard");
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

    tf_node_ = rclcpp::Node::make_shared("is_x_below_tf", "", tf_node_opts);
    RCLCPP_WARN(node_->get_logger(),
      "IsXBelowCondition: blackboard has no tf_buffer, falling back to local listener "
      "(remap /tf -> %s/tf)",
      tf_ns.empty() ? "(global)" : ("/" + tf_ns).c_str());

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, tf_node_, true);
  }

  initialized_ = true;
  return true;
}

BT::NodeStatus IsXBelowCondition::tick()
{
  if (!ensureInitialized()) {
    return BT::NodeStatus::FAILURE;
  }

  double x_threshold = 6.0;
  if (!getInput("x_threshold", x_threshold)) {
    RCLCPP_ERROR(node_->get_logger(), "Missing required input [x_threshold]");
    return BT::NodeStatus::FAILURE;
  }

  std::string base_frame = "base_footprint";
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
        "IsXBelow: TF unavailable %s -> %s, 返回 FAILURE",
        base_frame.c_str(), map_frame.c_str());
      return BT::NodeStatus::FAILURE;
    }
    tf_mb = tf_buffer_->lookupTransform(
      map_frame, base_frame, tf2::TimePointZero);
  } catch (const std::exception & e) {
    RCLCPP_WARN(node_->get_logger(),
      "IsXBelow: TF lookup failed (%s), 返回 FAILURE", e.what());
    return BT::NodeStatus::FAILURE;
  }

  const double cur_x = tf_mb.transform.translation.x;
  const bool below = (cur_x < x_threshold);

  RCLCPP_INFO(node_->get_logger(),
    "IsXBelow: cur_x=%.3f threshold=%.3f -> %s",
    cur_x, x_threshold, below ? "BELOW(SUCCESS)" : "ABOVE(FAILURE)");

  return below ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace nav2_bt_publish_goal

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_bt_publish_goal::IsXBelowCondition>("IsXBelow");
}
