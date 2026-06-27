// Copyright 2024
// Licensed under the Apache License, Version 2.0

#include "nav2_bt_publish_goal/follow_spin_path_action.hpp"

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace nav2_bt_publish_goal
{

FollowSpinPathAction::FollowSpinPathAction(
  const std::string & name,
  const BT::NodeConfig & config)
: BT::StatefulActionNode(name, config),
  goal_result_available_(false)
{
  RCLCPP_DEBUG(rclcpp::get_logger("FollowSpinPathAction"), "FollowSpinPathAction constructed");
}

BT::PortsList FollowSpinPathAction::providedPorts()
{
  return {
    BT::InputPort<rclcpp::Node::SharedPtr>("node", "ROS node"),

    // 目标位姿 (起点自动从 TF 读)
    BT::InputPort<double>("goal_x", "Target x in meters"),
    BT::InputPort<double>("goal_y", "Target y in meters"),
    BT::InputPort<double>("goal_yaw", "Target yaw in radians"),

    // 旋转区间
    BT::InputPort<double>("rotation_start_distance",
      "Distance from start (m) before rotation begins"),
    BT::InputPort<double>("rotation_end_distance",
      "Distance from goal (m) when rotation must already be done"),
    BT::InputPort<std::string>("rotation_direction", "ccw",
      "'cw' (clockwise) or 'ccw' (counter-clockwise)"),

    // 路径密度
    BT::InputPort<int>("num_points", 100, "Number of poses in generated path"),

    // 坐标系 / TF
    BT::InputPort<std::string>("frame_id", "map", "Path header frame"),
    BT::InputPort<std::string>("map_frame", "map", "Map TF frame"),
    BT::InputPort<std::string>("base_frame", "base_footprint", "Robot base TF frame"),
    BT::InputPort<double>("tf_timeout", 0.2, "TF lookup timeout (s)"),
    BT::InputPort<std::string>("tf_namespace", "AT_R2",
      "TF namespace for fallback listener"),

    // action
    BT::InputPort<std::string>("action_name", "/AT_R2/follow_path",
      "Controller server FollowPath action name"),
    BT::InputPort<std::string>("controller_id", "FollowPath",
      "Controller plugin id on controller_server"),
    BT::InputPort<std::string>("goal_checker_id", "general_goal_checker",
      "Goal checker plugin id on controller_server"),

    // 输出
    BT::OutputPort<geometry_msgs::msg::PoseStamped>("goal", "{goal}",
      "Final goal pose")
  };
}

bool FollowSpinPathAction::ensureInitialized()
{
  if (initialized_) {
    return true;
  }

  if (!getInput("node", node_) || !node_) {
    RCLCPP_ERROR(rclcpp::get_logger("FollowSpinPathAction"),
      "Missing or null required input [node]");
    return false;
  }

  // 优先用 simple_bt_runner 注入的共享 tf_buffer
  auto bb = config().blackboard;
  if (bb) {
    std::shared_ptr<tf2_ros::Buffer> shared_buf;
    if (bb->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer", shared_buf) && shared_buf) {
      tf_buffer_ = shared_buf;
      RCLCPP_INFO(node_->get_logger(),
        "FollowSpinPath: using shared tf_buffer from blackboard");
    }
  }

  // 兜底自建
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

    tf_node_ = rclcpp::Node::make_shared("follow_spin_path_tf", "", tf_node_opts);
    RCLCPP_WARN(node_->get_logger(),
      "FollowSpinPath: blackboard has no tf_buffer, falling back to local listener");
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, tf_node_, true);
  }

  std::string action_name = "/AT_R2/follow_path";
  (void)getInput("action_name", action_name);
  action_client_ = rclcpp_action::create_client<FollowPath>(node_, action_name);

  auto goal_qos = rclcpp::QoS(1).transient_local().reliable();
  current_goal_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
    "/AT_R2/current_nav_goal", goal_qos);

  RCLCPP_INFO(node_->get_logger(),
    "FollowSpinPathAction initialized with FollowPath action: %s",
    action_name.c_str());

  initialized_ = true;
  return true;
}

bool FollowSpinPathAction::readCurrentPose(
  const std::string & map_frame,
  const std::string & base_frame,
  double tf_timeout_s,
  double & sx, double & sy, double & syaw)
{
  geometry_msgs::msg::TransformStamped tf_mb;
  try {
    const tf2::Duration tf_timeout = tf2::durationFromSec(std::max(0.0, tf_timeout_s));
    if (!tf_buffer_->canTransform(
        map_frame, base_frame, tf2::TimePointZero, tf_timeout))
    {
      RCLCPP_ERROR(node_->get_logger(),
        "FollowSpinPath: TF unavailable %s -> %s",
        base_frame.c_str(), map_frame.c_str());
      return false;
    }
    tf_mb = tf_buffer_->lookupTransform(map_frame, base_frame, tf2::TimePointZero);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node_->get_logger(),
      "FollowSpinPath: TF lookup failed: %s", e.what());
    return false;
  }
  sx = tf_mb.transform.translation.x;
  sy = tf_mb.transform.translation.y;
  tf2::Quaternion q(
    tf_mb.transform.rotation.x,
    tf_mb.transform.rotation.y,
    tf_mb.transform.rotation.z,
    tf_mb.transform.rotation.w);
  double roll = 0.0, pitch = 0.0;
  tf2::Matrix3x3(q).getRPY(roll, pitch, syaw);
  return true;
}

double FollowSpinPathAction::normalizeAngle(double a)
{
  while (a > M_PI) {
    a -= 2.0 * M_PI;
  }
  while (a < -M_PI) {
    a += 2.0 * M_PI;
  }
  return a;
}

nav_msgs::msg::Path FollowSpinPathAction::buildPath(
  double sx, double sy, double syaw,
  double gx, double gy, double gyaw,
  double rot_start_d, double rot_end_d,
  const std::string & direction,
  int num_points,
  const std::string & frame_id)
{
  nav_msgs::msg::Path path;
  path.header.stamp = node_->now();
  path.header.frame_id = frame_id;

  if (num_points < 2) {
    num_points = 2;
  }

  const double dx = gx - sx;
  const double dy = gy - sy;
  const double total_len = std::hypot(dx, dy);

  // 计算 yaw 的 signed delta, 由 direction 决定走哪条路
  // 先取最短角差作为基准, 然后按 cw/ccw 调整到对应符号
  double delta_short = normalizeAngle(gyaw - syaw);
  double delta;
  if (direction == "cw") {
    // 顺时针: yaw 应该减小, delta 为负
    delta = (delta_short > 0.0) ? (delta_short - 2.0 * M_PI) : delta_short;
  } else {
    // ccw (默认): yaw 应该增大, delta 为正
    delta = (delta_short < 0.0) ? (delta_short + 2.0 * M_PI) : delta_short;
  }

  // 防御: 旋转区间不能挤掉中间段
  double rot_s = std::max(0.0, rot_start_d);
  double rot_e = std::max(0.0, rot_end_d);
  if (rot_s + rot_e >= total_len) {
    // 区间无效, 退化为全程线性插值
    RCLCPP_WARN(node_->get_logger(),
      "FollowSpinPath: rotation_start+end (%.2f+%.2f) >= path length (%.2f), "
      "fall back to interpolate over full path",
      rot_s, rot_e, total_len);
    rot_s = 0.0;
    rot_e = 0.0;
  }
  const double rot_len = total_len - rot_s - rot_e;  // 中间段长度

  for (int i = 0; i < num_points; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(num_points - 1);
    const double s = t * total_len;  // 弧长

    geometry_msgs::msg::PoseStamped p;
    p.header = path.header;
    p.pose.position.x = sx + t * dx;
    p.pose.position.y = sy + t * dy;
    p.pose.position.z = 0.0;

    double yaw;
    if (s <= rot_s || rot_len <= 1e-6) {
      yaw = syaw;
    } else if (s >= total_len - rot_e) {
      yaw = syaw + delta;
    } else {
      const double u = (s - rot_s) / rot_len;  // 0..1 在中间段的进度
      yaw = syaw + u * delta;
    }

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);
    p.pose.orientation = tf2::toMsg(q);
    path.poses.push_back(p);
  }

  return path;
}

BT::NodeStatus FollowSpinPathAction::onStart()
{
  if (!ensureInitialized()) {
    return BT::NodeStatus::FAILURE;
  }

  double gx, gy, gyaw;
  if (!getInput("goal_x", gx) || !getInput("goal_y", gy) || !getInput("goal_yaw", gyaw)) {
    RCLCPP_ERROR(node_->get_logger(),
      "FollowSpinPath: missing required input [goal_x/goal_y/goal_yaw]");
    return BT::NodeStatus::FAILURE;
  }

  double rot_s = 0.0, rot_e = 0.0;
  if (!getInput("rotation_start_distance", rot_s) ||
      !getInput("rotation_end_distance", rot_e))
  {
    RCLCPP_ERROR(node_->get_logger(),
      "FollowSpinPath: missing required input "
      "[rotation_start_distance/rotation_end_distance]");
    return BT::NodeStatus::FAILURE;
  }

  std::string direction = "ccw";
  (void)getInput("rotation_direction", direction);
  if (direction != "cw" && direction != "ccw") {
    RCLCPP_ERROR(node_->get_logger(),
      "FollowSpinPath: rotation_direction must be 'cw' or 'ccw', got '%s'",
      direction.c_str());
    return BT::NodeStatus::FAILURE;
  }

  int num_points = 100;
  (void)getInput("num_points", num_points);

  std::string frame_id = "map";
  std::string map_frame = "map";
  std::string base_frame = "base_footprint";
  double tf_timeout_s = 0.2;
  (void)getInput("frame_id", frame_id);
  (void)getInput("map_frame", map_frame);
  (void)getInput("base_frame", base_frame);
  (void)getInput("tf_timeout", tf_timeout_s);

  // 读起点
  double sx = 0.0, sy = 0.0, syaw = 0.0;
  if (!readCurrentPose(map_frame, base_frame, tf_timeout_s, sx, sy, syaw)) {
    return BT::NodeStatus::FAILURE;
  }

  // 生成 path
  auto path = buildPath(sx, sy, syaw, gx, gy, gyaw,
                        rot_s, rot_e, direction, num_points, frame_id);
  if (path.poses.empty()) {
    RCLCPP_ERROR(node_->get_logger(), "FollowSpinPath: generated empty path");
    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_INFO(node_->get_logger(),
    "FollowSpinPath: start=(%.2f,%.2f,yaw=%.2f) goal=(%.2f,%.2f,yaw=%.2f) "
    "rot_window=[%.2f,%.2f]m dir=%s poses=%zu",
    sx, sy, syaw, gx, gy, gyaw, rot_s, rot_e, direction.c_str(), path.poses.size());

  // 等 controller server
  if (!action_client_->wait_for_action_server(std::chrono::seconds(10))) {
    RCLCPP_ERROR(node_->get_logger(),
      "FollowSpinPath: FollowPath action server not available after 10s");
    return BT::NodeStatus::FAILURE;
  }

  // 发 goal
  auto action_goal = FollowPath::Goal();
  action_goal.path = path;
  (void)getInput("controller_id", action_goal.controller_id);
  (void)getInput("goal_checker_id", action_goal.goal_checker_id);

  auto send_goal_options = rclcpp_action::Client<FollowPath>::SendGoalOptions();

  send_goal_options.goal_response_callback =
    [this](const GoalHandleFollowPath::SharedPtr & gh) {
      if (!gh) {
        RCLCPP_ERROR(node_->get_logger(), "FollowSpinPath: goal rejected by server");
        goal_handle_ = nullptr;
      } else {
        RCLCPP_INFO(node_->get_logger(), "FollowSpinPath: goal accepted, following...");
        goal_handle_ = gh;
      }
    };

  send_goal_options.result_callback =
    [this](const GoalHandleFollowPath::WrappedResult & result) {
      goal_result_available_ = true;
      result_ = result;
      switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
          RCLCPP_INFO(node_->get_logger(), "✓ FollowSpinPath succeeded!");
          break;
        case rclcpp_action::ResultCode::ABORTED:
          RCLCPP_ERROR(node_->get_logger(), "✗ FollowSpinPath aborted");
          break;
        case rclcpp_action::ResultCode::CANCELED:
          RCLCPP_WARN(node_->get_logger(), "FollowSpinPath canceled");
          break;
        default:
          RCLCPP_ERROR(node_->get_logger(), "FollowSpinPath unknown result code");
          break;
      }
    };

  send_goal_options.feedback_callback =
    [this](GoalHandleFollowPath::SharedPtr,
           const std::shared_ptr<const FollowPath::Feedback> feedback) {
      RCLCPP_DEBUG(node_->get_logger(),
        "FollowSpinPath: feedback speed=%.2f, dist_to_goal=%.2f",
        feedback->speed, feedback->distance_to_goal);
    };

  goal_result_available_ = false;
  action_client_->async_send_goal(action_goal, send_goal_options);

  // 写终点 pose 到 blackboard & latched 话题
  const auto & final_goal = path.poses.back();
  setOutput("goal", final_goal);
  if (current_goal_pub_) {
    current_goal_pub_->publish(final_goal);
  }

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus FollowSpinPathAction::onRunning()
{
  if (goal_result_available_) {
    if (result_.code == rclcpp_action::ResultCode::SUCCEEDED) {
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::RUNNING;
}

void FollowSpinPathAction::onHalted()
{
  if (goal_handle_) {
    RCLCPP_INFO(node_->get_logger(), "Canceling FollowSpinPath goal...");
    action_client_->async_cancel_goal(goal_handle_);
    goal_handle_ = nullptr;
  }
  goal_result_available_ = false;
}

}  // namespace nav2_bt_publish_goal

#include "behaviortree_cpp/bt_factory.h"

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_bt_publish_goal::FollowSpinPathAction>("FollowSpinPath");
}
