// Copyright 2024
// Licensed under the Apache License, Version 2.0

#include "nav2_bt_publish_goal/dock_to_tag_action.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>

#include "behaviortree_cpp/bt_factory.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/time.h"

namespace nav2_bt_publish_goal
{

namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kDeg2Rad = kPi / 180.0;

double wrapAngle(double a)
{
  while (a > kPi) {a -= 2.0 * kPi;}
  while (a < -kPi) {a += 2.0 * kPi;}
  return a;
}

double saturate(double v, double lim)
{
  if (lim <= 0.0) {return 0.0;}
  return std::clamp(v, -lim, lim);
}

double slewLimit(double prev, double target, double accel, double dt)
{
  if (accel <= 0.0 || dt <= 0.0) {return target;}
  const double max_step = accel * dt;
  const double diff = target - prev;
  if (diff > max_step) {return prev + max_step;}
  if (diff < -max_step) {return prev - max_step;}
  return target;
}
}  // namespace

DockToTagAction::DockToTagAction(const std::string & name, const BT::NodeConfig & config)
: BT::StatefulActionNode(name, config),
  start_time_(0, 0, RCL_ROS_TIME),
  last_valid_tf_time_(0, 0, RCL_ROS_TIME),
  stall_start_time_(0, 0, RCL_ROS_TIME),
  prev_cmd_time_(0, 0, RCL_ROS_TIME),
  odom_ref_time_(0, 0, RCL_ROS_TIME)
{
}

BT::PortsList DockToTagAction::providedPorts()
{
  return {
    BT::InputPort<rclcpp::Node::SharedPtr>("node", "ROS node"),
    BT::InputPort<std::string>("target_frame", "dock_target", "Dock target TF frame (from vision)"),
    BT::InputPort<std::string>("base_frame", "base_footprint", "Robot base frame"),
    BT::InputPort<std::string>("cmd_vel_topic", "/AT_R2/cmd_vel_nav2_result", "cmd_vel topic"),
    BT::InputPort<std::string>("odom_topic", "/AT_R2/odometry", "Odometry topic for stall"),
    BT::InputPort<double>("pre_dock_offset_x", -0.5,
      "Pre-dock x in dock frame (signed; negative=approach from -X side)"),
    BT::InputPort<double>("target_yaw_in_dock_deg", -90.0, "Target robot yaw in dock frame (deg)"),
    BT::InputPort<double>("align_tol_lat", 0.005, "Lateral tolerance to enter APPROACH (m)"),
    BT::InputPort<double>("align_tol_yaw_deg", 0.5, "Yaw tolerance to enter APPROACH (deg)"),
    BT::InputPort<double>("approach_tol", 0.01, "Longitudinal tolerance for SUCCESS (m)"),
    BT::InputPort<double>("align_hyst_lat", 0.015, "Lateral hysteresis to fall back to ALIGN (m)"),
    BT::InputPort<double>("align_hyst_yaw_deg", 1.5, "Yaw hysteresis to fall back to ALIGN (deg)"),
    BT::InputPort<double>("kp_x", 1.0, "P gain along dock X (approach)"),
    BT::InputPort<double>("kp_y", 1.5, "P gain along dock Y (lateral)"),
    BT::InputPort<double>("kp_yaw", 2.0, "P gain on yaw"),
    BT::InputPort<double>("max_vx", 0.10, "Max body vx (m/s)"),
    BT::InputPort<double>("max_vy", 0.10, "Max body vy (m/s)"),
    BT::InputPort<double>("max_wz", 0.5, "Max angular speed (rad/s)"),
    BT::InputPort<double>("accel_vx", 0.3, "Slew limit on body vx (m/s^2)"),
    BT::InputPort<double>("accel_vy", 0.3, "Slew limit on body vy (m/s^2)"),
    BT::InputPort<double>("accel_wz", 1.5, "Slew limit on yaw rate (rad/s^2)"),
    BT::InputPort<double>("approach_min_speed", 0.015, "Min approach speed when error>tol (m/s)"),
    BT::InputPort<double>("approach_decel_dist", 0.05, "Decel ramp distance (m)"),
    BT::InputPort<double>("gate_sigma_lat", 0.015, "Soft gate scale on lateral error (m)"),
    BT::InputPort<double>("gate_sigma_yaw_deg", 3.0, "Soft gate scale on yaw error (deg)"),
    BT::InputPort<double>("stall_threshold", 0.003, "Stall position-change threshold (m)"),
    BT::InputPort<double>("stall_duration", 0.4, "Stall confirm duration (s)"),
    BT::InputPort<double>("tag_max_age", 0.5, "Max acceptable TF age (s)"),
    BT::InputPort<double>("tag_lost_freeze_time", 1.0,
      "If TF lost for this long, fail; below this hold last pose"),
    BT::InputPort<double>("timeout", 30.0, "Overall timeout (s)"),
    BT::InputPort<double>("publish_rate_hz", 50.0, "cmd_vel publish rate (Hz)"),
  };
}

BT::NodeStatus DockToTagAction::onStart()
{
  if (!node_) {
    if (!getInput("node", node_) || !node_) {
      RCLCPP_ERROR(rclcpp::get_logger("DockToTag"), "Missing/null required input [node]");
      return BT::NodeStatus::FAILURE;
    }
  }

  target_frame_ = "dock_target";
  base_frame_ = "base_footprint";
  cmd_vel_topic_ = "/AT_R2/cmd_vel_nav2_result";
  odom_topic_ = "/AT_R2/odometry";
  double target_yaw_deg = -90.0;
  double align_tol_yaw_deg = 0.5;
  double align_hyst_yaw_deg = 1.5;
  double gate_sigma_yaw_deg = 3.0;
  double publish_rate_hz = 50.0;

  (void)getInput("target_frame", target_frame_);
  (void)getInput("base_frame", base_frame_);
  (void)getInput("cmd_vel_topic", cmd_vel_topic_);
  (void)getInput("odom_topic", odom_topic_);
  (void)getInput("pre_dock_offset_x", pre_dock_offset_x_);
  (void)getInput("target_yaw_in_dock_deg", target_yaw_deg);
  (void)getInput("align_tol_lat", align_tol_lat_);
  (void)getInput("align_tol_yaw_deg", align_tol_yaw_deg);
  (void)getInput("approach_tol", approach_tol_);
  (void)getInput("align_hyst_lat", align_hyst_lat_);
  (void)getInput("align_hyst_yaw_deg", align_hyst_yaw_deg);
  (void)getInput("kp_x", kp_x_);
  (void)getInput("kp_y", kp_y_);
  (void)getInput("kp_yaw", kp_yaw_);
  (void)getInput("max_vx", max_vx_);
  (void)getInput("max_vy", max_vy_);
  (void)getInput("max_wz", max_wz_);
  (void)getInput("accel_vx", accel_vx_);
  (void)getInput("accel_vy", accel_vy_);
  (void)getInput("accel_wz", accel_wz_);
  (void)getInput("approach_min_speed", approach_min_speed_);
  (void)getInput("approach_decel_dist", approach_decel_dist_);
  (void)getInput("gate_sigma_lat", gate_sigma_lat_);
  (void)getInput("gate_sigma_yaw_deg", gate_sigma_yaw_deg);
  (void)getInput("stall_threshold", stall_threshold_);
  (void)getInput("stall_duration", stall_duration_);
  (void)getInput("tag_max_age", tag_max_age_);
  (void)getInput("tag_lost_freeze_time", tag_lost_freeze_time_);
  (void)getInput("timeout", timeout_);
  (void)getInput("publish_rate_hz", publish_rate_hz);

  target_yaw_in_dock_ = target_yaw_deg * kDeg2Rad;
  align_tol_yaw_ = align_tol_yaw_deg * kDeg2Rad;
  align_hyst_yaw_ = align_hyst_yaw_deg * kDeg2Rad;
  gate_sigma_yaw_ = gate_sigma_yaw_deg * kDeg2Rad;
  if (publish_rate_hz < 1.0) {publish_rate_hz = 1.0;}

  if (!ensureTfBuffer()) {
    return BT::NodeStatus::FAILURE;
  }

  cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
  odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
    odom_topic_, rclcpp::SensorDataQoS(),
    std::bind(&DockToTagAction::odomCallback, this, std::placeholders::_1));

  {
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    cmd_ = geometry_msgs::msg::Twist();
  }
  prev_vx_body_ = prev_vy_body_ = prev_wz_ = 0.0;
  phase_ = Phase::ALIGN;
  has_last_pose_ = false;
  stall_timer_active_ = false;
  {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    odom_received_ = false;
  }

  start_time_ = node_->now();
  prev_cmd_time_ = start_time_;
  last_valid_tf_time_ = start_time_;

  const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz);
  const auto period_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(period);
  publish_timer_ = node_->create_wall_timer(
    period_ns, std::bind(&DockToTagAction::publishCmdTimerCallback, this));
  {
    std::lock_guard<std::mutex> lock(publish_timer_mutex_);
    publish_timer_armed_ = true;
  }

  RCLCPP_INFO(node_->get_logger(),
    "DockToTag started: target=%s base=%s pre_dock_x=%.3f target_yaw=%.1f deg "
    "tol(lat/yaw/long)=%.4f/%.2fdeg/%.3f cmd_topic=%s",
    target_frame_.c_str(), base_frame_.c_str(), pre_dock_offset_x_, target_yaw_deg,
    align_tol_lat_, align_tol_yaw_deg, approach_tol_, cmd_vel_topic_.c_str());

  return BT::NodeStatus::RUNNING;
}

bool DockToTagAction::ensureTfBuffer()
{
  auto bb = config().blackboard;
  if (bb) {
    std::shared_ptr<tf2_ros::Buffer> shared_buf;
    if (bb->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer", shared_buf) && shared_buf) {
      tf_buffer_ = shared_buf;
      return true;
    }
  }

  std::string tf_ns = "AT_R2";
  rclcpp::NodeOptions tf_node_opts;
  bool use_sim_time = false;
  node_->get_parameter_or("use_sim_time", use_sim_time, false);
  tf_node_opts.parameter_overrides({rclcpp::Parameter("use_sim_time", use_sim_time)});
  const std::string ns_prefix = (tf_ns.front() == '/') ? tf_ns : ("/" + tf_ns);
  tf_node_opts.arguments({
    "--ros-args",
    "-r", "/tf:=" + ns_prefix + "/tf",
    "-r", "/tf_static:=" + ns_prefix + "/tf_static",
  });
  tf_node_ = rclcpp::Node::make_shared("dock_to_tag_tf", "", tf_node_opts);
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, tf_node_, true);
  RCLCPP_WARN(node_->get_logger(),
    "DockToTag: blackboard has no tf_buffer, using local listener (remap /tf -> %s/tf)",
    ns_prefix.c_str());
  return true;
}

bool DockToTagAction::lookupRobotInDock(
  double & x, double & y, double & yaw, rclcpp::Time & stamp)
{
  const tf2::Duration tf_timeout = tf2::durationFromSec(0.05);
  if (!tf_buffer_->canTransform(target_frame_, base_frame_, tf2::TimePointZero, tf_timeout)) {
    return false;
  }
  geometry_msgs::msg::TransformStamped tf;
  try {
    tf = tf_buffer_->lookupTransform(target_frame_, base_frame_, tf2::TimePointZero);
  } catch (const tf2::TransformException & e) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
      "DockToTag TF lookup failed: %s", e.what());
    return false;
  }
  x = tf.transform.translation.x;
  y = tf.transform.translation.y;
  tf2::Quaternion q(
    tf.transform.rotation.x, tf.transform.rotation.y,
    tf.transform.rotation.z, tf.transform.rotation.w);
  double roll, pitch;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  stamp = tf.header.stamp;
  return true;
}

void DockToTagAction::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(odom_mutex_);
  odom_cur_x_ = msg->pose.pose.position.x;
  odom_cur_y_ = msg->pose.pose.position.y;
  if (!odom_received_) {
    odom_ref_x_ = odom_cur_x_;
    odom_ref_y_ = odom_cur_y_;
    odom_ref_time_ = node_->now();
    odom_received_ = true;
  }
}

}  // namespace nav2_bt_publish_goal
