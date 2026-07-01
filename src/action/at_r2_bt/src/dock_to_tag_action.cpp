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

// 把 odom 系下从 t0 到 t 的相对运动，叠加到 t0 时刻 tag 锚定的 dock 系位姿上。
// dock 与 odom 都是世界固定系，二者间的常量旋转偏置在 t0 锚定时自动消去。
// T_dock_base(t) = T_dock_base(t0) * T_odom_base(t0)^-1 * T_odom_base(t)
void composeDeadReckon(
  double dock_x0, double dock_y0, double dock_yaw0,
  double odom_x0, double odom_y0, double odom_yaw0,
  double odom_x, double odom_y, double odom_yaw,
  double & out_x, double & out_y, double & out_yaw)
{
  // A) base(t) 相对 base(t0) 的位移/转角（在 base(t0) 系下表达）
  const double dyaw = wrapAngle(odom_yaw - odom_yaw0);
  const double ddx = odom_x - odom_x0;
  const double ddy = odom_y - odom_y0;
  const double c0 = std::cos(odom_yaw0);
  const double s0 = std::sin(odom_yaw0);
  const double dx_b = c0 * ddx + s0 * ddy;
  const double dy_b = -s0 * ddx + c0 * ddy;

  // B) 叠加到 dock 系锚点位姿
  const double cr = std::cos(dock_yaw0);
  const double sr = std::sin(dock_yaw0);
  out_x = dock_x0 + (cr * dx_b - sr * dy_b);
  out_y = dock_y0 + (sr * dx_b + cr * dy_b);
  out_yaw = wrapAngle(dock_yaw0 + dyaw);
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
    BT::InputPort<int>("lock_samples", 0,
      "If >0: sample N tag frames at start, lock target into odom, then dead-reckon only (no live TF)"),
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
  (void)getInput("lock_samples", lock_samples_);

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
  has_anchor_ = false;
  anchor_dock_x_ = anchor_dock_y_ = anchor_dock_yaw_ = 0.0;
  anchor_odom_x_ = anchor_odom_y_ = anchor_odom_yaw_ = 0.0;
  locked_ = false;
  lock_count_ = 0;
  lock_sum_x_ = lock_sum_y_ = lock_sum_sin_ = lock_sum_cos_ = 0.0;
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

bool DockToTagAction::collectLockSample(const rclcpp::Time & now)
{
  // 取一帧新鲜的 tag 位姿；不新鲜/取不到则本周期不计数。
  double rx = 0.0, ry = 0.0, ryaw = 0.0;
  rclcpp::Time tf_stamp;
  if (!lookupRobotInDock(rx, ry, ryaw, tf_stamp)) {return false;}
  if (tag_max_age_ > 0.0 && (now - tf_stamp).seconds() > tag_max_age_) {return false;}

  lock_sum_x_ += rx;
  lock_sum_y_ += ry;
  lock_sum_sin_ += std::sin(ryaw);  // yaw 用圆均值，避免 ±π 处跳变
  lock_sum_cos_ += std::cos(ryaw);
  lock_count_++;
  RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 200,
    "DockToTag: locking sample %d/%d", lock_count_, lock_samples_);

  if (lock_count_ < lock_samples_) {return false;}

  // 采够 N 帧：平均 dock 位姿，并捕获当前 odom 作为锚点。
  // 采样期间未下发任何速度，机器人静止，故 odom 取此刻即代表锚定时刻。
  const double n = static_cast<double>(lock_count_);
  const double avg_x = lock_sum_x_ / n;
  const double avg_y = lock_sum_y_ / n;
  const double avg_yaw = std::atan2(lock_sum_sin_, lock_sum_cos_);

  double ox = 0.0, oy = 0.0, oyaw = 0.0;
  bool odom_ok = false;
  {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    if (odom_received_) {
      ox = odom_cur_x_;
      oy = odom_cur_y_;
      oyaw = odom_cur_yaw_;
      odom_ok = true;
    }
  }
  if (!odom_ok) {
    // odom 尚未就绪：作废本轮平均，重新采样，等 odom 到位再锁。
    lock_count_ = 0;
    lock_sum_x_ = lock_sum_y_ = lock_sum_sin_ = lock_sum_cos_ = 0.0;
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 500,
      "DockToTag: tag samples ready but odom not received, retrying");
    return false;
  }

  anchor_dock_x_ = avg_x;
  anchor_dock_y_ = avg_y;
  anchor_dock_yaw_ = avg_yaw;
  anchor_odom_x_ = ox;
  anchor_odom_y_ = oy;
  anchor_odom_yaw_ = oyaw;
  has_anchor_ = true;
  locked_ = true;
  has_last_pose_ = true;
  last_robot_x_in_dock_ = avg_x;
  last_robot_y_in_dock_ = avg_y;
  last_robot_yaw_in_dock_ = avg_yaw;
  RCLCPP_INFO(node_->get_logger(),
    "DockToTag: LOCKED target into odom from %d samples -> dock_anchor=(%.4f,%.4f,%.3f)",
    lock_count_, avg_x, avg_y, avg_yaw);
  return true;
}

void DockToTagAction::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(odom_mutex_);
  odom_cur_x_ = msg->pose.pose.position.x;
  odom_cur_y_ = msg->pose.pose.position.y;
  {
    tf2::Quaternion q(
      msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    double roll, pitch;
    tf2::Matrix3x3(q).getRPY(roll, pitch, odom_cur_yaw_);
  }
  if (!odom_received_) {
    odom_ref_x_ = odom_cur_x_;
    odom_ref_y_ = odom_cur_y_;
    odom_ref_time_ = node_->now();
    odom_received_ = true;
  }
}

BT::NodeStatus DockToTagAction::onRunning()
{
  const auto now = node_->now();

  if ((now - start_time_).seconds() > timeout_) {
    RCLCPP_WARN(node_->get_logger(), "DockToTag timeout after %.1fs", timeout_);
    stopAll();
    return BT::NodeStatus::FAILURE;
  }

  // 1) 取我方在 dock 系下的位姿
  double rx = 0.0, ry = 0.0, ryaw = 0.0;

  if (lock_samples_ > 0) {
    // —— 快照模式：起步采 N 帧 tag 平均锁进 odom；锁定后只用 odom 推算，不再读 TF ——
    if (!locked_ && !collectLockSample(now)) {
      // 仍在采样(或等 TF/odom)：保持静止，等待锁定
      std::lock_guard<std::mutex> lock(cmd_mutex_);
      cmd_ = geometry_msgs::msg::Twist();
      return BT::NodeStatus::RUNNING;
    }
    double ox = 0.0, oy = 0.0, oyaw = 0.0;
    bool odom_ok_now = false;
    {
      std::lock_guard<std::mutex> lock(odom_mutex_);
      if (odom_received_) {
        ox = odom_cur_x_;
        oy = odom_cur_y_;
        oyaw = odom_cur_yaw_;
        odom_ok_now = true;
      }
    }
    if (!odom_ok_now) {
      // 锁定后 odom 断流：无法推算，保持静止等待恢复
      std::lock_guard<std::mutex> lock(cmd_mutex_);
      cmd_ = geometry_msgs::msg::Twist();
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 500,
        "DockToTag: locked but odom lost, holding still");
      return BT::NodeStatus::RUNNING;
    }
    // 用 odom 增量把锁定的 dock 锚点推到当前位姿
    composeDeadReckon(
      anchor_dock_x_, anchor_dock_y_, anchor_dock_yaw_,
      anchor_odom_x_, anchor_odom_y_, anchor_odom_yaw_,
      ox, oy, oyaw,
      rx, ry, ryaw);
  } else {
    // —— 实时 TF 模式(原行为)：每周期读 TF，丢失时退到 odom 航位推算 ——
    rclcpp::Time tf_stamp;
    bool tf_ok = lookupRobotInDock(rx, ry, ryaw, tf_stamp);
    if (tf_ok && tag_max_age_ > 0.0) {
      const double age = (now - tf_stamp).seconds();
      if (age > tag_max_age_) {tf_ok = false;}
    }

    if (tf_ok) {
      last_robot_x_in_dock_ = rx;
      last_robot_y_in_dock_ = ry;
      last_robot_yaw_in_dock_ = ryaw;
      has_last_pose_ = true;
      last_valid_tf_time_ = now;

      // 记录 dock 锚点 + 同一时刻的 odom 锚点，供 TF 丢失时航位推算。
      double ox = 0.0, oy = 0.0, oyaw = 0.0;
      bool odom_ok_now = false;
      {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        if (odom_received_) {
          ox = odom_cur_x_;
          oy = odom_cur_y_;
          oyaw = odom_cur_yaw_;
          odom_ok_now = true;
        }
      }
      if (odom_ok_now) {
        anchor_dock_x_ = rx;
        anchor_dock_y_ = ry;
        anchor_dock_yaw_ = ryaw;
        anchor_odom_x_ = ox;
        anchor_odom_y_ = oy;
        anchor_odom_yaw_ = oyaw;
        has_anchor_ = true;
      }
    } else {
      const double lost_for = (now - last_valid_tf_time_).seconds();
      if (!has_last_pose_ || lost_for > tag_lost_freeze_time_) {
        RCLCPP_ERROR(node_->get_logger(),
          "DockToTag: TF lost for %.2fs > %.2fs, abort", lost_for, tag_lost_freeze_time_);
        stopAll();
        return BT::NodeStatus::FAILURE;
      }

      // 读取当前 odom 活体位姿（跨线程，必须持锁），用于把 dock 位姿往前推。
      double ox = 0.0, oy = 0.0, oyaw = 0.0;
      bool odom_ok_now = false;
      {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        if (odom_received_) {
          ox = odom_cur_x_;
          oy = odom_cur_y_;
          oyaw = odom_cur_yaw_;
          odom_ok_now = true;
        }
      }
      if (has_anchor_ && odom_ok_now) {
        // 航位推算：把 odom 测得的相对运动叠加到 tag 锚定的 dock 位姿上。
        composeDeadReckon(
          anchor_dock_x_, anchor_dock_y_, anchor_dock_yaw_,
          anchor_odom_x_, anchor_odom_y_, anchor_odom_yaw_,
          ox, oy, oyaw,
          rx, ry, ryaw);
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 500,
          "DockToTag: TF stale %.2fs, dead-reckon via odom -> dock=(%.4f,%.4f,%.3f)",
          lost_for, rx, ry, ryaw);
      } else {
        // 无锚点或无 odom：退回冻结上一次 dock 位姿（旧行为）。
        rx = last_robot_x_in_dock_;
        ry = last_robot_y_in_dock_;
        ryaw = last_robot_yaw_in_dock_;
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 500,
          "DockToTag: TF stale %.2fs, no odom anchor, freezing last pose", lost_for);
      }
    }
  }

  // 2) 在 dock 系下算误差
  const double e_y = 0.0 - ry;
  const double e_yaw = wrapAngle(target_yaw_in_dock_ - ryaw);
  const double target_x = (phase_ == Phase::ALIGN) ? pre_dock_offset_x_ : 0.0;
  const double e_x = target_x - rx;

  // 3) 状态机切换（带滞回）
  const bool aligned =
    (std::fabs(e_y) < align_tol_lat_) && (std::fabs(e_yaw) < align_tol_yaw_);
  const bool fallen_back =
    (std::fabs(e_y) > align_hyst_lat_) || (std::fabs(e_yaw) > align_hyst_yaw_);

  if (phase_ == Phase::ALIGN && aligned) {
    phase_ = Phase::APPROACH;
    {
      std::lock_guard<std::mutex> lock(odom_mutex_);
      if (odom_received_) {
        odom_ref_x_ = odom_cur_x_;
        odom_ref_y_ = odom_cur_y_;
        odom_ref_time_ = now;
      }
    }
    stall_timer_active_ = false;
    RCLCPP_INFO(node_->get_logger(), "DockToTag: ALIGN -> APPROACH");
  } else if (phase_ == Phase::APPROACH && fallen_back) {
    phase_ = Phase::ALIGN;
    stall_timer_active_ = false;
    RCLCPP_WARN(node_->get_logger(),
      "DockToTag: APPROACH -> ALIGN (e_y=%.4f e_yaw=%.3f)", e_y, e_yaw);
  }

  // 4) APPROACH 阶段判到位（几何 OR stall）
  if (phase_ == Phase::APPROACH) {
    const bool geom_done = (std::fabs(e_x) < approach_tol_) && aligned;
    bool stall_done = false;
    {
      std::lock_guard<std::mutex> lock(odom_mutex_);
      if (odom_received_) {
        const double moved =
          std::hypot(odom_cur_x_ - odom_ref_x_, odom_cur_y_ - odom_ref_y_);
        if (moved < stall_threshold_) {
          if (!stall_timer_active_) {
            stall_start_time_ = now;
            stall_timer_active_ = true;
          } else if ((now - stall_start_time_).seconds() > stall_duration_) {
            stall_done = true;
          }
        } else {
          odom_ref_x_ = odom_cur_x_;
          odom_ref_y_ = odom_cur_y_;
          odom_ref_time_ = now;
          stall_timer_active_ = false;
        }
      }
    }
    if (geom_done || stall_done) {
      RCLCPP_INFO(node_->get_logger(),
        "DockToTag SUCCESS via %s: e_x=%.4f e_y=%.4f e_yaw=%.3frad",
        geom_done ? "geometry" : "stall", e_x, e_y, e_yaw);
      stopAll();
      return BT::NodeStatus::SUCCESS;
    }
  }

  // 5) dock 系下的目标速度（纯 P）
  double vx_dock = kp_x_ * e_x;
  double vy_dock = kp_y_ * e_y;
  double wz_cmd = kp_yaw_ * e_yaw;

  if (phase_ == Phase::ALIGN) {
    // 软门控：横向/yaw 偏得多就压住推进速度
    const double gy =
      (gate_sigma_lat_ > 0.0) ? (e_y / gate_sigma_lat_) : 0.0;
    const double gyaw =
      (gate_sigma_yaw_ > 0.0) ? (e_yaw / gate_sigma_yaw_) : 0.0;
    const double gate = std::exp(-(gy * gy + gyaw * gyaw));
    vx_dock *= gate;
  } else {
    // APPROACH：减速带 + 最低推进速度
    const double abs_ex = std::fabs(e_x);
    if (abs_ex > approach_tol_) {
      double v_mag = std::fabs(vx_dock);
      if (approach_decel_dist_ > 0.0 && abs_ex < approach_decel_dist_) {
        v_mag = std::min(v_mag, max_vx_ * abs_ex / approach_decel_dist_);
      }
      v_mag = std::max(v_mag, approach_min_speed_);
      vx_dock = std::copysign(v_mag, vx_dock);
    } else {
      vx_dock = 0.0;
    }
  }

  // 6) dock 系 -> body 系：v_body = R(-ryaw) * v_dock
  const double c = std::cos(ryaw);
  const double s = std::sin(ryaw);
  double vx_body = c * vx_dock + s * vy_dock;
  double vy_body = -s * vx_dock + c * vy_dock;

  // 7) 饱和
  vx_body = saturate(vx_body, max_vx_);
  vy_body = saturate(vy_body, max_vy_);
  wz_cmd = saturate(wz_cmd, max_wz_);

  // 8) 斜率限幅
  const double dt = std::max(0.001, (now - prev_cmd_time_).seconds());
  vx_body = slewLimit(prev_vx_body_, vx_body, accel_vx_, dt);
  vy_body = slewLimit(prev_vy_body_, vy_body, accel_vy_, dt);
  wz_cmd = slewLimit(prev_wz_, wz_cmd, accel_wz_, dt);
  prev_vx_body_ = vx_body;
  prev_vy_body_ = vy_body;
  prev_wz_ = wz_cmd;
  prev_cmd_time_ = now;

  // 9) 写入定时器要发的指令
  {
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    cmd_.linear.x = vx_body;
    cmd_.linear.y = vy_body;
    cmd_.angular.z = wz_cmd;
  }

  RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 500,
    "DockToTag[%s] e=(%.4f,%.4f,%.3f) v_body=(%.3f,%.3f,%.3f)",
    phase_ == Phase::ALIGN ? "ALIGN" : "APPROACH",
    e_x, e_y, e_yaw, vx_body, vy_body, wz_cmd);

  return BT::NodeStatus::RUNNING;
}

void DockToTagAction::publishCmdTimerCallback()
{
  geometry_msgs::msg::Twist out;
  {
    std::lock_guard<std::mutex> lock(publish_timer_mutex_);
    if (!publish_timer_armed_ || !cmd_vel_pub_) {
      return;
    }
    std::lock_guard<std::mutex> data_lock(cmd_mutex_);
    out = cmd_;
  }
  cmd_vel_pub_->publish(out);
}

void DockToTagAction::publishZero()
{
  if (!cmd_vel_pub_) {return;}
  geometry_msgs::msg::Twist zero;
  cmd_vel_pub_->publish(zero);
}

void DockToTagAction::stopAll()
{
  {
    std::lock_guard<std::mutex> lock(publish_timer_mutex_);
    publish_timer_armed_ = false;
    if (publish_timer_) {
      publish_timer_->cancel();
      publish_timer_.reset();
    }
  }
  {
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    cmd_ = geometry_msgs::msg::Twist();
  }
  publishZero();
  if (odom_sub_) {
    odom_sub_.reset();
  }
}

void DockToTagAction::onHalted()
{
  stopAll();
  RCLCPP_INFO(node_->get_logger(), "DockToTag halted, sent zero velocity");
}

}  // namespace nav2_bt_publish_goal

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_bt_publish_goal::DockToTagAction>("DockToTag");
}
