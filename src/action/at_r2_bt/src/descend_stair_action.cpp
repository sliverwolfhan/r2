// Copyright 2024
// Licensed under the Apache License, Version 2.0

#include "nav2_bt_publish_goal/descend_stair_action.hpp"

#include <algorithm>
#include <cmath>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/time.h"

namespace nav2_bt_publish_goal
{

DescendStairAction::DescendStairAction(
  const std::string & name,
  const BT::NodeConfig & config)
: BT::StatefulActionNode(name, config),
  current_status_(0),
  status_received_(false)
{
  // 不在构造函数中获取 node，而是在 onStart 中获取
}

BT::PortsList DescendStairAction::providedPorts()
{
  return {
    BT::InputPort<rclcpp::Node::SharedPtr>("node", "ROS node"),
    BT::InputPort<double>("height", "Descend height in meters (must be positive)"),
    BT::OutputPort<double>("descend_height", "Actual published descend height value"),

    // ===== 下台阶期间朝向纠偏（只发 angular.z），默认关闭，不影响现有调用 =====
    BT::InputPort<bool>("yaw_correct_enable", false,
      "总开关：true 才在下台阶期间发纠偏角速度；false/缺省完全不发"),
    BT::InputPort<double>("target_yaw",
      "纠偏目标朝向 (rad, map系)，一般绑下台阶前的准备朝向 {prep_yaw}"),
    BT::InputPort<double>("yaw_kp", 1.5, "比例增益 (rad/s per rad)"),
    BT::InputPort<double>("yaw_wz_max", 0.4, "角速度上限 (rad/s)"),
    BT::InputPort<double>("yaw_engage_threshold", 0.1,
      "启动阈值 (rad)：偏差 |err| 超过此才开始纠偏"),
    BT::InputPort<double>("yaw_deadband", 0.02,
      "停止阈值 (rad)：纠偏中偏差回落到此以下才停 (迟滞下界，需 <= 启动阈值)"),
    BT::InputPort<double>("yaw_publish_rate_hz", 30.0, "纠偏 cmd_vel 发布频率 (Hz)"),
    BT::InputPort<std::string>("yaw_cmd_topic", "/AT_R2/cmd_vel_bt", "纠偏速度话题"),
    BT::InputPort<std::string>("yaw_base_frame", "base_link", "当前朝向的 base TF frame"),
    BT::InputPort<std::string>("yaw_map_frame", "map", "map TF frame"),
    BT::InputPort<double>("yaw_tf_timeout", 0.1, "TF 查询超时 (s)"),

    // ===== 朝向偏移角发布（独立开关，与 yaw_correct_enable 无关）=====
    BT::InputPort<bool>("offset_pub_enable", false,
      "总开关：true 才发布朝向偏移角 (cur_yaw - target_yaw) 到 offset_topic"),
    BT::InputPort<std::string>("offset_topic", "/AT_R2/offset_angle",
      "朝向偏移角发布话题 (std_msgs/Float32, rad)")
  };
}

double DescendStairAction::normalizeAngle(double a)
{
  while (a > M_PI) {
    a -= 2.0 * M_PI;
  }
  while (a < -M_PI) {
    a += 2.0 * M_PI;
  }
  return a;
}

bool DescendStairAction::ensureYawCorrection()
{
  // 每次 onStart 重新读端口（允许不同台阶用不同参数）
  yaw_correct_enable_ = false;
  offset_pub_enable_ = false;
  (void)getInput("yaw_correct_enable", yaw_correct_enable_);
  (void)getInput("offset_pub_enable", offset_pub_enable_);
  // 两个功能都关：无需定时器
  if (!yaw_correct_enable_ && !offset_pub_enable_) {
    return false;
  }

  // 纠偏和偏移角发布都需要 target_yaw（偏移角 = cur_yaw - target_yaw）
  if (!getInput("target_yaw", yaw_target_)) {
    RCLCPP_WARN(node_->get_logger(),
      "DescendStair: yaw_correct_enable/offset_pub_enable=true 但缺少 [target_yaw]，本次不启用");
    yaw_correct_enable_ = false;
    offset_pub_enable_ = false;
    return false;
  }

  // TF / 通用参数（两功能共用）
  (void)getInput("yaw_tf_timeout", yaw_tf_timeout_);
  (void)getInput("yaw_base_frame", yaw_base_frame_);
  (void)getInput("yaw_map_frame", yaw_map_frame_);
  if (yaw_tf_timeout_ < 0.0) {
    yaw_tf_timeout_ = 0.0;
  }

  // ===== 纠偏专属参数与 cmd_vel publisher（仅纠偏开启时才建）=====
  if (yaw_correct_enable_) {
    (void)getInput("yaw_kp", yaw_kp_);
    (void)getInput("yaw_wz_max", yaw_wz_max_);
    (void)getInput("yaw_engage_threshold", yaw_engage_threshold_);
    (void)getInput("yaw_deadband", yaw_deadband_);
    (void)getInput("yaw_cmd_topic", yaw_cmd_topic_);
    // 迟滞要求 启动阈值 >= 停止阈值，否则退化：夹到不小于死区
    if (yaw_engage_threshold_ < yaw_deadband_) {
      RCLCPP_WARN(node_->get_logger(),
        "DescendStair: yaw_engage_threshold(%.3f) < yaw_deadband(%.3f)，已夹到 deadband",
        yaw_engage_threshold_, yaw_deadband_);
      yaw_engage_threshold_ = yaw_deadband_;
    }
    yaw_engaged_ = false;  // 每次下台阶重置迟滞状态

    // cmd_vel publisher（话题变更时重建）
    if (!cmd_vel_pub_ || cmd_vel_pub_->get_topic_name() != yaw_cmd_topic_) {
      cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(yaw_cmd_topic_, 10);
    }
  }

  // ===== 偏移角 publisher（仅发布开启时才建，话题变更时重建）=====
  if (offset_pub_enable_) {
    (void)getInput("offset_topic", offset_topic_);
    if (!offset_pub_ || offset_pub_->get_topic_name() != offset_topic_) {
      offset_pub_ = node_->create_publisher<std_msgs::msg::Float32>(offset_topic_, 10);
    }
  }

  // TF buffer：优先复用根黑板注入的共享 buffer（与 IsPrepSkippable 同源）
  if (!tf_buffer_) {
    auto bb = config().blackboard;
    if (bb) {
      std::shared_ptr<tf2_ros::Buffer> shared_buf;
      if (bb->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer", shared_buf) && shared_buf) {
        tf_buffer_ = shared_buf;
      }
    }
    // 兜底：自建 listener
    if (!tf_buffer_) {
      tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, node_, true);
      RCLCPP_WARN(node_->get_logger(),
        "DescendStair: 黑板无 tf_buffer，自建本地 TF listener");
    }
  }
  return true;
}

BT::NodeStatus DescendStairAction::onStart()
{
  // Get the ROS node from the blackboard (first time initialization)
  if (!node_) {
    if (!getInput("node", node_)) {
      RCLCPP_ERROR(rclcpp::get_logger("DescendStairAction"), "Missing required input [node]");
      return BT::NodeStatus::FAILURE;
    }

    if (!node_) {
      RCLCPP_ERROR(rclcpp::get_logger("DescendStairAction"), "Node pointer is null");
      return BT::NodeStatus::FAILURE;
    }

    // Create publisher for descend command
    descend_pub_ = node_->create_publisher<std_msgs::msg::Float64>(
      "/AT_R2/descend_stair", 10);

    // Create subscriber for climber status
    status_sub_ = node_->create_subscription<std_msgs::msg::Int32>(
      "/AT_R2/climber_status", 10,
      std::bind(&DescendStairAction::statusCallback, this, std::placeholders::_1));

    RCLCPP_INFO(node_->get_logger(), "DescendStairAction initialized");
  }

  // Read height parameter
  double height;
  if (!getInput("height", height)) {
    RCLCPP_ERROR(node_->get_logger(), "Missing required input [height]");
    return BT::NodeStatus::FAILURE;
  }

  // Validate height parameter
  if (height <= 0.0) {
    RCLCPP_ERROR(node_->get_logger(),
      "Invalid height: %.2f (must be positive)", height);
    return BT::NodeStatus::FAILURE;
  }

  // Reset state
  current_status_ = 0;
  status_received_ = false;

  // Publish descend command
  auto msg = std_msgs::msg::Float64();
  msg.data = height;
  descend_pub_->publish(msg);

  RCLCPP_INFO(node_->get_logger(),
    "Published descend command: %.2f meters", height);

  // Set output port
  setOutput("descend_height", height);

  // ===== 按需启动朝向纠偏定时器 =====
  stopYawTimer();
  if (ensureYawCorrection()) {
    double rate_hz = 30.0;
    (void)getInput("yaw_publish_rate_hz", rate_hz);
    if (rate_hz < 1.0) {
      rate_hz = 1.0;
    }
    const auto period_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / rate_hz));
    {
      std::lock_guard<std::mutex> lock(yaw_timer_mutex_);
      yaw_timer_armed_ = true;
    }
    yaw_timer_ = node_->create_wall_timer(
      period_ns, std::bind(&DescendStairAction::yawCorrectTimerCallback, this));
    RCLCPP_INFO(node_->get_logger(),
      "DescendStair: 定时器启用 target_yaw=%.3f rate=%.1fHz | 纠偏=%s(kp=%.2f wz_max=%.2f topic=%s) | 偏移角=%s(topic=%s)",
      yaw_target_, rate_hz,
      yaw_correct_enable_ ? "on" : "off", yaw_kp_, yaw_wz_max_, yaw_cmd_topic_.c_str(),
      offset_pub_enable_ ? "on" : "off", offset_topic_.c_str());
  }

  return BT::NodeStatus::RUNNING;
}

void DescendStairAction::yawCorrectTimerCallback()
{
  {
    std::lock_guard<std::mutex> lock(yaw_timer_mutex_);
    // 需要 TF；纠偏需 cmd_vel_pub_，发偏移角需 offset_pub_，至少一个功能可用
    if (!yaw_timer_armed_ || !tf_buffer_) {
      return;
    }
    if ((!yaw_correct_enable_ || !cmd_vel_pub_) &&
        (!offset_pub_enable_ || !offset_pub_)) {
      return;
    }
  }

  // 拿 map -> base 当前 yaw
  double cur_yaw = 0.0;
  try {
    const tf2::Duration to = tf2::durationFromSec(yaw_tf_timeout_);
    if (!tf_buffer_->canTransform(
        yaw_map_frame_, yaw_base_frame_, tf2::TimePointZero, to)) {
      return;  // TF 暂时不可用：本拍不发，等下一拍
    }
    const auto tf = tf_buffer_->lookupTransform(
      yaw_map_frame_, yaw_base_frame_, tf2::TimePointZero);
    tf2::Quaternion q(
      tf.transform.rotation.x, tf.transform.rotation.y,
      tf.transform.rotation.z, tf.transform.rotation.w);
    double roll = 0.0;
    double pitch = 0.0;
    tf2::Matrix3x3(q).getRPY(roll, pitch, cur_yaw);
  } catch (const std::exception & e) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
      "DescendStair yaw-correct: TF lookup 失败 (%s)", e.what());
    return;
  }

  // ===== 发布朝向偏移角 (cur_yaw - target_yaw) =====
  if (offset_pub_enable_ && offset_pub_) {
    std_msgs::msg::Float32 offset_msg;
    offset_msg.data = static_cast<float>(normalizeAngle(cur_yaw - yaw_target_));
    offset_pub_->publish(offset_msg);
  }

  // ===== 朝向纠偏（只发 angular.z），仅在纠偏开启时执行 =====
  if (!yaw_correct_enable_ || !cmd_vel_pub_) {
    return;
  }

  const double err = normalizeAngle(yaw_target_ - cur_yaw);
  const double abs_err = std::fabs(err);

  // 迟滞：|err| 超过启动阈值才开始纠偏，纠到回落到停止阈值(死区)以下才停；
  // 两阈值之间保持上一状态，避免在阈值边界反复启停抖动。
  if (!yaw_engaged_) {
    if (abs_err > yaw_engage_threshold_) {
      yaw_engaged_ = true;
    }
  } else {
    if (abs_err < yaw_deadband_) {
      yaw_engaged_ = false;
    }
  }

  double wz = 0.0;
  if (yaw_engaged_) {
    wz = std::clamp(yaw_kp_ * err, -yaw_wz_max_, yaw_wz_max_);
  }

  // 只纠偏角度：angular.z 之外的所有分量显式发 0
  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = 0.0;
  cmd.linear.y = 0.0;
  cmd.linear.z = 0.0;
  cmd.angular.x = 0.0;
  cmd.angular.y = 0.0;
  cmd.angular.z = wz;
  cmd_vel_pub_->publish(cmd);
}

void DescendStairAction::stopYawTimer()
{
  {
    std::lock_guard<std::mutex> lock(yaw_timer_mutex_);
    yaw_timer_armed_ = false;
  }
  if (yaw_timer_) {
    yaw_timer_->cancel();
    yaw_timer_.reset();
  }
}

void DescendStairAction::publishZeroCmd()
{
  if (cmd_vel_pub_) {
    cmd_vel_pub_->publish(geometry_msgs::msg::Twist());
  }
}

BT::NodeStatus DescendStairAction::onRunning()
{
  // If no status received yet, keep waiting
  if (!status_received_) {
    return BT::NodeStatus::RUNNING;
  }

  // Return result based on status
  switch (current_status_) {
    case STATUS_DESCENDING:
      RCLCPP_DEBUG(node_->get_logger(), "Still descending...");
      return BT::NodeStatus::RUNNING;

    case STATUS_SUCCESS:
      RCLCPP_INFO(node_->get_logger(), "✓ Descend succeeded!");
      stopYawTimer();
      publishZeroCmd();
      return BT::NodeStatus::SUCCESS;

    case STATUS_FAILURE:
      RCLCPP_ERROR(node_->get_logger(), "✗ Descend failed!");
      stopYawTimer();
      publishZeroCmd();
      return BT::NodeStatus::FAILURE;

    default:
      RCLCPP_WARN(node_->get_logger(),
        "Unknown status: %d", current_status_);
      return BT::NodeStatus::RUNNING;
  }
}

void DescendStairAction::onHalted()
{
  RCLCPP_INFO(node_->get_logger(), "DescendStair action halted");

  // 纠偏定时器停掉并把车停稳
  stopYawTimer();
  publishZeroCmd();

  // Reset state
  current_status_ = 0;
  status_received_ = false;

  // Note: publisher and subscriber will be automatically cleaned up
}

void DescendStairAction::statusCallback(
  const std_msgs::msg::Int32::SharedPtr msg)
{
  current_status_ = msg->data;
  status_received_ = true;

  RCLCPP_DEBUG(node_->get_logger(),
    "Received climber status: %d", current_status_);
}

}  // namespace nav2_bt_publish_goal

// Register the node with BehaviorTree.CPP v4
#include "behaviortree_cpp/bt_factory.h"

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_bt_publish_goal::DescendStairAction>("DescendStair");
}
