// Copyright 2024 Polaris Xia
// Copyright 2025 Lihan Chen
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "pb_nav2_plugins/behaviors/back_up_free_space.hpp"

#include <cmath>
#include <memory>

#include "tf2/utils.h"

namespace pb_nav2_behaviors
{

void BackUpFreeSpace::onConfigure()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  nav2_util::declare_parameter_if_not_declared(node, "global_frame", rclcpp::ParameterValue("map"));
  nav2_util::declare_parameter_if_not_declared(
    node, "goal_topic", rclcpp::ParameterValue("/AT_R2/current_nav_goal"));

  node->get_parameter("global_frame", global_frame_);
  node->get_parameter("goal_topic", goal_topic_);

  // Latched subscriber so we always pick up the most recent goal even if it
  // was published before this plugin was configured / activated.
  auto goal_qos = rclcpp::QoS(1).transient_local().reliable();
  goal_sub_ = node->create_subscription<geometry_msgs::msg::PoseStamped>(
    goal_topic_, goal_qos,
    std::bind(&BackUpFreeSpace::goalCallback, this, std::placeholders::_1));

  RCLCPP_INFO(
    logger_, "BackUpFreeSpace configured, listening goal on '%s'", goal_topic_.c_str());
}

void BackUpFreeSpace::onCleanup()
{
  goal_sub_.reset();
  std::lock_guard<std::mutex> lock(goal_mutex_);
  last_goal_.reset();
}

void BackUpFreeSpace::goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(goal_mutex_);
  last_goal_ = msg;
}

nav2_behaviors::Status BackUpFreeSpace::onRun(
  const std::shared_ptr<const BackUpAction::Goal> command)
{
  // Snapshot current goal under lock
  geometry_msgs::msg::PoseStamped goal_copy;
  {
    std::lock_guard<std::mutex> lock(goal_mutex_);
    if (!last_goal_) {
      RCLCPP_ERROR(
        logger_,
        "BackUpFreeSpace has no navigation goal yet (topic '%s'). Cannot decide direction.",
        goal_topic_.c_str());
      return nav2_behaviors::Status::FAILED;
    }
    goal_copy = *last_goal_;
  }

  if (!nav2_util::getCurrentPose(
        initial_pose_, *tf_, global_frame_, robot_base_frame_, transform_tolerance_)) {
    RCLCPP_ERROR(logger_, "Initial robot pose is not available.");
    return nav2_behaviors::Status::FAILED;
  }

  const double pose_x = initial_pose_.pose.position.x;
  const double pose_y = initial_pose_.pose.position.y;
  const double pose_yaw = tf2::getYaw(initial_pose_.pose.orientation);

  // 期望：goal 与机器人位姿都在 global_frame_ 下。如果 goal 的 frame_id 与
  // global_frame_ 不一致，这里会出现坐标系错位；目前先按同 frame 处理，遇到
  // 问题再补 TF。
  if (!goal_copy.header.frame_id.empty() && goal_copy.header.frame_id != global_frame_) {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 2000,
      "Goal frame_id '%s' differs from global_frame '%s'. Using raw coordinates.",
      goal_copy.header.frame_id.c_str(), global_frame_.c_str());
  }

  const double dx = goal_copy.pose.position.x - pose_x;
  const double dy = goal_copy.pose.position.y - pose_y;

  if (std::hypot(dx, dy) < 1e-6) {
    RCLCPP_WARN(logger_, "Robot is already at the goal, skipping back-up.");
    return nav2_behaviors::Status::FAILED;
  }

  // global frame 下的目标方向角 → base_link 系下
  const double goal_angle_global = std::atan2(dy, dx);
  const double angle_robot = goal_angle_global - pose_yaw;

  twist_x_ = std::cos(angle_robot) * command->speed;
  twist_y_ = std::sin(angle_robot) * command->speed;
  command_x_ = command->target.x;  // = backup_dist
  command_time_allowance_ = command->time_allowance;
  end_time_ = clock_->now() + command_time_allowance_;

  RCLCPP_WARN(
    logger_,
    "BackUpFreeSpace: moving %.3f m toward goal (global angle=%.2f rad, robot angle=%.2f rad, "
    "vx=%.3f vy=%.3f)",
    command_x_, goal_angle_global, angle_robot, twist_x_, twist_y_);

  return nav2_behaviors::Status::SUCCEEDED;
}

nav2_behaviors::Status BackUpFreeSpace::onCycleUpdate()
{
  rclcpp::Duration time_remaining = end_time_ - clock_->now();
  if (time_remaining.seconds() < 0.0 && command_time_allowance_.seconds() > 0.0) {
    stopRobot();
    RCLCPP_WARN(
      logger_,
      "Exceeded time allowance before reaching the "
      "DriveOnHeading goal - Exiting DriveOnHeading");
    return nav2_behaviors::Status::FAILED;
  }

  geometry_msgs::msg::PoseStamped current_pose;
  if (!nav2_util::getCurrentPose(
        current_pose, *tf_, global_frame_, robot_base_frame_, transform_tolerance_)) {
    RCLCPP_ERROR(logger_, "Current robot pose is not available.");
    return nav2_behaviors::Status::FAILED;
  }

  float diff_x = initial_pose_.pose.position.x - current_pose.pose.position.x;
  float diff_y = initial_pose_.pose.position.y - current_pose.pose.position.y;
  float distance = hypot(diff_x, diff_y);

  feedback_->distance_traveled = distance;
  action_server_->publish_feedback(feedback_);

  if (distance >= std::fabs(command_x_)) {
    stopRobot();
    return nav2_behaviors::Status::SUCCEEDED;
  }

  auto cmd_vel = std::make_unique<geometry_msgs::msg::Twist>();
  cmd_vel->linear.y = twist_y_;
  cmd_vel->linear.x = twist_x_;

  geometry_msgs::msg::Pose2D pose;
  pose.x = current_pose.pose.position.x;
  pose.y = current_pose.pose.position.y;
  pose.theta = tf2::getYaw(current_pose.pose.orientation);

  if (!isCollisionFree(distance, cmd_vel.get(), pose)) {
    stopRobot();
    RCLCPP_WARN(logger_, "Collision Ahead - Exiting DriveOnHeading");
    return nav2_behaviors::Status::FAILED;
  }

  vel_pub_->publish(std::move(cmd_vel));

  return nav2_behaviors::Status::RUNNING;
}

}  // namespace pb_nav2_behaviors

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(pb_nav2_behaviors::BackUpFreeSpace, nav2_core::Behavior)
