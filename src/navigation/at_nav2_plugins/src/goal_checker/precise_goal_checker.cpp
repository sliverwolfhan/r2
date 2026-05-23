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

#include "pb_nav2_plugins/goal_checker/precise_goal_checker.hpp"

#include <cmath>

#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/utils.h"

using nav2_util::declare_parameter_if_not_declared;
using rcl_interfaces::msg::ParameterType;

namespace pb_nav2_plugins
{

PreciseGoalChecker::PreciseGoalChecker()
: x_goal_tolerance_(0.05),
  y_goal_tolerance_(0.05),
  yaw_goal_tolerance_(0.05),
  stable_duration_(0.3),
  in_band_(false)
{
}

void PreciseGoalChecker::initialize(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, const std::string & plugin_name,
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> /*costmap_ros*/)
{
  node_ = parent;
  plugin_name_ = plugin_name;
  auto node = parent.lock();

  declare_parameter_if_not_declared(
    node, plugin_name_ + ".x_goal_tolerance", rclcpp::ParameterValue(0.05));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".y_goal_tolerance", rclcpp::ParameterValue(0.05));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".yaw_goal_tolerance", rclcpp::ParameterValue(0.05));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".stable_duration", rclcpp::ParameterValue(0.3));

  node->get_parameter(plugin_name_ + ".x_goal_tolerance", x_goal_tolerance_);
  node->get_parameter(plugin_name_ + ".y_goal_tolerance", y_goal_tolerance_);
  node->get_parameter(plugin_name_ + ".yaw_goal_tolerance", yaw_goal_tolerance_);
  node->get_parameter(plugin_name_ + ".stable_duration", stable_duration_);

  in_band_ = false;

  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(&PreciseGoalChecker::dynamicParametersCallback, this, std::placeholders::_1));
}

void PreciseGoalChecker::reset()
{
  in_band_ = false;
}

bool PreciseGoalChecker::isGoalReached(
  const geometry_msgs::msg::Pose & query_pose, const geometry_msgs::msg::Pose & goal_pose,
  const geometry_msgs::msg::Twist & /*velocity*/)
{
  double dx = std::fabs(query_pose.position.x - goal_pose.position.x);
  double dy = std::fabs(query_pose.position.y - goal_pose.position.y);
  bool x_ok = dx <= x_goal_tolerance_;
  bool y_ok = dy <= y_goal_tolerance_;

  double dyaw = std::fabs(tf2::getYaw(query_pose.orientation) - tf2::getYaw(goal_pose.orientation));
  if (dyaw > M_PI) {
    dyaw = 2.0 * M_PI - dyaw;
  }
  bool yaw_ok = dyaw <= yaw_goal_tolerance_;

  bool all_ok = x_ok && y_ok && yaw_ok;

  auto node = node_.lock();
  if (!node) {
    return false;
  }
  auto now = node->now();

  if (all_ok) {
    if (!in_band_) {
      // Just entered the tolerance band, start timing
      in_band_ = true;
      in_band_start_time_ = now;
      return false;
    }
    // Check if we've been in-band long enough
    double elapsed = (now - in_band_start_time_).seconds();
    return elapsed >= stable_duration_;
  } else {
    // Left the tolerance band, reset timer
    in_band_ = false;
    return false;
  }
}

bool PreciseGoalChecker::getTolerances(
  geometry_msgs::msg::Pose & pose_tolerance, geometry_msgs::msg::Twist & vel_tolerance)
{
  pose_tolerance.position.x = x_goal_tolerance_;
  pose_tolerance.position.y = y_goal_tolerance_;
  pose_tolerance.position.z = 0.0;

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw_goal_tolerance_);
  pose_tolerance.orientation = tf2::toMsg(q);

  vel_tolerance.linear.x = 0.0;
  vel_tolerance.linear.y = 0.0;
  vel_tolerance.angular.z = 0.0;

  return true;
}

rcl_interfaces::msg::SetParametersResult PreciseGoalChecker::dynamicParametersCallback(
  std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  for (const auto & parameter : parameters) {
    const auto & name = parameter.get_name();
    if (parameter.get_type() == ParameterType::PARAMETER_DOUBLE) {
      if (name == plugin_name_ + ".x_goal_tolerance") {
        x_goal_tolerance_ = parameter.as_double();
      } else if (name == plugin_name_ + ".y_goal_tolerance") {
        y_goal_tolerance_ = parameter.as_double();
      } else if (name == plugin_name_ + ".yaw_goal_tolerance") {
        yaw_goal_tolerance_ = parameter.as_double();
      } else if (name == plugin_name_ + ".stable_duration") {
        stable_duration_ = parameter.as_double();
      }
    }
  }
  result.successful = true;
  return result;
}

}  // namespace pb_nav2_plugins

PLUGINLIB_EXPORT_CLASS(pb_nav2_plugins::PreciseGoalChecker, nav2_core::GoalChecker)
