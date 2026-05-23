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

#ifndef PB_NAV2_PLUGINS__GOAL_CHECKER__PRECISE_GOAL_CHECKER_HPP_
#define PB_NAV2_PLUGINS__GOAL_CHECKER__PRECISE_GOAL_CHECKER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "nav2_core/goal_checker.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace pb_nav2_plugins
{

class PreciseGoalChecker : public nav2_core::GoalChecker
{
public:
  PreciseGoalChecker();
  void initialize(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, const std::string & plugin_name,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  void reset() override;
  bool isGoalReached(
    const geometry_msgs::msg::Pose & query_pose, const geometry_msgs::msg::Pose & goal_pose,
    const geometry_msgs::msg::Twist & velocity) override;
  bool getTolerances(
    geometry_msgs::msg::Pose & pose_tolerance,
    geometry_msgs::msg::Twist & vel_tolerance) override;

protected:
  double x_goal_tolerance_;
  double y_goal_tolerance_;
  double yaw_goal_tolerance_;
  double stable_duration_;  // seconds the conditions must hold
  std::string plugin_name_;

  // Stable timer state
  rclcpp::Time in_band_start_time_;
  bool in_band_;

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
  rcl_interfaces::msg::SetParametersResult dynamicParametersCallback(
    std::vector<rclcpp::Parameter> parameters);
};

}  // namespace pb_nav2_plugins

#endif  // PB_NAV2_PLUGINS__GOAL_CHECKER__PRECISE_GOAL_CHECKER_HPP_
