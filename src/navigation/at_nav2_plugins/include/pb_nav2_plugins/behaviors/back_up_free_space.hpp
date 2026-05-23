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

#ifndef PB_NAV2_PLUGINS__BEHAVIORS__BACK_UP_FREE_SPACE_HPP_
#define PB_NAV2_PLUGINS__BEHAVIORS__BACK_UP_FREE_SPACE_HPP_

#include <memory>
#include <mutex>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_behaviors/plugins/drive_on_heading.hpp"
#include "nav2_msgs/action/back_up.hpp"
#include "rclcpp/rclcpp.hpp"

using BackUpAction = nav2_msgs::action::BackUp;

namespace pb_nav2_behaviors
{

/**
 * @class pb_nav2_behaviors::BackUpFreeSpace
 * @brief Back up toward the current navigation goal direction.
 *
 * The plugin subscribes to a latched topic (default `/AT_R2/current_nav_goal`)
 * that carries the latest navigation goal as a `PoseStamped`. When invoked it
 * computes the angle from the robot to the goal in the global frame, converts
 * it into the robot base frame and drives a small Twist along that direction
 * for the requested distance. Collision avoidance is delegated to the parent
 * `DriveOnHeading::onCycleUpdate`, which will fail the action if the heading
 * is blocked, letting the upstream `RecoveryNode` move on to other recoveries.
 */
class BackUpFreeSpace : public nav2_behaviors::DriveOnHeading<nav2_msgs::action::BackUp>
{
public:
  BackUpFreeSpace() = default;

  void onConfigure() override;
  void onCleanup() override;

  nav2_behaviors::Status onRun(const std::shared_ptr<const BackUpAction::Goal> command) override;
  nav2_behaviors::Status onCycleUpdate() override;

protected:
  void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  geometry_msgs::msg::PoseStamped::SharedPtr last_goal_;
  std::mutex goal_mutex_;

  // 缓存的本次执行命令分量（base_link 系下）
  double twist_x_{0.0};
  double twist_y_{0.0};

  // parameters
  std::string goal_topic_;
};

}  // namespace pb_nav2_behaviors

#endif  // PB_NAV2_PLUGINS__BEHAVIORS__BACK_UP_FREE_SPACE_HPP_
