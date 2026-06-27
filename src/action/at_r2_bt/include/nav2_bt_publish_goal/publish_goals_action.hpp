// Copyright 2024
// Licensed under the Apache License, Version 2.0

#ifndef NAV2_BT_PUBLISH_GOAL__PUBLISH_GOALS_ACTION_HPP_
#define NAV2_BT_PUBLISH_GOAL__PUBLISH_GOALS_ACTION_HPP_

#include <string>
#include <vector>
#include <memory>

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"

namespace nav2_bt_publish_goal
{

// 调用 nav2 NavigateThroughPoses, 一次性把多个路径点送进去,
// global planner 用 ComputePathThroughPoses 拼成一整条路径,
// controller 连续跟踪, 中间点不停。
class PublishGoalsAction : public BT::StatefulActionNode
{
public:
  using NavigateThroughPoses = nav2_msgs::action::NavigateThroughPoses;
  using GoalHandleNavigateThroughPoses =
    rclcpp_action::ClientGoalHandle<NavigateThroughPoses>;

  PublishGoalsAction(
    const std::string & name,
    const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<NavigateThroughPoses>::SharedPtr action_client_;
  GoalHandleNavigateThroughPoses::SharedPtr goal_handle_;
  bool goal_result_available_;
  rclcpp_action::ClientGoalHandle<NavigateThroughPoses>::WrappedResult result_;

  // 把最后一个目标点 latched 出去, 与 PublishGoal 一致, 供 BackUpFreeSpace 读取
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr current_goal_pub_;

  // 解析 "x1,y1,yaw1; x2,y2,yaw2; ..." -> PoseStamped[]
  bool parsePoses(
    const std::string & poses_str,
    const std::string & frame_id,
    std::vector<geometry_msgs::msg::PoseStamped> & out);

  geometry_msgs::msg::PoseStamped createGoalMessage(
    double x, double y, double yaw, const std::string & frame_id);
};

}  // namespace nav2_bt_publish_goal

#endif  // NAV2_BT_PUBLISH_GOAL__PUBLISH_GOALS_ACTION_HPP_
