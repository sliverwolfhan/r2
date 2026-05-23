// Copyright 2024
// Licensed under the Apache License, Version 2.0

#ifndef NAV2_BT_PUBLISH_GOAL__IS_PREP_SKIPPABLE_CONDITION_HPP_
#define NAV2_BT_PUBLISH_GOAL__IS_PREP_SKIPPABLE_CONDITION_HPP_

#include <memory>
#include <string>

#include "behaviortree_cpp/condition_node.h"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace nav2_bt_publish_goal
{

// 条件节点：判断当前位姿是否已经离 prep 点足够近，
// 三个容差全部满足即返回 SUCCESS（可跳过导航到 prep 点的步骤），
// 任意一个不满足或 TF 取不到则返回 FAILURE（继续走导航）。
class IsPrepSkippableCondition : public BT::ConditionNode
{
public:
  IsPrepSkippableCondition(
    const std::string & name,
    const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  // 第一次 tick 时初始化，从黑板取共享 tf_buffer，避免 lazy 自建撞 discovery
  bool ensureInitialized();

  // 把角度差归一化到 [-pi, pi]
  static double normalizeAngle(double a);

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

  // 仅在黑板没有共享 buffer 时使用的兜底
  rclcpp::Node::SharedPtr tf_node_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  bool initialized_{false};
};

}  // namespace nav2_bt_publish_goal

#endif  // NAV2_BT_PUBLISH_GOAL__IS_PREP_SKIPPABLE_CONDITION_HPP_
