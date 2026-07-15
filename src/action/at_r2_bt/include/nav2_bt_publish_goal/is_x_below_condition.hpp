// Copyright 2024
// Licensed under the Apache License, Version 2.0

#ifndef NAV2_BT_PUBLISH_GOAL__IS_X_BELOW_CONDITION_HPP_
#define NAV2_BT_PUBLISH_GOAL__IS_X_BELOW_CONDITION_HPP_

#include <memory>
#include <string>

#include "behaviortree_cpp/condition_node.h"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace nav2_bt_publish_goal
{

// 条件节点：读 map->base_footprint 的 x, 判断是否小于阈值。
//   cur_x < x_threshold -> SUCCESS; 否则(含 TF 取不到) -> FAILURE。
// 用于中断重来后按位置二选一进哪个重试树 (x 小=还在梅林段, x 大=已到放块段)。
class IsXBelowCondition : public BT::ConditionNode
{
public:
  IsXBelowCondition(
    const std::string & name,
    const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  // 第一次 tick 时初始化，从黑板取共享 tf_buffer，避免 lazy 自建撞 discovery
  bool ensureInitialized();

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

  // 仅在黑板没有共享 buffer 时使用的兜底
  rclcpp::Node::SharedPtr tf_node_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  bool initialized_{false};
};

}  // namespace nav2_bt_publish_goal

#endif  // NAV2_BT_PUBLISH_GOAL__IS_X_BELOW_CONDITION_HPP_
