// Copyright 2024
// Licensed under the Apache License, Version 2.0

#ifndef NAV2_BT_PUBLISH_GOAL__SET_ZONE_MODE_ACTION_HPP_
#define NAV2_BT_PUBLISH_GOAL__SET_ZONE_MODE_ACTION_HPP_

#include <string>
#include <memory>

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

namespace nav2_bt_publish_goal
{

/**
 * SetZoneMode: 发布一个整数到 /AT_R2/zone_mode (std_msgs/Int32)，用于切换
 * 机器人的工作区模式。这是一个"持久状态"，使用 transient_local(latched) QoS，
 * 后启动的节点也能立刻拿到当前模式。
 *
 *   mode = 1  -> 一区模式 (抓武器头、对接)
 *   mode = 2  -> 二区模式 (上下台阶、抓块)
 *   mode = 3  -> 三区模式 (预留)
 *
 * 这是一个同步动作节点：发布后立即返回 SUCCESS (fire-and-forget)。
 *
 * 输入端口:
 *   - node   (ROS node, 来自黑板)
 *   - mode   (int)    区模式: 1/2/3
 *   - topic  (string, 默认 "/AT_R2/zone_mode")
 */
class SetZoneModeAction : public BT::SyncActionNode
{
public:
  SetZoneModeAction(
    const std::string & name,
    const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr mode_pub_;
  std::string topic_;
};

}  // namespace nav2_bt_publish_goal

#endif  // NAV2_BT_PUBLISH_GOAL__SET_ZONE_MODE_ACTION_HPP_
