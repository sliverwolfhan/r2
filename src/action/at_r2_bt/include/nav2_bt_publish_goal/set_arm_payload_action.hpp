// Copyright 2024
// Licensed under the Apache License, Version 2.0

#ifndef NAV2_BT_PUBLISH_GOAL__SET_ARM_PAYLOAD_ACTION_HPP_
#define NAV2_BT_PUBLISH_GOAL__SET_ARM_PAYLOAD_ACTION_HPP_

#include <future>
#include <memory>
#include <string>
#include <vector>

#include "behaviortree_cpp/action_node.h"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"

namespace nav2_bt_publish_goal
{

// 切换机械臂运动学模型: 设置 arm_calc 节点的 use_payload 参数。
//   payload=true  -> 带块模型 (model_2, payload KDL 链)
//   payload=false -> 不带块模型 (model, nominal KDL 链)
// 通过 rclcpp::AsyncParametersClient 调用目标节点的 set_parameters。
//
// 服务不可用或 set 失败: WARN, 仍返回 SUCCESS (与 SetMppiParams / SetCostmapInflation 一致),
// 避免一次切换失败把整个流程卡死。
class SetArmPayloadAction : public BT::StatefulActionNode
{
public:
  SetArmPayloadAction(
    const std::string & name,
    const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  using SetParametersResultFuture =
    std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>>;

  rclcpp::Node::SharedPtr node_;

  std::string arm_node_name_;
  bool payload_{false};

  rclcpp::AsyncParametersClient::SharedPtr client_;
  SetParametersResultFuture future_;
  bool request_sent_{false};
  bool finished_{false};

  rclcpp::Time start_time_;
  rclcpp::Duration wait_after_set_{0, 0};
};

}  // namespace nav2_bt_publish_goal

#endif  // NAV2_BT_PUBLISH_GOAL__SET_ARM_PAYLOAD_ACTION_HPP_
