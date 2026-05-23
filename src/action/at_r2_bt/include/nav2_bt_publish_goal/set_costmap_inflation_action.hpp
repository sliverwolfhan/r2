// Copyright 2024
// Licensed under the Apache License, Version 2.0

#ifndef NAV2_BT_PUBLISH_GOAL__SET_COSTMAP_INFLATION_ACTION_HPP_
#define NAV2_BT_PUBLISH_GOAL__SET_COSTMAP_INFLATION_ACTION_HPP_

#include <future>
#include <memory>
#include <string>
#include <vector>

#include "behaviortree_cpp/action_node.h"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"

namespace nav2_bt_publish_goal
{

// 运行时修改 nav2 局部 / 全局 costmap 的 inflation_layer 参数
// (inflation_radius 和 cost_scaling_factor)。
// 通过 rclcpp::AsyncParametersClient 调远端节点的 set_parameters 服务。
//
// 失败处理：服务连接 / SetParameters 返回失败 -> WARN, 仍然返回 SUCCESS，
// 让上层 BT 继续执行（避免一次参数失败把整个流程卡死）。
class SetCostmapInflationAction : public BT::StatefulActionNode
{
public:
  SetCostmapInflationAction(
    const std::string & name,
    const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

  // StatefulActionNode interface
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  using SetParametersResultFuture =
    std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>>;

  struct Target
  {
    std::string node_name;
    std::vector<rclcpp::Parameter> params;
    rclcpp::AsyncParametersClient::SharedPtr client;
    SetParametersResultFuture future;
    bool valid{false};      // 是否要发请求
    bool finished{false};   // future 是否已处理过
  };

  void buildParams(
    Target & target,
    const std::string & port_radius_key,
    const std::string & port_scaling_key);

  void sendIfReady(Target & target, double service_timeout_s);

  void evaluate(Target & target);

  rclcpp::Node::SharedPtr node_;
  Target local_;
  Target global_;

  rclcpp::Time start_time_;
  rclcpp::Duration wait_after_set_{0, 0};
};

}  // namespace nav2_bt_publish_goal

#endif  // NAV2_BT_PUBLISH_GOAL__SET_COSTMAP_INFLATION_ACTION_HPP_
