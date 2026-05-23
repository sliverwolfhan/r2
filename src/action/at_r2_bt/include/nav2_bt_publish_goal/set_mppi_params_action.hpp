// Copyright 2024
// Licensed under the Apache License, Version 2.0

#ifndef NAV2_BT_PUBLISH_GOAL__SET_MPPI_PARAMS_ACTION_HPP_
#define NAV2_BT_PUBLISH_GOAL__SET_MPPI_PARAMS_ACTION_HPP_

#include <future>
#include <memory>
#include <string>
#include <vector>

#include "behaviortree_cpp/action_node.h"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"

namespace nav2_bt_publish_goal
{

// 运行时修改 nav2 controller_server 上 MPPI 插件实例的参数
//（参数名为 plugin_prefix + "." + 短名，例如 FollowPath.batch_size）。
// 通过 rclcpp::AsyncParametersClient 调用远端 set_parameters。
//
// 未连接的端口会被忽略；若没有任何参数可设，返回 SUCCESS。
// 服务不可用或 set 失败：WARN，仍返回 SUCCESS（与 SetCostmapInflation 一致）。
class SetMppiParamsAction : public BT::StatefulActionNode
{
public:
  SetMppiParamsAction(
    const std::string & name,
    const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

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
    bool valid{false};
    bool finished{false};
  };

  void buildParams(Target & target, const std::string & prefix);

  void sendIfReady(Target & target, double service_timeout_s);

  void evaluate(Target & target);

  rclcpp::Node::SharedPtr node_;
  Target controller_;

  rclcpp::Time start_time_;
  rclcpp::Duration wait_after_set_{0, 0};
};

}  // namespace nav2_bt_publish_goal

#endif  // NAV2_BT_PUBLISH_GOAL__SET_MPPI_PARAMS_ACTION_HPP_
