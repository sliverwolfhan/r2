// Copyright 2024
// Licensed under the Apache License, Version 2.0

#ifndef NAV2_BT_PUBLISH_GOAL__MATCH_START_EDGE_GUARD_CONDITION_HPP_
#define NAV2_BT_PUBLISH_GOAL__MATCH_START_EDGE_GUARD_CONDITION_HPP_

#include <atomic>
#include <climits>
#include <string>

#include "behaviortree_cpp/condition_node.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

namespace nav2_bt_publish_goal
{

// 比赛"中途重来"守卫条件节点 (整局常驻)。
//   - 同时订阅两个启动话题 (std_msgs/Int32):
//       red_topic  (默认 /AT_R2/red_start_cmd)
//       blue_topic (默认 /AT_R2/blue_start_cmd)
//   - 检测联合跳变沿: red 跳到 red_trigger_value(默认0) 的下降沿
//     且 blue 跳到 blue_trigger_value(默认1) 的上升沿, 触发 -> 返回 FAILURE。
//   - 放在顶层 ReactiveSequence 首位: 平时返回 SUCCESS 放行主流程;
//     触发时返回 FAILURE, ReactiveSequence 会 halt 掉正在 RUNNING 的主流程
//     (级联取消导航等), 触发上层 RetryUntilSuccessful 重来。
//   - 边沿用 latch(各自锁存)容忍两话题异步到达: 任一话题出现对应边沿即置各自 latch,
//     两 latch 同时 true 才中断, 触发后清 latch 防重复。
//   - 防上电误触发: 每个话题第一次收到值时只记 prev 作基线, 不判沿。
class MatchStartEdgeGuardCondition : public BT::ConditionNode
{
public:
  MatchStartEdgeGuardCondition(const std::string & name, const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  // 第一次 tick 时惰性建订阅 (ConditionNode 无 onStart); 订阅整局常驻。
  bool ensureInitialized();

  void redCallback(const std_msgs::msg::Int32::SharedPtr msg);
  void blueCallback(const std_msgs::msg::Int32::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr red_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr blue_sub_;

  // 回调收到的最新值; INT_MIN = 尚未收到任何值 (区分未收到与合法 0)。
  std::atomic<int> red_latest_{INT_MIN};
  std::atomic<int> blue_latest_{INT_MIN};

  int prev_red_{0};
  int prev_blue_{0};
  bool have_prev_red_{false};
  bool have_prev_blue_{false};
  bool red_edge_latched_{false};
  bool blue_edge_latched_{false};
  // 边沿锁存的时刻, 用于时间窗: 先到的 latch 只在 edge_window_ 秒内有效, 超时清掉。
  rclcpp::Time red_latch_time_;
  rclcpp::Time blue_latch_time_;

  int red_trig_{0};
  int blue_trig_{1};
  double edge_window_{1.0};   // 两边沿必须在此秒数内先后到达才算联合触发
  bool initialized_{false};
};

}  // namespace nav2_bt_publish_goal

#endif  // NAV2_BT_PUBLISH_GOAL__MATCH_START_EDGE_GUARD_CONDITION_HPP_
