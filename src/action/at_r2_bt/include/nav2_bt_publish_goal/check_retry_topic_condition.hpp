// Copyright 2024
// Licensed under the Apache License, Version 2.0

#ifndef NAV2_BT_PUBLISH_GOAL__CHECK_RETRY_TOPIC_CONDITION_HPP_
#define NAV2_BT_PUBLISH_GOAL__CHECK_RETRY_TOPIC_CONDITION_HPP_

#include <atomic>
#include <string>

#include "behaviortree_cpp/condition_node.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

namespace nav2_bt_publish_goal
{

// 放块顶墙中断条件节点。
//   - 订阅重试话题 (std_msgs/Int32, 默认 /AT_R2/place_retry), 整局常驻。
//   - 平时 (无待处理值) 返回 SUCCESS -> 放行后面的顶墙动作继续跑。
//   - 收到值 v ∈ {1,2,3,4} 时返回 FAILURE, 并把 v 写到输出口 retry_val
//     -> 放在 ReactiveSequence 里可 halt 掉正在 RUNNING 的顶墙动作, 触发退回重试。
//   - 消费即清 (exchange), 一次发布 = 一次中断。
//   - 防陈旧: 每次"重新进入顶墙" (与上一 tick 不连续) 的首个 tick 先排空,
//            保证只有顶墙进行中收到的值才触发中断, 回合间/导航中发的值不误触发。
class CheckRetryTopicCondition : public BT::ConditionNode
{
public:
  CheckRetryTopicCondition(const std::string & name, const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  // 第一次 tick 时惰性建订阅 (ConditionNode 无 onStart); 订阅整局常驻。
  bool ensureInitialized();

  void retryCallback(const std_msgs::msg::Int32::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_;

  std::atomic<int> latest_{0};      // 回调收到的最新有效值 (0 = 无)
  bool initialized_{false};
  rclcpp::Time last_tick_time_;     // 上次 tick 的时刻, 用于判断是否重新进入
  bool have_last_tick_{false};
};

}  // namespace nav2_bt_publish_goal

#endif  // NAV2_BT_PUBLISH_GOAL__CHECK_RETRY_TOPIC_CONDITION_HPP_
