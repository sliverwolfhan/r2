// Copyright 2024
// Licensed under the Apache License, Version 2.0

#ifndef NAV2_BT_PUBLISH_GOAL__WAIT_GRASP_RESULT_ACTION_HPP_
#define NAV2_BT_PUBLISH_GOAL__WAIT_GRASP_RESULT_ACTION_HPP_

#include <memory>
#include <string>
#include <mutex>

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

namespace nav2_bt_publish_goal
{

/**
 * @brief 闭合爪子后等待抓取结果反馈(双话题), 成功返回 SUCCESS, 失败返回 FAILURE
 *
 * 竞技赛(单项赛)抓武器头: 发完抓取命令闭爪后调用本节点。
 * 同时订阅两个 std_msgs/Int32 话题 topic_a / topic_b, 记住各自启动后最新的值,
 * 每次消息到达用两值 *组合* 判定:
 *   - 成功: topic_a==success_a(默认1) 且 topic_b==success_b(默认0) -> SUCCESS;
 *   - 失败: topic_a==fail_a(默认0)    且 topic_b==fail_b(默认1)    -> FAILURE;
 *   - 其它任意组合(含只收到一个话题) -> 继续等待(RUNNING)。
 * 两路订阅只接受本节点启动之后到达的消息, 避免收到上一阶段的残留值。
 * timeout>0 且超时未凑齐成功/失败组合 -> 返回 timeout_result(默认 FAILURE), 交外层重试逻辑接管。
 *
 * 默认 topic_a=/AT_R2/meilin_mission_start, topic_b=/AT_R2/zone3_cmd:
 *   成功 = mission_start==1 且 zone3_cmd==0; 失败 = mission_start==0 且 zone3_cmd==1。
 *
 * 典型用法: 放在 RetryUntilSuccessful 的子 Sequence 末尾 —— FAILURE 触发外层从激光对齐重来。
 */
class WaitGraspResultAction : public BT::StatefulActionNode
{
public:
  WaitGraspResultAction(
    const std::string & name,
    const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  void topicACallback(const std_msgs::msg::Int32::SharedPtr msg);
  void topicBCallback(const std_msgs::msg::Int32::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_a_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_b_;

  std::mutex mutex_;
  // 两话题各自"启动后最新收到的值", has_* 标记是否收到过
  bool has_a_{false};
  bool has_b_{false};
  int32_t last_a_{0};
  int32_t last_b_{0};
  rclcpp::Time start_time_;
  double timeout_{0.0};

  // 两话题组合判定: 成功=(a==success_a && b==success_b), 失败=(a==fail_a && b==fail_b)
  std::string topic_a_;
  std::string topic_b_;
  int32_t success_a_{1};
  int32_t success_b_{0};
  int32_t fail_a_{0};
  int32_t fail_b_{1};
  bool timeout_result_{false};  // 超时返回值: true=SUCCESS, false=FAILURE
};

}  // namespace nav2_bt_publish_goal

#endif  // NAV2_BT_PUBLISH_GOAL__WAIT_GRASP_RESULT_ACTION_HPP_
