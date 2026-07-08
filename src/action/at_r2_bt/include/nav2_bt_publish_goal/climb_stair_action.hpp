// Copyright 2024
// Licensed under the Apache License, Version 2.0

#ifndef NAV2_BT_PUBLISH_GOAL__CLIMB_STAIR_ACTION_HPP_
#define NAV2_BT_PUBLISH_GOAL__CLIMB_STAIR_ACTION_HPP_

#include <string>
#include <memory>
#include <mutex>

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/int32.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace nav2_bt_publish_goal
{

class ClimbStairAction : public BT::StatefulActionNode
{
public:
  ClimbStairAction(
    const std::string & name,
    const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

  // StatefulActionNode interface
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  // ROS2 components
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr climb_pub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr status_sub_;

  // State management
  int32_t current_status_;
  bool status_received_;

  // Status constants
  static constexpr int32_t STATUS_CLIMBING = 1;
  static constexpr int32_t STATUS_SUCCESS = 2;
  static constexpr int32_t STATUS_FAILURE = 3;

  // Callback function
  void statusCallback(const std_msgs::msg::Int32::SharedPtr msg);

  // ===== 爬台阶期间的朝向纠偏（只发 angular.z）=====
  // 目标朝向取自爬台阶前的准备朝向 (target_yaw / prep_yaw)，
  // 每次 tick 用 TF 拿 map->base 当前 yaw，比例控制回到目标朝向。
  bool yaw_correct_enable_{false};
  double yaw_target_{0.0};
  double yaw_kp_{1.5};
  double yaw_wz_max_{0.4};
  double yaw_deadband_{0.02};       // 停止阈值：纠到 |err| 回落到此以下才停
  double yaw_engage_threshold_{0.1};  // 启动阈值：|err| 超过此才开始纠偏
  bool yaw_engaged_{false};         // 迟滞状态位：当前是否处于"纠偏中"
  double yaw_tf_timeout_{0.1};
  std::string yaw_base_frame_{"base_link"};
  std::string yaw_map_frame_{"map"};
  std::string yaw_cmd_topic_{"/AT_R2/cmd_vel_bt"};

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::TimerBase::SharedPtr yaw_timer_;
  std::mutex yaw_timer_mutex_;
  bool yaw_timer_armed_{false};

  static double normalizeAngle(double a);
  bool ensureYawCorrection();      // 读端口 + 建 pub/tf，返回是否启用
  void yawCorrectTimerCallback();  // 定时器回调：算 wz 并发 cmd_vel
  void stopYawTimer();
  void publishZeroCmd();
};

}  // namespace nav2_bt_publish_goal

#endif  // NAV2_BT_PUBLISH_GOAL__CLIMB_STAIR_ACTION_HPP_
