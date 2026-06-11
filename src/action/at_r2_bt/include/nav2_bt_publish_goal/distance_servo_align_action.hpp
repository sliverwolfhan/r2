// Copyright 2024
// Licensed under the Apache License, Version 2.0

#ifndef NAV2_BT_PUBLISH_GOAL__DISTANCE_SERVO_ALIGN_ACTION_HPP_
#define NAV2_BT_PUBLISH_GOAL__DISTANCE_SERVO_ALIGN_ACTION_HPP_

#include <memory>
#include <mutex>
#include <string>

#include "behaviortree_cpp/action_node.h"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

namespace nav2_bt_publish_goal
{

// 根据测距误差做比例速度闭环，对准到目标距离。
// error = target_distance - current_distance，速度方向可通过 cmd_axis 和
// positive_error_direction 在 XML 中调整，便于现场调参。
class DistanceServoAlignAction : public BT::StatefulActionNode
{
public:
  DistanceServoAlignAction(
    const std::string & name,
    const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  void distanceCallback(const std_msgs::msg::Float64::SharedPtr msg);
  void publishCmdTimerCallback();
  void publishZero();
  void stopAll();
  void setZeroCommand();
  double computeSpeed(double error) const;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr distance_sub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;

  std::mutex mutex_;
  std::mutex publish_timer_mutex_;
  bool publish_timer_armed_{false};

  std::string cmd_vel_topic_;
  std::string distance_topic_;
  std::string cmd_axis_{"x"};

  double latest_distance_{0.0};
  rclcpp::Time latest_distance_time_;
  bool has_distance_{false};

  geometry_msgs::msg::Twist cmd_;

  rclcpp::Time start_time_;
  int stable_count_current_{0};

  double distance_scale_{1.0};
  double target_distance_{0.0};
  double tolerance_{0.005};
  double kp_{0.8};
  double max_speed_{0.06};
  double min_speed_{0.015};
  double max_data_age_{0.5};
  double timeout_{8.0};
  double positive_error_direction_{1.0};
  int stable_count_required_{3};
};

}  // namespace nav2_bt_publish_goal

#endif  // NAV2_BT_PUBLISH_GOAL__DISTANCE_SERVO_ALIGN_ACTION_HPP_
