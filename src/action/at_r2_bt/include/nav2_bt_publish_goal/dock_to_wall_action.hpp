// Copyright 2024
// Licensed under the Apache License, Version 2.0

#ifndef NAV2_BT_PUBLISH_GOAL__DOCK_TO_WALL_ACTION_HPP_
#define NAV2_BT_PUBLISH_GOAL__DOCK_TO_WALL_ACTION_HPP_

#include <string>
#include <memory>
#include <mutex>
#include <deque>

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float64.hpp"

namespace nav2_bt_publish_goal
{

/// @brief BT 动作节点：以固定速度侧向移动(y方向)，判断车体是否已贴靠到墙/柜台。
///        两种贴住判据(二选一)：
///          1) 里程计位移(默认)：一段时间内位置变化量小于阈值 -> 贴住;
///          2) 激光测距(配置 distance_topic 时启用)：对爪子侧激光测距滤波后，
///             一段时间内距离变化量小于阈值 -> 压不动了 -> 贴住。
///        判定贴住后停止发送速度并返回 SUCCESS。
class DockToWallAction : public BT::StatefulActionNode
{
public:
  DockToWallAction(
    const std::string & name,
    const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  // ROS2 components
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr distance_sub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;

  // 参数
  // 基础压墙速度(车体系)，方向指向墙面，保持恒定
  double vy_{0.0};            // y方向速度
  double vx_{0.0};            // x方向速度
  double wz_{0.0};            // 角速度(一般为0)
  // 摇摆速度(车体系)，应设为平行墙面方向，其符号每隔 swing_period 反转一次
  double swing_vx_{0.0};      // x方向摇摆分量(m/s)
  double swing_vy_{0.0};      // y方向摇摆分量(m/s)
  double swing_period_{0.5};  // 摇摆方向反转周期(s)，<=0 表示不摇摆
  double stall_threshold_;    // 位置变化阈值(m)，低于此值认为被阻挡
  double stall_duration_;     // 需要持续多长时间位置不变才判定贴住(s)
  double timeout_;            // 最大超时时间(s)
  std::string cmd_vel_topic_;
  std::string active_cmd_vel_topic_;
  std::string odom_topic_;

  // 激光测距贴住判据(可选)：distance_topic 非空时启用，取代 odom 判据
  std::string distance_topic_;      // 爪子侧激光话题，空则用 odom
  double distance_scale_{0.001};    // 原始值(mm)转米
  bool filter_enable_{true};        // 是否中值+EMA滤波
  int median_window_{5};            // 中值窗口
  double ema_tau_{0.1};             // EMA 时间常数(s)
  bool use_distance_{false};        // 本次运行是否走激光判据
  // 距离下限门槛(激光防误检)：滤波后距离必须 < distance_stall_max 才允许把
  // "距离不变"判为贴住。避免激光被挡/读数卡住时把大距离的假不变误判成贴住。
  // <=0 表示不设门槛(仅靠"距离不变")。
  double distance_stall_max_{0.0};

  // 状态
  rclcpp::Time start_time_;
  rclcpp::Time stall_start_time_;
  bool is_stalling_{false};
  bool odom_received_{false};

  // 摇摆状态
  int swing_sign_{1};                  // 当前摇摆速度符号(+1/-1)
  rclcpp::Time swing_last_flip_time_;  // 上次反转时刻

  // 里程计位置记录
  double last_x_{0.0};
  double last_y_{0.0};
  rclcpp::Time last_odom_time_;

  // 激光测距记录 + 滤波状态(持 mutex_ 访问)
  bool distance_received_{false};
  double last_distance_{0.0};          // 上次采样的滤波后距离(m)
  rclcpp::Time last_distance_time_;    // 上次采样时刻
  std::deque<double> median_buf_;      // 中值滤波窗口
  bool ema_initialized_{false};
  double ema_value_{0.0};
  rclcpp::Time last_filter_time_;

  std::mutex mutex_;

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void distanceCallback(const std_msgs::msg::Float64::SharedPtr msg);
  double filterDistance(double raw, const rclcpp::Time & stamp);  // 持 mutex_ 调用
  void publishCmdTimerCallback();
  void publishZero();
  void stopAll();
};

}  // namespace nav2_bt_publish_goal

#endif  // NAV2_BT_PUBLISH_GOAL__DOCK_TO_WALL_ACTION_HPP_
