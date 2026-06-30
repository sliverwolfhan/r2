// Copyright 2024
// Licensed under the Apache License, Version 2.0

#ifndef NAV2_BT_PUBLISH_GOAL__DOCK_TO_TAG_ACTION_HPP_
#define NAV2_BT_PUBLISH_GOAL__DOCK_TO_TAG_ACTION_HPP_

#include <memory>
#include <mutex>
#include <string>

#include "behaviortree_cpp/action_node.h"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace nav2_bt_publish_goal
{

// 基于 tag(TF) 的全向舵轮对接控制器。
// 误差在 dock_target 系下计算，纯 P + 饱和 + 斜率限幅。
// 两阶段：
//   ALIGN    —— 收敛 dock 系下的 Y(横向) 和 yaw；vx_dock 由软门控压制
//   APPROACH —— 锁住横向/yaw，沿 dock 系 X 推进；几何或 stall 触发到位
class DockToTagAction : public BT::StatefulActionNode
{
public:
  DockToTagAction(const std::string & name, const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  enum class Phase { ALIGN, APPROACH };

  bool ensureTfBuffer();
  bool lookupRobotInDock(double & x, double & y, double & yaw, rclcpp::Time & stamp);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void publishCmdTimerCallback();
  void publishZero();
  void stopAll();

  rclcpp::Node::SharedPtr node_;
  rclcpp::Node::SharedPtr tf_node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;

  std::mutex publish_timer_mutex_;
  std::mutex cmd_mutex_;
  std::mutex odom_mutex_;
  bool publish_timer_armed_{false};

  // 端口参数
  std::string target_frame_;
  std::string base_frame_;
  std::string cmd_vel_topic_;
  std::string odom_topic_;
  double pre_dock_offset_x_{-0.5};
  double target_yaw_in_dock_{-1.5707963};  // rad, 默认 -90°
  double align_tol_lat_{0.005};
  double align_tol_yaw_{0.00873};          // rad, 默认 0.5°
  double approach_tol_{0.01};
  double align_hyst_lat_{0.015};
  double align_hyst_yaw_{0.0262};          // rad, 默认 1.5°
  double kp_x_{1.0}, kp_y_{1.5}, kp_yaw_{2.0};
  double max_vx_{0.10}, max_vy_{0.10}, max_wz_{0.5};
  double accel_vx_{0.3}, accel_vy_{0.3}, accel_wz_{1.5};
  double approach_min_speed_{0.015};
  double approach_decel_dist_{0.05};
  double gate_sigma_lat_{0.015};
  double gate_sigma_yaw_{0.0524};          // rad, 默认 3°
  double stall_threshold_{0.003};
  double stall_duration_{0.4};
  double tag_max_age_{0.5};
  double tag_lost_freeze_time_{1.0};
  double timeout_{30.0};

  // 运行态
  Phase phase_{Phase::ALIGN};
  rclcpp::Time start_time_;
  rclcpp::Time last_valid_tf_time_;
  rclcpp::Time stall_start_time_;
  bool stall_timer_active_{false};
  double last_robot_x_in_dock_{0.0};
  double last_robot_y_in_dock_{0.0};
  double last_robot_yaw_in_dock_{0.0};
  bool has_last_pose_{false};

  // 航位推算锚点：TF 新鲜时同时记录 tag 测得的 dock 位姿与同一时刻的 odom 位姿，
  // TF 丢失时用 odom 增量把 dock 位姿往前推。仅 tick 线程(onRunning)读写，无需锁。
  double anchor_dock_x_{0.0}, anchor_dock_y_{0.0}, anchor_dock_yaw_{0.0};
  double anchor_odom_x_{0.0}, anchor_odom_y_{0.0}, anchor_odom_yaw_{0.0};
  bool has_anchor_{false};

  geometry_msgs::msg::Twist cmd_;          // 当前下发指令(body 系)
  double prev_vx_body_{0.0};
  double prev_vy_body_{0.0};
  double prev_wz_{0.0};
  rclcpp::Time prev_cmd_time_;

  // odom stall 监测
  double odom_ref_x_{0.0}, odom_ref_y_{0.0};
  double odom_cur_x_{0.0}, odom_cur_y_{0.0}, odom_cur_yaw_{0.0};
  bool odom_received_{false};
  rclcpp::Time odom_ref_time_;
};

}  // namespace nav2_bt_publish_goal

#endif  // NAV2_BT_PUBLISH_GOAL__DOCK_TO_TAG_ACTION_HPP_
