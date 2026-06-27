#ifndef NAV2_BT_PUBLISH_GOAL__ARM_CARTESIAN_TASK_ACTION_HPP_
#define NAV2_BT_PUBLISH_GOAL__ARM_CARTESIAN_TASK_ACTION_HPP_

#include <memory>
#include <string>
#include <vector>

#include "behaviortree_cpp/action_node.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robot_interfaces/action/arm_task.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace nav2_bt_publish_goal
{

/**
 * ArmCartesianTask: 笛卡尔模式末端运动 (默认 task_id=4)。
 *
 * 输入 cube_x / cube_y (map 系) -> lookupTransform(base_frame, map_frame) ->
 * 把 map 系下的点变换到 base_frame 下, 发给 robotic_task action server。
 *
 * 发送约定 (与现有客户端对齐):
 *   task_id = 4 (默认, 可改)
 *   data    = [x, y, z, qx, qy, qz, qw]   7 维
 *             x, y = TF 变换后 base_frame 系下的值 (来自 map 系 cube_x / cube_y)
 *             z    = 输入 cube_z 固定值 (不做 TF 变换)
 *             机械臂端忽略四元数, 但仍要凑齐 7 维。
 *
 * 与 ArmTask 的差异: 没有 grasp_height 字段, data 是 7 维不是 8 维。
 */
class ArmCartesianTaskAction : public BT::StatefulActionNode
{
public:
  using ArmTask = robot_interfaces::action::ArmTask;
  using GoalHandleArmTask = rclcpp_action::ClientGoalHandle<ArmTask>;

  ArmCartesianTaskAction(
    const std::string & name,
    const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  void onGoalResponse(const GoalHandleArmTask::SharedPtr & goal_handle);
  void onFeedback(
    GoalHandleArmTask::SharedPtr,
    const std::shared_ptr<const ArmTask::Feedback> feedback);
  void onResult(const GoalHandleArmTask::WrappedResult & result);

  bool buildGoalDataFromMap(
    const std::string & base_frame,
    const std::string & map_frame,
    double tf_timeout_sec,
    double cube_x,
    double cube_y,
    double cube_z,
    std::vector<double> & out_goal_data);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Node::SharedPtr tf_node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp_action::Client<ArmTask>::SharedPtr action_client_;

  GoalHandleArmTask::SharedPtr goal_handle_;
  bool goal_rejected_{false};
  bool result_received_{false};
  int32_t result_err_code_{-1};
  std::string result_reason_;
  BT::NodeStatus final_status_{BT::NodeStatus::RUNNING};
};

}  // namespace nav2_bt_publish_goal

#endif  // NAV2_BT_PUBLISH_GOAL__ARM_CARTESIAN_TASK_ACTION_HPP_
