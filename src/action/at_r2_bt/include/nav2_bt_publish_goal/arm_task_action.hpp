#ifndef NAV2_BT_PUBLISH_GOAL__ARM_TASK_ACTION_HPP_
#define NAV2_BT_PUBLISH_GOAL__ARM_TASK_ACTION_HPP_

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

class ArmTaskAction : public BT::StatefulActionNode
{
public:
  using ArmTask = robot_interfaces::action::ArmTask;
  using GoalHandleArmTask = rclcpp_action::ClientGoalHandle<ArmTask>;

  ArmTaskAction(
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

  bool loadGoalDataFromTfOrFallback(
    const std::string & base_frame,
    const std::string & target_frame,
    const std::string & map_frame,
    double tf_timeout_sec,
    double grasp_height,
    double cube_x,
    double cube_y,
    double block_height,
    std::vector<double> & out_goal_data);

  rclcpp::Node::SharedPtr node_;
  // 专用于订阅 TF 的子节点（带命名空间，例如 /AT_R2 → /AT_R2/tf, /AT_R2/tf_static）。
  // 用 spin_thread=true 自带后台线程 spin，所以不会和 BT 主 node 抢资源。
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

#endif  // NAV2_BT_PUBLISH_GOAL__ARM_TASK_ACTION_HPP_
