#ifndef NAV2_BT_PUBLISH_GOAL__ARM_MOVE_JOINT_ACTION_HPP_
#define NAV2_BT_PUBLISH_GOAL__ARM_MOVE_JOINT_ACTION_HPP_

#include <memory>
#include <string>
#include <vector>

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robot_interfaces/action/arm_task.hpp"

namespace nav2_bt_publish_goal
{

/** Sends ArmTask with joint-space goal (task_id=1 move_kfs): 6 or 7 floats in goal.data. */
class ArmMoveJointAction : public BT::StatefulActionNode
{
public:
  using ArmTask = robot_interfaces::action::ArmTask;
  using GoalHandleArmTask = rclcpp_action::ClientGoalHandle<ArmTask>;

  ArmMoveJointAction(
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

  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<ArmTask>::SharedPtr action_client_;

  GoalHandleArmTask::SharedPtr goal_handle_;
  bool goal_rejected_{false};
  bool result_received_{false};
  int32_t result_err_code_{-1};
  std::string result_reason_;
  BT::NodeStatus final_status_{BT::NodeStatus::RUNNING};
};

}  // namespace nav2_bt_publish_goal

#endif  // NAV2_BT_PUBLISH_GOAL__ARM_MOVE_JOINT_ACTION_HPP_
