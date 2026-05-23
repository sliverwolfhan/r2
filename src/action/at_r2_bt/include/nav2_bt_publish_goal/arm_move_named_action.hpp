#ifndef NAV2_BT_PUBLISH_GOAL__ARM_MOVE_NAMED_ACTION_HPP_
#define NAV2_BT_PUBLISH_GOAL__ARM_MOVE_NAMED_ACTION_HPP_

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robot_interfaces/action/arm_task.hpp"

namespace nav2_bt_publish_goal
{

/**
 * ArmMoveNamed: send a joint-space move (default task_id=1) to the
 * `robotic_task` action server, looking up the 6 joint angles by name in
 * arm_task/config/arm_position.yaml.
 *
 * Input ports:
 *   - node          (ROS node, from blackboard)
 *   - name          (string)   key in arm_position.yaml under arm_positions:
 *   - server_name   (string, default "robotic_task")
 *   - task_id       (int32_t, default 1)  arm_task move_kfs id
 *   - arm_yaml      (string, optional)    absolute path to arm_position.yaml.
 *                                         empty = default share/arm_task/config/arm_position.yaml
 *
 * Returns FAILURE if the name is empty, the yaml load fails, the name is
 * not found, the joint vector size != 6, or the action goal is rejected /
 * aborted. The XML is expected to wrap this node with <ForceSuccess> when
 * its failure shouldn't propagate.
 */
class ArmMoveNamedAction : public BT::StatefulActionNode
{
public:
  using ArmTask = robot_interfaces::action::ArmTask;
  using GoalHandleArmTask = rclcpp_action::ClientGoalHandle<ArmTask>;

  ArmMoveNamedAction(
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

  bool ensureLoaded(const std::string & yaml_path);

  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<ArmTask>::SharedPtr action_client_;

  GoalHandleArmTask::SharedPtr goal_handle_;
  bool goal_rejected_{false};
  bool result_received_{false};
  int32_t result_err_code_{-1};
  std::string result_reason_;
  BT::NodeStatus final_status_{BT::NodeStatus::RUNNING};

  // Lazy-loaded named-pose cache (name -> 6 joints)
  std::map<std::string, std::vector<double>> named_positions_;
  std::string loaded_yaml_path_;
};

}  // namespace nav2_bt_publish_goal

#endif  // NAV2_BT_PUBLISH_GOAL__ARM_MOVE_NAMED_ACTION_HPP_
