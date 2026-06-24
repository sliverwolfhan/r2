#include "nav2_bt_publish_goal/arm_move_named_async_action.hpp"

#include "behaviortree_cpp/bt_factory.h"
#include "rclcpp/rclcpp.hpp"

namespace nav2_bt_publish_goal
{

ArmMoveNamedAsyncAction::ArmMoveNamedAsyncAction(
  const std::string & name,
  const BT::NodeConfig & config)
: BT::SyncActionNode(name, config)
{
}

BT::PortsList ArmMoveNamedAsyncAction::providedPorts()
{
  return {
    BT::InputPort<std::string>("arm_runner_key", "arm_runner",
      "Blackboard key holding shared ArmJobRunner"),
    BT::InputPort<std::string>("pose_name", "Named pose key in arm_position.yaml"),
    BT::InputPort<int32_t>("task_id", 1, "Arm task id (move_kfs = 1)"),
    BT::InputPort<double>("duration", 3.0, "Trajectory duration in seconds"),
    BT::InputPort<std::string>("arm_yaml", "",
      "Optional override path to arm_position.yaml; empty = runner default"),
  };
}

BT::NodeStatus ArmMoveNamedAsyncAction::tick()
{
  std::string runner_key{"arm_runner"};
  (void)getInput("arm_runner_key", runner_key);

  std::shared_ptr<ArmJobRunner> runner;
  if (!config().blackboard ||
      !config().blackboard->get<std::shared_ptr<ArmJobRunner>>(runner_key, runner) ||
      !runner) {
    RCLCPP_ERROR(rclcpp::get_logger("ArmMoveNamedAsync"),
      "Missing shared ArmJobRunner at blackboard key [%s]", runner_key.c_str());
    return BT::NodeStatus::FAILURE;
  }

  std::string pose_name;
  if (!getInput("pose_name", pose_name) || pose_name.empty()) {
    RCLCPP_ERROR(runner->node()->get_logger(),
      "ArmMoveNamedAsync: empty [pose_name]");
    return BT::NodeStatus::FAILURE;
  }

  int32_t task_id = 1;
  (void)getInput("task_id", task_id);
  double duration = 3.0;
  (void)getInput("duration", duration);
  std::string yaml_path;
  (void)getInput("arm_yaml", yaml_path);

  const bool ok = runner->enqueueNamed(pose_name, task_id, duration, yaml_path);
  if (!ok) {
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::SUCCESS;
}

}  // namespace nav2_bt_publish_goal

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_bt_publish_goal::ArmMoveNamedAsyncAction>(
    "ArmMoveNamedAsync");
}
