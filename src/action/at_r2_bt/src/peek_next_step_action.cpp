#include "nav2_bt_publish_goal/peek_next_step_action.hpp"

#include "rclcpp/rclcpp.hpp"

namespace nav2_bt_publish_goal
{

PeekNextStepAction::PeekNextStepAction(
  const std::string & name,
  const BT::NodeConfig & config)
: BT::SyncActionNode(name, config)
{
}

BT::PortsList PeekNextStepAction::providedPorts()
{
  return {
    BT::InputPort<std::shared_ptr<PlanStepVec>>("plan", "Shared plan vector"),
    BT::OutputPort<std::string>("next_step_type", "MOVE | PICK | PUSH | NONE"),
    BT::OutputPort<int32_t>("next_target_id", "Target node id of the next step"),
    BT::OutputPort<int32_t>("next_from_id", "From node id of the next step"),
    BT::OutputPort<double>("next_cube_x", "Cube x of the next step (PICK/PUSH)"),
    BT::OutputPort<double>("next_cube_y", "Cube y of the next step (PICK/PUSH)"),
    BT::OutputPort<double>("next_block_height", "Block height of the next step")
  };
}

BT::NodeStatus PeekNextStepAction::tick()
{
  std::shared_ptr<PlanStepVec> plan;
  if (!getInput("plan", plan) || !plan || plan->empty()) {
    // 队列空（当前步是最后一步）：写默认值并 SUCCESS，
    // 让外层 Switch 走默认分支（不动机械臂），且不影响 Repeat 计数。
    setOutput<std::string>("next_step_type", "NONE");
    setOutput<int32_t>("next_target_id", 0);
    setOutput<int32_t>("next_from_id", 0);
    setOutput<double>("next_cube_x", 0.0);
    setOutput<double>("next_cube_y", 0.0);
    setOutput<double>("next_block_height", 0.0);
    return BT::NodeStatus::SUCCESS;
  }

  const auto & step = plan->front();

  using PS = robot_interfaces::msg::PlanStep;
  const char * type_str = "UNKNOWN";
  switch (step.type) {
    case PS::TYPE_MOVE: type_str = "MOVE"; break;
    case PS::TYPE_PICK: type_str = "PICK"; break;
    case PS::TYPE_PUSH: type_str = "PUSH"; break;
    default: break;
  }
  setOutput<std::string>("next_step_type", type_str);
  setOutput<int32_t>("next_target_id", step.target_id);
  setOutput<int32_t>("next_from_id", step.from_id);
  setOutput<double>("next_cube_x", step.cube_x);
  setOutput<double>("next_cube_y", step.cube_y);
  setOutput<double>("next_block_height", step.block_height);

  RCLCPP_INFO(
    rclcpp::get_logger("PeekNextStep"),
    "Peek next step: type=%s from=%d target=%d  remaining=%zu",
    type_str, step.from_id, step.target_id, plan->size());

  return BT::NodeStatus::SUCCESS;
}

}  // namespace nav2_bt_publish_goal
