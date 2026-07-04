#include "nav2_bt_publish_goal/pop_next_step_action.hpp"

#include "rclcpp/rclcpp.hpp"

namespace nav2_bt_publish_goal
{

PopNextStepAction::PopNextStepAction(
  const std::string & name,
  const BT::NodeConfig & config)
: BT::SyncActionNode(name, config)
{
}

BT::PortsList PopNextStepAction::providedPorts()
{
  return {
    BT::BidirectionalPort<std::shared_ptr<PlanStepVec>>("plan", "Shared plan vector"),
    BT::OutputPort<std::string>("step_type", "MOVE | PICK | PUSH"),
    BT::OutputPort<int32_t>("target_id", "Target node id"),
    BT::OutputPort<int32_t>("from_id", "Robot's current node id at the moment of this step"),

    // MOVE
    BT::OutputPort<double>("prep_x", "Preparation pose x (MOVE)"),
    BT::OutputPort<double>("prep_y", "Preparation pose y (MOVE)"),
    BT::OutputPort<double>("prep_yaw", "Preparation pose yaw (MOVE)"),
    BT::OutputPort<double>("abs_dh", "|height diff| in meters (MOVE)"),
    BT::OutputPort<std::string>("stair_dir", "UP | DOWN | NONE (MOVE)"),

    // PICK / PUSH
    BT::OutputPort<double>("cube_x", "Cube x in map (PICK/PUSH)"),
    BT::OutputPort<double>("cube_y", "Cube y in map (PICK/PUSH)"),
    BT::OutputPort<double>("block_height", "Block height in meters (PICK/PUSH)"),
    BT::OutputPort<double>("grasp_yaw", "Preparation yaw for PICK/PUSH")
  };
}

BT::NodeStatus PopNextStepAction::tick()
{
  std::shared_ptr<PlanStepVec> plan;
  if (!getInput("plan", plan) || !plan) {
    RCLCPP_ERROR(rclcpp::get_logger("PopNextStep"), "Missing/empty [plan]");
    return BT::NodeStatus::FAILURE;
  }
  if (plan->empty()) {
    return BT::NodeStatus::FAILURE;
  }

  const auto step = plan->front();
  plan->erase(plan->begin());
  setOutput<std::shared_ptr<PlanStepVec>>("plan", plan);

  using PS = robot_interfaces::msg::PlanStep;
  const char * type_str = "UNKNOWN";
  switch (step.type) {
    case PS::TYPE_MOVE: type_str = "MOVE"; break;
    case PS::TYPE_PICK: type_str = "PICK"; break;
    case PS::TYPE_PUSH: type_str = "PUSH"; break;
    case PS::TYPE_WAIT: type_str = "WAIT"; break;
    default: break;
  }
  setOutput<std::string>("step_type", type_str);
  setOutput<int32_t>("target_id", step.target_id);
  setOutput<int32_t>("from_id", step.from_id);

  setOutput<double>("prep_x", step.prep_pose.x);
  setOutput<double>("prep_y", step.prep_pose.y);
  setOutput<double>("prep_yaw", step.prep_pose.theta);
  setOutput<double>("abs_dh", step.abs_dh);

  const char * dir_str = "NONE";
  if (step.stair_dir == PS::STAIR_UP) {
    dir_str = "UP";
  } else if (step.stair_dir == PS::STAIR_DOWN) {
    dir_str = "DOWN";
  }
  setOutput<std::string>("stair_dir", dir_str);

  setOutput<double>("cube_x", step.cube_x);
  setOutput<double>("cube_y", step.cube_y);
  setOutput<double>("block_height", step.block_height);
  setOutput<double>("grasp_yaw", step.grasp_yaw);

  RCLCPP_INFO(
    rclcpp::get_logger("PopNextStep"),
    "Pop step: type=%s target=%d from=%d  remaining=%zu",
    type_str, step.target_id, step.from_id, plan->size());

  return BT::NodeStatus::SUCCESS;
}

}  // namespace nav2_bt_publish_goal
