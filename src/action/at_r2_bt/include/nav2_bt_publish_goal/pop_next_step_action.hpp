#ifndef NAV2_BT_PUBLISH_GOAL__POP_NEXT_STEP_ACTION_HPP_
#define NAV2_BT_PUBLISH_GOAL__POP_NEXT_STEP_ACTION_HPP_

#include <memory>
#include <string>
#include <vector>

#include "behaviortree_cpp/action_node.h"
#include "robot_interfaces/msg/plan_step.hpp"

namespace nav2_bt_publish_goal
{

/**
 * PopNextStep: pop the first step from the shared plan vector and write all
 * fields into the blackboard. Returns FAILURE when the plan is empty (so a
 * surrounding Repeat decorator naturally exits).
 */
class PopNextStepAction : public BT::SyncActionNode
{
public:
  using PlanStepVec = std::vector<robot_interfaces::msg::PlanStep>;

  PopNextStepAction(const std::string & name, const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
};

}  // namespace nav2_bt_publish_goal

#endif  // NAV2_BT_PUBLISH_GOAL__POP_NEXT_STEP_ACTION_HPP_
