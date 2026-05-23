#ifndef NAV2_BT_PUBLISH_GOAL__PEEK_NEXT_STEP_ACTION_HPP_
#define NAV2_BT_PUBLISH_GOAL__PEEK_NEXT_STEP_ACTION_HPP_

#include <memory>
#include <string>
#include <vector>

#include "behaviortree_cpp/action_node.h"
#include "robot_interfaces/msg/plan_step.hpp"

namespace nav2_bt_publish_goal
{

/**
 * PeekNextStep: read the front PlanStep of the shared plan vector WITHOUT
 * popping it, and write its key fields onto the blackboard with `next_*`
 * prefix so the currently-executing step's subtree can react to what comes
 * next (e.g. pre-position the arm during a MOVE if the upcoming step is
 * a PICK).
 *
 * If the plan is empty (i.e. the step we just popped was the last one) the
 * node still returns SUCCESS and writes next_step_type = "NONE" so that the
 * surrounding Repeat decorator is not disturbed.
 */
class PeekNextStepAction : public BT::SyncActionNode
{
public:
  using PlanStepVec = std::vector<robot_interfaces::msg::PlanStep>;

  PeekNextStepAction(const std::string & name, const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
};

}  // namespace nav2_bt_publish_goal

#endif  // NAV2_BT_PUBLISH_GOAL__PEEK_NEXT_STEP_ACTION_HPP_
