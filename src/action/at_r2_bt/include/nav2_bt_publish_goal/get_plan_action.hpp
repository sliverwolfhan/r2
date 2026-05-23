#ifndef NAV2_BT_PUBLISH_GOAL__GET_PLAN_ACTION_HPP_
#define NAV2_BT_PUBLISH_GOAL__GET_PLAN_ACTION_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "robot_interfaces/msg/plan.hpp"
#include "robot_interfaces/msg/plan_step.hpp"

namespace nav2_bt_publish_goal
{

/**
 * GetPlan: subscribe to /r2_planner/plan (transient_local) and write the latest
 * plan into the blackboard. Returns RUNNING until the first message arrives.
 */
class GetPlanAction : public BT::StatefulActionNode
{
public:
  using PlanStepVec = std::vector<robot_interfaces::msg::PlanStep>;

  GetPlanAction(const std::string & name, const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  void onPlan(const robot_interfaces::msg::Plan::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<robot_interfaces::msg::Plan>::SharedPtr sub_;

  std::mutex mtx_;
  std::shared_ptr<PlanStepVec> latest_plan_;
};

}  // namespace nav2_bt_publish_goal

#endif  // NAV2_BT_PUBLISH_GOAL__GET_PLAN_ACTION_HPP_
