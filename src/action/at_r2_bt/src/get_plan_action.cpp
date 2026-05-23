#include "nav2_bt_publish_goal/get_plan_action.hpp"

namespace nav2_bt_publish_goal
{

GetPlanAction::GetPlanAction(const std::string & name, const BT::NodeConfig & config)
: BT::StatefulActionNode(name, config)
{
}

BT::PortsList GetPlanAction::providedPorts()
{
  return {
    BT::InputPort<rclcpp::Node::SharedPtr>("node", "ROS node"),
    BT::InputPort<std::string>("topic", "/r2_planner/plan", "Plan topic name"),
    BT::OutputPort<std::shared_ptr<PlanStepVec>>("plan", "Shared pointer to PlanStep vector"),
    BT::OutputPort<int32_t>("num_steps", "Number of steps in the plan")
  };
}

BT::NodeStatus GetPlanAction::onStart()
{
  if (!node_) {
    if (!getInput("node", node_) || !node_) {
      RCLCPP_ERROR(rclcpp::get_logger("GetPlanAction"), "Missing/null input [node]");
      return BT::NodeStatus::FAILURE;
    }
  }

  std::string topic = "/r2_planner/plan";
  (void)getInput("topic", topic);

  {
    std::lock_guard<std::mutex> lock(mtx_);
    latest_plan_.reset();
  }

  if (!sub_) {
    auto qos = rclcpp::QoS(1).transient_local().reliable();
    sub_ = node_->create_subscription<robot_interfaces::msg::Plan>(
      topic, qos,
      std::bind(&GetPlanAction::onPlan, this, std::placeholders::_1));
    RCLCPP_INFO(node_->get_logger(),
      "GetPlanAction: subscribed to %s (transient_local)", topic.c_str());
  }

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus GetPlanAction::onRunning()
{
  std::shared_ptr<PlanStepVec> plan;
  {
    std::lock_guard<std::mutex> lock(mtx_);
    plan = latest_plan_;
  }
  if (!plan) {
    return BT::NodeStatus::RUNNING;
  }
  setOutput<std::shared_ptr<PlanStepVec>>("plan", plan);
  setOutput<int32_t>("num_steps", static_cast<int32_t>(plan->size()));
  RCLCPP_INFO(node_->get_logger(),
    "GetPlanAction: received plan with %zu steps", plan->size());
  return BT::NodeStatus::SUCCESS;
}

void GetPlanAction::onHalted()
{
  std::lock_guard<std::mutex> lock(mtx_);
  latest_plan_.reset();
}

void GetPlanAction::onPlan(const robot_interfaces::msg::Plan::SharedPtr msg)
{
  auto vec = std::make_shared<PlanStepVec>(msg->steps);
  std::lock_guard<std::mutex> lock(mtx_);
  latest_plan_ = std::move(vec);
}

}  // namespace nav2_bt_publish_goal
