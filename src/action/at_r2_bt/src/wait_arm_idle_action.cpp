#include "nav2_bt_publish_goal/wait_arm_idle_action.hpp"

#include "behaviortree_cpp/bt_factory.h"
#include "rclcpp/rclcpp.hpp"

namespace nav2_bt_publish_goal
{

WaitArmIdleAction::WaitArmIdleAction(
  const std::string & name,
  const BT::NodeConfig & config)
: BT::StatefulActionNode(name, config)
{
}

BT::PortsList WaitArmIdleAction::providedPorts()
{
  return {
    BT::InputPort<std::string>("arm_runner_key", "arm_runner",
      "Blackboard key holding shared ArmJobRunner"),
    BT::InputPort<double>("timeout", 0.0,
      "Seconds before giving up. <=0 means wait forever."),
  };
}

BT::NodeStatus WaitArmIdleAction::onStart()
{
  runner_.reset();
  has_deadline_ = false;

  std::string runner_key{"arm_runner"};
  (void)getInput("arm_runner_key", runner_key);
  if (!config().blackboard ||
      !config().blackboard->get<std::shared_ptr<ArmJobRunner>>(runner_key, runner_) ||
      !runner_) {
    RCLCPP_ERROR(rclcpp::get_logger("WaitArmIdle"),
      "Missing shared ArmJobRunner at blackboard key [%s]", runner_key.c_str());
    return BT::NodeStatus::FAILURE;
  }

  timeout_seconds_ = 0.0;
  (void)getInput("timeout", timeout_seconds_);
  if (timeout_seconds_ > 0.0) {
    deadline_ = std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(timeout_seconds_));
    has_deadline_ = true;
  }

  if (!runner_->busy()) {
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus WaitArmIdleAction::onRunning()
{
  if (!runner_) {
    return BT::NodeStatus::FAILURE;
  }
  if (!runner_->busy()) {
    return BT::NodeStatus::SUCCESS;
  }
  if (has_deadline_ && std::chrono::steady_clock::now() >= deadline_) {
    RCLCPP_ERROR(runner_->node()->get_logger(),
      "WaitArmIdle: timeout after %.2fs", timeout_seconds_);
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::RUNNING;
}

void WaitArmIdleAction::onHalted()
{
  // BT halted；不取消臂队列，让它自然结束。需要清空请用 cancelAll。
  runner_.reset();
  has_deadline_ = false;
}

}  // namespace nav2_bt_publish_goal

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_bt_publish_goal::WaitArmIdleAction>("WaitArmIdle");
}
