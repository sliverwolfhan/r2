#include "nav2_bt_publish_goal/delay_decorator.hpp"
#include <thread>

namespace nav2_bt_publish_goal
{

DelayDecorator::DelayDecorator(const std::string & name, const BT::NodeConfig & config)
: BT::DecoratorNode(name, config)
{
}

BT::NodeStatus DelayDecorator::tick()
{
  if (!delay_started_) {
    unsigned msec = 0;
    if (!getInput("delay_msec", msec)) {
      throw BT::RuntimeError("DelayDecorator: missing required input [delay_msec]");
    }
    msec_ = msec;
    delay_started_ = true;
    start_time_ = std::chrono::steady_clock::now();

    if (msec_ == 0) {
      // 无延时，直接 tick 子节点
      return child_node_->executeTick();
    }
    return BT::NodeStatus::RUNNING;
  }

  // 检查延时是否已到
  auto elapsed = std::chrono::steady_clock::now() - start_time_;
  auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count();

  if (static_cast<unsigned>(elapsed_ms) < msec_) {
    return BT::NodeStatus::RUNNING;
  }

  // 延时完成，tick 子节点
  auto child_status = child_node_->executeTick();
  if (child_status != BT::NodeStatus::RUNNING) {
    // 子节点结束，重置状态以备下次使用
    delay_started_ = false;
    resetChild();
  }
  return child_status;
}

void DelayDecorator::halt()
{
  delay_started_ = false;
  DecoratorNode::halt();
}

}  // namespace nav2_bt_publish_goal
