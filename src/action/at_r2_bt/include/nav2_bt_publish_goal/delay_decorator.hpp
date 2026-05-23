#ifndef NAV2_BT_PUBLISH_GOAL__DELAY_DECORATOR_HPP_
#define NAV2_BT_PUBLISH_GOAL__DELAY_DECORATOR_HPP_

#include <chrono>
#include <string>

#include "behaviortree_cpp/decorator_node.h"

namespace nav2_bt_publish_goal
{

/**
 * @brief DelayDecorator: 等待指定毫秒后再 tick 子节点。
 *
 * 第一次被 tick 时开始计时，持续返回 RUNNING，
 * 直到延时结束后 tick 子节点并转发子节点的状态。
 *
 * XML 用法:
 *   <Delay delay_msec="4000">
 *     <AlwaysSuccess/>
 *   </Delay>
 */
class DelayDecorator : public BT::DecoratorNode
{
public:
  DelayDecorator(const std::string & name, const BT::NodeConfig & config);

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<unsigned>("delay_msec", "Delay in milliseconds before ticking child")};
  }

  BT::NodeStatus tick() override;

  void halt() override;

private:
  bool delay_started_{false};
  std::chrono::steady_clock::time_point start_time_;
  unsigned msec_{0};
};

}  // namespace nav2_bt_publish_goal

#endif  // NAV2_BT_PUBLISH_GOAL__DELAY_DECORATOR_HPP_
